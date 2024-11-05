#include <iostream>

// Program for conducting measurement on the testbed using Sun Sensor and Inclinometer
// Van K. Vu, Josh G. Zhou, Tatsuyoshi C. Kurumiya, David M. Auslander
// Space Sciences Lab, UC Berkeley
// May 20th, 2024
// Update: June 21st: Updated full position control and Master Library.
//         June 22nd: Full control software with TORP control and Master control
//         June 25th: Add Finding Sun state and change to faster ramping up time for the TORP

#include <unistd.h>
#include <iostream>
#include <string>
#include <string.h>
#include <time.h>
#include <fstream>
#include <iomanip>
#include <math.h>
#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <errno.h>
#include <signal.h>
#include <stdlib.h>

// Link Master Library
#include "Master Library/GrotiferMaster.hpp"


// Libraries for altitude determination
using namespace std;

// Definitions of paths for serial components
#define SUNSENSOR_SERIAL_PORT "/dev/digital_sun_sensor"
#define RIGHT_STEPPER_1 "/dev/right_boom_stepper_1"
#define RIGHT_STEPPER_2 "/dev/right_boom_stepper_2"
#define LEFT_STEPPER_1 "/dev/left_boom_stepper_1"
#define LEFT_STEPPER_2 "/dev/left_boom_stepper_2"

// Timer functions
double g_t0;   // Time at timer initialization

// ========================================================================================================================================

// DEFINITIONS OF BASIC CLASS FOR ALL THE TASKS OF GROTIFER'S CONTROL SOFTWARE ============================================================
class TorpControl : public BaseTask
{
    public:
        TorpControl(char* taskName, unsigned int taskID, double taskPeriod,
                    homingProfilePara &hpp, PIControl &pi, MaxonMotor &mm, LJEncoder3Channels &ljEnc3C,
                    bool useMotEncFlag, double gearRatio, ofstream &torpControlDataFile, ofstream &auditTrailDataFile)
        {
            p_taskName = taskName;
            p_taskID = taskID;
            p_deltaTaskTime = taskPeriod;
            p_auditTrailDataFile = &auditTrailDataFile;
            p_torpControlDataFile = &torpControlDataFile;
            p_homingVel = hpp.homingVel;
            p_maxAcc = hpp.maxAcc;
            p_offsetPos = hpp.offsetPos;
            p_offsetPosLim = hpp.offsetPosLim;
            p_startPosLim = hpp.startPosLim;
            p_gearRatio = gearRatio;
            p_flipSign = (p_homingVel >= 0) ? false : true;
            p_pi = &pi;
            p_mm = &mm;
            p_ljEnc3C = &ljEnc3C;
            p_useMotEncFlag = useMotEncFlag;
            p_state = INITIALIZING; p_nextState = INITIALIZING;
            p_nextStateName = "Initializing";
        }
        ~TorpControl() {}

        int Run() override
        {
            if (!p_allowedToRun)    // The task is not allow to run
            {
                (*p_mm).HaltMotion();
                return -1;
            }

            int done = 1;   // Default return value indicating done for this event
            p_numScans++;   // Increasement the scan count

            // Entry code -> check to see if the state is synchronizing state
            if (p_state == SYNCHRONIZING)
                goto syncExecution;

            if(GetTimeNow() < p_nextTaskTime)  // It's not the time for the task to run
            {
                p_runSuccess = false;
                return done; // Done for now. Not the propriate time for the task to run
            }

            p_timeStart = GetTimeNow();
            p_nextTaskTime += p_deltaTaskTime; // Increase the next time that the task can run
            p_runSuccess = true;    // Mark this as a successful run

            // Calculate the actual time duration between two runs
            p_time = GetTimeNow();
            p_deltaT = p_time - p_preTime;  // Actual delta T for integral
            p_preTime = p_time;

            p_state = p_nextState;  // Update the current state

            switch (p_state)
            {
                case INITIALIZING:
                    // Entry code
                    p_stateName = "Initializing";

                    // Check to see if the system is homed manually or automatically
                    if (p_readySync) // Check to see if the system is homed manually
                    {
                        p_state = SYNCHRONIZING;    // Switch current and next state to synchronizing immediately
                                                    // Use only when homing is done by hand
                        p_stateName = "Synchronizing";
                        p_nextState = SYNCHRONIZING;
                        p_nextStateName = "Synchronizing";

                        p_startPosAct = (p_useMotEncFlag) ? (*p_mm).GetPositionIs() : (*p_ljEnc3C).GetAngularPosDeg();    // Record the starting value
                    }
                    else    // Homing is done automatically
                    {
                        p_nextState = FINDING_HOME_INDEX;   // Set next state into "Finding home index"
                        p_nextStateName = "FindingHomeIndex";
                        p_tA = GetTimeNow();    p_tB = p_tA + abs(p_homingVel / p_maxAcc) - p_deltaTaskTime;    // Calculater tA and tB for the traperzoidal profile
                        p_refAcc = ((double) GetSignDir(p_flipSign)) * p_maxAcc;
                        p_preTime = GetTimeNow();
                    }

                    break;

                case FINDING_HOME_INDEX:
                    // Entry code
                    p_stateName = "FindingHomeIndex";
                    p_motPos = (*p_mm).GetPositionIs();
                    p_motVel = (*p_mm).GetVelocityIs();
                    p_torpPos = (*p_ljEnc3C).GetAngularPosDeg();
                    p_torpVel = p_velMAFilter.addSample(((p_torpPos - p_torpPrePos) / 360.0)/ (p_deltaT / 60.0)); // Convert from deg/s -> rpm
                    p_torpPrePos = p_torpPos;

                    // Calculate the reference velocity
                    if (GetTimeNow() > p_tB)    // Check to switch between accelerating and holding const velocity
                    {
                        p_refAcc = 0;
                        p_refVel = p_homingVel;
                    }
                    else
                    {
                        p_refVel = p_refVel + p_refAcc * p_deltaT;
                    }

                    // Condition to switch state: Check to see if index has been reached
                    if ((*p_ljEnc3C).GetIndexFlag())
                    {
                        p_indexFlag = true;
                        p_nextState = MOVING_TO_STARTING_POS;
                        p_nextStateName = "Moving2StartPos";
                        p_homeTorpPos = p_torpPos;  // Capture the home position of the Torp
                        p_homeMotPos = p_motPos;    // Capture the home position of the motor
                        p_startPosRef = ((!p_useMotEncFlag) ? (p_homeTorpPos + ((double) GetSignDir(p_flipSign)) * p_offsetPos)
                                                            : (p_homeMotPos + ((double) GetSignDir(p_flipSign)) * p_offsetPos));    // Calculate the starting position

                        if (abs(p_offsetPos) >= p_offsetPosLim)
                        {
                            double a_min = abs((pow(p_refVel, 2) / (2*p_offsetPos)) * 6);
                            if (a_min >= p_maxAcc) p_accMag = a_min;
                            else if (a_min <= 0.2 * p_maxAcc) p_accMag = p_maxAcc;
                            else p_accMag = 5.0 * a_min;

                            p_tA = abs((p_offsetPos / CONV_RAD_TO_DEG) / (p_refVel * CONV_RPM_TO_RADpSEC)) + GetTimeNow() - abs(p_refVel / (2 * p_accMag));
                            p_tB = abs((p_offsetPos / CONV_RAD_TO_DEG) / (p_refVel * CONV_RPM_TO_RADpSEC)) + GetTimeNow() + abs(p_refVel / (2 * p_accMag));
                        }
                        else
                        {
                            p_accMag = 0;
                            p_tA = GetTimeNow(); p_tB = p_tA;
                        }
                    }

                    break;


                case MOVING_TO_STARTING_POS:
                    // Entry code
                    p_stateName = "Moving2StartPos";
                    p_motPos = (*p_mm).GetPositionIs();
                    p_motVel = (*p_mm).GetVelocityIs();
                    p_torpPos = (*p_ljEnc3C).GetAngularPosDeg();
                    p_torpVel = p_velMAFilter.addSample(((p_torpPos - p_torpPrePos) / 360.0)/ (p_deltaT / 60.0)); // Convert from deg/s -> rpm
                    p_torpPrePos = p_torpPos;

                    // Execution code
                    p_refAcc = ((GetTimeNow() >= p_tA && GetTimeNow() <= p_tB) ? (((double) GetSignDir(p_flipSign)) * (-p_accMag)) : 0);

                    p_refVel = ((GetTimeNow() > p_tB) ? 0 : (p_refVel + p_refAcc * p_deltaT));

                    // Condition to switch state: check to see the current position is within the limit
                    if (p_useMotEncFlag)    // In case of using motor encoder
                    {
                        if (abs(p_motPos - p_startPosRef) <= 1.25 * p_startPosLim || abs(p_refVel) <= 0.035 * p_gearRatio)
                        {
                            if ((abs(p_motPos - p_startPosRef) <= p_startPosLim) && (abs(p_refVel) <= 0.01 * p_gearRatio) && (abs(p_motVel) <= 0.25 * p_gearRatio))
                            {
                                p_nextState = WAITING;
                                p_nextStateName = "Waiting";
                                p_startPosAct = p_startPosRef;
                                (*p_mm).HaltMotion();
                            }
                            p_startPosRegionFlag = true;
                        }
                    }
                    else
                    {
                        if (abs(p_torpPos - p_startPosRef) <= 1.25 * p_startPosLim || abs(p_refVel) <= 0.035)
                        {
                            if ((abs(p_torpPos - p_startPosRef) <= p_startPosLim) && (abs(p_refVel) <= 0.01) && (abs(p_torpVel) <= 0.25))
                            {
                                p_nextState = WAITING;
                                p_nextStateName = "Waiting";
                                p_startPosAct = p_startPosRef;
                                (*p_mm).HaltMotion();
                            }
                            p_startPosRegionFlag = true;
                        }
                    }

                    break;


                case WAITING:
                    // Entry Code
                    p_stateName = "Waiting";
                    p_motPos = (*p_mm).GetPositionIs();
                    p_motVel = (*p_mm).GetVelocityIs();
                    p_torpPos = (*p_ljEnc3C).GetAngularPosDeg();
                    p_torpVel = p_velMAFilter.addSample(((p_torpPos - p_torpPrePos) / 360.0)/ (p_deltaT / 60.0)); // Convert from deg/s -> rpm
                    p_torpPrePos = p_torpPos;

                    p_doneHomingFlag = true;

                    p_refAcc = 0;
                    p_refVel = 0;

                    if (p_readySync)
                    {
                        // Switch current and next state to synchronizing immediately
                        p_state = SYNCHRONIZING;
                        p_stateName = "Synchronizing";
                        p_nextState = SYNCHRONIZING;
                        p_nextStateName = "Synchronizing";
                    }

                    break;

                case SYNCHRONIZING:
                    syncExecution:
                        if (p_enableRunning)
                        {
                            p_timeStart = GetTimeNow();
                            p_runSuccess = true;    // Mark this as a successful run

                            // Calculate the actual time duration between two runs
                            p_time = GetTimeNow();
                            p_deltaT = p_time - p_preTime;  // Actual delta T for integral
                            p_preTime = p_time;

                            // Get the motion's data
                            p_motPos = (*p_mm).GetPositionIs();
                            p_motVel = (*p_mm).GetVelocityIs();
                            p_torpPos = (*p_ljEnc3C).GetAngularPosDeg();
                            p_torpVel = p_velMAFilter.addSample(((p_torpPos - p_torpPrePos) / 360.0)/ (p_deltaT / 60.0)); // Convert from deg/s -> rpm
                            p_torpPrePos = p_torpPos;

                            // Run the PI Control to calculate the desired velocity
                            p_desVel = (*p_pi).PICalculation(p_refPos, (p_useMotEncFlag) ? p_motPos : p_torpPos) / CONV_TURN2DEG + p_refVel;
                            p_posErr = (*p_pi).GetError();

                            // Actuate the motor
                            (*p_mm).RunWithVelocity(roundingFunc(p_desVel * ((p_useMotEncFlag) ? 1.0 : p_gearRatio)));

                            // Flip the enable flag back to false
                            p_enableRunning = false;

                            // Log Data
                            *p_torpControlDataFile << left << setw(w) << GetTimeNow() << left << setw(w) << GetHomeTorpPos() << left << setw(w) << GetStartingPosRef()
                                                                      << left << setw(w) << GetRefPosition() << left << setw(w) << GetTorpPos()
                                                                      << left << setw(w) << GetRefVelocity() << left << setw(w) << GetTorpVel()
                                                                      << left << setw(w) << GetMotorPos() << left << setw(w) << GetMotorVel()
                                                                      << left << setw(w) << GetPosError() << left << setw(w) << GetDesVelCmd() << endl;

                            // Get the timestamp and finish the task
                            p_timeEnd = GetTimeNow();  // Get the end time stamp for the task
                            AuditTrailRecord();  // Record the state here
                            return done;
                        }

                        else
                        {
                            p_runSuccess = false;
                            return done;
                        }
                    break;

            }

            // Calculate the value for the reference position
            p_refPos = (p_startPosRegionFlag || p_doneHomingFlag) ? p_startPosRef : (p_refPos + p_refVel * (p_deltaT / 60.0) * (CONV_TURN2DEG));

            // Run the PI Control
            p_desVel = (*p_pi).PICalculation(p_refPos, (p_useMotEncFlag) ? p_motPos : p_torpPos) / CONV_TURN2DEG + p_refVel;
            p_posErr = (*p_pi).GetError();

            // Actuate the motor
            (*p_mm).RunWithVelocity(roundingFunc(p_desVel * ((p_useMotEncFlag) ? 1.0 : p_gearRatio)));

            // Log the data
            *p_torpControlDataFile << left << setw(w) << GetTimeNow() << left << setw(w) << GetHomeTorpPos() << left << setw(w) << GetStartingPosRef()
                                                                      << left << setw(w) << GetRefPosition() << left << setw(w) << GetTorpPos()
                                                                      << left << setw(w) << GetRefVelocity() << left << setw(w) << GetTorpVel()
                                                                      << left << setw(w) << GetMotorPos() << left << setw(w) << GetMotorVel()
                                                                      << left << setw(w) << GetPosError() << left << setw(w) << GetDesVelCmd() << endl;

            // Get the timestamp and finish the task
            p_timeEnd = GetTimeNow();  // Get the end time stamp for the task
            AuditTrailRecord();  // Record the state here
            return done;


        }

        // Get data function
        double GetRefPosition() {return p_refPos;}  // Get the value of reference position, [deg]
        double GetRefVelocity() {return p_refVel;}  // Get the value of the reference velocity, [rpm]
        double GetRefAcc()  {return p_refAcc;}  // Get the value of the reference acceleration, [rpm/s]
        double GetHomeTorpPos() {return p_homeTorpPos;}    // Get the home position of Torp, [deg]
        double GetHomeMotPos()  {return p_homeMotPos;}  // Get the home position of the motor, [deg]
        double GetStartingPosAct() {return p_startPosAct;}    // Get the Actual starting position, [deg]
        double GetStartingPosRef() {return p_startPosRef;}  // Get the Reference starting position, [deg]
        bool GetDoneHomingFlag() {return p_doneHomingFlag;} // Get the "Done Homing" flag
        bool GetIndexFlag() {return p_indexFlag;}   // Get the "Index" flag
        double GetMotorPos() {return p_motPos;} // Get the position of the motor, [deg]
        double GetMotorVel() {return p_motVel;} // Get the velocity of the motor, [rpm]
        double GetPosError() {return p_posErr;}   // Get the position error, [deg]
        double GetTorpPos() {return p_torpPos;} // Get the position of the torp, [deg]
        double GetTorpVel() {return p_torpVel;} // Get the velocity of the torp, [rpm]
        double GetDesVelCmd() {return p_desVel;}    // Get the desired velocity command sent to Maxon, [rpm]
        bool GetUseMotEncFlag() {return p_useMotEncFlag;};  // Get the use motor's encoder flag

        // Halt motion function
        void HaltMotion()
        {
            (*p_mm).HaltMotion();
        }


        // Function to set reference position and velocity
        void SetRefPositionVelocity(double refPos, double refVel)
        {
            p_refPos = p_startPosAct + ((double) GetSignDir(p_flipSign)) * refPos;
            p_refVel = ((double) GetSignDir(p_flipSign)) * refVel;
        }

        // Function to get the sign of the direction
        int GetSignDir(bool flipSignFlag)
        {
            int signResult = (!flipSignFlag) ? 1 : -1;
            return signResult;
        }

        // Functions to set the flags
        void SetReadySyncFlag(bool val)
        {
            p_readySync = val;
        }

        void SetEnableFlag(bool val)
        {
            p_enableRunning = val;
        }

        // Rounding function
        int roundingFunc(double val)
        {
            double threshold = 0.5;
            double diff = abs(val - (int) val);
            int result = 0;
            if (val >= 0)   // Check if the value is positive
            {
                result = (int) (diff >= threshold) ? (int) val + 1 : (int) val;
            }
            else    // Value is negative
            {
                result = (int) (diff >= threshold) ? (int) val - 1 : (int) val;
            }
            return result;
        }


    protected:
        MaxonMotor *p_mm = NULL;
        PIControl *p_pi = NULL;
        LJEncoder3Channels *p_ljEnc3C = NULL;

        double p_torpPos = 0,       // Torp angular position, [deg]
               p_torpPrePos = 0,    // Torp previous angular position, [deg]
               p_torpVel = 0,       // Torp angular velocity, [rpm]
               p_motPos = 0,        // Motor position, [deg]
               p_motVel = 0,        // Motor Velocity, [RPM]
               p_posErr = 0;        // Error in position, [deg]

        double p_gearRatio = 1;     // Gear ratio between the motor and the torp

        double p_time = 0, p_preTime = 0, p_deltaT = 0; // Time value to calculate the integral [sec]

        double p_tA = 0, p_tB = 0;  // Time to change the homing velocity profile. [sec]

        double p_homingVel = 0,     // Homing velocity, [rpm]
               p_maxAcc = 0,        // Maximum acceleration, [RPM/s]
               p_accMag = 0,        // Magnitude of acceleration, [RPM/s]
               p_offsetPos = 0,     // Offset position from the home position, [deg]
               p_offsetPosLim = 0,  // Offset position limit, [deg]
               p_startPosLim = 0,   // Limit of how close the system to the starting position, [deg]
               p_homeTorpPos = 0,   // Home position of the torp, [deg]
               p_homeMotPos = 0,    // Home position of the motor, [deg]
               p_startPosRef = 0,   // Reference starting position, [deg]
               p_startPosAct = 0,   // Actual starting position, [deg]
               p_refPos = 0,        // Reference position, [deg]
               p_refAcc = 0,        // Reference acceleration profile [RPM/s]
               p_refVel = 0,        // Reference velocity profile, [RPM]
               p_desVel = 0;        // Desired velocity to send to Maxon, [RPM]

        bool p_flipSign = false;    // flag to know if the direction is CCW (+) or CW (-)

        bool p_useMotEncFlag = false;   // Flag to know if the main encoder is the motor's encoder

        bool p_doneHomingFlag = false,      // Done homing flag
             p_indexFlag = false,           // Index flag
             p_readySync = false,           // Ready for synchronization flag
             p_startPosRegionFlag = false,  // Position is within starting pos region flag
             p_enableRunning = false;       // Enable to run in synchronization state

        // Moving average filter for angular velocity
        int p_movingAverageFilterSpan = 5;
        MovingAverage p_velMAFilter{p_movingAverageFilterSpan};

        ofstream *p_torpControlDataFile = NULL;
        unsigned int w = 25;

        static const unsigned int INITIALIZING = 0;             // Initializing state
        static const unsigned int FINDING_HOME_INDEX = 1;       // Finding home/index state
        static const unsigned int MOVING_TO_STARTING_POS = 2;   // Moving to starting position state
        static const unsigned int WAITING = 3;                  // Waiting state
        static const unsigned int SYNCHRONIZING = 4;            // Synchronization state

};

// Master TORP Control Task
class TorpMasterControl : public BaseTask
{
    public:
        TorpMasterControl(char* taskName, unsigned int taskID, double taskPeriod,
                          double oprVelMag, double maxAccMag, double tAccDec, double tCruise,
                          bool homingManualFlag, bool deployingSensorsFlag, bool startDeployBySoftwareFlag, TorpControl &l_tc, TorpControl &r_tc,
                          StepperMotor &l_st1, StepperMotor &l_st2, StepperMotor &r_st1, StepperMotor &r_st2, ofstream &masterTorpDataFile, ofstream &auditTrailDataFile)
        {
            p_taskName = taskName;
            p_taskID = taskID;
            p_deltaTaskTime = taskPeriod;
            p_l_tc = &l_tc;
            p_r_tc = &r_tc;
            p_l_st1 = &l_st1;
            p_l_st2 = &l_st2;
            p_r_st1 = &r_st1;
            p_r_st2 = &r_st2;
            p_homingManualFlag = homingManualFlag;
            p_allowedDeploySensors = deployingSensorsFlag;
            p_startDeployBySoftwareFlag = startDeployBySoftwareFlag;
            p_masterTorpDataFile = &masterTorpDataFile;
            p_auditTrailDataFile = &auditTrailDataFile;
            p_oprVelMag = oprVelMag;
            p_tAccDec = tAccDec;
            double maxAccAllowed = abs(2 * p_oprVelMag / p_tAccDec);
            if (maxAccMag < maxAccAllowed)  // Check to see if the input maximum acceleration satisfies the condition
            {
                p_maxAccMag = maxAccMag;
            }
            else
            {
                cout << "Max acceleration magnitude exceeds allowable value. Use allowable value instead." << endl;
                p_maxAccMag = maxAccAllowed;
            }
            p_maxAccMag = maxAccMag;
            p_tCruise = tCruise;
            p_state = INITIALIZING;
            p_nextState = p_state;
        }
        ~TorpMasterControl() {};

        int Run() override
        {
            if (!p_allowedToRun)    // The task is not allow to run
            {
                return -1;
            }

            if (p_firstTimeRun) // Update the pre-time for the first time run
            {
                p_preTime = GetTimeNow();
                p_firstTimeRun = false;
                return -1;
            }


            int done = 1;
            p_numScans++;

            if(GetTimeNow() < p_nextTaskTime)  // It's not the time for the task to run
            {
                p_runSuccess = false;
                return done; // Done for now. Not the propriate time for the task to run
            }

            p_timeStart = GetTimeNow();
            p_nextTaskTime += p_deltaTaskTime; // Increase the next time that the task can run
            p_runSuccess = true;    // Mark this as a successful run

            // Calculate the actual time duration between two runs
            p_time = GetTimeNow();
            p_deltaT = p_time - p_preTime;  // Actual delta T for integral
            p_preTime = p_time;

            p_state = p_nextState;

            switch (p_state)
            {
                case INITIALIZING:
                    // Entry Code
                    p_stateName = "Initializing";
                    // Check to see if homing is done manually or automatically
                    if (p_homingManualFlag)
                    {
                        p_doneHomingFlag = true;
                        (*p_l_tc).SetReadySyncFlag(true);   // Set two torp control task to ready to synch mode
                        (*p_r_tc).SetReadySyncFlag(true);

                        // Switch to next state - Accelerating
                        p_nextState = ACCELERATING;
                        p_nextStateName = "Accelerating";

                        // Execute code
                        p_T2 = abs(2 * p_oprVelMag / p_maxAccMag) - p_tAccDec;  // Calculate T2
                        p_T1 = 0.5 * (p_tAccDec - p_T2);    // Calculate T1

                        p_Ta = p_time + p_T1;
                        p_Tb = p_time + p_T1 + p_T2;
                        p_Tc = p_time + p_tAccDec;

                        p_jerkProfVal = abs(p_maxAccMag / p_T1);
                    }
                    else
                    {
                        // Switch to next state - Homing
                        p_nextState = HOMING;
                        p_nextStateName = "Homing";
                    }

                    break;
                case HOMING:
                    // Entry Code
                    p_stateName = "Homing";

                    // Check to see if both torp has been done homing
                    if ((*p_l_tc).GetDoneHomingFlag() && (*p_r_tc).GetDoneHomingFlag())
                    {
                        p_doneHomingFlag = true;

                        // Switch ready to synch flag for both task
                        (*p_l_tc).SetReadySyncFlag(true);
                        (*p_r_tc).SetReadySyncFlag(true);

                        // Switch to next state - Accelerating
                        p_nextState = ACCELERATING;
                        p_nextStateName = "Accelerating";

                        // Execute code
                        p_T2 = abs(2 * p_oprVelMag / p_maxAccMag) - p_tAccDec;  // Calculate T2
                        p_T1 = 0.5 * (p_tAccDec - p_T2);    // Calculate T1

                        p_Ta = p_time + p_T1;
                        p_Tb = p_time + p_T1 + p_T2;
                        p_Tc = p_time + p_tAccDec;

                        p_jerkProfVal = abs(p_maxAccMag / p_T1);

                    }

                    break;

                case ACCELERATING:
                    // Entry Code
                    p_stateName = "Accelerating";

                    // Check conditions code
                    if (p_time >= p_Tc)
                    {
                        p_nextState = CRUISING; // Update next state
                        p_nextStateName = "Cruising";
                        p_readyToDeploySensorsFlag = true;
                        if (p_allowedDeploySensors)
                        {
                            p_Td = p_time + p_tCruise;  // Update time for cruising phase
                        }
                        else
                        {
                            p_Td = p_time + 2.0*p_tCruise + 10.0;  // Update time for cruising phase
                        }

                        p_velProfVal = p_oprVelMag; // Update velocity profile
                        p_accProfVal = 0;   // Set acceleration to be 0
                        p_jerkProfVal = 0;  // Set jerk to be 0
                    }

                    // Execution code
                    else
                    {
                        if (p_time >= p_Ta && p_time <= p_Tb)
                        {
                            p_jerkProfVal = 0;
                        }
                        else if (p_time > p_Tb)
                        {
                            p_jerkProfVal = -abs(p_maxAccMag / p_T1);
                        }
                    }

                    break;

                case CRUISING:
                    // Entry code
                    p_stateName = "Cruising";
                    if (!p_allowedDeploySensors)
                    {
                        p_deployingDoneFlag = true;
                    }


                    // Check conditions code
                    if (p_time >= p_Td)
                    {
                        if (p_allowedDeploySensors) // Check to see if the program allowed to deploy the sensors/masses
                        {
                            if (!p_startDeployBySoftwareFlag)   // Check to see if the start deploying-retracting is done by timing or software
                            {
                                if (p_deployingDoneFlag && !p_retractingDoneFlag)    // Check to see if the deploying has been done
                                {
                                    p_nextState = RETRACTING_MASS;      // Switch to Retracting state
                                    p_nextStateName = "RetractingMass";
                                    // Capture the start time to deploy
                                    p_tEndDeployRetract = p_time + p_tDeployRetract;
                                    // Keep the velocity the same
                                    p_velProfVal = p_oprVelMag;
                                    p_accProfVal = 0;
                                    p_jerkProfVal = 0;

                                }

                                else if (p_retractingDoneFlag && p_deployingDoneFlag)
                                {
                                    p_nextState = DECELERATING; // Update next state
                                    p_nextStateName = "Decelerating";
                                    p_Ta = p_time + p_T1;
                                    p_Tb = p_time + p_T1 + p_T2;
                                    p_Tc = p_time + p_tAccDec;
                                    p_jerkProfVal = -abs(p_maxAccMag / p_T1);
                                }

                                else
                                {
                                    p_nextState = DEPLOYING_MASS;   // Switch to deploying boom
                                    p_nextStateName = "DeployingMass";
                                    // Capture the start time to deploy
                                    p_tEndDeployRetract = p_time + p_tDeployRetract;
                                    // Keep the velocity the same
                                    p_velProfVal = p_oprVelMag;
                                    p_accProfVal = 0;
                                    p_jerkProfVal = 0;
                                }
                            }

                            else
                            {
                                if (p_startDeployingFlag && !p_deployingDoneFlag)
                                {
                                    p_nextState = DEPLOYING_MASS;
                                    p_nextStateName = "DeployingMass";
                                    p_tEndDeployRetract = p_time + p_tDeployRetract;
                                    // Keep the velocity the same
                                    p_velProfVal = p_oprVelMag;
                                    p_accProfVal = 0;
                                    p_jerkProfVal = 0;
                                }

                                else if (p_startRetractingFlag && !p_retractingDoneFlag)
                                {
                                    p_nextState = RETRACTING_MASS;      // Switch to Retracting state
                                    p_nextStateName = "RetractingMass";
                                    // Capture the start time to deploy
                                    p_tEndDeployRetract = p_time + p_tDeployRetract;
                                    // Keep the velocity the same
                                    p_velProfVal = p_oprVelMag;
                                    p_accProfVal = 0;
                                    p_jerkProfVal = 0;
                                }

                                else if (p_retractingDoneFlag && p_deployingDoneFlag)
                                {
                                    p_nextState = DECELERATING; // Update next state
                                    p_nextStateName = "Decelerating";
                                    p_Ta = p_time + p_T1;
                                    p_Tb = p_time + p_T1 + p_T2;
                                    p_Tc = p_time + p_tAccDec;
                                    p_jerkProfVal = -abs(p_maxAccMag / p_T1);
                                }

                                else
                                {
                                    p_velProfVal = p_oprVelMag;
                                    p_accProfVal = 0;
                                    p_jerkProfVal = 0;
                                }

                            }

                        }
                        else
                        {
                            p_nextState = DECELERATING; // Update next state
                            p_nextStateName = "Decelerating";
                            p_Ta = p_time + p_T1;
                            p_Tb = p_time + p_T1 + p_T2;
                            p_Tc = p_time + p_tAccDec;
                            p_jerkProfVal = -abs(p_maxAccMag / p_T1);
                        }

                    }

                    // Execution code
                    else
                    {
                        p_velProfVal = p_oprVelMag;
                        p_accProfVal = 0;
                        p_jerkProfVal = 0;
                    }

                    break;

                case DEPLOYING_MASS:
                    // Entry code
                    p_stateName = "DeployingMass";

                    // Execution code
                    if (!p_deployingDoneFlag)
                    {
                        // Calculate the distance to deploy all booms
                        double distPerStep = 0.04;  // Linear distance per step, [mm]
                        double lBoom = 200;         // Travel distance of each boom
                        int32_t deployTargetPos = (int32_t) lBoom/distPerStep;

                        // Deploy the masses
                        (*p_l_st1).RunToTargetPosition(deployTargetPos);
                        (*p_l_st2).RunToTargetPosition(deployTargetPos);
                        (*p_r_st1).RunToTargetPosition(deployTargetPos);
                        (*p_r_st2).RunToTargetPosition(deployTargetPos);

                    }

                    // Keep the velocity the same
                    p_velProfVal = p_oprVelMag;
                    p_accProfVal = 0;
                    p_jerkProfVal = 0;

                    // Check to see if time to switch state
                    if (p_time >= p_tEndDeployRetract)
                    {
                        p_nextState = CRUISING; // Switch back to cruising state
                        p_nextStateName = "Cruising";
                        p_Td = p_time + p_tCruise;  // Calculate the time for next cruising phase
                        // Flip the flag
                        p_deployingDoneFlag = true;
                    }

                    break;

                case RETRACTING_MASS:
                    // Entry code
                    p_stateName = "RetractingMass";

                    // Execution code
                    if (!p_retractingDoneFlag)
                    {
                        // Calculate the distance to deploy all booms
                        double distPerStep = 0.04;  // Linear distance per step, [mm]
                        double lBoom = 200;         // Travel distance of each boom
                        double lOffset = 25;        // Offset distance when retracting
                        int32_t retractTargetPos = -(int32_t) (lBoom - 10)/distPerStep;

                        // Retract the masses
                        (*p_l_st1).RunToTargetPosition(retractTargetPos);
                        (*p_l_st2).RunToTargetPosition(retractTargetPos);
                        (*p_r_st1).RunToTargetPosition(retractTargetPos);
                        (*p_r_st2).RunToTargetPosition(retractTargetPos);

                        // Flip the flag
                        p_retractingDoneFlag = true;
                    }

                    // Keep the velocity the same
                    p_velProfVal = p_oprVelMag;
                    p_accProfVal = 0;
                    p_jerkProfVal = 0;

                    // Check to see if time to switch state
                    if (p_time >= p_tEndDeployRetract)
                    {
                        // De-energize all the steppers
                        (*p_l_st1).DeEnergize();
                        (*p_l_st2).DeEnergize();
                        (*p_r_st1).DeEnergize();
                        (*p_r_st2).DeEnergize();

                        // Switch to Next State
                        p_nextState = CRUISING; // Switch back to cruising state
                        p_nextStateName = "Cruising";
                        p_Td = p_time + p_tCruise;  // Calculate the time for next cruising phase
                    }

                    break;

                case DECELERATING:
                    // Entry code
                    p_stateName = "Decelerating";

                    // Check condition code
                    if(p_time >= p_Tc)
                    {
                        p_nextState = STOPPING;
                        p_nextStateName = "Stopping";
                        p_velProfVal = 0;
                        p_accProfVal = 0;
                        p_jerkProfVal = 0;
                    }

                    // Execution code
                    else
                    {
                        if (p_time >= p_Ta && p_time <= p_Tb)
                        {
                            p_jerkProfVal = 0;
                        }
                        else if (p_time > p_Tb)
                        {
                            p_jerkProfVal = abs(p_maxAccMag / p_T1);
                        }
                    }

                    break;

                case STOPPING:
                    // Entry Code
                    p_stateName = "Stopping";

                    // Halt all motion
                    (*p_l_tc).HaltMotion();
                    (*p_r_tc).HaltMotion();

                    break;
            }

            if (p_doneHomingFlag)
            {
                // Update the profile
                p_accProfVal = p_accProfVal + p_jerkProfVal * p_deltaT;                     // Acceleration profile
                p_velProfVal = p_velProfVal + p_accProfVal * p_deltaT;                      // Velocity profile
                p_posPofVal = p_posPofVal + (p_velProfVal * (p_deltaT / 60.0) * 360.0);     // Position profile

                // Actuate the motor
                ActuatingTorp(p_l_tc, p_posPofVal, p_velProfVal);
                ActuatingTorp(p_r_tc, p_posPofVal, p_velProfVal);
            }

            // Log the data
            *p_masterTorpDataFile << left << setw(w) << GetTimeNow() << left << setw(w) << GetPosProfVal() << left << setw(w) << GetVelProfVal() << left << setw(w) << GetAccProfVal() << endl;

            // Get the timestamp and finish the task
            p_timeEnd = GetTimeNow();  // Get the end time stamp for the task
            AuditTrailRecord();  // Record the state here
            return done;
        }


        // Get data functions
        double GetAccProfVal() {return p_accProfVal;}       // Function to get acceleration
        double GetVelProfVal() {return p_velProfVal;}       // Function to get velocity
        double GetJerkProfVal() {return p_jerkProfVal;}     // Function to get jerk value
        double GetPosProfVal() {return p_posPofVal;}        // Function to get position value
        bool GetDeployingDoneFlag() {return p_deployingDoneFlag;}   // Function to get deploying done flag
        bool GetReTractingDoneFlag() {return p_retractingDoneFlag;} // Function to get retracting done flag
        bool GetFirstTimeRunFlag() {return p_firstTimeRun;} // Function to get first time run flag
        bool GetReadyToDeploySensorsFlag() {return p_readyToDeploySensorsFlag;}

        // Set functions
        void SetDeployingFlag(bool flagVal)
        {
            p_startDeployingFlag = flagVal;
        }

        void SetRetractingFlag(bool flagVal)
        {
            p_startRetractingFlag = flagVal;
        }

        // Actuating functions
        void ActuatingTorp(TorpControl *tc, double posVal, double velVal)
        {
            TorpControl *p_tc = NULL;
            p_tc = tc;

            // Set anable flag for torp control task
            (*p_tc).SetEnableFlag(true);

            // Set reference positon and velocity
            (*p_tc).SetRefPositionVelocity(posVal, velVal);

            // Actuate
            (*p_tc).Run();
        }



    protected:
        TorpControl *p_l_tc = NULL,
                    *p_r_tc = NULL;         // Pointer for torp control task

        StepperMotor *p_l_st1 = NULL,       // Left Stepper 1
                     *p_l_st2 = NULL,       // Left Stepper 2
                     *p_r_st1 = NULL,       // Right stepper 1
                     *p_r_st2 = NULL;       // Right stepper 2
        bool p_homingManualFlag = false;    // Flag for perfoming homing manually
        bool p_allowedDeploySensors = true; // Flag to allow deploy the masses
        bool p_startDeployBySoftwareFlag = false;   // Flag to allow deploy by software
        bool p_startDeployingFlag = false;      // Start deploying flag
        bool p_startRetractingFlag = false; // Start retracting process flag
        bool p_doneHomingFlag = false;      // Flag to signal the homing is done
        bool p_deployingDoneFlag = false;   // Flag to signal the deploying is done
        bool p_retractingDoneFlag = false;  // Flag to signal the retracting is done
        bool p_firstTimeRun = true;    // Flag for first time run
        bool p_readyToDeploySensorsFlag = false;

        double p_oprVelMag = 0;             // Operation velocity magnitude, [RPM]
        double p_maxAccMag = 1;             // Max acceleration/deceleration mafnitude, [RPM/s]
        double p_tAccDec = 1;               // Time to accelerate/decelerate, [s]
        double p_tCruise = 1;               // Time for cruising, [s]
        double p_tDeployRetract = 65;       // Time to deploy or retract the boom, [s]
        double p_tEndDeployRetract = 0;     // Ending time to deploy or retract

        double p_posPofVal = 0;     // Value of the profile position, [deg]
        double p_accProfVal = 0;    // Value of the profile acceleration, [RPM/s]
        double p_velProfVal = 0;    // Value of the profile velocity, [RPM]
        double p_jerkProfVal = 0;   // Value of the jerk,. [RPM/s^2]
        double p_T1 = 0, p_T2 = 0;  // Interval time value for the profile, [s]
        double p_time = 0, p_preTime = 0, p_deltaT = 0; // Time value to calculate the integral [sec]
        double p_Ta = 0, p_Tb = 0, p_Tc = 0, p_Td = 0;  // Time stamp for the profile, [sec]
                                                        // Ta - time to ramp up acceleration
                                                        // Tb - time to hold acceleration const
                                                        // Tc - time to ramp down acceleration
                                                        // Td - time for cruising

        ofstream *p_masterTorpDataFile = NULL;
        unsigned int w = 25;

        static const unsigned int INITIALIZING = 0;     // Initializing state
        static const unsigned int HOMING = 1;           // Homing state
        static const unsigned int ACCELERATING = 2;     // ACCELERATING state
        static const unsigned int CRUISING = 3;         // Cruising state
        static const unsigned int DEPLOYING_MASS = 4;   // Deploying boom state
        static const unsigned int RETRACTING_MASS = 5;  // Retracting state
        static const unsigned int DECELERATING = 6;     // Decelerating state
        static const unsigned int STOPPING = 7;         // Stopping state
};

// Altitude Control Task
class AltitudeControl : public BaseTask
{
    public:
        AltitudeControl(char* taskName, unsigned int taskID, double taskPeriod,
                        ModbusSunSensor &mss, LabJackInclinometer &ljIncl, bool onlyDoMeasurement,
                        double tIniTorque, double iniTorqueWidth, Vector3d iniTorqueVec, bool moveWithVelocityOnlyFlag, bool holdPosFlag,
                        Vector3d axisOfRotation, double deltaThetaTarget, double refVelocity, double refAcceleration, PIControl &piBody,
                        double momOfInertiaX, double kControlX, PIControl &piX, long int maxVelX, MaxonMotor &mmX,
                        double momOfInertiaY, double kControlY, PIControl &piY, long int maxVelY, MaxonMotor &mmY,
                        double momOfInertiaZ, double kControlZ, PIControl &piZ, long int maxVelZ, MaxonMotor &mmZ,
                        ofstream &bwDataFile, ofstream &sensorsDataFile, ofstream &angularVelDataFile, ofstream &momentumWheelsData, ofstream &auditTrailDataFile)
        {
            p_taskName = taskName;
            p_taskID = taskID;
            p_deltaTaskTime = taskPeriod;
            p_mss = &mss;
            p_ljIncl = &ljIncl;
            p_onlyDoMeasurement = onlyDoMeasurement;
            p_tIniTorque = tIniTorque;
            p_iniTorqueWidth = iniTorqueWidth;
            p_iniTorqueVec = iniTorqueVec;
            p_moveWithVelocityOnlyFlag = moveWithVelocityOnlyFlag;
            p_holdPosFlag = holdPosFlag;
            p_axisOfRotationNorm = axisOfRotation;
            p_axisOfRotationNorm.normalize();
            p_deltaThetaTarget = deltaThetaTarget;
            p_refVelocity = refVelocity;
            p_refAcceleration = refAcceleration;
            p_piBody = &piBody;
            p_momOfInertiaX = momOfInertiaX;
            p_momOfInertiaY = momOfInertiaY;
            p_momOfInertiaZ = momOfInertiaZ;
            p_maxVelX = maxVelX;
            p_maxVelY = maxVelY;
            p_maxVelZ = maxVelZ;
            p_kControlX = kControlX;
            p_kControlY = kControlY;
            p_kControlZ = kControlZ;
            p_piX = &piX;
            p_piY = &piY;
            p_piZ = &piZ;
            p_mmX = &mmX;
            p_mmY = &mmY;
            p_mmZ = &mmZ;
            p_bwDataFile = &bwDataFile;
            p_sensorsDataFile = &sensorsDataFile;
            p_angularVelDataFile = &angularVelDataFile;
            p_momentumWheelsData = &momentumWheelsData;
            p_auditTrailDataFile = &auditTrailDataFile;
            p_state = INITIALIZING;
            p_nextState = 0;
            p_xb << 1.0, 0.0, 0.0;
            p_yb << 0.0, 1.0, 0.0;
            p_zb << 0.0, 0.0, 1.0;
            p_xeBw << 1.0, 0.0, 0.0;
            p_yeBw << 0.0, 1.0, 0.0;
            p_zeBw << 0.0, 0.0, 1.0;
            p_angularVelocityVecFiltered << 0.0, 0.0, 0.0;
            p_preAngularVelocityVecFiltered << 0.0, 0.0, 0.0;
            p_refAngularVelVec << 0.0, 0.0, 0.0;
            p_angVelErrVec << 0.0, 0.0, 0.0;
        }
        ~AltitudeControl(){};   // Destructor

        int Run() override  // Main running function of the controller
        {
            if (!p_allowedToRun)    // The task is not allow to run
            {
                return -1;
            }

            int done = 1;   // Default return value indicating done for this event
            p_numScans++;   // Increasement the scan count

            if(GetTimeNow() < p_nextTaskTime)  // It's not the time for the task to run
            {
                p_runSuccess = false;
                return done; // Done for now. Not the propriate time for the task to run
            }

            p_timeStart = GetTimeNow();
            p_nextTaskTime += p_deltaTaskTime;

            p_runSuccess = true;    // Mark this as a successful run

            p_state = p_nextState;  // Switch current state

            switch (p_state)
            {
                case INITIALIZING:
                    p_stateName = "Initializing";
                    // Calculate the max acceleration
                    p_maxAccCmdX = (*p_mmX).GetMaxTorque() / p_momOfInertiaX;
                    p_maxAccCmdY = (*p_mmY).GetMaxTorque() / p_momOfInertiaY;
                    p_maxAccCmdZ = (*p_mmZ).GetMaxTorque() / p_momOfInertiaZ;
                    // Calculate the end time for initiating motion
                    p_tEnd = GetTimeNow() + p_iniTorqueWidth;
                    // Update time of initializing motion state
                    p_preTimeIni = GetTimeNow();
                    // Switch to nextState
                    if (p_onlyDoMeasurement)
                    {
                        p_nextState = DETERMINING_ALTITUDE;   // Switch to Determining Altitude state
                        p_nextStateName = "DeterminingAlt";
                    }
                    else
                    {
                        p_nextState = INITIALIZING_MOTION;      // Switch to Initializing motion state
                        p_nextStateName = "IniMotion";
                    }

                    break;

                case DETERMINING_ALTITUDE:
                    p_stateName = "DeterminingAlt";

                    // Read the Sun Sensor
                    if (!(*p_mss).GetAngles(p_thzSun, p_thySun, p_sunInfo))
                    {
                        cout << "Fail to read Sun Sensor!" << endl;
                    }

                    p_thySun = Deg2Rad(p_thySun);    // Get Y-angle from Sun Sensor
                    p_thzSun = Deg2Rad(p_thzSun);    // Get Z-angle from Sun Sensor

                    if (p_sunInfo != 0)
                    {
                        cout << "Sun Message: " << (*p_mss).GetAddMessage(p_sunInfo) << endl;
                    }

                    // Read the Inclinometer
                    p_thxIncl = Deg2Rad((*p_ljIncl).GetAngleY());    // Check to see if the Inclinometer's Y-axis allign with the X-body-fixed
                    p_thzIncl = Deg2Rad((*p_ljIncl).GetAngleX());    // Check to see if the Inclinometer's X-axis allign with the Z-body-fixed


                    // Calculating the altitude
                    p_solRotMatBackward = BackwardSol::AlgebraicSolutionMatrix(p_thxIncl, p_thzIncl, p_thySun, p_thzSun);
                    p_time = GetTimeNow();  // Get the current time

                    p_xeBw = p_solRotMatBackward * p_xb;
                    p_yeBw = p_solRotMatBackward * p_yb;
                    p_zeBw = p_solRotMatBackward * p_zb;

                    // Calculate Euler Angles
                    BackwardSol::GetEulerFromRot(p_solRotMatBackward, &p_thxEuler, &p_thyEuler, &p_thzEuler);

                    // Calculate the angular velocity
                    if (!p_readyForAngVelocity)
                    {
                        p_iniRotMat = p_solRotMatBackward;
                        p_prevRotMat = p_iniRotMat;
                        p_readyForAngVelocity = true; //Initial state determined
                        p_angularVelocityVec << 0.0, 0.0, 0.0;
                        p_rotAngle = 0;
                        p_preTime = p_time;
                    }
                    else
                    {
                        p_curRotMat = p_solRotMatBackward * p_iniRotMat.transpose();
                        AngularVel::time_interval = p_time - p_preTime;  // Calculate the actual delta_t between two consecutive calls
                        p_angularVelocityVec = AngularVel::GetAngularVelVec(p_prevRotMat, p_curRotMat);
                        p_rotAngle = AngularVel::RotAngleAboutRotAxis(p_curRotMat);
                        p_prevRotMat = p_curRotMat;
                    }

                    p_angularVelocityVecFiltered = emaFilter3d(p_fc, p_time - p_preTime, p_angularVelocityVec, p_preAngularVelocityVecFiltered); // Use Exponential-moving-average filter

                    // Log the data
                    *p_angularVelDataFile << left << setw(w) << GetTimeNow() << left << setw(w) << p_angularVelocityVecFiltered(0)
                                                                             << left << setw(w) << p_angularVelocityVecFiltered(1)
                                                                             << left << setw(w) << p_angularVelocityVecFiltered(2)
                                                                             << left << setw(w) << p_refAngularVelVec(0)
                                                                             << left << setw(w) << p_refAngularVelVec(1)
                                                                             << left << setw(w) << p_refAngularVelVec(2)
                                                                             << left << setw(w) << p_angVelErrVec(0)
                                                                             << left << setw(w) << p_angVelErrVec(1)
                                                                             << left << setw(w) << p_angVelErrVec(2)
                                                                             << left << setw(w) << Rad2Deg(p_rotAngle) << left << setw(w) << Rad2Deg(p_thetaProf) << left << setw(w) << p_wProf << left << setw(w) << p_eProf << endl;

                    *p_bwDataFile << left << setw(w) << GetTimeNow() << left << setw(w) << p_xeBw(0) << left << setw(w) << p_xeBw(1) << left << setw(w) << p_xeBw(2)
                                                                     << left << setw(w) << p_yeBw(0) << left << setw(w) << p_yeBw(1) << left << setw(w) << p_yeBw(2)
                                                                     << left << setw(w) << p_zeBw(0) << left << setw(w) << p_zeBw(1) << left << setw(w) << p_zeBw(2) << endl;

                    *p_sensorsDataFile << left << setw(w) << GetTimeNow() << left << setw(w) << Rad2Deg(p_thxIncl) << left << setw(w) << Rad2Deg(p_thzIncl)
                                                                          << left << setw(w) << Rad2Deg(p_thySun) << left << setw(w) << Rad2Deg(p_thzSun)
                                                                          << left << setw(w) << Rad2Deg(p_thxEuler) << left << setw(w) << Rad2Deg(p_thyEuler) << left << setw(w) << Rad2Deg(p_thzEuler) << endl;

                    // Update
                    p_preAngularVelocityVecFiltered = p_angularVelocityVecFiltered;
                    p_preTime = p_time;

                    // Condition to switch state
                    if (p_onlyDoMeasurement)
                    {
                        p_nextState = DETERMINING_ALTITUDE;
                        p_nextStateName = "DeterminingAlt";
                    }
                    else
                    {
                        if (p_iniMotionDone)
                        {
                            // Check to see if the de-tumbling is done
                            if (p_detumblingDone)
                            {
                                if (!p_doneFindingSunFlag)  // Check to see if the body has moved to the Sun's Center
                                {
                                    if (!p_readyToFindSun)  // Check to see if this is the first time to find Sun
                                    {
                                        p_readyToFindSun = true;
                                        // Update the time
                                        p_preTimeFindSun = GetTimeNow();

                                        // Calculate the delta angle to the Sun
                                        p_rotMatBodyToInertialFrame = p_inertialFrameMat * p_solRotMatBackward.transpose();
                                        p_deltaThetaSun = AngularVel::RotAngleAboutRotAxis(p_rotMatBodyToInertialFrame);
                                        p_axisOfRotationFindSunNorm = AngularVel::RotAxisFromRotMat(p_rotMatBodyToInertialFrame);

                                        // Plan the path
                                        double eDesMin = abs(pow(p_refVelocity, 2) / p_deltaThetaSun);
                                        if (eDesMin >= abs(p_refAcceleration))
                                        {
                                            p_refAcceleration = eDesMin;
                                            cout << "Use minimum acceleration value" << endl;
                                        }

                                        p_t0 = GetTimeNow();
                                        p_tA = abs(p_refVelocity / p_refAcceleration) + 2 * p_deltaTaskTime;
                                        p_tB = abs(p_deltaThetaSun/p_refVelocity) - p_tA - 2 * p_deltaTaskTime;
                                        p_t1 = p_t0 + p_tA;
                                        p_t2 = p_t1 + p_tB;
                                        p_t3 = p_t2 + p_tA;

                                        // Calculate the target angle
                                        p_thetaTarget = p_rotAngle + p_deltaThetaSun;

                                        // Update on the profile angle
                                        p_thetaProf = p_rotAngle;
                                    }

                                    p_nextState = FINDING_SUN;
                                    p_nextStateName = "FindingSun";
                                }

                                else if (p_startMovingFlag)  // Check to see if body is allowed to move
                                {
                                    if (!p_holdPosFlag)
                                    {
                                        if (!p_readyToMove) // Check to see if this is the first time switch to move state
                                        {
                                            p_readyToMove = true;
                                            // Update the time for moving state
                                            p_preTimeMoving = GetTimeNow();

                                            if (p_moveWithVelocityOnlyFlag)
                                            {
                                                p_tEndMoving = GetTimeNow() + p_tMoveWithVelocityOnly;
                                            }
                                            else
                                            {
                                                // Calculate the time for the profile
                                                double eDesMin = abs(pow(p_refVelocity, 2) / p_deltaThetaTarget);
                                                if (eDesMin >= abs(p_refAcceleration))
                                                {
                                                    p_refAcceleration = eDesMin;
                                                    cout << "Use minimum acceleration value" << endl;
                                                }
                                                p_t0 = GetTimeNow();
                                                p_tA = abs(p_refVelocity / p_refAcceleration) + 2 * p_deltaTaskTime;
                                                p_tB = abs(p_deltaThetaTarget/p_refVelocity) - p_tA - 2 * p_deltaTaskTime;
                                                p_t1 = p_t0 + p_tA;
                                                p_t2 = p_t1 + p_tB;
                                                p_t3 = p_t2 + p_tA;

                                                // Calculate the target angle
                                                p_thetaTarget = p_rotAngle + p_deltaThetaTarget;

                                                // Update on the profile angle
                                                p_thetaProf = p_rotAngle;
                                            }
                                        }

                                        p_nextState = MOVING;
                                        p_nextStateName = "Moving";
                                    }

                                    else
                                    {
                                        p_nextState = HOLDING_POSITION;
                                        p_nextStateName = "HoldingPos";
                                    }
                                }


                                else
                                {
                                    p_nextState = HOLDING_POSITION;
                                    p_nextStateName = "HoldingPos";
                                }

                            }
                            else
                            {

                                if ((p_time >= p_tPossibleEndDetumbling) && (abs(p_angularVelocityVecFiltered(0)) <= p_anglVelLimitX)  && (abs(p_angularVelocityVecFiltered(1)) <= p_anglVelLimitY) && (abs(p_angularVelocityVecFiltered(2)) <= p_anglVelLimitZ))
                                {
                                    p_detumblingDone = true;

                                    // Switch to moving state
                                    p_nextState = HOLDING_POSITION;
                                    p_nextStateName = "HoldingPos";
                                }
                                else
                                {
                                    p_nextState = DETUMBLING;
                                    p_nextStateName = "Detumbling";
                                }
                            }
                        }
                        else
                        {
                            p_nextState = INITIALIZING_MOTION;
                            p_nextStateName = "IniMotion";
                        }
                    }

                    break;

                case INITIALIZING_MOTION:
                    bool moveMomtWheelResult;
                    p_stateName = "IniMotion";
                    p_timeIni = GetTimeNow();
                    p_deltaTIni = p_timeIni - p_preTimeIni;
                    p_preTimeIni = p_timeIni;

                    // Execution code
                    if (p_timeIni <= p_tEnd)
                    {
                        // Move X
                        if(moveMomentumWheelWithTorque(p_iniTorqueVec(0), p_deltaTIni, &p_torqueCmdX, 'x')) cout << "Saturate X Momentum Wheel" << endl;
                        // Move Y
                        if(moveMomentumWheelWithTorque(p_iniTorqueVec(1), p_deltaTIni, &p_torqueCmdY, 'y')) cout << "Saturate Y Momentum Wheel" << endl;
                        // Move Z
                        if(moveMomentumWheelWithTorque(p_iniTorqueVec(2), p_deltaTIni, &p_torqueCmdZ, 'z')) cout << "Saturate Z Momentum Wheel" << endl;
                    }
                    else
                    {
                        // Move X
                        if(moveMomentumWheelWithTorque(0, p_deltaTIni, &p_torqueCmdX, 'x')) cout << "Saturate X Momentum Wheel" << endl;
                        // Move Y
                        if(moveMomentumWheelWithTorque(0, p_deltaTIni, &p_torqueCmdY, 'y')) cout << "Saturate Y Momentum Wheel" << endl;
                        // Move Z
                        if(moveMomentumWheelWithTorque(0, p_deltaTIni, &p_torqueCmdZ, 'z')) cout << "Saturate Z Momentum Wheel" << endl;
                    }

                    // Get the data
                    p_momtWheelXVel = (*p_mmX).GetVelocityIs();
                    p_momtWheelYVel = (*p_mmY).GetVelocityIs();
                    p_momtWheelZVel = (*p_mmZ).GetVelocityIs();

                    // Log the data
                    *p_momentumWheelsData << left << setw(w) << GetTimeNow() << left << setw(w) << p_torqueCmdX << left << setw(w) << p_momtWheelXVelCmd << left << setw(w) << p_momtWheelXVel
                                                                             << left << setw(w) << p_torqueCmdY << left << setw(w) << p_momtWheelYVelCmd << left << setw(w) << p_momtWheelYVel
                                                                             << left << setw(w) << p_torqueCmdZ << left << setw(w) << p_momtWheelZVelCmd << left << setw(w) << p_momtWheelZVel << endl;

                    cout << left << setw(w) << GetTimeNow() << left << setw(w) << p_angularVelocityVecFiltered(0) << left << setw(w) << p_angularVelocityVecFiltered(1) << left << setw(w) << p_angularVelocityVecFiltered(2)
                                                            << left << setw(w) << p_momtWheelXVel << left << setw(w) <<  p_momtWheelYVel << left << setw(w) << p_momtWheelZVel << endl;

                    // Condition to switch state/switch mode
                    if (p_timeIni + p_deltaTIni >= p_tIniTorque) // Done with initializing motion
                    {
                        p_iniMotionDone = true;
                        // Update time for de-tumbling state
                        p_preTimeDetumb = GetTimeNow();
                        p_tPossibleEndDetumbling = GetTimeNow() + p_tPossibleDetumbling;
                    }

                    p_nextState = DETERMINING_ALTITUDE;
                    p_nextStateName = "DeterminingAlt";

                    break;

                case DETUMBLING:
                    p_stateName = "Detumbling";
                    p_timeDetumb = GetTimeNow();
                    p_deltaTDetumb = p_timeDetumb - p_preTimeDetumb;
                    p_preTimeDetumb = p_timeDetumb;
                    // Execution code
                    // Move X
                    if(moveMomentumWheelWithTorque(-p_kControlX * p_angularVelocityVecFiltered(0), p_deltaTDetumb, &p_torqueCmdX, 'x')) cout << "Saturate X momentum wheel" << endl;
                    // Move Y
                    if(moveMomentumWheelWithTorque(-p_kControlY * p_angularVelocityVecFiltered(1), p_deltaTDetumb, &p_torqueCmdY, 'y')) cout << "Saturate Y momentum wheel" << endl;
                    // Move Z
                    if(moveMomentumWheelWithTorque(-p_kControlZ * p_angularVelocityVecFiltered(2), p_deltaTDetumb, &p_torqueCmdZ, 'z')) cout << "Saturate Z momentum wheel" << endl;

                    // Get the data
                    p_momtWheelXVel = (*p_mmX).GetVelocityIs();
                    p_momtWheelYVel = (*p_mmY).GetVelocityIs();
                    p_momtWheelZVel = (*p_mmZ).GetVelocityIs();

                    // Log the data
                    *p_momentumWheelsData << left << setw(w) << GetTimeNow() << left << setw(w) << p_torqueCmdX << left << setw(w) << p_momtWheelXVelCmd << left << setw(w) << p_momtWheelXVel
                                                                             << left << setw(w) << p_torqueCmdY << left << setw(w) << p_momtWheelYVelCmd << left << setw(w) << p_momtWheelYVel
                                                                             << left << setw(w) << p_torqueCmdZ << left << setw(w) << p_momtWheelZVelCmd << left << setw(w) << p_momtWheelZVel << endl;

                    cout << left << setw(w) << GetTimeNow() << left << setw(w) << p_angularVelocityVecFiltered(0) << left << setw(w) << p_angularVelocityVecFiltered(1) << left << setw(w) << p_angularVelocityVecFiltered(2)
                                                            << left << setw(w) << p_momtWheelXVel << left << setw(w) <<  p_momtWheelYVel << left << setw(w) << p_momtWheelZVel << endl;

                    p_nextState = DETERMINING_ALTITUDE;
                    p_nextStateName = "DeterminingAlt";
                    break;

                case FINDING_SUN:
                    p_stateName = "FindingSun";
                    p_timeFindSun = GetTimeNow();
                    p_deltaTFindSun = p_timeFindSun - p_preTimeFindSun;
                    p_preTimeFindSun = p_timeFindSun;

                    // Execution code
                    // Calculate the profile
                    if (p_t0 <= p_timeFindSun && p_timeFindSun < p_t1)
                    {
                        p_eProf = GetSign(p_deltaThetaSun) * p_refAcceleration;
                        p_wProf = p_wProf + p_eProf * p_deltaTFindSun;
                        p_thetaProf = p_thetaProf + p_wProf * p_deltaTFindSun;
                    }
                    else if (p_t1 <= p_timeFindSun && p_timeFindSun < p_t2)
                    {
                        p_eProf = 0;
                        p_wProf = GetSign(p_deltaThetaSun) * p_refVelocity;
                        p_thetaProf = p_thetaProf + p_wProf * p_deltaTFindSun;
                    }
                    else if (p_t2 <= p_timeFindSun && p_timeFindSun < p_t3)
                    {
                        p_eProf = -GetSign(p_deltaThetaSun) * p_refAcceleration;
                        p_wProf = p_wProf + p_eProf * p_deltaTFindSun;
                        p_thetaProf = p_thetaProf + p_wProf * p_deltaTFindSun;
                    }
                    else
                    {
                        p_eProf = 0;
                        p_wProf = 0;
                        p_thetaProf = p_thetaTarget;
                        p_stopFindingSun = true;
                    }

                    // Run PI Control for the body to calculate the reference angular velocity vector
                    p_refAngularVelVec = ((*p_piBody).PICalculation(p_thetaProf, p_rotAngle) + p_wProf) * p_axisOfRotationFindSunNorm;

                    // Calculate the desired torque
                    p_torqueDes << (*p_piX).PICalculation(p_refAngularVelVec(0), p_angularVelocityVecFiltered(0)), (*p_piY).PICalculation(p_refAngularVelVec(1), p_angularVelocityVecFiltered(1)), (*p_piZ).PICalculation(p_refAngularVelVec(2), p_angularVelocityVecFiltered(2));
                    // Calculate the error
                    p_angVelErrVec << (*p_piX).GetError(), (*p_piY).GetError(), (*p_piZ).GetError();
                    // Move the momentum wheels accordingly
                    // Move X
                    if(moveMomentumWheelWithTorque(p_torqueDes(0), p_deltaTFindSun, &p_torqueCmdX, 'x')) cout << "Saturate X momentum wheel" << endl;
                    // Move Y
                    if(moveMomentumWheelWithTorque(p_torqueDes(1), p_deltaTFindSun, &p_torqueCmdY, 'y')) cout << "Saturate Y momentum wheel" << endl;
                    // Move Z
                    if(moveMomentumWheelWithTorque(p_torqueDes(2), p_deltaTFindSun, &p_torqueCmdZ, 'z')) cout << "Saturate Z momentum wheel" << endl;

                    // Get the data
                    p_momtWheelXVel = (*p_mmX).GetVelocityIs();
                    p_momtWheelYVel = (*p_mmY).GetVelocityIs();
                    p_momtWheelZVel = (*p_mmZ).GetVelocityIs();

                    // Log the data
                    *p_momentumWheelsData << left << setw(w) << GetTimeNow() << left << setw(w) << p_torqueCmdX << left << setw(w) << p_momtWheelXVelCmd << left << setw(w) << p_momtWheelXVel
                                                                             << left << setw(w) << p_torqueCmdY << left << setw(w) << p_momtWheelYVelCmd << left << setw(w) << p_momtWheelYVel
                                                                             << left << setw(w) << p_torqueCmdZ << left << setw(w) << p_momtWheelZVelCmd << left << setw(w) << p_momtWheelZVel << endl;


                    cout << left << setw(w) << GetTimeNow() << left << setw(w) << p_angularVelocityVecFiltered(0) << left << setw(w) << p_angularVelocityVecFiltered(1) << left << setw(w) << p_angularVelocityVecFiltered(2)
                                                            << left << setw(w) << p_momtWheelXVel << left << setw(w) <<  p_momtWheelYVel << left << setw(w) << p_momtWheelZVel << endl;

                    // Check the position to set flag done moving
                    if (p_stopFindingSun && abs(((*p_piBody).GetError()) <= Deg2Rad(0.008)))
                    {
                        p_doneFindingSunFlag = true;
                        p_preTimeHolding = GetTimeNow();
                    }

                    p_nextState = DETERMINING_ALTITUDE;
                    p_nextStateName = "DeterminingAlt";

                    break;

                case HOLDING_POSITION:
                    p_stateName = "HoldingPos";
                    p_timeHolding = GetTimeNow();
                    p_deltaTHolding = p_timeHolding - p_preTimeHolding;
                    p_preTimeHolding = p_timeHolding;

                    // Execution code
                    p_eProf = 0;
                    p_wProf = 0;
                    p_thetaProf = p_thetaTarget;
                    p_refAngularVelVec = ((*p_piBody).PICalculation(p_thetaProf, p_rotAngle) + p_wProf) * p_axisOfRotationFindSunNorm;

                    // Calculate the desired torque
                    p_torqueDes << (*p_piX).PICalculation(p_refAngularVelVec(0), p_angularVelocityVecFiltered(0)), (*p_piY).PICalculation(p_refAngularVelVec(1), p_angularVelocityVecFiltered(1)), (*p_piZ).PICalculation(p_refAngularVelVec(2), p_angularVelocityVecFiltered(2));
                    // Calculate the error
                    p_angVelErrVec << (*p_piX).GetError(), (*p_piY).GetError(), (*p_piZ).GetError();
                    // Move the momentum wheels accordingly
                    // Move X
                    if(moveMomentumWheelWithTorque(p_torqueDes(0), p_deltaTHolding, &p_torqueCmdX, 'x')) cout << "Saturate X momentum wheel" << endl;
                    // Move Y
                    if(moveMomentumWheelWithTorque(p_torqueDes(1), p_deltaTHolding, &p_torqueCmdY, 'y')) cout << "Saturate Y momentum wheel" << endl;
                    // Move Z
                    if(moveMomentumWheelWithTorque(p_torqueDes(2), p_deltaTHolding, &p_torqueCmdZ, 'z')) cout << "Saturate Z momentum wheel" << endl;

                    // Get the data
                    p_momtWheelXVel = (*p_mmX).GetVelocityIs();
                    p_momtWheelYVel = (*p_mmY).GetVelocityIs();
                    p_momtWheelZVel = (*p_mmZ).GetVelocityIs();

                    // Log the data
                    *p_momentumWheelsData << left << setw(w) << GetTimeNow() << left << setw(w) << p_torqueCmdX << left << setw(w) << p_momtWheelXVelCmd << left << setw(w) << p_momtWheelXVel
                                                                             << left << setw(w) << p_torqueCmdY << left << setw(w) << p_momtWheelYVelCmd << left << setw(w) << p_momtWheelYVel
                                                                             << left << setw(w) << p_torqueCmdZ << left << setw(w) << p_momtWheelZVelCmd << left << setw(w) << p_momtWheelZVel << endl;


                    cout << left << setw(w) << GetTimeNow() << left << setw(w) << p_angularVelocityVecFiltered(0) << left << setw(w) << p_angularVelocityVecFiltered(1) << left << setw(w) << p_angularVelocityVecFiltered(2)
                                                            << left << setw(w) << p_momtWheelXVel << left << setw(w) <<  p_momtWheelYVel << left << setw(w) << p_momtWheelZVel << endl;

                    p_nextState = DETERMINING_ALTITUDE;
                    p_nextStateName = "DeterminingAlt";

                    break;

                case MOVING:
                    p_stateName = "Moving";
                    p_timeMoving = GetTimeNow();
                    p_deltaTMoving = p_timeMoving - p_preTimeMoving;
                    p_preTimeMoving = p_timeMoving;

                    // Execution code
                    // Calculate the desired angular velocity vector
                    if (p_moveWithVelocityOnlyFlag) // Check to see if only needs to move with velocity
                    {
                        if (p_timeMoving <= p_tEndMoving)
                        {
                            p_refAngularVelVec = p_refVelocity * p_axisOfRotationNorm;
                        }
                        else
                        {
                            p_refAngularVelVec << 0.0, 0.0, 0.0;
                        }
                    }
                    else
                    {
                        // Calculate the profile
                        if (p_t0 <= p_timeMoving && p_timeMoving < p_t1)
                        {
                            p_eProf = GetSign(p_deltaThetaTarget) * p_refAcceleration;
                            p_wProf = p_wProf + p_eProf * p_deltaTMoving;
                            p_thetaProf = p_thetaProf + p_wProf * p_deltaTMoving;
                        }
                        else if (p_t1 <= p_timeMoving && p_timeMoving < p_t2)
                        {
                            p_eProf = 0;
                            p_wProf = GetSign(p_deltaThetaTarget) * p_refVelocity;
                            p_thetaProf = p_thetaProf + p_wProf * p_deltaTMoving;
                        }
                        else if (p_t2 <= p_timeMoving && p_timeMoving < p_t3)
                        {
                            p_eProf = -GetSign(p_deltaThetaTarget) * p_refAcceleration;
                            p_wProf = p_wProf + p_eProf * p_deltaTMoving;
                            p_thetaProf = p_thetaProf + p_wProf * p_deltaTMoving;
                        }
                        else
                        {
                            p_eProf = 0;
                            p_wProf = 0;
                            p_thetaProf = p_thetaTarget;
                            p_stopMovingFlag = true;
                        }

                        // Run PI Control for the body to calculate the reference angular velocity vector
                        p_refAngularVelVec = ((*p_piBody).PICalculation(p_thetaProf, p_rotAngle) + p_wProf) * p_axisOfRotationNorm;
                    }

                    // Calculate the desired torque
                    p_torqueDes << (*p_piX).PICalculation(p_refAngularVelVec(0), p_angularVelocityVecFiltered(0)), (*p_piY).PICalculation(p_refAngularVelVec(1), p_angularVelocityVecFiltered(1)), (*p_piZ).PICalculation(p_refAngularVelVec(2), p_angularVelocityVecFiltered(2));
                    // Calculate the error
                    p_angVelErrVec << (*p_piX).GetError(), (*p_piY).GetError(), (*p_piZ).GetError();
                    // Move the momentum wheels accordingly
                    // Move X
                    if(moveMomentumWheelWithTorque(p_torqueDes(0), p_deltaTMoving, &p_torqueCmdX, 'x')) cout << "Saturate X momentum wheel" << endl;
                    // Move Y
                    if(moveMomentumWheelWithTorque(p_torqueDes(1), p_deltaTMoving, &p_torqueCmdY, 'y')) cout << "Saturate Y momentum wheel" << endl;
                    // Move Z
                    if(moveMomentumWheelWithTorque(p_torqueDes(2), p_deltaTMoving, &p_torqueCmdZ, 'z')) cout << "Saturate Z momentum wheel" << endl;

                    // Get the data
                    p_momtWheelXVel = (*p_mmX).GetVelocityIs();
                    p_momtWheelYVel = (*p_mmY).GetVelocityIs();
                    p_momtWheelZVel = (*p_mmZ).GetVelocityIs();

                    // Log the data
                    *p_momentumWheelsData << left << setw(w) << GetTimeNow() << left << setw(w) << p_torqueCmdX << left << setw(w) << p_momtWheelXVelCmd << left << setw(w) << p_momtWheelXVel
                                                                             << left << setw(w) << p_torqueCmdY << left << setw(w) << p_momtWheelYVelCmd << left << setw(w) << p_momtWheelYVel
                                                                             << left << setw(w) << p_torqueCmdZ << left << setw(w) << p_momtWheelZVelCmd << left << setw(w) << p_momtWheelZVel << endl;


                    cout << left << setw(w) << GetTimeNow() << left << setw(w) << p_angularVelocityVecFiltered(0) << left << setw(w) << p_angularVelocityVecFiltered(1) << left << setw(w) << p_angularVelocityVecFiltered(2)
                                                            << left << setw(w) << p_momtWheelXVel << left << setw(w) <<  p_momtWheelYVel << left << setw(w) << p_momtWheelZVel << endl;

                    // Check the position to set flag done moving
                    if (p_stopMovingFlag && abs(((*p_piBody).GetError()) <= Deg2Rad(0.008)))
                    {
                        p_doneMovingFlag = true;
                    }

                    p_nextState = DETERMINING_ALTITUDE;
                    p_nextStateName = "DeterminingAlt";

                    break;

            }

            // Get the timestamp and finish the task
            p_timeEnd = GetTimeNow();  // Get the end time stamp for the task
            AuditTrailRecord();  // Record the state here
            return done;
        }

        // EMA filter
        Vector3d emaFilter3d(double fc, double dt, Vector3d curVector, Vector3d prevVector)
        {
            double a = exp(-2 * M_PI * fc * dt);
            return (1.0 - a) * curVector + a * prevVector;
        }

        // Rounding function
        int roundingFunc(double val)
        {
            double threshold = 0.5;
            double diff = abs(val - (int) val);
            int result = 0;
            if (val >= 0)   // Check if the value is positive
            {
                result = (int) (diff >= threshold) ? (int) val + 1 : (int) val;
            }
            else    // Value is negative
            {
                result = (int) (diff >= threshold) ? (int) val - 1 : (int) val;
            }
            return result;
        }

        // Pick which motor to run
        void pickMotor(char axisName, double torqueVal)
        {

            switch (axisName)   // Choose the axis to operate
            {
                case 'x':
                    p_torqueCmd = -torqueVal;
                    p_momOfInertia = &p_momOfInertiaX;
                    p_maxAccCmd = &p_maxAccCmdX;
                    p_preMomtWheelVelCmd = &p_preMomtWheelXVelCmd;
                    p_momtWheelVelCmd = &p_momtWheelXVelCmd;
                    p_maxVel = &p_maxVelX;
                    break;

                case 'y':
                    p_torqueCmd = -torqueVal;
                    p_momOfInertia = &p_momOfInertiaY;
                    p_maxAccCmd = &p_maxAccCmdY;
                    p_preMomtWheelVelCmd = &p_preMomtWheelYVelCmd;
                    p_momtWheelVelCmd = &p_momtWheelYVelCmd;
                    p_maxVel = &p_maxVelY;
                    break;

                case 'z':
                    p_torqueCmd = torqueVal;
                    p_momOfInertia = &p_momOfInertiaZ;
                    p_maxAccCmd = &p_maxAccCmdZ;
                    p_preMomtWheelVelCmd = &p_preMomtWheelZVelCmd;
                    p_momtWheelVelCmd = &p_momtWheelZVelCmd;
                    p_maxVel = &p_maxVelZ;
                    break;
            }
        }
        // Function to actuate momentum wheel
        bool moveMomentumWheelWithTorque(double torqueVal, double deltaT, double *torqueCmdVal, char axisName)
        {
            bool saturateMomtWheelFlag = false;
            double accCmd = 0;          // Command value for acceleration
            double velCmd = 0;


            pickMotor(axisName, torqueVal);

            accCmd = p_torqueCmd / *p_momOfInertia;

            if (accCmd >= (*p_maxAccCmd)) accCmd = *p_maxAccCmd;
            else if (accCmd <= -*p_maxAccCmd) accCmd = -*p_maxAccCmd;

            p_torqueCmd = accCmd * (*p_momOfInertia);
            velCmd = (*p_preMomtWheelVelCmd) + (accCmd * deltaT) * (30 / M_PI);

            //Check for saturation
            if (velCmd > *p_maxVel)
                {
                    saturateMomtWheelFlag = true;
                    *p_momtWheelVelCmd = *p_maxVel;
                }
            else if (velCmd < -*p_maxVel)
                {
                    saturateMomtWheelFlag = true;
                    *p_momtWheelVelCmd = -*p_maxVel;
                }
            else
                {
                    *p_momtWheelVelCmd = velCmd;
                }


            switch (axisName){
                case 'x':
                    (*p_mmX).RunWithVelocity(roundingFunc(*p_momtWheelVelCmd));
                    *torqueCmdVal = -p_torqueCmd;
                    break;
                 case 'y':
                    (*p_mmY).RunWithVelocity(roundingFunc(*p_momtWheelVelCmd));
                    *torqueCmdVal = -p_torqueCmd;
                    break;
                case 'z':
                    (*p_mmZ).RunWithVelocity(roundingFunc(*p_momtWheelVelCmd));
                    *torqueCmdVal = p_torqueCmd;
                    break;
            }                                               // Sign convention for the torque applied to Grotifer's body


            *p_preMomtWheelVelCmd = *p_momtWheelVelCmd;     //Update

            return saturateMomtWheelFlag;
        }

        // Get sign function
        double GetSign(double val)
        {
            double returnVal = 0;
            if (val >= 0) returnVal = 1.0;
            else returnVal = -1.0;
            return returnVal;
        }

        // Function to halt motions of the momentum wheel
        void HaltMotion()
        {
            (*p_mmX).HaltMotion();
            (*p_mmY).HaltMotion();
            (*p_mmZ).HaltMotion();
        }

        // Function to set the values
        void SetAxisRotation(Vector3d axisOfRotation)
        {
            p_axisOfRotationNorm = axisOfRotation;
            p_axisOfRotationNorm.normalize();
        }

        void SetRefAngularVel(double refAngularVelVal)
        {
            p_refVelocity = refAngularVelVal;
        }

        void SetRefAcceleration(double refAngularAccVal)
        {
            p_refAcceleration = refAngularAccVal;
        }

        void SetMoveWithVelDuration(double duration)
        {
            p_tMoveWithVelocityOnly = duration;
        }

        void SetMovePosition(double posInRad)
        {
            p_deltaThetaTarget = posInRad;
        }

        void SetStartMovingFlag(bool flagVal)
        {
            p_startMovingFlag = flagVal;
        }

        void SetDoneMovingFlag(bool flagVal)
        {
            p_doneMovingFlag = flagVal;
        }

        // Functions to get values
        bool GetDetumblingDoneFlag() {return p_detumblingDone;}
        bool GetFindingSunDoneFlag() {return p_doneFindingSunFlag;}
        bool GetDoneMovingFlag()  {return p_doneMovingFlag;}

    protected:
        ModbusSunSensor *p_mss = NULL;
        LabJackInclinometer *p_ljIncl = NULL;
        double p_thxIncl; // X-angle given from Inclinometer, [rad]
        double p_thzIncl; // Z-angle given from Inclinometer, [rad]
        double p_thySun;  // Y-angle given from the Sun Sensor, [rad]
        double p_thzSun;  // Z-angle given from the Sun Sensor, [rad]
        int p_sunInfo = 0;    // Information about the Sun
        double p_thxEuler;
        double p_thyEuler;
        double p_thzEuler;
        Matrix3d p_solRotMatBackward{{1.0, 0.0, 0.0},{0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};   // Rotation matrix
        Matrix3d p_solRotMatNM{{1.0, 0.0, 0.0},{0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};   // Rotation matrix
        Matrix3d p_curRotMat{{0.0, 0.0, 0.0},{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
        Matrix3d p_iniRotMat{{0.0, 0.0, 0.0},{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
        Matrix3d p_prevRotMat{{0.0, 0.0, 0.0},{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
        Matrix3d p_incRotMat{{0.0, 0.0, 0.0},{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
        Matrix3d p_inertialFrameMat{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}; // Inertial frame matrix
        Matrix3d p_rotMatBodyToInertialFrame{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}; // Rotation matrix to move from the body to inertial frame
        Vector3d p_xb, p_yb, p_zb;  // Body-fixed coordinate in body-fixed frame
        Vector3d p_xeBw, p_yeBw, p_zeBw;  // Body-fixed coordinate in global-frame - estimated based on Backward sln
        Vector3d p_localRotAxis;

        // Motion profile variables
        Vector3d p_refAngularVelVec;            // Reference angular velocity Vector, [rad/s]
        Vector3d p_axisOfRotationNorm;          // Normalized axis of rotation
        Vector3d p_axisOfRotationFindSunNorm;   // Normalized axis of rotation to find the sun
        Vector3d p_angVelErrVec;                // Vector for error between desired angular velocity and actual angular velocity
        double p_deltaThetaTarget = 0;          // Delta target position to move the Grotifer's body, [rad]
        double p_deltaThetaSun = 0;             // Delta target position to move the Grotifer's body back to the Sun, [rad]
        double p_thetaTarget = 0;               // Target position to move the body, [rad]
        double p_thetaSet = 0;                  // Set position for holding position, [rad]
        double p_refVelocity = 0;               // Value of moving velocity, [rad/s]
        double p_refAcceleration = 0;           // Value of moving acceleration, [rad/s^2]
        double p_tMoveWithVelocityOnly = 20.0;  // Default time duration to move with velocity only, [s]
        double p_tEndMoving = 0;                // Time to end moving
        double p_tPossibleDetumbling = 60.0;    // Possible time to fully detumbling
        double p_tPossibleEndDetumbling = 0;    // Possible time to end detumbling
        double p_thetaProf = 0,                 // Profile angle, [rad]
               p_wProf = 0,                     // Profile angular velocity [rad/s]
               p_eProf = 0;                     // Profile angular acceleration [rad/s^2]

        double p_t0 = 0, p_tA = 0, p_tB = 0, p_tC = 0,
               p_t1 = 0, p_t2 = 0, p_t3 = 0;    // Time step for position profile, [s]


        // Initial state and variables needed for angular velocity calculations
        double p_time = 0;
        double p_preTime = 0;
        double p_rotAngle = 0;                  // Rotation angle, [rad]
        Vector3d p_angularVelocityVec;
        Vector3d p_angularVelocityVecFiltered;
        Vector3d p_preAngularVelocityVecFiltered;
        bool p_readyForAngVelocity = false;     //First time step has no angular velocity

        double p_fc = 4.0;                      // Default cut-off frequency for angular velocity calculation

        // Variables for actuating the momentum wheels
        double p_tIniTorque = 0,        // Time for initializing the motion
               p_tEnd = 0,              // Time to end ramping up
               p_iniTorqueWidth = 0,    // Width of the torque pulse
               p_torqueCmd = 0;
        Vector3d p_iniTorqueVec;        // Initiate torque vector
        Vector3d p_torqueDes;           // Desired torque vector

        double p_anglVelLimitX = 4.5e-3,    // Limit for seeing the body is not moving
               p_anglVelLimitY = 1.1e-3,    // Limit for seeing the body is not moving
               p_anglVelLimitZ = 1.1e-3;    // Limit for seeing the body is not moving

        bool p_iniMotionDone = false;               // Flag for finishing initiate motion
        bool p_detumblingDone = false;              // Flag for  finishing de-tumbling
        bool p_holdPosFlag = false;                 // Flag for holding position
        bool p_movePosFlag = false;                 // Flag for moving position
        bool p_onlyDoMeasurement = false;           // Flag for only do measurement
        bool p_moveWithVelocityOnlyFlag = false;    // Flag for only move with velocity
        bool p_startMovingFlag = false;             // Flag for start moving process
        bool p_readyToMove = false;                 // Flag for ready to move
        bool p_readyToFindSun = false;              // Flag for ready to find Sun
        bool p_stopFindingSun = false;              // Flag for stop finding Sun
        bool p_stopMovingFlag = false;              // Flag for stop moving
        bool p_doneMovingFlag = false;              // Flag for done moving process
        bool p_doneFindingSunFlag = false;          // Flag for finding Sun

        double p_momOfInertiaX = 1,     // Moment of inertia for X-momentum wheel
               p_momOfInertiaY = 1,     // Moment of inertia for Y-momentum wheel
               p_momOfInertiaZ = 1;     // Moment of inertia for Z-momentum wheel

        double p_kControlX = 1,         // Controller gain for the X-momentum wheel
               p_kControlY = 1,         // Controller gain for the Y-momentum wheel
               p_kControlZ = 1;         // Controller gain for the Z-momentum wheel

        PIControl *p_piX = NULL,        // PI controller for X
                  *p_piY = NULL,        // PI controller for Y
                  *p_piZ = NULL,        // PI controller for Z
                  *p_piBody = NULL;     // PI Controller for body

        MaxonMotor *p_mmX = NULL,       // Maxon motor for X-momentum wheel
                   *p_mmY = NULL,       // Maxon motor for Y-momentum wheel
                   *p_mmZ = NULL;       // Maxon motor for Z-momentum wheel

        long int p_maxVelX = 9000,      // Max velocity for X-momentum wheel
                 p_maxVelY = 9000,      // Max velocity for Y-momentum wheel
                 p_maxVelZ = 9000;      // Max velocity for Z-momentum wheel

        long int p_momtWheelXVelCmd = 0,       // Cmd Velocity of X-momentum wheel
                 p_preMomtWheelXVelCmd = 0,    // Cmd Previous velocity of X-momentum wheel
                 p_momtWheelYVelCmd = 0,       // Cmd Velocity of Y-momentum wheel
                 p_preMomtWheelYVelCmd = 0,    // Cmd Previous velocity of Y-momentum wheel
                 p_momtWheelZVelCmd = 0,       // Cmd Velocity of Z-momentum wheel
                 p_preMomtWheelZVelCmd = 0;    // Cmd Previous velocity of Z-momentum wheel

        double p_maxAccCmdX = 10000,      // Maximum acceleration for X-momentum wheel
               p_maxAccCmdY = 10000,      // Maximum acceleration for Y-momentum wheel
               p_maxAccCmdZ = 10000;      // Maximum acceleration for Z-momentum wheel

        double p_torqueCmdX = 0,        // Torque command for X
               p_torqueCmdY = 0,        // Torque command for Y
               p_torqueCmdZ = 0,        // Torque command for Z
               p_torqueCmdValTemp;      // Temporary variable to store torque cmd

        long int p_momtWheelXVel = 0,       // Velocity of X-momentum wheel
                 p_momtWheelYVel = 0,       // Velocity of Y-momentum wheel
                 p_momtWheelZVel = 0;       // Velocity of Z-momentum wheel

        // Placeholders to keep track of the motor data you are using
        double *p_momOfInertia = NULL;
        double *p_maxAccCmd = NULL;
        long int *p_preMomtWheelVelCmd = NULL;
        long int *p_momtWheelVelCmd = NULL;
        long int *p_maxVel = NULL;

        double p_timeIni = 0, p_preTimeIni = 0, p_deltaTIni = 0;
        double p_timeDetumb = 0, p_preTimeDetumb = 0, p_deltaTDetumb = 0;
        double p_timeFindSun = 0,   p_preTimeFindSun = 0, p_deltaTFindSun = 0;
        double p_timeMoving = 0, p_preTimeMoving = 0, p_deltaTMoving = 0;
        double p_timeHolding = 0,   p_preTimeHolding = 0, p_deltaTHolding = 0;

        // Data file
        ofstream *p_bwDataFile = NULL, *p_sensorsDataFile = NULL, *p_angularVelDataFile = NULL, *p_momentumWheelsData = NULL;
        unsigned int w = 25;

        // State related variables
        static const unsigned int INITIALIZING = 0;         // Initializing state
        static const unsigned int DETERMINING_ALTITUDE = 1; // Calculating altitude/angular velocity
        static const unsigned int INITIALIZING_MOTION = 2;  // Initializing motion state
        static const unsigned int DETUMBLING = 3;           // De-tumbling state
        static const unsigned int FINDING_SUN = 4;
        static const unsigned int HOLDING_POSITION = 5;     // Holding position state
        static const unsigned int MOVING = 6;               // Moving state
};

// Coordinate task
class TaskCordinate : public BaseTask   // Coordinate between each task
{
    public:
        TaskCordinate(char* taskName, unsigned int taskID, double taskPeriod,
                      bool holdPosAfterDeploying, bool onlyDoMeasurement, bool onlyControlBody,
                      TorpControl &tct_l, TorpControl &tct_r, TorpMasterControl &mtct,
                      AltitudeControl &atc, ofstream &auditTrailDataFile)
        {
            p_taskName = taskName;
            p_taskID = taskID;
            p_deltaTaskTime = taskPeriod;
            p_holdPosAfterDeploy = holdPosAfterDeploying;
            p_onlyDoMeasurement = onlyDoMeasurement;
            p_onlyControlBody = onlyControlBody;
            p_tct_l = &tct_l;
            p_tct_r = &tct_r;
            p_mtct = &mtct;
            p_atc = &atc;
            p_auditTrailDataFile = &auditTrailDataFile;
            p_state = INITIALIZING;
            p_stateName = "Initializing";
            p_nextState = p_state;
            p_nextStateName = p_stateName;
        }

        ~TaskCordinate() {};

        int Run()
        {
            if (!p_allowedToRun)    // The task is not allow to run
            {
                return -1;
            }

            int done = 1;   // Default return value indicating done for this event
            p_numScans++;   // Increasement the scan count

            if(GetTimeNow() < p_nextTaskTime)  // It's not the time for the task to run
            {
                p_runSuccess = false;
                return done; // Done for now. Not the propriate time for the task to run
            }

            p_timeStart = GetTimeNow();
            p_nextTaskTime += p_deltaTaskTime; // Increase the next time that the task can run
            p_runSuccess = true;    // Mark this as a successful run

            p_state = p_nextState;  // Switch current state

            switch (p_state)
            {
                case INITIALIZING:
                    // Entry code
                    p_stateName = "Initializing";

                    // Execution code
                    // Check to see if only do measurement
                    if (p_onlyDoMeasurement)
                    {
                        p_nextState = DEPLOYING;
                        p_nextStateName = "Deploying";

                    }
                    else
                    {
                        // Check to see if the altitude control is done with detumbling
                        if ((*p_atc).GetFindingSunDoneFlag())
                        {
                            if (!p_onlyControlBody) // Check to see if the program only control the body
                            {
                                (*p_mtct).SetAllowedToRunFlag(true);
                                if (!(*p_mtct).GetFirstTimeRunFlag())
                                {
                                    p_nextState = DEPLOYING;
                                    p_nextStateName = "Deploying";
                                    (*p_tct_l).SetAllowedToRunFlag(true);
                                    (*p_tct_r).SetAllowedToRunFlag(true);
                                }


                            }

                            else
                            {
                                p_nextState = MOVING;
                                p_nextStateName = "Moving";
                                // Stop the TORP controller
                                (*p_tct_l).SetAllowedToRunFlag(false);
                                (*p_tct_r).SetAllowedToRunFlag(false);
                                (*p_mtct).SetAllowedToRunFlag(false);
                            }
                        }

                        else
                        {
                            p_nextState = INITIALIZING;
                            p_nextStateName = "Initializing";
                            // Stop the TORP controller
                            (*p_tct_l).SetAllowedToRunFlag(false);
                            (*p_tct_r).SetAllowedToRunFlag(false);
                            (*p_mtct).SetAllowedToRunFlag(false);
                        }
                    }


                    break;

                case DEPLOYING:
                    // Entry Code
                    p_stateName = "Deploying";

                    // Execution code
                    // Check conditions
                    if (!p_holdPosAfterDeploy)
                    {
                        if ((*p_mtct).GetReadyToDeploySensorsFlag())
                        {
                            if (p_tStartDeploySensors <= 0)
                            {
                                p_tStartDeploySensors = GetTimeNow() + 10.0;
                            }

                            else
                            {
                                if (GetTimeNow() >= p_tStartDeploySensors)
                                {
                                    (*p_mtct).SetDeployingFlag(true);
                                }
                                else
                                {
                                    (*p_mtct).SetDeployingFlag(false);
                                }

                            }

                        }
                        if((*p_mtct).GetDeployingDoneFlag())
                        {
                            p_nextState = MOVING; // Switch to moving state
                            p_nextStateName = "Moving";

                            // Enable moving the body
                            (*p_atc).SetStartMovingFlag(true);
                        }

                        else (*p_atc).SetStartMovingFlag(false);
                    }

                    else (*p_atc).SetStartMovingFlag(false);

                    break;

                case MOVING:
                    // Entry Code
                    p_stateName = "Moving";

                    // Execution code
                    if (p_onlyControlBody)
                    {
                        (*p_tct_l).SetAllowedToRunFlag(false);
                        (*p_tct_r).SetAllowedToRunFlag(false);
                        (*p_mtct).SetAllowedToRunFlag(false);

                    }

                    // Enable moving the body
                    (*p_atc).SetStartMovingFlag(true);

                    // Check condition
                    if ((*p_atc).GetDoneMovingFlag())
                    {
                        p_nextState = STOPPING;
                        p_nextStateName = "Stopping";

                        // start retracting the masses
                        (*p_mtct).SetRetractingFlag(true);
                    }

                    break;

                case STOPPING:
                    // Entry Code
                    p_stateName = "Stopping";

                    // Execution code
                    if (p_onlyControlBody)
                    {
                        (*p_tct_l).SetAllowedToRunFlag(false);
                        (*p_tct_r).SetAllowedToRunFlag(false);
                        (*p_mtct).SetAllowedToRunFlag(false);

                    }

                    // Enable moving the body
                    (*p_atc).SetStartMovingFlag(true);

                    break;
            }

            // Get the timestamp and finish the task
            p_timeEnd = GetTimeNow();  // Get the end time stamp for the task
            AuditTrailRecord();  // Record the state here
            return done;
        }


    protected:
        TorpControl *p_tct_l = NULL,
                    *p_tct_r = NULL;

        TorpMasterControl *p_mtct = NULL;
        AltitudeControl *p_atc = NULL;
        bool p_holdPosAfterDeploy = false;
        bool p_onlyDoMeasurement = false;
        bool p_onlyControlBody = false;

        double p_tStartDeploySensors = 0;

        static const unsigned int INITIALIZING = 0;         // Initializing state
        static const unsigned int DEPLOYING = 1;            // Deploying state
        static const unsigned int MOVING = 2;               // Moving state
        static const unsigned int STOPPING = 3;             // Stopping state

};

int main()
{
    // Configuration of the program
    // --- Configuration flag --- //
    bool manualHomingFlag = false;                              // Change to true for manual homing
    bool onlyControlBody = false;                                // Flag to perform control body only
    bool onlyDoMeasurementFlag = false;                         // Flag to perform only measurement
    bool onlyMoveWithVelocityFlag = false;                      // Flag to move with velocity only
    bool holdPosAfterDeploy = false;                             // Flag to hold position after deploying
    bool deployingSensorsFlag = true;                          // Flag to know if sensors are allowed to deploy
    bool startDeployBySoftwareFlag = true;                      // Flag to start deploying by software
    bool exciteByMotor = true;                                  // Flag to extie the motion by moving momentum wheels automatically

    // -- Initial conditions setup -- //
    double tIni = 1.5 * ((double) exciteByMotor);               // Time to excite the motion, [s]
    double torqueWidth = 500.0e-3 * ((double) exciteByMotor);   // Time width for the torque pulse, [s]
    double scaleFactor = 12 * ((double) exciteByMotor);         // Scale factor for initial torque magnitude
    double iniTorqueMag = 0.1* ((double) exciteByMotor);        // Initial magnitude of torque
    Vector3d iniTorqueVec;
    iniTorqueVec << 0.5 * scaleFactor * iniTorqueMag, scaleFactor * iniTorqueMag, -0.5 * scaleFactor * iniTorqueMag;  // Initial torque vector, [Nm]

    // --- Movement configuration --- //
    double deltaThetaTarget = 20.0;            // Angle to rotate, [deg]
    Vector3d rotAxis;
    rotAxis << 0.0, 1.0, 0.0;                  // Axis of rotation
    double anglVelRef = 0.015;                 // Reference Angular velocity
    double anglAccRef = 2.0e-3;                // Reference Angular acceleration
    double tRun = 20.0;                        // Run time

    // --- TORP Profile configuration --- //
    double oprVel = 12;                        // Operation velocity, [RPM]
    double accScaleFactor = 1.0;               // Scale factor for max acceleration of the TORP, use 1.0 if weights are mounted
    double maxAccMag = accScaleFactor * 0.321; // Max acceleration, [RPM/s]
    double tAccDec = 60;                       // Acc/Dec time, [s], use 60 secs if weights are mounted
    double tCruise = 20;                       // Cruising time, [s]
    double homingVelMag = 0.643;               // Homing velocity at the TORP
    double stepperSpeed = (uint32_t) ((200.0 / 60) * (1.0e4/0.04));    // Speed for the stepper motor, [steps/10000s]



    // Parameters of the system
    // --- PI Body motion controller --- //
    PIControlPara PIBodyCtrlPara;
    PIBodyCtrlPara.kp = 0.2;
    PIBodyCtrlPara.ki = 0.0;
    PIBodyCtrlPara.hLim = 0.1;
    PIBodyCtrlPara.lLim = -0.1;

    PIControl PIBodyCtrl(PIBodyCtrlPara);

    // --- Momentum Wheels Controller --- //
    // X-momentum wheel
    double kControlX = 0.175;                   // Controller gain for X
    PIControlPara PIMomXPara;
    PIMomXPara.kp = 1.0 * kControlX;
    PIMomXPara.ki = 1.5e-3 * kControlX;
    PIMomXPara.hLim = 0.5;
    PIMomXPara.lLim = -0.5;

    PIControl PIMomXCtrl(PIMomXPara);

    // Y-momentum wheel
    double kControlY = 0.2;                     // Controller gain for Y
    PIControlPara PIMomYPara;
    PIMomYPara.kp = 1.0 * kControlY;
    PIMomYPara.ki = 1.5e-3 * kControlY;
    PIMomYPara.hLim = 1.0;
    PIMomYPara.lLim = 1.0;

    PIControl PIMomYCtrl(PIMomYPara);

    // Z-momentum wheel
    double kControlZ = 0.5;                     // Controller gain for Z
    PIControlPara PIMomZPara;
    PIMomZPara.kp = 1.0 * kControlZ;
    PIMomZPara.ki = 1.5e-3 * kControlZ;
    PIMomZPara.hLim = 0.5;
    PIMomZPara.lLim = -0.5;

    PIControl PIMomZCtrl(PIMomZPara);

    // Left Motor
    PIControlPara leftPICntrlPara;
    leftPICntrlPara.kp = 20; // Best KP = 8
    leftPICntrlPara.ki = 18.5; // Best Ki = 0.075
    leftPICntrlPara.hLim = 32.0;
    leftPICntrlPara.lLim = -32.0;

    PIControl leftTorpPi(leftPICntrlPara);

    // Right Motor
    PIControlPara rightPICntrlPara;
    rightPICntrlPara.kp = 10;
    rightPICntrlPara.ki = 8.5;
    rightPICntrlPara.hLim = 32.0;
    rightPICntrlPara.lLim = -32.0;

    PIControl rightTorpPi(rightPICntrlPara);

    // --- Momentum Wheels Configurations --- //
    // X-momentum wheel
    double momOfInertiaX = 3.6383e-5;           // Moment of inertia for X, [kg.m^2]
    long int maxVelX = 9000;                    // Maximum speed for X-momentum wheel, [rpm]

    // Y-momentum wheel
    double momOfInertiaY = 3.6383e-5;           // Moment of inertia for Y, [kg.m^2]
    long int maxVelY = 9000;                    // Maximum speed for Y-momentum wheel, [rpm]

    // Z-momentum wheel
    double momOfInertiaZ = 3.6383e-5;           // Moment of inertia for Z, [kg.m^2]
    long int maxVelZ = 9000;                    // Maximum speed for Z-momentum wheel, [rpm]

    // --- Maxon Boards --- //
    // X-motor
    maxon xMotorPara;
    xMotorPara.nameMotor = "xMomMotor";
    xMotorPara.serialNo = 0x37058243;
    xMotorPara.K_P = 269805;
    xMotorPara.K_I = 2549658;
    xMotorPara.KFF_VEL = 4917;
    xMotorPara.KFF_ACC = 7138;
    xMotorPara.K_TUNING = 1;
    xMotorPara.KP_CURR = 1591597;
    xMotorPara.KI_CURR = 2453833;
    xMotorPara.KT = 24.6 * 1.0e-3;
    xMotorPara.NUM_OF_PULSE_PER_REV = 512;
    xMotorPara.SENSOR_POLARITY = 0;

    // Y-motor
    maxon yMotorPara;
    yMotorPara.nameMotor = "yMomMotor";
    yMotorPara.serialNo = 0x37058261;
    yMotorPara.K_P = 266911;
    yMotorPara.K_I = 2522306;
    yMotorPara.KFF_VEL = 4801;
    yMotorPara.KFF_ACC = 7061;
    yMotorPara.K_TUNING = 1;
    yMotorPara.KP_CURR = 1573403;
    yMotorPara.KI_CURR = 2395082;
    yMotorPara.KT = 24.6 * 1.0e-3;
    yMotorPara.NUM_OF_PULSE_PER_REV = 512;
    yMotorPara.SENSOR_POLARITY = 0;

    // Z-motor
    maxon zMotorPara;
    zMotorPara.nameMotor = "zMomMotor";
    zMotorPara.serialNo = 0x37059351 ;
    zMotorPara.K_P = 259697;
    zMotorPara.K_I = 2454133;
    zMotorPara.KFF_VEL = 4895;
    zMotorPara.KFF_ACC = 6870;
    zMotorPara.K_TUNING = 1;
    zMotorPara.KP_CURR = 1598083;
    zMotorPara.KI_CURR = 2575556;
    zMotorPara.KT = 24.6 * 1.0e-3;
    zMotorPara.NUM_OF_PULSE_PER_REV = 512;
    zMotorPara.SENSOR_POLARITY = 0;

    // Left Motor
    homingProfilePara leftHomingProfPara;
    leftHomingProfPara.homingVel = homingVelMag;
    leftHomingProfPara.maxAcc = maxAccMag;
    leftHomingProfPara.offsetPos = 51.75 - 3.25;
    leftHomingProfPara.offsetPosLim = 2.5;
    leftHomingProfPara.startPosLim = 2.0;

    maxon leftMotorPara;
    leftMotorPara.nameMotor = "LeftTORPMOT";
    leftMotorPara.serialNo = 0x40011322;
    leftMotorPara.K_P = 41985;
    leftMotorPara.K_I = 2219421;
    leftMotorPara.KFF_VEL = 2058;
    leftMotorPara.KFF_ACC = 397;
    leftMotorPara.K_TUNING = 1.5013;
    leftMotorPara.KP_CURR = 1171880;
    leftMotorPara.KI_CURR = 3906250;
    leftMotorPara.KT = 24.6 * 1.0e-3;
    leftMotorPara.NUM_OF_PULSE_PER_REV = 512;
    leftMotorPara.SENSOR_POLARITY = 0;

    stepperPara leftStepper1Para;
    leftStepper1Para.sm = 0;
    leftStepper1Para.baudrate = 9600;
    leftStepper1Para.maxCurr = 3500;   // 1500 for Tic 249, 3500 for Tic36v4
    leftStepper1Para.stepMode = 0;
    leftStepper1Para.maxSpeed = stepperSpeed;
    leftStepper1Para.startSpeed = stepperSpeed;

    stepperPara leftStepper2Para;
    leftStepper2Para.sm = 0;
    leftStepper2Para.baudrate = 9600;
    leftStepper2Para.maxCurr = 3500;   // 1500 for Tic 249, 3500 for Tic36v4
    leftStepper2Para.stepMode = 0;
    leftStepper2Para.maxSpeed = stepperSpeed;
    leftStepper2Para.startSpeed = stepperSpeed;

    // Right Motor
    homingProfilePara rightHomingProfPara;
    rightHomingProfPara.homingVel = -homingVelMag;
    rightHomingProfPara.maxAcc = maxAccMag;
    rightHomingProfPara.offsetPos = 244.35 - 0.75;
    rightHomingProfPara.offsetPosLim = 1.5;
    rightHomingProfPara.startPosLim = 1.5;

    maxon rightMotorPara;
    rightMotorPara.nameMotor = "RightTORPMOT";
    rightMotorPara.serialNo = 0x40006029;
    rightMotorPara.K_P = 59128;
    rightMotorPara.K_I = 3488178;
    rightMotorPara.KFF_VEL = 888;
    rightMotorPara.KFF_ACC = 0;
    rightMotorPara.K_TUNING = 1.14568;
    rightMotorPara.KP_CURR = 1847321;
    rightMotorPara.KI_CURR = 3129455;
    rightMotorPara.KT = 24.6 * 1.0e-3;
    rightMotorPara.NUM_OF_PULSE_PER_REV = 512;
    rightMotorPara.SENSOR_POLARITY = 0;

    stepperPara rightStepper1Para;
    rightStepper1Para.sm = 0;
    rightStepper1Para.baudrate = 9600;
    rightStepper1Para.maxCurr = 3500;   // 1500 for Tic 249, 3500 for Tic36v4
    rightStepper1Para.stepMode = 0;
    rightStepper1Para.maxSpeed = stepperSpeed;
    rightStepper1Para.startSpeed = stepperSpeed;

    stepperPara rightStepper2Para;
    rightStepper2Para.sm = 0;
    rightStepper2Para.baudrate = 9600;
    rightStepper2Para.maxCurr = 3500;   // 1500 for Tic 249, 3500 for Tic36v4
    rightStepper2Para.stepMode = 0;
    rightStepper2Para.maxSpeed = stepperSpeed;
    rightStepper2Para.startSpeed = stepperSpeed;



    // Data files creation
    ofstream bwDataFile, sensorsDataFile, auditTrailDataFile, angularVelDataFile, momentumWheelsData, eposErrFile, leftTorpDataFile, rightTorpDataFile, masterTorpDataFile;

    // Open all the data files
    eposErrFile.open("SystemData/Maxon EPOS Error Log.txt");
    auditTrailDataFile.open("SystemData/Audit Trail Data.txt");
    bwDataFile.open("SystemData/Sensors Experiment Data Backward Solution.txt");
    sensorsDataFile.open("SystemData/Sensor Data.txt");
    angularVelDataFile.open("SystemData/Angular Velocity Data.txt");
    momentumWheelsData.open("SystemData/Momentum Wheels Data.txt");
    leftTorpDataFile.open("SystemData/Left Torp Data.txt");
    rightTorpDataFile.open("SystemData/Right Torp Data.txt");
    masterTorpDataFile.open("SystemData/Master Torp Data.txt");

    // Formate the data files
    int w = 25;
    auditTrailDataFile << left << setw(w) << "Start" << left << setw(w) << "End" << left << setw(w) << "Duration[ms]" << left << setw(w) << "TaskName"
                       << left << setw(w) << "TaskID" << left << setw(w) << "CurrState" << left << setw(w) << "NextState" << endl;

    bwDataFile << left << setw(w) << "Time (s)" << left << setw(w) << "Xb_x" << left << setw(w) << "Xb_y" << left << setw(w) << "Xb_z"
                                                << left << setw(w) << "Yb_x" << left << setw(w) << "Yb_y" << left << setw(w) << "Yb_z"
                                                << left << setw(w) << "Zb_x" << left << setw(w) << "Zb_y" << left << setw(w) << "Zb_z" << endl;
    sensorsDataFile << left << setw(w) << "Time (s)" << left << setw(w) << "thxIncl(deg)" << left << setw(w) << "thzIncl(deg)"
                                                     << left << setw(w) << "thySun(deg)" << left << setw(w) << "thzSun(deg)"
                                                     << left << setw(w) << "thxEuler(deg)" << left << setw(w) << "thyEuler(deg)" << left << setw(w) << "thzEuler(deg)" << endl;
    angularVelDataFile << left << setw(w) << "Time(s)" << left << setw(w) << "AngVel-x(rad/s)" << left << setw(w) << "AngVel-y(rad/s)" << left << setw(w) << "AngVel-z(rad/s)"
                                                       << left << setw(w) << "Ref.AngVelX(rad/s)" << left << setw(w) << "Ref.AngVelY(rad/s)" << left << setw(w) << "Ref.AngVelZ(rad/s)"
                                                       << left << setw(w) << "Err.AngVelX(rad/s)" << left << setw(w) << "Err.AngVelY(rad/s)" << left << setw(w) << "Err.AngVelZ(rad/s)"
                                                       << left << setw(w) << "Rot.Angle(deg)" << left << setw(w) << "Prof.Angle(deg)"  << left << setw(w) << "Prof.w(rad/s)" << left << setw(w) << "Prof.e(rad/s^2)"<< endl;

    momentumWheelsData << left << setw(w) << "Time(s)" << left << setw(w) <<  "TorqueX(Nm)" << left << setw(w) << "XWheelVelCmd(rpm)" << left << setw(w) << "XWheelVelAct(rpm)"
                                                       << left << setw(w) <<  "TorqueY(Nm)" << left << setw(w) << "YWheelVelCmd(rpm)" << left << setw(w) << "YWheelVelAct(rpm)"
                                                       << left << setw(w) <<  "TorqueZ(Nm)" << left << setw(w) << "ZWheelVelCmd(rpm)" << left << setw(w) << "ZWheelVelAct(rpm)" << endl;

    leftTorpDataFile << left << setw(w) << "Time (s)" << left << setw(w) << "HomePos (deg)" << left << setw(w) << "StartingPos (deg)"
                                                      << left << setw(w) << "RefPos (deg)" << left << setw(w) << "TorpPos (deg)"
                                                      << left << setw(w) << "RefVel (rpm)" << left << setw(w) << "TorpVel (rpm)"
                                                      << left << setw(w) << "MotorPos (deg)" << left << setw(w) << "MotorVel (rpm)"
                                                      << left << setw(w) << "PositionErr (deg)" << left << setw(w) << "DesVelCmd (rpm)" << endl;

    rightTorpDataFile << left << setw(w) << "Time (s)"  << left << setw(w) << "HomePos (deg)" << left << setw(w) << "StartingPos (deg)"
                                                        << left << setw(w) << "RefPos (deg)" << left << setw(w) << "TorpPos (deg)"
                                                        << left << setw(w) << "RefVel (rpm)" << left << setw(w) << "TorpVel (rpm)"
                                                        << left << setw(w) << "MotorPos (deg)" << left << setw(w) << "MotorVel (rpm)"
                                                        << left << setw(w) << "PositionErr (deg)" << left << setw(w) << "DesVelCmd (rpm)" << endl;

    masterTorpDataFile << left << setw(w) << "Time (s)" << left << setw(w) << "ProfilePos (deg)" << left << setw(w) << "ProfileVel (rpm)" << left << setw(w) << "ProfileAcc (rpm/s)" << endl;

    // Open all the components
    // LabJack U6
    HANDLE hDevice;
    if(!OpenLabJackU6(hDevice))
    {
        exit(0);    // Fail to open the LabJack
    }

    // Sun Sensor
    modbus_t *ctx;                  // Modbus pointer
    const int BIT_RATE = 115200;    // Default bit rate is 19200, one of the sun sensor has 115200 baudrate
    if(!OpenSunSensor(SUNSENSOR_SERIAL_PORT, BIT_RATE, ctx))
    {
        exit(0);    // Fail to establish connection with the Sun Sensor
    }

    // Maxon Motors
    unsigned int noOfOpenedPorts = 5;
    void* keyHandle[noOfOpenedPorts];
    uint32_t serialNo[noOfOpenedPorts];
    string* availPortNameList;
    if(!openMaxonMotors(keyHandle, serialNo, availPortNameList))
    {
        exit(0);    // Fail to connect to Maxon boards
    }
    // Scan to match handle with Maxon board
    // X-motor
    for(int i=0; i<noOfOpenedPorts; i++)
    {
        if(serialNo[i] == xMotorPara.serialNo)   // Compare serial number to identify the Maxon motor
        {
            xMotorPara.keyHandle = keyHandle[i];
            xMotorPara.portName = &availPortNameList[i][0];
        }
    }

    // Y-motor
    for(int i=0; i<noOfOpenedPorts; i++)
    {
        if(serialNo[i] == yMotorPara.serialNo)   // Compare serial number to identify the Maxon motor
        {
            yMotorPara.keyHandle = keyHandle[i];
            yMotorPara.portName = &availPortNameList[i][0];
        }
    }

    // Z-motor
    for(int i=0; i<noOfOpenedPorts; i++)
    {
        if(serialNo[i] == zMotorPara.serialNo)   // Compare serial number to identify the Maxon motor
        {
            zMotorPara.keyHandle = keyHandle[i];
            zMotorPara.portName = &availPortNameList[i][0];
        }
    }

    // Left Motor
    for(int i=0; i<noOfOpenedPorts; i++)
    {
        if(serialNo[i] == leftMotorPara.serialNo)   // Compare serial number to identify the Maxon motor
        {
            leftMotorPara.keyHandle = keyHandle[i];
            leftMotorPara.portName = &availPortNameList[i][0];
        }
    }

    // Right Motor
    for(int i=0; i<noOfOpenedPorts; i++)
    {
        if(serialNo[i] == rightMotorPara.serialNo)   // Compare serial number to identify the Maxon motor
        {
            rightMotorPara.keyHandle = keyHandle[i];
            rightMotorPara.portName = &availPortNameList[i][0];
        }
    }

    // Stepper motors
    leftStepper1Para.sm = open(LEFT_STEPPER_1, O_RDWR | O_NOCTTY);
    leftStepper2Para.sm = open(LEFT_STEPPER_2, O_RDWR | O_NOCTTY);
    rightStepper1Para.sm = open(RIGHT_STEPPER_1, O_RDWR | O_NOCTTY);
    rightStepper2Para.sm = open(RIGHT_STEPPER_2, O_RDWR | O_NOCTTY);

    if (leftStepper1Para.sm == -1)
    {
        perror(LEFT_STEPPER_1);
        return -1;
    }

    if (leftStepper2Para.sm == -1)
    {
        perror(LEFT_STEPPER_2);
        return -1;
    }

    if (rightStepper1Para.sm == -1)
    {
        perror(RIGHT_STEPPER_1);
        return -1;
    }

    if (rightStepper2Para.sm == -1)
    {
        perror(RIGHT_STEPPER_2);
        return -1;
    }



    // Initiate all objects for all components of the system
    // LabJack U6
    LabJackU6 grotiferLJU6(hDevice, 0);

    //Initiate Inclinometer Objeck
    LabJackInclinometer inclinometer(grotiferLJU6, 0, 1);    // [TODO] CHECK THE ACTUAL WIRE CONNECTION ON THE TESTBED

    // Initiate the US Digital Object
    LJEncoder3Channels rightUSDigEnc(grotiferLJU6, 400, 1);
    LJEncoder3Channels leftUSDigEnc(grotiferLJU6, 400, 2);

    // Initiate the Sun Sensor object
    ModbusSunSensor sunSensor(ctx);

    // Create objects for each Maxon boards
    MaxonMotor XMomentumMotor(eposErrFile, xMotorPara, 'v');
    MaxonMotor YMomentumMotor(eposErrFile, yMotorPara, 'v');
    MaxonMotor ZMomentumMotor(eposErrFile, zMotorPara, 'v');
    MaxonMotor leftTorpMotor(eposErrFile, leftMotorPara, 'v');
    MaxonMotor rightTorpMotor(eposErrFile, rightMotorPara, 'v');

    // Create objects for Steppers
    StepperMotor leftStepper1(leftStepper1Para);
    StepperMotor leftStepper2(leftStepper2Para);
    StepperMotor rightStepper1(rightStepper1Para);
    StepperMotor rightStepper2(rightStepper2Para);

    // Initiate the ACS task
    AltitudeControl GrotiferAltitudeControl("AltitudeCntrl", 1, 75e-3, sunSensor, inclinometer, onlyDoMeasurementFlag,
                                            tIni, torqueWidth, iniTorqueVec, onlyMoveWithVelocityFlag, holdPosAfterDeploy,
                                            rotAxis, Deg2Rad(deltaThetaTarget), anglVelRef, anglAccRef, PIBodyCtrl,
                                            momOfInertiaX, kControlX, PIMomXCtrl, maxVelX, XMomentumMotor,
                                            momOfInertiaY, kControlY, PIMomYCtrl, maxVelY, YMomentumMotor,
                                            momOfInertiaZ, kControlZ, PIMomZCtrl, maxVelZ, ZMomentumMotor,
                                            bwDataFile, sensorsDataFile, angularVelDataFile, momentumWheelsData, auditTrailDataFile);
    // Initiate the TORP Control Task
    TorpControl leftTorpControl("LeftTorpCtrl", 3, 150.0e-3, leftHomingProfPara, leftTorpPi, leftTorpMotor, leftUSDigEnc, false, 31.1, leftTorpDataFile, auditTrailDataFile);
    TorpControl rightTorpControl("RightTorpCtrl", 4, 150.0e-3, rightHomingProfPara, rightTorpPi, rightTorpMotor, rightUSDigEnc, false, 31.1, rightTorpDataFile, auditTrailDataFile);

    // Initiate the Master TORP Control task
    TorpMasterControl torpMasterControl("MasterTorpCtrl", 2, 150.0e-3, oprVel, maxAccMag, tAccDec, tCruise, manualHomingFlag, deployingSensorsFlag, startDeployBySoftwareFlag, leftTorpControl, rightTorpControl,
                                        leftStepper1, leftStepper2, rightStepper1, rightStepper2, masterTorpDataFile, auditTrailDataFile);

    // Initiate Coordinating task
    TaskCordinate taskCoordinator("TaskCoord", 0, 80e-3, holdPosAfterDeploy, onlyDoMeasurementFlag, onlyControlBody, leftTorpControl, rightTorpControl, torpMasterControl, GrotiferAltitudeControl, auditTrailDataFile);

    int numOfTasks = 5;
    BaseTask* TaskList[numOfTasks];
    TaskList[0] = &taskCoordinator;
    TaskList[1] = &GrotiferAltitudeControl;
    TaskList[2] = &torpMasterControl;
    TaskList[3] = &leftTorpControl;
    TaskList[4] = &rightTorpControl;

    // Main loop
    double T_PROGRAM = 8.6 * 60;  //Run time of the program, [sec]
    StartTimer();
    while (GetTimeNow() <= T_PROGRAM)
    {
        // Scan through the list of tasks -> run -> log audit trail data for each task
        for (int i = 0; i < numOfTasks; i++)
        {
            TaskList[i]->Run();
        }
    }

    // Halt the motions
    GrotiferAltitudeControl.HaltMotion();

    // Close all files
    auditTrailDataFile.close();
    bwDataFile.close();
    sensorsDataFile.close();
    angularVelDataFile.close();
    momentumWheelsData.close();
    eposErrFile.close();
    leftTorpDataFile.close();
    rightTorpDataFile.close();
    masterTorpDataFile.close();

    return 0;
}

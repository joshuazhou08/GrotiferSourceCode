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

using namespace std;

// Global variables
extern double g_t0;   // Initial time

// Struct-type variables
struct PIControlPara   // Struct-type variable to store parameters for PI control
{
    double  kp, // P Gain Coefficient
            ki, // I Gain coefficient
            hLim,  // High Limit for anti wind up
            lLim;  // Low Limit for anti wind up
};

struct homingProfilePara // Struct type variable to store parameters for homing profile
{
    double homingVel;
    double maxAcc;
    double offsetPos;
    double offsetPosLim;
    double startPosLim;
};

// Function to convert from deg -> rad
double Deg2Rad(double angleVal);

// Function to convert from rad -> deg
double Rad2Deg(double angleVal);

// Timer functions
void StartTimer(); // Function to initialize timer -> receive initial time stamp variable to initiate the time for the program

double GetTimeNow();   // Function to get elapsed time from the initial time t0, [s]

// PI Position Control
class PIControl
{
    public:
        PIControl(PIControlPara &pic); // Constructor
        ~PIControl();

        // Get Data functions
        double GetKp();  // Get k_p
        double GetKi();  // Get k_i
        double GetError();   // Get the error from the PI controller

        // Main Controller Code Function
        double PICalculation(double setpoint, double actVal);

    protected:
        double p_deltaT = 0;    // Discrete time to calculate the integral, [sec]
        double p_time = 0;     // Time when the PI Controller is called, [sec]
        double p_preTime = 0;   // Previous time instance when the PI Controller was called, [sec]
        double p_setpoint = 0;  // Setpoint/reference of the controller, [deg]
        double p_actVal = 0;    // Actual value -> actual position
        double p_kp, p_ki;  // Controller Gains
        double p_P_part = 0, p_I_part = 0;  // P_part and I_part of the controller
        double p_uPI = 0;   // Control signal
        double p_error = 0; // Error
        double p_hLim, p_lLim;  // Limit of the controller
};

// BaseTask class for control
class BaseTask
{
    public:
        virtual int Run()=0;  // Main running function of the task. NEED "OVERRIDE" KEYWORD IF BEING USED
        char* GetTaskName();    // Function to get task name
        unsigned int GetTaskID(); // Function to get taskID
        long unsigned int GetNumScan();  // Get the number of scan
        bool GetSuccessRunFlag(); // Get the run success flag
        double GetStartTimeStamp();    // Get the time stamp when the task is run
        double GetEndTimeStamp();    // Get the time stamp when the task is done running
        void SetAllowedToRunFlag(bool flagVal); // Function to set
        void AuditTrailRecord(); // Function to record the trail of the program

    protected:
        char* p_taskName;   // Name of the task
        unsigned int p_taskID;  // ID or order of the task
        unsigned int p_state, p_nextState;  // State ID of the task
        char* p_stateName;  // Name of the current state
        char* p_nextStateName; // Name of the and next state
        ofstream *p_auditTrailDataFile = NULL;  // Pointer for audit trail datafile
        double p_nextTaskTime = 0; // The next time of this task should run, [sec]
        double p_deltaTaskTime;   // Period for this task [sec]
        double p_timeStart, p_timeEnd, p_duration;  // Time stamp for running the task, [sec]
        double p_currTime;  // Current time of the task, [sec]
        long unsigned int p_numScans = 0;  // Number of scan - for performance tracking
        bool p_runSuccess = true;   // Flag for successful run - for performance tracking
        bool p_allowedToRun = true; // Flag for allowing to run the task
};

void runTaskList(double T_PROGRAM, BaseTask** TaskList, int numOfTasks);






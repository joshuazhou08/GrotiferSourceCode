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
#include "Definitions.h"

using namespace std;

// Maxon variables
struct maxon  // Struct type variable to store Maxon global data
{
    // Setting parameters
    char* nameMotor;
    void* keyHandle;
    char* portName;
    uint32_t serialNo;
    long long unsigned int K_P;
    long long unsigned int K_I;
    double K_TUNING;
    long long unsigned int KFF_VEL;
    long long unsigned int KFF_ACC;
    long long unsigned int KP_CURR;
    long long unsigned int KI_CURR;
    unsigned int NUM_OF_PULSE_PER_REV;
    double KT;  // Torque constant
    int SENSOR_POLARITY;
};

char* InterfaceName2PortName(char* p_interfaceName); // Find Interface Name from Port Name
string* ScanAvailablePortNames(char* p_deviceName, char* p_protocolStackName, char* p_interfaceName, unsigned int* p_availPort, unsigned int* p_errorCode);
bool openMaxonMotors (void* keyHandleArr[5], uint32_t serialNoArr[5], string* &availPortNameList);

class MaxonMotor
/*
    [TODO] WRITE DESCRIPTION OF THE CLASS
*/
{
    public:
    MaxonMotor(ofstream &errFile, maxon &mx, char controlMode);   // Basic constructor for Maxon Motor

    ~MaxonMotor();   // Destructor for the class

    // Setting all the parameters of the motor
    void InitSetting();  // Function to fully set the following parameters of the motor: Encoder, Controller (Velocity and Current)
                         // Log error if there's any occured during the call to Maxon library

    // Get the position of the motor
    double GetPositionIs();   // Function to get the position of the motor [deg]

    // Get the velocity of the motor
    double GetVelocityIs();    // Function to get the (averaged) velocity of the motor [rpm]

    // Get the current of the motor
    int GetCurrentIs();   // Function to get the (averaged) current of the motor [mA]

    // Get the operational mode
    int GetOperationalMode(); // Function to get the operational mode

    // Get the torque constant
    double GetKT();  // Function to get torque constant K_T

    // Get Max torque
    double GetMaxTorque();

    // Run the motor with velocity value
    bool RunWithVelocity(int desVel);

    // Run the motor in current control mode with a desired current
    bool RunWithCurrent(int desCur);

    // Halt motion of the motor
    void HaltMotion();

    // Disable the motor
    void SetDisableState();  // Function to disable the motor - used for experiment

    protected:
        void* p_handle; // Handle for all the VCS Functions
        char* p_portName;   // Port name that is corresponding to the EPOS board
        unsigned int p_serialNo; // Serial number for the motor
        char* p_nameMotor;  // Name of the motor
        unsigned int p_nodeID = 1;  // Default NodeID of the motor
        unsigned int p_baudrate = 1000000; // Default baudrate
        unsigned int p_timeout = 500;

        char p_controlMode;   // Control Mode of the motor, either velocity control or current control
                            // "c" - for current control or torque control
                            // "v" - for velocity control
        int p_oprMode;   // Operational Mode;

        long long unsigned int p_K_P, p_K_I, p_KFF_VEL, p_KFF_ACC, p_KP_CURR, p_KI_CURR;    // Controller gains
        double p_K_TUNING;    // Offset gain for the PI Velocity loop

        double p_KT;    // Torque constant
        double p_maxTorque = 36.8e-3;   // Maximum available torque, [Nm]

        unsigned int p_NUM_OF_PULSE_PER_REV = 1;
        int p_SENSOR_POLARITY; // Parameters of the encoder

        int p_motorVel = 0;   // Averaged velocity of the motor, [rpm]
        int p_motorPos = 0;   // Position of the motor, [inc]
        int p_motorCur = 0;   // (Averaged) current of the motor
        int p_desVel = 0; // Desired velocity -> input for the run motor function, [rpm]
        int p_desCur = 0;   // Desired current in current control mode, [mA]
        unsigned int p_errorCode = 0;   // Error Code when calling Maxon library

        ofstream *p_errDataFile;    // Data file to store error during operation

        int LogError(ofstream &errFileName, unsigned int errorCocde); // Logging error data
};

// Stepper Motor variables
struct stepperPara
{
    int sm; // Serial port
    uint32_t baudrate;  // Baudrate
    uint8_t stepMode;
    uint32_t maxSpeed;      // Maximum speed, [microstep/10000sec]
    uint32_t startSpeed;    // Starting speed, [microstep/10000sec]
    uint8_t maxCurr;    // Current limit, [mA]
};

// Stepper Motor class
class StepperMotor
{
    public:
        StepperMotor(stepperPara &sp);     // Constructor for the class
        ~StepperMotor();

        // Initial Setting function
        void InitSetting();

        // Function to write bytes to the serial port, returning 0 on success and -1 on failure
        int write_port(int sm, uint8_t * buffer, size_t size);

        // Funtion to reads bytes from the serial port.
        // Returns after all the desired bytes have been read, or if there is a timeout or other error.
        // Returns the number of bytes successfully read into the buffer, or -1 if there was an error reading.
        ssize_t read_port(int sm, uint8_t * buffer, size_t size);

        // Gets one or more variables from the Tic. Returns 0 for success, -1 for failure.
        int tic_get_variable(int fd, uint8_t offset, uint8_t * buffer, uint8_t length);

        // Function to clear all driver error
        void ClearDriverError();

        // Function to reset the driver
        void Reset();

        // Function to reset command timeout
        void ResetCmdTimeout();

        // Function to de-energizethe motor
        void DeEnergize();

        // Function to energize the motor
        void Energize();

        // Function to exit safe start
        void ExitSafeStart();

        // Function to enter safe start
        void EnterSafeStart();

        // Function to set step mode
        void SetStepMode(uint8_t stepMode);

        // Function to set current limit
        void SetCurrentLimit(uint8_t maxCurr);

        // Function to set target position
        void SetTargetPosition(int32_t position);

        // Function to set velocity
        void SetTargetVelocity(int32_t velocity);

        // Function to set max speed
        void SetMaxSpeed(uint32_t maxSpeed);

        // Function to set starting speed
        void SetStartSpeed(uint32_t startSpeed);

        // Function to get current position
        int32_t GetCurrentPositionIs();

        // Functions to get error
        uint16_t GetErrorStatusIs();

        uint32_t GetErrorOccuredIs();

        // Function to run the motor
        void RunToTargetPosition(int32_t position);

    protected:
        int p_sm;                   // Serial port
        uint32_t p_baudrate = 9600; // Baudrate for the serial communication
        uint8_t p_stepMode = 0;     // Step mode
        uint32_t p_maxSpeed;        // Maximum speed, [microstep/10000sec]
        uint32_t p_startSpeed;      // Starting speed, [microstep/10000sec]
        uint8_t p_maxCurr = 200;    // Current limit, [mA]
};

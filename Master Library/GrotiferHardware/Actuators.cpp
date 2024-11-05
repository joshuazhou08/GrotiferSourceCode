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

#include "Actuators.hpp"

using namespace std;

// Maxon

char* InterfaceName2PortName(char* p_interfaceName) // Find Interface Name from Port Name
{
    if (!strcmp(p_interfaceName, "USB")) return "USB";          // Interface is USB -> port name starts with USB
    else if (!strcmp(p_interfaceName, "RS232")) return "COM";   // Interface is RS232 -> port name starts with COM
    else return "CAN";                                          // Interface is CANOpen -> port name starts with CAN
}

string* ScanAvailablePortNames(char* p_deviceName, char* p_protocolStackName, char* p_interfaceName, unsigned int* p_availPort, unsigned int* p_errorCode)  // Scan the available port name
{

    const unsigned int MAX_STR_SIZE = 100;      // String size to store available port name
    unsigned int noOfAvailablePortName = 0;     // Number of available port names or available ports

    // Convert the interface name into the port name that is corresponding to that interface: USB -> USB, RS232 -> COM, CANOpen -> CAN
    char* portName = InterfaceName2PortName(p_interfaceName);

    // Reset all the ports name selection using the function from Maxon Library - VCS_ResetPortNameSelection();
    VCS_ResetPortNameSelection(p_deviceName, p_protocolStackName, p_interfaceName, p_errorCode);

    // Scanning for the available ports
    char portNameTemp[MAX_STR_SIZE];
    int endOfAvailablePortFlag = false;
    VCS_GetPortNameSelection(p_deviceName, p_protocolStackName, p_interfaceName, true, portNameTemp, MAX_STR_SIZE, &endOfAvailablePortFlag, p_errorCode);
    noOfAvailablePortName++;
    while (!endOfAvailablePortFlag)
    {
        // Increase the number of available ports
        noOfAvailablePortName++;

        // Scan again for other available ports
        VCS_GetPortNameSelection(p_deviceName, p_protocolStackName, p_interfaceName, false, portNameTemp, MAX_STR_SIZE, &endOfAvailablePortFlag, p_errorCode);
    }

    // Return the number of available ports
    *p_availPort = noOfAvailablePortName;

    // Construct the  list of available port names
    string* portNameList = new string[noOfAvailablePortName];
    for (unsigned int i = 0; i < noOfAvailablePortName; i++)
    {
        portNameList[i] = portName + to_string(i);
    }

    return portNameList;

    // Free memory for the portNameList
    delete[] portNameList;
}


// Function to open all Maxon boards
bool openMaxonMotors (void* keyHandleArr[5], uint32_t serialNoArr[5], string* &availPortNameList)
{
    // Variables to open Maxon's boards
    char* deviceName = "EPOS4";
    char* protocolStackName = "MAXON SERIAL V2";
    char* interfaceName = "USB";
    unsigned int nodeID = 1;
    unsigned int errorCode = 0;

    // Reset all portname
    VCS_ResetPortNameSelection(deviceName, protocolStackName, interfaceName, &errorCode);

    // Get all available portname and number of available ports
    unsigned int noOfAvailPorts = 0;    // Number of available ports (USB)
    availPortNameList = ScanAvailablePortNames(deviceName, protocolStackName, interfaceName, &noOfAvailPorts, &errorCode);

    // Scan through all available ports to check for open EPOS devices
    unsigned int noOfOpenedPorts = 0;
    for(int i=0; i<noOfAvailPorts; i++)
    {
        keyHandleArr[i] = VCS_OpenDevice(deviceName, protocolStackName, interfaceName, &availPortNameList[i][0], &errorCode);
        if(keyHandleArr[i] != 0)
        {
            noOfOpenedPorts++;
            VCS_ClearFault(keyHandleArr[i], nodeID, &errorCode);
            VCS_SetDisableState(keyHandleArr[i], nodeID, &errorCode);  // Diable all state for the connecting boards
            unsigned int pNumOfBytesToRead = 4;
            unsigned int pNumOfBytesActRead = 0;
            VCS_GetObject(keyHandleArr[i], nodeID, 0x1018, 0x04, &serialNoArr[i], pNumOfBytesToRead, &pNumOfBytesActRead, &errorCode);
        }
    }

    // Condition to return
    bool returnVal = false;
    if(noOfOpenedPorts != 5)   // Not equal to the amount of Maxon motors
    {
        cout << "Failed to connect to all Maxon Motors" << endl;
        returnVal = false;
    }
    else
    {
        cout << "Successfully connect all Maxon Motors" << endl;
        cout << "Number of motors: " << noOfOpenedPorts << endl;
        returnVal = true;
    }

    // Return
    return returnVal;
}

MaxonMotor::MaxonMotor(ofstream &errFile, maxon &mx, char controlMode)  // Constructor for Maxon Motor class
{
    // Asigned all the constructor parameters into private variables
    p_handle = mx.keyHandle;
    p_portName = mx.portName;
    p_nameMotor = mx.nameMotor;
    p_K_P = mx.K_P;
    p_K_I = mx.K_I;
    p_KFF_VEL = mx.KFF_VEL;
    p_KFF_ACC = mx.KFF_ACC;
    p_KP_CURR = mx.KP_CURR;
    p_KI_CURR = mx.KI_CURR;
    p_K_TUNING = mx.K_TUNING;
    p_NUM_OF_PULSE_PER_REV = mx.NUM_OF_PULSE_PER_REV;
    p_SENSOR_POLARITY = mx.SENSOR_POLARITY;
    p_KT = mx.KT;
    p_controlMode = controlMode;
    p_errDataFile = &errFile;
    // Run all the settings
    InitSetting();
}

MaxonMotor::~MaxonMotor()
{
    if(!VCS_HaltVelocityMovement(p_handle, p_nodeID, &p_errorCode)) // Stop the motor
    {
        LogError(*p_errDataFile, p_errorCode);
    }

    if(!VCS_SetDisableState(p_handle, p_nodeID, &p_errorCode))  // Disable the motor
    {
        LogError(*p_errDataFile, p_errorCode);
    }

    if(!VCS_CloseDevice(p_handle, &p_errorCode))  // Close epos device
    {
        LogError(*p_errDataFile, p_errorCode);
    }
}

void MaxonMotor::InitSetting()
{
    // Clear all fault
    if(!VCS_ClearFault(p_handle, p_nodeID, &p_errorCode))
    {
        LogError(*p_errDataFile, p_errorCode);
    }

    // Switch to disable state
    if(!VCS_SetDisableState(p_handle, p_nodeID, &p_errorCode))
    {
        LogError(*p_errDataFile, p_errorCode);
    }

    // Set Protocol Stack Setting
    if(!VCS_SetProtocolStackSettings(p_handle, p_baudrate, p_timeout, &p_errorCode))
    {
        LogError(*p_errDataFile, p_errorCode);
    }

    // Set Encoder
    if(!VCS_SetSensorType(p_handle, p_nodeID, ST_INC_ENCODER_2CHANNEL, &p_errorCode))
    {
        LogError(*p_errDataFile, p_errorCode);
    }
    if(!VCS_SetIncEncoderParameter(p_handle, p_nodeID, p_NUM_OF_PULSE_PER_REV, p_SENSOR_POLARITY, &p_errorCode))
    {
        LogError(*p_errDataFile, p_errorCode);
    }

    // Set Controller Gains - Current Control loop
    if(!VCS_SetControllerGain(p_handle, p_nodeID, EC_PI_CURRENT_CONTROLLER, EG_PICC_P_GAIN, p_KP_CURR, &p_errorCode))
    {
        LogError(*p_errDataFile, p_errorCode);
    }
    if(!VCS_SetControllerGain(p_handle, p_nodeID, EC_PI_CURRENT_CONTROLLER, EG_PICC_I_GAIN, p_KI_CURR, &p_errorCode))
    {
        LogError(*p_errDataFile, p_errorCode);
    }

    // Set Controller Gains - Velocity loop
    if(!VCS_SetControllerGain(p_handle, p_nodeID, EC_PI_VELOCITY_CONTROLLER, EG_PIVC_P_GAIN, p_K_P*p_K_TUNING, &p_errorCode))
    {
        LogError(*p_errDataFile, p_errorCode);
    }
    if(!VCS_SetControllerGain(p_handle, p_nodeID, EC_PI_VELOCITY_CONTROLLER, EG_PIVC_I_GAIN, p_K_I*pow(p_K_TUNING, 2), &p_errorCode))
    {
        LogError(*p_errDataFile, p_errorCode);
    }
    if(!VCS_SetControllerGain(p_handle, p_nodeID, EC_PI_VELOCITY_CONTROLLER, EG_PIVC_FEED_FORWARD_VELOCITY_GAIN, p_KFF_VEL, &p_errorCode))
    {
        LogError(*p_errDataFile, p_errorCode);
    }
    if(!VCS_SetControllerGain(p_handle, p_nodeID, EC_PI_VELOCITY_CONTROLLER, EG_PIVC_FEED_FORWARD_ACCELERATION_GAIN, p_KFF_ACC, &p_errorCode))
    {
        LogError(*p_errDataFile, p_errorCode);
    }

    // Set Operation mode
    switch (p_controlMode)
    {
        case 'v':   // Velocity Control Mode
            if(!VCS_SetOperationMode(p_handle, p_nodeID, OMD_VELOCITY_MODE, &p_errorCode))
            {
                LogError(*p_errDataFile, p_errorCode);
            }

            // Activate velocity mode
            if(!VCS_ActivateVelocityMode(p_handle, p_nodeID, &p_errorCode))
            {
                LogError(*p_errDataFile, p_errorCode);
            }
            break;
        case 'c': // Current Control Mode
            if(!VCS_SetOperationMode(p_handle, p_nodeID, OMD_CURRENT_MODE, &p_errorCode))
            {
                LogError(*p_errDataFile, p_errorCode);
            }

            // Activate velocity mode
            if(!VCS_ActivateCurrentMode(p_handle, p_nodeID, &p_errorCode))
            {
                LogError(*p_errDataFile, p_errorCode);
            }
            break;
        default:
            cout << "Invalid Mode. Set to Velocity Mode!" << endl;
            if(!VCS_SetOperationMode(p_handle, p_nodeID, OMD_VELOCITY_MODE, &p_errorCode))
            {
                LogError(*p_errDataFile, p_errorCode);
            }

            // Activate velocity mode
            if(!VCS_ActivateVelocityMode(p_handle, p_nodeID, &p_errorCode))
            {
                LogError(*p_errDataFile, p_errorCode);
            }
    }

    // Enable state
    if(!VCS_SetEnableState(p_handle, p_nodeID, &p_errorCode))
    {
        LogError(*p_errDataFile, p_errorCode);
    }
}
const double PI = M_PI; // Value of pi, 3.14159265358979323846
const double CONV_INC2TURN = 2048;  // Number of increasement per turn
const double CONV_TURN2DEG = 360;   // deg/turn
const double CONV_RPM_TO_RADpSEC = PI/30;    // Convert from rpm to rad/s
const double CONV_RAD_TO_DEG = 180/PI;   // convert from rad to deg

double MaxonMotor::GetPositionIs() // Function to get the position of the motor
{
    if(!VCS_GetPositionIs(p_handle, p_nodeID, &p_motorPos, &p_errorCode))
    {
        LogError(*p_errDataFile, p_errorCode);
    }
    return (double) p_motorPos * (360.0/(4.0*p_NUM_OF_PULSE_PER_REV));
}

double MaxonMotor::GetVelocityIs()  // Function to get the (averaged) velocity of the motor [rpm]
{
    if(!VCS_GetVelocityIsAveraged(p_handle, p_nodeID, &p_motorVel, &p_errorCode))
    {
        LogError(*p_errDataFile, p_errorCode);
    }
    return (double) p_motorVel;
}

// Get the current of the motor
int MaxonMotor::GetCurrentIs()   // Function to get the (averaged) current of the motor [mA]
{
    if(!VCS_GetCurrentIsAveragedEx(p_handle, p_nodeID, &p_motorCur, &p_errorCode))
    {
        LogError(*p_errDataFile, p_errorCode);
    }
    return p_motorCur;
}

// Get the operational mode
int MaxonMotor::GetOperationalMode() // Function to get the operational mode
{
    char p_oprModeTemp; // Temporary variable
    if(!VCS_GetOperationMode(p_handle, p_nodeID, &p_oprModeTemp, &p_errorCode))
    {
        LogError(*p_errDataFile, p_errorCode);
    }
    p_oprMode = (int) p_oprModeTemp;
    return p_oprMode;
}

// Get the torque constant
double MaxonMotor::GetKT()  // Function to get torque constant K_T
{
    return p_KT;
}

// Get Max torque
double MaxonMotor::GetMaxTorque()
{
    return p_maxTorque;
}

// Run the motor with velocity value
bool MaxonMotor::RunWithVelocity(int desVel)
{
    p_desVel = desVel;
    if(!VCS_SetVelocityMust(p_handle, p_nodeID, p_desVel, &p_errorCode))    // Run the motor
    {
        LogError(*p_errDataFile, p_errorCode);
        return false;  // This indicate that there's an error in running the motor
    }
        else return true;  // Successfully run the motor
}

// Run the motor in current control mode with a desired current
bool MaxonMotor::RunWithCurrent(int desCur)
{
    p_desCur = desCur;
    if(!VCS_SetCurrentMustEx(p_handle, p_nodeID, p_desCur, &p_errorCode))    // Run the motor
    {
        LogError(*p_errDataFile, p_errorCode);
        return false;  // This indicate that there's an error in running the motor
    }
    else return true;  // Successfully run the motor
}

// Halt motion of the motor
void MaxonMotor::HaltMotion()
{
    if(!VCS_HaltVelocityMovement(p_handle, p_nodeID, &p_errorCode))
    {
        LogError(*p_errDataFile, p_errorCode);
    }
}

// Disable the motor
void MaxonMotor::SetDisableState()  // Function to disable the motor - used for experiment
{
    if(!VCS_SetDisableState(p_handle, p_nodeID, &p_errorCode))
    {
        LogError(*p_errDataFile, p_errorCode);
    }
}

int MaxonMotor::LogError(ofstream &errFileName, unsigned int errorCocde)  // Logging error data
{
    time_t errTimeStamp = time(NULL);       // Variable to record timestamp
    const unsigned int MAX_STR_SIZE = 100;  // Maximum size of string
    char errMessage[MAX_STR_SIZE];          // Variable to store the error message

    // Get error message corresponding to the error code
    VCS_GetErrorInfo(errorCocde, errMessage, MAX_STR_SIZE);

    // Print the error, and store them into a file with timestamp
    cerr << "At " << p_nameMotor << ": Error Code is 0x" << std::hex << errorCocde << " and Information is: " << errMessage << endl;
    errFileName << left << setw(15) << p_nameMotor << left << setw(15) <<"0x" << left << setw(15) << std::hex << errorCocde << " - " << errMessage << ". " << left << setw(15) << "[Time Stamp]:" <<  ctime(&errTimeStamp) <<  endl;

    return 0;
}

// Stepper Motor class
StepperMotor::StepperMotor(stepperPara &sp)     // Constructor for the class
{
    p_sm = sp.sm;
    if(sp.baudrate != 4800 && sp.baudrate != 9600 && sp.baudrate != 19200 && sp.baudrate != 38400 && sp.baudrate != 115200)
    {
        cout << "Unsupported Baudrate! Use 9600 instead." << endl;
    }
    else p_baudrate = sp.baudrate;

    if(sp.stepMode > 9)
    {
        cout << "Invalid Step Mode! Use Full-Step - Mode 0 instead." << endl;
    }
    else p_stepMode = sp.stepMode;


    if(sp.maxCurr > 4000)
    {
        cout << "Current limit is too large! Use 4A instead." << endl;
    }
    else p_maxCurr = sp.maxCurr;
    p_maxCurr = sp.maxCurr;
    p_maxSpeed = sp.maxSpeed;
    p_startSpeed = sp.startSpeed;

    // Run the InitSetting() function to set up the serial port
    InitSetting();

    // Reset the driver
    ExitSafeStart();
    Reset();

    // Reset cmd timeout
    ExitSafeStart();
    ResetCmdTimeout();

    // Clear all the error
    ExitSafeStart();
    ClearDriverError();

    // Set Current Limit
    ExitSafeStart();
    SetCurrentLimit(p_maxCurr);

    // Set the step mode
    ExitSafeStart();
    SetStepMode(p_stepMode);

    // Set the max speed
    ExitSafeStart();
    SetMaxSpeed(p_maxSpeed);

    // Set the starting speed
    ExitSafeStart();
    SetStartSpeed(p_startSpeed);

    // Energize the motor
    ExitSafeStart();
    Energize();
}

StepperMotor::~StepperMotor()
{
    // De-energize the motor
    DeEnergize();

    // Close the serial port
    close(p_sm);
}

void StepperMotor::InitSetting()
{
    // Flush away any bytes previously read or written
    tcflush(p_sm, TCIOFLUSH);

    // Get the current configuration of the serial port
    struct termios smConfigutation;
    tcgetattr(p_sm, &smConfigutation);

    // Turn off any options that might interfere with our ability to send and receive raw binary bytes.
    smConfigutation.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
    smConfigutation.c_oflag &= ~(ONLCR | OCRNL);
    smConfigutation.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    // Set up timeouts: Calls to read() will return as soon as there is at least one byte available or when 100 ms has passed.
    smConfigutation.c_cc[VTIME] = 1;
    smConfigutation.c_cc[VMIN] = 0;

    // Set the baudrate for the serial port
    switch(p_baudrate)
    {
        case 4800:   cfsetospeed(&smConfigutation, B4800);   break;
        case 9600:   cfsetospeed(&smConfigutation, B9600);   break;
        case 19200:  cfsetospeed(&smConfigutation, B19200);  break;
        case 38400:  cfsetospeed(&smConfigutation, B38400);  break;
        case 115200: cfsetospeed(&smConfigutation, B115200); break;
    }
    cfsetispeed(&smConfigutation, cfgetospeed(&smConfigutation));

    // Save all the configurations
    tcsetattr(p_sm, TCSANOW, &smConfigutation);
}

int StepperMotor::write_port(int sm, uint8_t * buffer, size_t size)
{
    ssize_t result = write(sm, buffer, size);
    if (result != (ssize_t)size)
    {
        perror("failed to write to port");
        return -1;
    }
        return 0;
}

// Funtion to reads bytes from the serial port.
// Returns after all the desired bytes have been read, or if there is a timeout or other error.
// Returns the number of bytes successfully read into the buffer, or -1 if there was an error reading.
ssize_t StepperMotor::read_port(int sm, uint8_t * buffer, size_t size)
{
    size_t received = 0;
    while (received < size)
    {
        ssize_t r = read(sm, buffer + received, size - received);
        if (r < 0)
        {
            perror("failed to read from port");
        return -1;
        }
        if (r == 0)
        {
            // Timeout
            break;
        }
        received += r;
    }
    return received;
}

// Gets one or more variables from the Tic. Returns 0 for success, -1 for failure.
int StepperMotor::tic_get_variable(int fd, uint8_t offset, uint8_t * buffer, uint8_t length)
{
    uint8_t command[] = { 0xA2, offset, length };
    int result = write_port(fd, command, sizeof(command));
    if (result) { return -1; }
    ssize_t received = read_port(fd, buffer, length);
    if (received < 0) { return -1; }
    if (received != length)
    {
        fprintf(stderr, "read timeout: expected %u bytes, got %zu\n", length, received);
        return -1;
    }
    return 0;
}

// Function to clear all driver error
void StepperMotor::ClearDriverError()
{
    uint8_t command[] = {0x8A};
    if(write_port(p_sm, command, sizeof(command)) < 0)
    {
        uint32_t error = GetErrorOccuredIs();
        cout << "Failed to clear driver, error code is: " << error << endl;
    }
}

// Function to reset the driver
void StepperMotor::Reset()
{
    uint8_t command[] = {0xB0};
    if(write_port(p_sm, command, sizeof(command)) < 0)
    {
        uint32_t error = GetErrorOccuredIs();
        cout << "Failed to reset driver, error code is: " << error << endl;
    }
}

// Function to reset command timeout
void StepperMotor::ResetCmdTimeout()
{
    uint8_t command[] = {0x8C};
    if(write_port(p_sm, command, sizeof(command)) < 0)
    {
        uint32_t error = GetErrorOccuredIs();
        cout << "Failed to reset command timeout, error code is: " << error << endl;
    }
}

// Function to de-energizethe motor
void StepperMotor::DeEnergize()
{
    uint8_t command[] = {0x86};
    if(write_port(p_sm, command, sizeof(command)) < 0)
    {
        uint32_t error = GetErrorOccuredIs();
        cout << "Failed to de-energize, error code is: " << error << endl;
    }
}

// Function to energize the motor
void StepperMotor::Energize()
{
    uint8_t command[] = {0x85};
    if(write_port(p_sm, command, sizeof(command)) < 0)
    {
        uint32_t error = GetErrorOccuredIs();
        cout << "Failed to energize, error code is: " << error << endl;
    }
}

// Function to exit safe start
void StepperMotor::ExitSafeStart()
{
    uint8_t command[] = {0x83};
    if(write_port(p_sm, command, sizeof(command)) < 0)
    {
        cout << "Failed to exit safe start" << endl;
        }
}

// Function to enter safe start
void StepperMotor::EnterSafeStart()
{
    uint8_t command[] = {0x8F};
    if(write_port(p_sm, command, sizeof(command)) < 0)
    {
        cout << "Failed to enter safe start" << endl;
    }
}

// Function to set step mode
void StepperMotor::SetStepMode(uint8_t stepMode)
{
    uint8_t command[2];
    command[0] = 0x94;
    command[1] = stepMode;
    if(write_port(p_sm, command, sizeof(command)) < 0)
    {
        uint32_t error = GetErrorOccuredIs();
        cout << "Failed to set step mode, error code is: " << error << endl;
    }
}

// Function to set current limit
void StepperMotor::SetCurrentLimit(uint8_t maxCurr)
{
    uint8_t command[2];
    command[0] = 0x91;
    command[1] = maxCurr;
    if(write_port(p_sm, command, sizeof(command)) < 0)
    {
        uint32_t error = GetErrorOccuredIs();
        cout << "Failed to set current limit, error code is: " << error << endl;
    }
}

// Function to set target position
void StepperMotor::SetTargetPosition(int32_t position)
{
    uint32_t value = position;
    uint8_t command[6];
    command[0] = 0xE0;
    command[1] = ((value >>  7) & 1) |
                 ((value >> 14) & 2) |
                 ((value >> 21) & 4) |
                 ((value >> 28) & 8);
    command[2] = value >> 0 & 0x7F;
    command[3] = value >> 8 & 0x7F;
    command[4] = value >> 16 & 0x7F;
    command[5] = value >> 24 & 0x7F;
    if(write_port(p_sm, command, sizeof(command)) < 0)
    {
        cout << "Failed to set target position" << endl;;
    }
}

// Function to set velocity
void StepperMotor::SetTargetVelocity(int32_t velocity)
{
    uint32_t value = velocity;
    uint8_t command[6];
    command[0] = 0xE3;
    command[1] = ((value >>  7) & 1) |
                 ((value >> 14) & 2) |
                 ((value >> 21) & 4) |
                 ((value >> 28) & 8);
    command[2] = value >> 0 & 0x7F;
    command[3] = value >> 8 & 0x7F;
    command[4] = value >> 16 & 0x7F;
    command[5] = value >> 24 & 0x7F;
    if(write_port(p_sm, command, sizeof(command)) < 0)
    {
        cout << "Failed to set target velocity" << endl;
    }
}

// Function to set max speed
void StepperMotor::SetMaxSpeed(uint32_t maxSpeed)
{
    uint32_t value = maxSpeed;
    uint8_t command[6];
    command[0] = 0xE6;
    command[1] = ((value >>  7) & 1) |
                 ((value >> 14) & 2) |
                 ((value >> 21) & 4) |
                 ((value >> 28) & 8);
    command[2] = value >> 0 & 0x7F;
    command[3] = value >> 8 & 0x7F;
    command[4] = value >> 16 & 0x7F;
    command[5] = value >> 24 & 0x7F;
    if(write_port(p_sm, command, sizeof(command)) < 0)
    {
        uint32_t error = GetErrorOccuredIs();
        cout << "Failed to set max speed, error code is: " << error << endl;
    }
}

// Function to set starting speed
void StepperMotor::SetStartSpeed(uint32_t startSpeed)
{
    uint32_t value = startSpeed;
    uint8_t command[6];
    command[0] = 0xE5;
    command[1] = ((value >>  7) & 1) |
                 ((value >> 14) & 2) |
                 ((value >> 21) & 4) |
                 ((value >> 28) & 8);
    command[2] = value >> 0 & 0x7F;
    command[3] = value >> 8 & 0x7F;
    command[4] = value >> 16 & 0x7F;
    command[5] = value >> 24 & 0x7F;
    if(write_port(p_sm, command, sizeof(command)) < 0)
    {
        uint32_t error = GetErrorOccuredIs();
        cout << "Failed to set start speed, error code is: " << error << endl;
    }
}

// Function to get current position
int32_t StepperMotor::GetCurrentPositionIs()
{
    int32_t output = 0;
    uint8_t buffer[4];
    if(tic_get_variable(p_sm, 0x22, buffer, sizeof(buffer)) < 0)
    {
        uint32_t error = GetErrorOccuredIs();
        cout << "Failed to get current position, error code is: " << error << endl;
        output = 0;
    }
    else output = buffer[0] + ((uint32_t)buffer[1] << 8) + ((uint32_t)buffer[2] << 16) + ((uint32_t)buffer[3] << 24);
    return output;
}

// Functions to get error
uint16_t StepperMotor::GetErrorStatusIs()
{
    uint16_t output = 0;
    uint8_t buffer[2];
    if(tic_get_variable(p_sm, 0x02, buffer, sizeof(buffer)) < 0)
    {
        cout << "Failed to get error status" << endl;
        output = 0;
    }
    else output = buffer[0] + ((uint16_t)buffer[1] << 8);
    return output;
}

uint32_t StepperMotor::GetErrorOccuredIs()
{
    uint32_t output = 0;
    uint8_t buffer[4];
    if(tic_get_variable(p_sm, 0x04, buffer, sizeof(buffer)) < 0)
    {
        cout << "Failed to get error occured" << endl;
        output = 0;
    }
    else output = buffer[0] + ((uint32_t)buffer[1] << 8) + ((uint32_t)buffer[2] << 16) + ((uint32_t)buffer[3] << 24);
    return output;
}

// Function to run the motor
void StepperMotor::RunToTargetPosition(int32_t position)
{
    ExitSafeStart();
    SetTargetPosition(position);
}

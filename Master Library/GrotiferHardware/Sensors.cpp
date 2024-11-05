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
#include <modbus/modbus-rtu.h>  // Modbus library
#include "u6.h" // U6 Labjack library

#include "Sensors.hpp"

// Libraries for altitude determination

using namespace std;

// Definition of class for Labjack U6
bool OpenLabJackU6(HANDLE &hDevice) // Function to open Labjack U6
{
    hDevice = openUSBConnection(-1);
    if( hDevice  == NULL )
    {
        cout <<"Failed to open LabJack U6" << endl;
        return false;
    }

    // Initiate LabJack U6 object
    return true;
}

LabJackU6::LabJackU6(HANDLE hDevice, long TCPinOffset)  // Constructor
{
    p_hDevice = hDevice;
    p_lngTCPinOffset = TCPinOffset;

    // Get Calibration info
    GetCalibrationInfo();

    // Configure up the Timers and Counters
    ConfigTimersCounters();
}

LabJackU6::~LabJackU6() {} // Destructor

// Get calibration Info function
void LabJackU6::GetCalibrationInfo()
{
    long error = getCalibrationInfo(p_hDevice, &p_caliInfo);
    if (error < 0) LabJackLogError(error);
}

// Config timers and counters
void LabJackU6::ConfigTimersCounters()
{
    long error = 0; // Error code
    //Enable all timers and counters; Set Timers operation mode; and Reset and disable reset
    for (int i = 0; i < p_numOfTimers; i++)
    {
        p_alngEnableTimers[i] = 1;          // Enable Timer-i
        p_alngTimerModes[i] = LJ_tmQUAD;    // Timer-i is at quadrature mode
        p_adblTimerValues[i] = 0;           // Initiate reading
        p_alngReadTimers[i] = 1;            // Enable read Timers
        p_alngUpdateResetTimers[i] = 1;     // Reset Timer-i
    }

    for (int k = 0; k < p_numOfCounters; k++)
    {
        p_alngEnableCounters[k] = 1;        // Enbale Counter-k
        p_alngReadCounters[k] = 1;          // Enable read Counter-k
        p_alngResetCounters[k] = 1;         // Reset Counter-k
    }


    // Config the LabJack Timers and Counters
    error = eTCConfig(p_hDevice, p_alngEnableTimers, p_alngEnableCounters, p_lngTCPinOffset, LJ_tc48MHZ, 0, p_alngTimerModes, p_adblTimerValues, 0, 0);
    if (error < 0)
    {
     cout << "Failed to config the LabJack" << endl;
     LabJackLogError(error);
    }

    // Reset the LabJack
    error = eTCValues(p_hDevice, p_alngReadTimers, p_alngUpdateResetTimers, p_alngReadCounters, p_alngResetCounters, p_adblTimerValues, p_adblCounterValues, 0, 0);
    if (error < 0)
    {
        cout << "Failed to reset the LabJack" << endl;
        LabJackLogError(error);
    }

    // Disable Reset
    for (int i = 0; i < p_numOfTimers; i++)
    {
        p_alngUpdateResetTimers[i] = 0;     // Disable Reset Timer-i
    }

    for (int k = 0; k < p_numOfCounters; k++)
    {
        p_alngResetCounters[k] = 0;         // Disable Reset Counter-k
    }
}

// Get counts value at Timer
long LabJackU6::GetCountsValueAtTimer(unsigned int timerChannel)
{
    // Check the validity of the requested timer
    unsigned int p_timerChannel = 0;
    if (timerChannel > p_numOfTimers)
    {
        cout << "Invalid Timer. Use Timer 0" << endl;
    }
    else
    {
        p_timerChannel = timerChannel;
    }

    // Read all timers and counters
    long error = 0;
    error = eTCValues(p_hDevice, p_alngReadTimers, p_alngUpdateResetTimers, p_alngReadCounters, p_alngResetCounters, p_adblTimerValues, p_adblCounterValues, 0, 0);
    if (error < 0)
    {
        cout << "Failed to read Timer LabJack" << endl;
        LabJackLogError(error);
    }

    // Extract the value
    long countVal = 0;
    if (p_adblTimerValues[p_timerChannel] >= 2147483648.0)
    {
        countVal = (int32_t)-1 * (4294967296 - p_adblTimerValues[p_timerChannel]);
    }
    else
    {
        countVal = (int32_t) p_adblTimerValues[p_timerChannel];
     }

    return countVal;
}

// Get value of Counter
double LabJackU6::GetCounterValueAtCounter(unsigned int counterChannel)
{
    // Check the validity of the requested counter
    unsigned int p_counterChannel = 0;
    if (counterChannel > p_numOfCounters)
    {
        cout << "Invalid Counter. Use Counter 0" << endl;
    }
    else
    {
        p_counterChannel = counterChannel;
    }

    // Read all timers and counters
    long error = 0;
    error = eTCValues(p_hDevice, p_alngReadTimers, p_alngUpdateResetTimers, p_alngReadCounters, p_alngResetCounters, p_adblTimerValues, p_adblCounterValues, 0, 0);
    if (error < 0)
    {
        cout << "Failed to read Counter LabJack" << endl;
        LabJackLogError(error);
    }

    return p_adblCounterValues[p_counterChannel];
}

// Get the voltage at channel
double LabJackU6::GetRawVoltageAtChannel(long channel)
{
    double voltage = 0.0; // Voltage read at the channel
    long error = eAIN(p_hDevice, &p_caliInfo, channel, 15, &voltage, LJ_rgBIP10V, 0, 0, 0, 0, 0);
    if (error != 0) LabJackLogError(error);
    return voltage;
}

double LabJackU6::GetAveVoltageAtChannel(long channel)
{
    double aveVoltage = 0.0;
    double tempVoltage = 0.0, sumVoltage = 0.0; // Voltage value to calculate the average
    double noOfRead = 5;    // Number of read to get the average
    long error = 0;
    for(int i=0; i<noOfRead; i++)
    {
        error = eAIN(p_hDevice, &p_caliInfo, channel, 15, &tempVoltage, LJ_rgBIP10V, 0, 0, 0, 0, 0);
        if (error != 0) LabJackLogError(error);
        sumVoltage += tempVoltage;
    }
    aveVoltage = sumVoltage/noOfRead;
    return aveVoltage;
}

// Log error function
void LabJackU6::LabJackLogError(long error)
{
    cout << "Received an error code of " << error << endl;
}

// Encoder using LabJack to read
LJEncoder3Channels::LJEncoder3Channels(LabJackU6 &ljU6, uint16_t cpr, unsigned int tcOrder)    // Constructor
{
    p_ljU6 = &ljU6;
    p_cpr = 4*cpr;    // Quadrature mode
    p_timerChannel = tcOrder;
    p_counterChannel = tcOrder - 1;
}
LJEncoder3Channels::~LJEncoder3Channels() {} // Destructor

// Get angular position function
double LJEncoder3Channels::GetAngularPosDeg()
{
    double countVal = (double) (*p_ljU6).GetCountsValueAtTimer(p_timerChannel);
    p_pos = (countVal / p_cpr) * 360.0;
    return p_pos;
}

// Get counter/index
uint16_t LJEncoder3Channels::GetIndexCounterSignal()
{
    uint16_t counterVal = (uint16_t) (*p_ljU6).GetCounterValueAtCounter(p_counterChannel);
    p_counterVal = counterVal;
    return p_counterVal;
}

// Function to get flag for reaching the index
bool LJEncoder3Channels::GetIndexFlag()
{
    bool result = (GetIndexCounterSignal() >= 1) ? true : false;
    return result;
}

// Inclinometer
LabJackInclinometer::LabJackInclinometer(LabJackU6 &lju6, long xChannel, long yChannel)  // Contructor, xchannel is 0, ychannel is 1
{
    p_lju6 = &lju6;
    p_xChannel = xChannel;  // Channel on the Labjack that's corresponding to X-incl
    p_yChannel = yChannel;  // Channel on the Labjack that's corresponding to Y-incl
}

LabJackInclinometer::~LabJackInclinometer(){}   // Destructor


// Function to get the X-angle, assuming linear (-25 deg -> 25 deg)
// Tatsu's calibration results
double LabJackInclinometer::GetAngleX()
{
    double voltageX = (*p_lju6).GetAveVoltageAtChannel(p_xChannel);
    double angleX = 12.5317 * voltageX - 30.4959;
    return angleX;
}

// Function to get the Y-angle, assuming linear (-25 deg -> 25 deg)
// Tatsu's calibration results
double LabJackInclinometer::GetAngleY()
{
    double voltageY = (*p_lju6).GetAveVoltageAtChannel(p_yChannel);
    double angleY = 12.4982 * voltageY - 29.4607;
    return angleY;
}

// Definition of the Digital Sun Sensor class using ModbusRTU Communication
bool OpenSunSensor(const char *PATH_NAME, const int BIT_RATE, modbus_t* &ctx)    // Function to open Sun Sensor
{
    const int SUNSENSOR_ID = 1; // Modbus default ID of the sun sensor
    char PARITY = 'N';          // Default parity
    const int DATA_BITS = 8;    // Default number of bits of data
    const int STOP_BIT = 1;     // Default stop bit

    ctx = modbus_new_rtu(PATH_NAME, BIT_RATE, PARITY, DATA_BITS, STOP_BIT); // Create modbus_t structure when opening the device
    if (ctx == NULL)
    {
        fprintf(stderr, "Unable to create the libmodbus context\n");
        return false;
    }

    // Connect to the Sun sensor via modbus
    if(modbus_connect(ctx) == -1)
    {
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return false;
    }

    // Set the Sun Sensor ID
    modbus_set_slave(ctx, SUNSENSOR_ID);

    return true;
}

ModbusSunSensor::ModbusSunSensor(modbus_t *ctx)  // Basic constructor for the Sun Sensor
{
    p_ctx = ctx;

    // Set RS485 mode
    modbus_rtu_set_serial_mode(p_ctx, MODBUS_RTU_RS485);

    // Get the field of view of the Sun Sensor immediately
    p_fov = GetFov();

}
ModbusSunSensor::~ModbusSunSensor() { modbus_flush(p_ctx);};  // Destructor

// Function to get the field of view of the Sun sensor
int ModbusSunSensor::GetFov()
{
    uint16_t readResult = 0;
    modbus_read_registers(p_ctx, 2, 1, &readResult);
    int fov = readResult;
    modbus_flush(p_ctx);
    return fov;
}

char* ModbusSunSensor::GetAddMessage(int addInfoCode)
{
    char* msg;
    switch (addInfoCode)
    {
        case 0xFF:
            msg = "Radiation is less than 300 W/m^2";
            break;

        case 0x33:
            msg = "Sun is out of FOV";
            break;

        case 0x01:
            msg = "Sun is to X positive reference";
            break;

        case 0x02:
            msg = "Sun is to X negative reference";
            break;

        case 0x10:
            msg = "Sun is to Y positive reference";
            break;

        case 0x20:
            msg = "Sun is to Y negative reference";
            break;

        case 0x11:
            msg = "Sun is to X positive and Y positive reference";
            break;

        case 0x12:
            msg = "Sun is to X negative and Y positive reference";
            break;

        case 0x21:
            msg = "Sun is to X positive and Y negative reference";
            break;

        case 0x22:
            msg = "Sun is to X negative and Y negative reference";
            break;

        case 0x00:
            msg = "Normal / No Information";
            break;
    }
    return msg;
}

// Function to convert 2's compliment number into signed decimal
int16_t ModbusSunSensor::Convert2CompToSignedDecimal(uint16_t numb)
{
    return ((numb & 0x8000) == 0) ? numb : -(~numb + 1);
}

// Function to get radiation data from the sun sensor
int ModbusSunSensor::GetRadiation()
{
    uint16_t readResult = 0;
    modbus_read_registers(p_ctx, 9, 1, &readResult);
    int radiation = readResult;
    modbus_flush(p_ctx);
    return radiation;
}

// Function to get temperature in degC
double ModbusSunSensor::GetTemperatureC()
{
    uint16_t readResult = 0;
    modbus_read_registers(p_ctx, 10, 1, &readResult);
    int resultTemp  = Convert2CompToSignedDecimal(readResult);
    double temperatureDegC = (double) resultTemp * 0.1;
    modbus_flush(p_ctx);
    return temperatureDegC;
}

// Function to get the angles (with Butterworth filter) and additional info
bool ModbusSunSensor::GetAngles(double &angleX, double &angleY, int &addInfoCode)
{

    uint16_t dest[5];
    if (!modbus_read_registers(p_ctx, 8, 5, dest) == 5)
    {
        cout << "did not read all info";
        angleX = 0;
        angleY = 0;
        addInfoCode = 0;
        return false;
    }

    //index 0 is additional info code, index 3 is angleX, and index 4 is angleY
    int16_t angleX_reg = Convert2CompToSignedDecimal(dest[3]);
    int16_t angleY_reg = Convert2CompToSignedDecimal(dest[4]);

    // Calculate the angles
    double scaleFactor;
    if (p_fov == 60)
    {
        scaleFactor = 0.01;
    }
    else if (p_fov == 5 || p_fov == 15 || p_fov == 25)
    {
        scaleFactor = 0.001;
    }

    angleX = (double) angleX_reg * scaleFactor;
    angleY = (double) angleY_reg * scaleFactor;

    // Convert the additional info
    addInfoCode = (int) dest[0];

    return true;
}

/* =================================== Archived functions ========================================= */
// Functions to get additional information and its message
int ModbusSunSensor::GetAddInfo()
{
    uint16_t readResult = 0;
    modbus_read_registers(p_ctx, 8, 1, &readResult);
    int addInfoCode = readResult;
    modbus_flush(p_ctx);
    return addInfoCode;
}

// Function to get the angle (with Butterworth filter).
// Depending on the input "axis", if "axis" is specified as 'X' -> Get the angle X
//                                if "axis" is specified as 'Y' -> Get the angle Y
double ModbusSunSensor::GetAngleWithFilter(char axis)
{

    int addr;       // Address for the register of the angle data
    if (axis == 'X')
    {
        addr = 11;
    }
    else if (axis == 'Y')
    {
        addr = 12;
    }

    uint16_t readResult = 0;
    modbus_read_registers(p_ctx, addr, 1, &readResult);
    double scaleFactor;
    if (p_fov == 60)
    {
        scaleFactor = 0.01;
    }
    else if (p_fov == 5 || p_fov == 15 || p_fov == 25)
    {
        scaleFactor = 0.001;
    }

    int16_t resultTemp = Convert2CompToSignedDecimal(readResult);
    double AngleWithFilter = (double) resultTemp * scaleFactor;
    modbus_flush(p_ctx);
    return AngleWithFilter;
}

double ModbusSunSensor::GetAngleX()
{
    return GetAngleWithFilter('X');
}

double ModbusSunSensor::GetAngleY()
{
    return GetAngleWithFilter('Y');
}

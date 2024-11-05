#include <iostream>
#include <modbus/modbus-rtu.h>  // Modbus library
#include "u6.h" // U6 Labjack library

using namespace std;

// Definition of class for Labjack U6
bool OpenLabJackU6(HANDLE &hDevice);
class LabJackU6
{
    public:
        LabJackU6(HANDLE hDevice, long TCPinOffset); // Constructor
        ~LabJackU6(); // Destructor
        // Setup Timers and Counter function
        void ConfigTimersCounters();
        // Get calibration Info function
        void GetCalibrationInfo();
        // Get counts value at Timer
        long GetCountsValueAtTimer(unsigned int timerChannel);
        // Get value of Counter
        double GetCounterValueAtCounter(unsigned int counterChannel);
        // Get the voltage at channel
        double GetRawVoltageAtChannel(long channel);

        double GetAveVoltageAtChannel(long channel);

        // Log error function
        void LabJackLogError(long error);
    protected:
        HANDLE p_hDevice;
        u6CalibrationInfo p_caliInfo;
        static const unsigned int p_numOfTimers = 4, p_numOfCounters = 2;
        long p_lngTCPinOffset;
        long p_lngTimerClockBaseIndex;
        long p_lngTimerClockDivisor;
        long p_alngEnableTimers[p_numOfTimers];
        long p_alngTimerModes[p_numOfTimers];
        double p_adblTimerValues[p_numOfTimers];
        long p_alngEnableCounters[p_numOfCounters];
        long p_alngReadTimers[p_numOfTimers];
        long p_alngUpdateResetTimers[p_numOfTimers];
        long p_alngReadCounters[p_numOfCounters];
        long p_alngResetCounters[p_numOfCounters];
        double p_adblCounterValues[p_numOfCounters];
};

// Encoder 3 channels using LabJack to read
class LJEncoder3Channels
{
    public:
        LJEncoder3Channels(LabJackU6 &ljU6, uint16_t cpr, unsigned int tcOrder);    // Constructor
        ~LJEncoder3Channels(); // Destructor

        // Get angular position function
        double GetAngularPosDeg();

        // Get counter/index
        uint16_t GetIndexCounterSignal();

        // Function to get flag for reaching the index
        bool GetIndexFlag();

    protected:
        LabJackU6 *p_ljU6 = NULL;
        uint16_t p_cpr = 1;                 // Count per revolution
        double p_pos = 0;                   // Angular position, [deg]
        uint16_t p_counterVal = 0;          // Value for counter
        bool p_indexFlag = false;           // Flag for reaching the index
        unsigned int p_timerChannel = 0;    // Timer channel
        unsigned int p_counterChannel = 0;  // Counter channel
};

// Inclinometer
class LabJackInclinometer
{
    public:
        LabJackInclinometer(LabJackU6 &lju6, long xChannel, long yChannel);
        ~LabJackInclinometer();

        // Function to get the X-angle, assuming linear (-25 deg -> 25 deg)
        // Tatsu's calibration results
        double GetAngleX();

        // Function to get the Y-angle, assuming linear (-25 deg -> 25 deg)
        // Tatsu's calibration results
        double GetAngleY();

    protected:
        LabJackU6 *p_lju6 = NULL;
        long p_xChannel, p_yChannel;    // Default channel for the X-angle and Y-angle
        double p_xAngle, p_yAngle;  // X-Angle and Y-Angle respectively, [deg]
};


// Definition of the Digital Sun Sensor class using ModbusRTU Communication
bool OpenSunSensor(const char *PATH_NAME, const int BIT_RATE, modbus_t* &ctx);
class ModbusSunSensor
{
    public:
        ModbusSunSensor(modbus_t *ctx);
        ~ModbusSunSensor();
        // Function to get the field of view of the Sun sensor
        int GetFov();

        // Functions to get additional information and its message
        int GetAddInfo();

        char* GetAddMessage(int addInfoCode);

        // Function to convert 2's compliment number into signed decimal
        int16_t Convert2CompToSignedDecimal(uint16_t numb);

        // Function to get radiation data from the sun sensor
        int GetRadiation();

        // Function to get temperature in degC
        double GetTemperatureC();

        // Function to get both angles and additional information
        bool GetAngles(double &angleX, double &angleY, int &addInfoCode);

        // Function to get the angle (with Butterworth filter).
        // Depending on the input "axis", if "axis" is specified as 'X' -> Get the angle X
        //                                if "axis" is specified as 'Y' -> Get the angle Y
        double GetAngleWithFilter(char axis);

        double GetAngleX();
        double GetAngleY();

    protected:
        modbus_t *p_ctx;        // Private struct variable
        uint16_t p_fov;         // Field of view of the Sun Sensor
        int p_addInfoCode = 0;  // Additional info code about the data from the sensor,
                                // Initialized as 0 -> assume it's normal
};



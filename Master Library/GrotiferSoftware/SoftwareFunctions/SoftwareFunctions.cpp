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

#include "SoftwareFunctions.hpp"

using namespace std;

// Converting functions
double Deg2Rad(double angleVal) // Function to convert from deg to radian
{
    return angleVal * M_PI / 180.0;
}

double Rad2Deg(double angleVal) // Function to convert from rad to deg
{
    return angleVal * 180.0 / M_PI;
}

// Timer functions
void StartTimer() // Function to initialize timer -> receive initial time stamp variable to initiate the time for the program
{
    struct timespec tvals;
    clock_gettime(CLOCK_MONOTONIC, &tvals);
    g_t0 = tvals.tv_sec + 1.0e-9 * tvals.tv_nsec;
}

double GetTimeNow()   // Function to get elapsed time from the initial time t0 [s]
{
    struct timespec tvals;
    clock_gettime(CLOCK_MONOTONIC, &tvals);
    return (tvals.tv_sec + 1.0e-9 * tvals.tv_nsec - g_t0);    // Return current time interval
}

// PI Position control
PIControl::PIControl(PIControlPara &pic)
{
    p_kp = pic.kp;
    p_ki = pic.ki;
    p_hLim = pic.hLim;
    p_lLim = pic.lLim;
}

PIControl::~PIControl() {};

// Get data function
double PIControl::GetKp() {return p_kp;}   // Get k_p
double PIControl::GetKi() {return p_ki;}   // Get k_i
double PIControl::GetError() {return p_error;}   // Get the error from the PI controller

// Main Controller function
double PIControl::PICalculation(double setpoint, double actVal)
{
    p_setpoint = setpoint;
    p_actVal = actVal;

    p_time = GetTimeNow();          // Get the time when starting to calculate the control signal
    p_deltaT = p_time - p_preTime;  // Calculate the actual time it take between two calls
    p_preTime = p_time;             // Assign current time into previous time

    p_error = p_setpoint - p_actVal;    // Calculate error

    // Calculate each part of the controller
    p_P_part = p_kp*p_error;

    // Calculate P-part of the controller
    p_P_part = p_kp*p_error;
    // Calculate the I-part with anti-windup
    if((p_P_part+p_I_part > p_hLim) && (p_error > 0))   p_I_part = p_I_part;
    else if((p_P_part+p_I_part < p_hLim) && (p_error < 0))   p_I_part = p_I_part;
    else p_I_part += p_ki*p_error*p_deltaT;

    // Calculate the control signal
    p_uPI = (p_P_part + p_I_part);

    // Return the control signal
    return p_uPI;
}

// BaseTask Class for control
char* BaseTask::GetTaskName() {return p_taskName;}    // Function to get task name
unsigned int BaseTask::GetTaskID() {return p_taskID;} // Function to get taskID
long unsigned int BaseTask::GetNumScan() {return p_numScans;}  // Get the number of scan
bool BaseTask::GetSuccessRunFlag() {return p_runSuccess;} // Get the run success flag
double BaseTask::GetStartTimeStamp() {return p_timeStart;}    // Get the time stamp when the task is run
double BaseTask::GetEndTimeStamp() {return p_timeEnd;}    // Get the time stamp when the task is done running
void BaseTask::SetAllowedToRunFlag(bool flagVal)
{
    p_allowedToRun = flagVal;
}

void BaseTask::AuditTrailRecord() // Function to record the trail of the program
{
    unsigned int w = 25;
    p_duration = p_timeEnd - p_timeStart;
    *p_auditTrailDataFile << left << setw(w) << p_timeStart << left << setw(w) << p_timeEnd << left << setw(w) << p_duration*1e3 << left << setw(w) << p_taskName
    << left << setw(w) << p_taskID << left << setw(w) << p_stateName << left << setw(w) << p_nextStateName << endl;
}

void runTaskList(double T_PROGRAM, BaseTask** TaskList, int numOfTasks)
{
    while (GetTimeNow() <= T_PROGRAM)
    {
        // Scan through the list of tasks -> run -> log audit trail data for each task
        for (int i = 0; i < numOfTasks; i++)
        {
            TaskList[i]->Run();
        }
    }
}


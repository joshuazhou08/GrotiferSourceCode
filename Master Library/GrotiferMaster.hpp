// Include all the header files
#pragma once
#include <math.h>
#include "GrotiferSoftware/AttitudeDetermination/AttitudeDetermination.hpp"
#include "GrotiferSoftware/SoftwareFunctions/SoftwareFunctions.hpp"
#include "GrotiferSoftware/SoftwareFunctions/MovingAverage.h"
#include "GrotiferHardware/Sensors.hpp"
#include "GrotiferHardware/Actuators.hpp"
#include "GrotiferHardware/Definitions.h"

// Definitions of constants
const double PI = M_PI;
const double CONV_TURN2DEG = 360;               // convert from turn to deg
const double CONV_RPM_TO_RADpSEC = PI/30;       // Convert from rpm to rad/s
const double CONV_RAD_TO_DEG = 180/PI;          // convert from rad to deg


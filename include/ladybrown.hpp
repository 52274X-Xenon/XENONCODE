#pragma once

#include "main.h"

using namespace pros;

// Declare global variables for PID
extern bool lbPID;
extern double ladyBrownCorrectPosition;
extern double ladyBrownCurrentPosition;

// Move these to a separate declaration for clarity
extern double ladyBrownTotalError;
extern double ladyBrownPrevError;

// Modify the function signature to avoid unnecessary default parameters
extern double ladyBrownPID(double error, double kP, double kI, double kD, double integralThreshold=30, double maxI=500);

extern void ladyBrownTask();
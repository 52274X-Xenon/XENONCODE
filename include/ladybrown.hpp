#pragma once

#include "main.h"

using namespace pros;

// Declare global variables for PID
extern bool lbPID;
extern int ladyBrownState;

extern double ladyBrownCorrectPosition;
extern double ladyBrownCurrentPosition;

extern double ladyBrownMogo;
extern double ladyBrownAlliance;
extern double ladyBrownLoadFirst;
extern double ladyBrownLoadSecond; 
extern double ladyBrownPassive;

/*below loading pos
//extern double ladyBrownLoad1;
//above loading pos
//extern double ladyBrownLoad2;
*/


// Move these to a separate declaration for clarity
extern double ladyBrownTotalError;
extern double ladyBrownPrevError;

// Modify the function signature to avoid unnecessary default parameters
extern double ladyBrownPID(double error, double kP, double kI, double kD, double integralThreshold=30, double maxI=500);

extern void ladyBrownTask();




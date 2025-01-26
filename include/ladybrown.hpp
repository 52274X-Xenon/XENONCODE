#pragma once

#include "main.h"

using namespace pros;

bool lbPID;
double ladyBrownCorrectPosition;
double ladyBrownCurrentPosition;

double ladyBrownPID(double error, double kP=5, double kI=0, double kD=0, double totalError=0, double prevError=0, double integralThreshold=30, double maxI=500);

void ladyBrownTask();
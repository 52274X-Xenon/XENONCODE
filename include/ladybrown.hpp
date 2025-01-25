#pragma once

#include "api.h"

extern pros::Motor ladybrown;
extern pros::Rotation ldbrotation;



  double ldbcurrentpos;
  double correctloadpos;
  double correctalliancepos;
  double correctmogopos;
  double ldberrorload;
  double ldberrormogo;
  double ldberrorall;
  bool ldbPID;
  double ladyBrownPID(double error = 0, double kP=-0, double kI=0, double kD=0, double totalError = 0, double prevError = 0, double integralThreshold=30, double maxI=500);
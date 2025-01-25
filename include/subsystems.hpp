#pragma once

#include "api.h"

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor ladybrown(7);
inline pros::MotorGroup intake({5,-6});

inline pros::Optical csortoptical(11);
inline pros::Rotation ldbrotation(12);
#pragma once

#include "api.h"

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

inline pros::Motor intake(5);
inline pros::Motor ladybrown(6);


inline ez::Piston clamp(8);
inline ez::Piston doinker(6);
inline ez::Piston intlift(7);
inline ez::Piston csortpist(3);
inline pros::Optical csortoptical(11);
inline pros::Rotation ldbrotation(7);
#pragma once

#include "api.h"

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor ladybrown(7);
inline pros::Motor flex(-6);
inline pros::Motor hooks(5);
inline pros::MotorGroup intake({5,-6});


inline pros::Optical csortoptical(11);
inline pros::Rotation ldbrotation(12);

inline ez::Piston clamps(8);
inline ez::Piston csortpist(3);
inline ez::Piston doinker(2);
inline ez::Piston intlift(4);
inline ez::Piston rushmech(1);
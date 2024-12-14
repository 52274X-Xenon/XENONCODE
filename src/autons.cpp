#include "main.h"
#include <cmath>
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Out of 127
const int DRIVE_SPEED = 127;
const int TURN_SPEED = 110;
const int SWING_SPEED = 90;

///
// Constants
///
void default_constants() {
  chassis.pid_heading_constants_set(11, 0, 20);
  chassis.pid_drive_constants_set(8, 0, 50);
  chassis.pid_turn_constants_set(5.3, 0, 45);
  chassis.pid_swing_constants_set(10, 0, 65);

  chassis.pid_turn_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(80_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);

  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(2_in);

  chassis.slew_drive_constants_set(7_in, 80);
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.drive_sensor_reset();
  chassis.pid_drive_set(24_in, DRIVE_SPEED, false);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

// . . .
// Make your own autonomous functions here!
// . . .



//pos side red nonrush solo awp (2 on mogo 1 on alliance)
void solo_awp_red() {
  //default configuration + definitions
  ez::Piston clamp(8);
  ez::Piston rightdoinker(6);
  ez::Piston intlift(7);
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  ladybrown.set_brake_mode(MOTOR_BRAKE_HOLD);

  // SCORE RED RING 1 ON RED ALLIANCE STAKE
  chassis.pid_drive_set(-15.5, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(1.75, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-60, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-3, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-65, 80);
  chassis.pid_wait_quick_chain();
  pros::delay(200);
  intake.move(-127);
  pros::delay(500);
  intake.move(0);

  // GRAB SAFE MOBILE GOAL
  chassis.pid_turn_set(-50, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(20, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(103, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-8, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(300);
  clamp.set(true);
  pros::delay(100);

  // SCORE RED RING 2 ON MOBILE GOAL
  chassis.pid_turn_set(138, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  intlift.set(true);
  intake.move(-127);
  chassis.pid_drive_set(16.75, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(300);
  intlift.set(false); 
  pros::delay(500);
  chassis.pid_drive_set(-10, 100);
  chassis.pid_wait_quick_chain();

  // SCORE RED RING 3 ON MOBILE GOAL
  chassis.pid_turn_set(-22, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(26, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(50);
  chassis.pid_drive_set(5, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  // TOUCH BAR
  chassis.pid_drive_set(-12, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(180, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(25, DRIVE_SPEED);
  
}

//pos side red nonrush solo awp (2 on mogo 1 on alliance)
void solo_awp_blue() {
  //default configuration + definitions
  ez::Piston clamp(8);
  ez::Piston rightdoinker(6);
  ez::Piston intlift(7);
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  ladybrown.set_brake_mode(MOTOR_BRAKE_HOLD);

  // SCORE RED RING 1 ON RED ALLIANCE STAKE
  chassis.pid_drive_set(-15.5, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(1.75, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(56, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-1.5, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(65, 80);
  chassis.pid_wait_quick_chain();
  pros::delay(200);
  intake.move(-127);
  pros::delay(500);
  intake.move(0);

  // GRAB SAFE MOBILE GOAL
  chassis.pid_turn_set(50, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(20, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  
  chassis.pid_turn_set(-103, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-8, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(300);
  clamp.set(true);
  pros::delay(100);
  
  // SCORE RED RING 2 ON MOBILE GOAL
  chassis.pid_turn_set(-138, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  intlift.set(true);
  intake.move(-127);
  chassis.pid_drive_set(16, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(300);
  intlift.set(false); 
  pros::delay(700);
  chassis.pid_drive_set(-10, 80);
  chassis.pid_wait_quick_chain();

  // SCORE RED RING 3 ON MOBILE GOAL
  chassis.pid_turn_set(22, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(25, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(50);
  chassis.pid_drive_set(5, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(350);
  
  // TOUCH BAR
  chassis.pid_drive_set(-12, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-190, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(19, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(6, 90);
  
}

///
// RUSH AUTONOMOUS RED
///
void rush_auto_red() {
  //Initializing pneumatics, sensors, and etc.
  ez::Piston clamp(8);
  ez::Piston rightdoinker(6);
  ez::Piston intlift(7);
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  ladybrown.set_brake_mode(MOTOR_BRAKE_HOLD);

  //Side: Red Positive-Corner Quadrant
  //Objectives: Steals middle mobile goal, and scores 1 rings on each goal
  //Starting position: On starting line between mobile goal and ring stake
  //Ending position: Near autonomous line, near ladder, facing blue positive corner
  //Consistency: 6-9 out of 10
  //Relative Programming/Logic Complexity: Easy
  //Total points: 6 points (not including autonomous bonus or AWP)


  // RUSH TOWARDS MIDDLE MOBILE GOAL
  chassis.pid_drive_set(32, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(100);
  rightdoinker.set(true);
  chassis.pid_turn_set(5, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(7.5, DRIVE_SPEED);
  pros::delay(600);
  rightdoinker.set(false);

  // DRIVE BACK AND CLAMP 
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-36.5, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(30, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(400);
  chassis.pid_drive_set(-6, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(205, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-22, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(300);
  clamp.set(true);
  pros::delay(50);
  chassis.pid_drive_set(-2, DRIVE_SPEED);

  // SCORE PRELOAD ON MOGO 1
  chassis.pid_drive_set(8, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  intake.move(-127);
  pros::delay(400);
  intake.move(0);
  clamp.set(false);

  // DRIVE TO AND CLAMP MOGO 2
  chassis.pid_drive_set(4, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(160, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-27, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(400);
  clamp.set(true);

  // INTAKE RED RING ONTO MOGO 2
  chassis.pid_turn_set(230, TURN_SPEED);
  intlift.set(true);
  chassis.pid_wait_quick_chain();
  intake.move(-127);
  chassis.pid_drive_set(22, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  intlift.set(false);
  pros::delay(300);
  chassis.pid_drive_set(-8, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  // GO TO AUTONOMOUS LINE
  chassis.pid_turn_set(30, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(25, DRIVE_SPEED);
  intake.move(0);

}

///
// RUSH AUTONOMOUS BLUE
///
void rush_auto_blue() {
  //Initializing pneumatics, sensors, and etc.
  ez::Piston clamp(8);
  ez::Piston leftdoinker(5);
  ez::Piston intlift(7);
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  ladybrown.set_brake_mode(MOTOR_BRAKE_HOLD);

  //Side: Red Positive-Corner Quadrant
  //Objectives: Steals middle mobile goal, and scores 1 rings on each goal
  //Starting position: On starting line between mobile goal and ring stake
  //Ending position: Near autonomous line, near ladder, facing blue positive corner
  //Consistency: 6-9 out of 10
  //Relative Programming/Logic Complexity: Easy
  //Total points: 6 points (not including autonomous bonus or AWP)


  // RUSH TOWARDS MIDDLE MOBILE GOAL
  chassis.pid_drive_set(32, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(100);
  leftdoinker.set(true);
  chassis.pid_turn_set(-5, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(7.5, DRIVE_SPEED);
  pros::delay(600);
  leftdoinker.set(false);

  // DRIVE BACK AND CLAMP 
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-36.5, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-30, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(400);
  chassis.pid_drive_set(-6, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-205, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-22, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(300);
  clamp.set(true);
  pros::delay(50);
  chassis.pid_drive_set(-2, DRIVE_SPEED);

  // SCORE PRELOAD ON MOGO 1
  chassis.pid_drive_set(8, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  intake.move(-127);
  pros::delay(400);
  intake.move(0);
  clamp.set(false);

  // DRIVE TO AND CLAMP MOGO 2
  chassis.pid_drive_set(4, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-160, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-27, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(400);
  clamp.set(true);

  // INTAKE RED RING ONTO MOGO 2
  chassis.pid_turn_set(-230, TURN_SPEED);
  intlift.set(true);
  chassis.pid_wait_quick_chain();
  intake.move(-127);
  chassis.pid_drive_set(22, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  intlift.set(false);
  pros::delay(300);
  chassis.pid_drive_set(-8, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  // GO TO AUTONOMOUS LINE
  chassis.pid_turn_set(-30, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(25, DRIVE_SPEED);
  intake.move(0);

}

///
// XENON SKILLS AUTONOMOUS
///
void skills_auto() {
  //default configuration + definitions
  ez::Piston clamp(8);
  ez::Piston intlift(7);
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  ladybrown.set_brake_mode(MOTOR_BRAKE_HOLD);

  // RED ALLIANCE STAKE
  intake.move(-127);
  pros::delay(500);
  intake.move(0);

  // GRAB MOGO 1
  chassis.pid_drive_set(7, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-90, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-17, DRIVE_SPEED);
  pros::delay(500);
  clamp.set(true);
  pros::delay(100);
  
  // FILL MOGO 1
  chassis.pid_wait();
  chassis.pid_drive_set(-2, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(0, DRIVE_SPEED);
  intake.move(-127);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(15, DRIVE_SPEED);  
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(52, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(42, DRIVE_SPEED);  
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-6, 80);  
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-163.5, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(300);
  chassis.pid_drive_set(13, 65);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-180, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(37, 35);
  chassis.pid_wait_quick_chain();
  pros::delay(300);
  chassis.pid_drive_set(-30, DRIVE_SPEED);
  chassis.pid_turn_set(52, TURN_SPEED);





}
#include "main.h"
#include <cmath>
#include "subsystems.hpp"


/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Out of 127
const int DRIVE_SPEED = 127;
const int TURN_SPEED = 127;
const int SWING_SPEED = 127;

///
// Constants
///
void default_constants() {
  chassis.pid_heading_constants_set(11, 0, 20);
  chassis.pid_drive_constants_set(10.85, 0, 50);
  chassis.pid_turn_constants_set(4.8, 0, 45);
  chassis.pid_swing_constants_set(1.32, 0, 10);

  chassis.pid_turn_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(80_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);

  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(2_in);

  chassis.slew_drive_constants_set(7_in, 80);
}

  double ldbcurrentpos2 = (ldbrotation.get_angle()/100);
  double correctloadpos2 = 40.00;
  double correctalliancepos2 = 206.00;
  double correctmogopos2 = 256.00;
  double ldberrorload2 = correctloadpos2 - ldbcurrentpos2;
  double ldberrorall2 = correctalliancepos2 - ldbcurrentpos2;
  double ldberrormogo2 = correctmogopos2 - ldbcurrentpos2;
  double ladyBrownPID2(double error2 = 0, double kP2=-0, double kI2=0, double kD2=0, double totalError2 = 0, double prevError2 = 0, double integralThreshold2=30, double maxI2=500) {
      // calculate integral
      if (abs(error2) < integralThreshold2) {
        totalError2 += error2;
      }

      if (error2 > 0){
        totalError2 = std::min(totalError2, maxI2);
      }
      else {
        totalError2 = std::max(totalError2, -maxI2);
      }

    // calculate derivative
      float derivative2 = error2 - prevError2;
      prevError2 = error2;

    // calculate output
      double speed2 = (error2 * kP2) + (totalError2 * kI2) + (derivative2 * kD2);

      if (speed2 > 127){
        speed2 = 127;
      }
      else if (speed2 < -127){
        speed2 = -127;
      }

      return speed2;
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
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 90, SWING_SPEED, 50);
  chassis.pid_wait_quick_chain();
}

// . . .
// Make your own autonomous functions here!
// . . .

void newredsoloawp() {
  //default configuration + definitions
  ez::Piston clamp(8);
  ez::Piston doinker(1);
  ez::Piston intlift(4);
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  ladybrown.set_brake_mode(MOTOR_BRAKE_HOLD);

  while(abs(ldberrorall2)>2) {
    ldberrorall2 = correctalliancepos2 - ldbcurrentpos2;
    ladybrown.move(ladyBrownPID2(ldberrorall2, 0.5, 0, 0));
  }

}

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
  //intake.move(-127);
  pros::delay(500);
  //intake.move(0);

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
  //intake.move(-127);
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
  chassis.pid_turn_set(50, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-2.25, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(65, 80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-1.375, 80);
  chassis.pid_wait_quick_chain();
  pros::delay(200);
  //intake.move(-127);
  pros::delay(450);
  //intake.move(0);

  // GRAB SAFE MOBILE GOAL
  chassis.pid_turn_set(41.5, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(20, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  
  chassis.pid_turn_set(-103, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-12, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(250);
  clamp.set(true);
  pros::delay(100);
  chassis.pid_drive_set(2, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  
  // SCORE RED RING 2 ON MOBILE GOAL
  chassis.pid_turn_set(-138, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  intlift.set(true);
  //intake.move(-127);
  chassis.pid_drive_set(18.25, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(450);
  intlift.set(false); 
  pros::delay(450);
  chassis.pid_drive_set(-15, 100);
  chassis.pid_wait_quick_chain();

  
  // SCORE RED RING 3 ON MOBILE GOAL
  chassis.pid_turn_set(17, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(25, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(50);
  chassis.pid_drive_set(5, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(350);
  
  // TOUCH BAR
  chassis.pid_drive_set(-17, 80);
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
  chassis.pid_turn_set(9, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(8.375, DRIVE_SPEED);
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
  //intake.move(-127);
  pros::delay(400);
  //intake.move(0);
  clamp.set(false);

  // DRIVE TO AND CLAMP MOGO 2
  chassis.pid_drive_set(4, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(160, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-22, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-5, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(400);
  clamp.set(true);
  chassis.pid_drive_set(-2.5, 70);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(2.5, 70);
  chassis.pid_wait_quick_chain();


  // INTAKE RED RING ONTO MOGO 2
  chassis.pid_turn_set(230, TURN_SPEED);
  intlift.set(true);
  chassis.pid_wait_quick_chain();
  //intake.move(-127);
  chassis.pid_drive_set(20, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  intlift.set(false);
  pros::delay(600);
  chassis.pid_drive_set(-11, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  
  // GO TO AUTONOMOUS LINE
  chassis.pid_turn_set(30, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(16, DRIVE_SPEED);
  //intake.move(0);
  
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
  //intake.move(-127);
  pros::delay(400);
  //intake.move(0);
  clamp.set(false);

  // DRIVE TO AND CLAMP MOGO 2
  chassis.pid_drive_set(4, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-160, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-22, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-9, 90);
  pros::delay(400);
  clamp.set(true);
  chassis.pid_wait_quick_chain();

  // INTAKE RED RING ONTO MOGO 2
  chassis.pid_turn_set(-215, TURN_SPEED);
  intlift.set(true);
  chassis.pid_wait_quick_chain();
  //intake.move(-127);
  chassis.pid_drive_set(24, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  intlift.set(false);
  pros::delay(500);
  chassis.pid_drive_set(-10, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  // GO TO AUTONOMOUS LINE
  chassis.pid_turn_set(-30, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(22, DRIVE_SPEED);
  //intake.move(0);

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
  //intake.move(-127);
  pros::delay(500);
  //intake.move(0);

  // GRAB MOGO 1
  chassis.pid_drive_set(9, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-90, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-17.75, DRIVE_SPEED);
  pros::delay(500);
  clamp.set(true);
  pros::delay(100);
  
  // FILL MOGO 1
  chassis.pid_wait();
  chassis.pid_drive_set(-2.25, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(250);
  chassis.pid_turn_set(6, DRIVE_SPEED);
  chassis.pid_wait();
  
  //intake.move(-127);
  chassis.pid_drive_set(16, DRIVE_SPEED);  
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(46.5, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(45, DRIVE_SPEED);  
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-13.75, 100);  
  chassis.pid_wait_quick_chain();
  pros::delay(200);
  chassis.pid_turn_set(175, TURN_SPEED);
  pros::delay(600);
  
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(33.5, 75);
  pros::delay(950);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(7, 75);
  chassis.pid_wait_quick_chain();
  pros::delay(300);
  
  chassis.pid_drive_set(-3.75, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(250);
  chassis.pid_turn_set(100, 60);
  pros::delay(100);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(10, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  // SCORE IN CORNER
  chassis.pid_turn_set(0, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-8, DRIVE_SPEED);
  pros::delay(400);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-20, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(200);
  //intake.move(0);
  clamp.set(false);

  // GRAB MOGO 2
  chassis.pid_drive_set(3, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(90, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-64, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(60, TURN_SPEED);
  chassis.pid_drive_set(-9, 80);
  pros::delay(400);
  clamp.set(true);
  pros::delay(50);
  chassis.pid_wait();

  //////////////////////////////////

  // FILL MOGO 2 
  
  chassis.pid_wait();
  chassis.pid_drive_set(-1.5, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(0, DRIVE_SPEED);
  //intake.move(-127);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(15, DRIVE_SPEED);  
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-52, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(42, DRIVE_SPEED);  
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-7.5, 100);  
  chassis.pid_wait_quick_chain();
  pros::delay(200);
  chassis.pid_turn_set(163.5, TURN_SPEED);
  pros::delay(300);
  
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(13, 65);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(180, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(22, 65);
  pros::delay(700);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(5.5, 65);
  chassis.pid_wait_quick_chain();
  pros::delay(300);
  chassis.pid_drive_set(-3.75, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(250);
  chassis.pid_turn_set(-100, 60);
  pros::delay(100);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(10, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  // SCORE IN CORNER
  chassis.pid_turn_set(-180, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-8, DRIVE_SPEED);
  pros::delay(400);
  chassis.pid_wait_quick_chain();

}
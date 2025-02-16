#include "main.h"
#include <cmath>
#include "subsystems.hpp"


using namespace pros;


/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Out of 127
const int DRIVE_SPEED = 127;
const int TURN_SPEED = 127;
const int SWING_SPEED = 127;
const int SKILLS_DRIVE = 110;
const int SKILLS_TURN = 110;

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
  chassis.pid_drive_exit_condition_set(65_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);

  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(2_in);

  chassis.slew_drive_constants_set(7_in, 80);
}

void defaultconfig(){
  //default configuration + definitions
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  ladybrown.set_brake_mode(MOTOR_BRAKE_HOLD);
  lbPID = false;
  csortoptical.set_led_pwm(100);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void redsoloawp() {
  defaultconfig();

  //alliance stake
  chassis.pid_turn_set(37, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(2.25, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  ladyBrownCorrectPosition = ladyBrownAlliance;
  lbPID = true;
  delay(950);
  chassis.pid_drive_set(-4.25, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  ladyBrownCorrectPosition = ladyBrownPassive;
  
  //grab mogo
  chassis.pid_turn_set(20, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-20, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  delay(100);
  chassis.pid_drive_set(-10, 90);
  delay(200);
  clamps.set(true);

  // get ring 2
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(43, TURN_SPEED);
  intake.move(-127);
  intlift.set(true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(15, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(10.5, 40);
  chassis.pid_wait_quick_chain();
  delay(900);
  intlift.set(false);
  chassis.pid_drive_set(-17, (DRIVE_SPEED/2));
  chassis.pid_wait_quick_chain();

  // get ring 3
  chassis.pid_turn_set(-85, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(20, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-3, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  delay(200);

  //touch
  chassis.pid_turn_set(90, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(25, (DRIVE_SPEED/2));
  ladyBrownCorrectPosition = (ladyBrownAlliance/1.75);

  // get ring 4
  /*
  chassis.pid_turn_set(-48, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  delay(100);
  chassis.pid_drive_set(12, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-22, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  intake.move(127);

  chassis.pid_drive_set(27, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  delay(100);
  intake.move(-127);
  delay(100);
  chassis.pid_drive_set(-20, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  delay(200);
  
  //touch bar
  chassis.pid_turn_set(105, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  ladyBrownCorrectPosition = (ladyBrownAlliance/2.25);
  chassis.pid_drive_set(45, (DRIVE_SPEED/2.25));
  chassis.pid_wait_quick_chain();
  delay(400);*/
  
}

void bluesoloawp() {
  defaultconfig();

  //alliance stake
  chassis.pid_turn_set(-40, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(2.25, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  ladyBrownCorrectPosition = ladyBrownAlliance;
  lbPID = true;
  delay(950);
  chassis.pid_drive_set(-4.25, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  ladyBrownCorrectPosition = ladyBrownPassive;
  
  //grab mogo
  chassis.pid_turn_set(-20, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-20, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  delay(100);
  chassis.pid_drive_set(-10, 90);
  delay(200);
  clamps.set(true);

  // get ring 2
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-43, TURN_SPEED);
  intake.move(-127);
  intlift.set(true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(15, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(10.5, 40);
  chassis.pid_wait_quick_chain();
  delay(900);
  intlift.set(false);
  chassis.pid_drive_set(-17, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  // get ring 3
  chassis.pid_turn_set(85, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(20, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-3, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  delay(200);

  //touch
  chassis.pid_turn_set(-90, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(25, (DRIVE_SPEED/2));
  ladyBrownCorrectPosition = (ladyBrownAlliance/1.75);

  // get ring 4
  /*chassis.pid_turn_set(48, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  delay(100);
  chassis.pid_drive_set(12, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(35, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  intake.move(127);

  chassis.pid_drive_set(30, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  delay(100);
  intake.move(-127);
  delay(100);
  chassis.pid_drive_set(-35, (DRIVE_SPEED/1.5));
  chassis.pid_wait_quick_chain();
  delay(200);
  chassis.pid_turn_set(-93, (TURN_SPEED/1.25));
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(28, (DRIVE_SPEED/1.5));
  ladyBrownCorrectPosition = (ladyBrownAlliance/1.5);
  */

}

void rednegativeside() {
  defaultconfig();

  //alliance stake
  chassis.pid_turn_set(-41, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(1.85, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  ladyBrownCorrectPosition = ladyBrownAlliance;
  lbPID = true;
  delay(950);
  chassis.pid_drive_set(-3.625, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  ladyBrownCorrectPosition = ladyBrownPassive;

  //mogo
  chassis.pid_turn_set(-18, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-29, (DRIVE_SPEED/2));
  chassis.pid_wait_quick_chain();
  delay(200);
  clamps.set(true);

  //auton line rings
  chassis.pid_turn_set(135, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  intlift.set(true);
  intake.move(127);
  chassis.pid_drive_set(22.5, DRIVE_SPEED/2);
  chassis.pid_wait_quick_chain();
  delay(100);
  
  chassis.pid_turn_set(125, (TURN_SPEED/2.25));
  chassis.pid_wait_quick_chain();
  intake.move(-127);
  intlift.set(false);
  delay(50);
  chassis.pid_turn_set(90, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(14, (DRIVE_SPEED/3));
  chassis.pid_wait_quick_chain();
  delay(500);
  intake.move(0);
  flex.move(127);

  chassis.pid_drive_set(-14, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(34, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  delay(50);
  flex.move(0);
  delay(50);
  intake.move(-127);
  chassis.pid_drive_set(12, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  delay(400);

  chassis.pid_turn_set(-90, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(48, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  ladyBrownCorrectPosition = (ladyBrownAlliance/2);
  /*chassis.pid_drive_set(10, (DRIVE_SPEED));
  chassis.pid_wait_quick_chain();
  intake.move(127);
  chassis.pid_turn_set(45, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  
  chassis.pid_drive_set(35, DRIVE_SPEED/2.25);
  chassis.pid_wait_quick_chain();
  delay(200);
  intake.move(-127);
  delay(300);
  chassis.pid_drive_set(-25, (DRIVE_SPEED));
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-105, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(20, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(8, (DRIVE_SPEED/2));*/


}

void bluerushside() {
  defaultconfig();

  flex.move(-127);
  chassis.pid_drive_set(20, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  doinker.set(true);
  chassis.pid_drive_set(17.75, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  rushmech.set(true);
  chassis.pid_drive_set(-15, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  flex.move(0);

  rushmech.set(false);
  delay(350);
  chassis.pid_drive_set(-1, DRIVE_SPEED);
  doinker.set(false);
  chassis.pid_wait_quick_chain();
  
  chassis.pid_turn_set(-24, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(4.5, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  lbPID = true;
  ladyBrownCorrectPosition = ladyBrownMogo;
  delay(700);

  chassis.pid_drive_set(-2, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-120, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  ladyBrownCorrectPosition = ladyBrownPassive;
  chassis.pid_drive_set(-34, (DRIVE_SPEED/2));
  chassis.pid_wait_quick_chain();
  clamps.set(true);
  delay(100);

  chassis.pid_turn_set(-235, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  delay(300);
  intlift.set(true);
  intake.move(-127);
  chassis.pid_drive_set(19, (DRIVE_SPEED/2));
  chassis.pid_wait_quick_chain();
  delay(1000);
  chassis.pid_drive_set(-20, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(45, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-40, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  intake.move(127);
  
  delay(100);
  clamps.set(false);
  chassis.pid_drive_set(10, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(110, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-8, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();


  


  


}

void redrushside() {
  defaultconfig();

  flex.move(-127);
  chassis.pid_drive_set(12.5, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  doinker.set(true);
  chassis.pid_drive_set(24.25, (DRIVE_SPEED));
  chassis.pid_wait_quick_chain();
  rushmech.set(true);
  chassis.pid_drive_set(-25, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  rushmech.set(false);
  flex.move(-(127/2));
  delay(50);
  chassis.pid_drive_set(-8, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(130, TURN_SPEED);
  doinker.set(false);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-22.5, (DRIVE_SPEED/2));
  chassis.pid_wait_quick_chain();
  clamps.set(true);
  
  chassis.pid_drive_set(-5, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(68, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  delay(100);
  lbPID = true;
  ladyBrownCorrectPosition = ladyBrownMogo;
  intake.move(-127);
  delay(1400);
  ladyBrownCorrectPosition = ladyBrownPassive;
  delay(300);
  chassis.pid_turn_set(180, TURN_SPEED);
  intake.move(0);
  chassis.pid_wait_quick_chain();

  intlift.set(true);
  chassis.pid_drive_set(14, (DRIVE_SPEED));
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(210, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  intake.move(-127);
  chassis.pid_drive_set(12, (DRIVE_SPEED/2.5));
  chassis.pid_wait_quick_chain();
  delay(500);
  intlift.set(false);

  chassis.pid_drive_set(-10, (DRIVE_SPEED/1.5));
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(250, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-35, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  intake.move(127);
  delay(200);
  clamps.set(false);

  chassis.pid_drive_set(15, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(185, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-22, (DRIVE_SPEED/1.5));
  chassis.pid_wait_quick_chain();
  clamps.set(true);
  chassis.pid_turn_set(50, TURN_SPEED);
  
  


  


  

}

void xenon_skills() {
  defaultconfig();

  intake.move(-127);
  delay(350);
  chassis.pid_drive_set(7, SKILLS_DRIVE);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-90, SKILLS_TURN);
  chassis.pid_wait_quick_chain();
  intake.move(0);

  chassis.pid_drive_set(-12, SKILLS_DRIVE);
  chassis.pid_wait_quick_chain();
  delay(300);
  clamps.set(true);

  chassis.pid_drive_set(-4, SKILLS_DRIVE);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(0, SKILLS_TURN);
  chassis.pid_wait_quick_chain();
  intake.move(-127);
  chassis.pid_drive_set(14, SKILLS_DRIVE);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(53, SKILLS_TURN);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(36, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(90, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-5.5, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(176, (SKILLS_TURN/1.5));
  delay(300);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(20, 40);
  chassis.pid_wait_quick_chain();
  delay(400);
  chassis.pid_turn_set(180, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(34, 50);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(45, 80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(8, SKILLS_DRIVE);
  chassis.pid_wait_quick_chain();
  delay(300);
  chassis.pid_turn_set(-30, SKILLS_TURN);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-8, SKILLS_DRIVE);
  chassis.pid_wait_quick_chain();
  delay(900);
  intake.move(127);
  delay(300);
  clamps.set(false);
  chassis.pid_drive_set(8, SKILLS_DRIVE);
  chassis.pid_wait_quick_chain();
  
  
  //////////////////////////////////////////////////////
  // MOGO 2
  //////////////////////////////////////////////////////
  chassis.pid_turn_set(84, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  intake.move(0);
  chassis.pid_drive_set(-50, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(90, SKILLS_TURN);
  chassis.pid_wait_quick_chain();
  
  chassis.pid_drive_set(-5.5, SKILLS_DRIVE);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(90, SKILLS_TURN);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-12, (SKILLS_DRIVE/2.25));
  chassis.pid_wait_quick_chain();
  delay(300);
  clamps.set(true);

  chassis.pid_drive_set(-2, SKILLS_DRIVE);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(0, SKILLS_TURN);
  chassis.pid_wait_quick_chain();
  intake.move(-127);
  chassis.pid_drive_set(18, SKILLS_DRIVE);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-55, SKILLS_TURN);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(36, SKILLS_DRIVE);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-90, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  
  chassis.pid_drive_set(-7.5, (SKILLS_DRIVE/1.5));
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-177, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(20, 40);
  chassis.pid_wait_quick_chain();
  delay(400);
  chassis.pid_turn_set(-173, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(34, 50);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-45, 80);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(8, SKILLS_DRIVE);
  chassis.pid_wait_quick_chain();
  delay(300);
  chassis.pid_turn_set(30, SKILLS_TURN);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-8, SKILLS_DRIVE);
  chassis.pid_wait_quick_chain();
  delay(900);
  intake.move(127);
  delay(300);
  clamps.set(false);
  chassis.pid_drive_set(8, SKILLS_DRIVE);
  chassis.pid_wait_quick_chain();

  //////////////////////////////////////////////////////
  // BLUE MOGO
  ////////////////////////////////////////////////////// 
  
  chassis.pid_turn_set(45, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  flex.move(127);
  chassis.pid_drive_set(25, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  delay(100);
  flex.move(0);
  chassis.pid_turn_set(240, SKILLS_TURN);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-15, SKILLS_DRIVE);
  chassis.pid_wait_quick_chain();
  delay(100);
  clamps.set(true);
  delay(100);

  


  
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


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
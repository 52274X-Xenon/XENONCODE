#include "main.h"
#include <cmath>

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-8, 9, -10},     // Left Chassis Ports (negative port will reverse it!)
    {1, -2, 3},  // Right Chassis Ports (negative port will reverse it!)

    4,      // IMU Port
    3.25 ,  // Wheel Diameter
    450);   // Wheel RPM


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);  // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0);    // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0, 0);     // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      Auton("SKILLS", skills_auto),
      Auton("solo awp blue", solo_awp_blue),
      Auton("solo awp red", solo_awp_red),
      Auton("rush red", rush_auto_red),
      Auton("rush blue", rush_auto_blue),
      Auton("Example Drive\n\nDrive forward and come back.", drive_example),
      Auton("Example Turn\n\nTurn 3 times.", turn_example),
      Auton("Drive and Turn\n\nDrive forward, turn, come back. ", drive_and_turn),
      Auton("Drive and Turn\n\nSlow down during drive.", wait_until_change_speed),
      Auton("Swing Example\n\nSwing in an 'S' curve", swing_example),
      Auton("Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining),
      Auton("Combine all 3 movements", combining_movements),
      Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),
  });



  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(".");
}


// LADY BROWN PID

  double ladyBrownPID(double error = 0, double kP=-0, double kI=0, double kD=0, double totalError = 0, double prevError = 0, double integralThreshold=30, double maxI=500) {
      // calculate integral
      if (abs(error) < integralThreshold) {
          totalError += error;
      }

      if (error > 0){
          totalError = std::min(totalError, maxI);
      }
      else {
          totalError = std::max(totalError, -maxI);
      }

    // calculate derivative
      float derivative = error - prevError;
      prevError = error;

    // calculate output
      double speed = (error * kP) + (totalError * kI) + (derivative * kD);

      if (speed > 127){
          speed = 127;
      }
      else if (speed < -127){
          speed = -127;
      }

      return speed;
  }

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  // This is preference to what you like to drive on
  pros::motor_brake_mode_e_t driver_preference_brake = MOTOR_BRAKE_COAST;
  /*pros::motor_brake_mode_e_t ladybrown_default_brake = MOTOR_BRAKE_HOLD;
  ladybrown.set_brake_mode(ladybrown_default_brake);*/
  chassis.drive_brake_set(driver_preference_brake);

//DEFINE PISTONS AND SENSORS + LDB BRAKE MODE AND MORE
  ez::Piston clamp(8);
  ez::Piston rightdoinker(6);
  ez::Piston leftdoinker(5);
  ez::Piston intlift(7);
  ez::Piston csortpist(3);
  pros::Optical csortoptical(11);
  pros::Rotation ldbrotation(7);
  ladybrown.set_brake_mode(MOTOR_BRAKE_HOLD); 
  bool ldbPID = false; // Lady Brown PID is off by default
  csortoptical.set_led_pwm(100); // Turn on light with 100% intensity
  double csorthue_val = csortoptical.get_hue(); // variable returns optical sensor's detected hue value
  


  while (true) {
    // PID Tuner
    // After you find values that you're happy with, you'll have to set them in auton.cpp
    if (!pros::competition::is_connected()) {
      // Enable / Disable PID Tuner
      //  When enabled:
      //  * use A and Y to increment / decrement the constants
      //  * use the arrow keys to navigate the constants
      if (master.get_digital_new_press(DIGITAL_X))
        chassis.pid_tuner_toggle();

      // Trigger the selected autonomous routine
      if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
        autonomous();
        chassis.drive_brake_set(driver_preference_brake);
      }

      chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
    }

    // chassis.opcontrol_tank();  // Tank control
    chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    //chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    //chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .
    

    // LADY BROWN MACRO
    double ldbcurrentpos = (ldbrotation.get_angle()/100.0); //CENTI DEGREES SO WATCH OUT FOR THE /100
        double ldbcorrectpos1 = -140; //loading pos if ldb above loading pos
        double ldbcorrectpos2 = 58 //loading pos if ldb below loadpos1
        double lberror1 = (ldbcorrectpos1 - ldbcurrentpos); //Finds position that you want to go to based on how much movement
        double lberror2 = (ldbcorrectpos2 - ldbcurrentpos);


        if (master.get_digital_new_press(DIGITAL_DOWN)){
          ldbPID = true; //Sets PID to true, which activates loop setting macro active
        } 
        // experimental start
      
        else { 
            if (master.get_digital(DIGITAL_L1)) {
              ladybrown.move(127);
              ldbPID = false;
            }

            else if (master.get_digital(DIGITAL_L2)) {
              ladybrown.move(-127);
              ldbPID = false;
            }

            else {
              ladybrown.move(0);
            }
        }
    
      
        if ((ldbPID == true) && ldbcurrentpos > 35) { // if ladybrown is above loading position
          ladybrown.move(ladyBrownPID(lberror1, 0.30, 0, 0.15)); // position, kP, kI, kD

        }
        else if ((ldbPID == true) && ldbcurrentpos < 35) { // if ladybrown is between zero and loading position
          ladybrown.move(ladyBrownPID(lberror2, 0.40, 0, 0.15)); // position, kP, kI, kD
        }
      

    // INTAKE FUNCTIONS
    
    if (master.get_digital(DIGITAL_R2)) {
      intake.move(127);
    }
    else if (master.get_digital(DIGITAL_R1)) {
      intake.move(-127);
    }
    else {
      intake.move(0);
    }

    // PISTON FUNCTIONS
    rightdoinker.button_toggle(master.get_digital(DIGITAL_Y));
    clamp.button_toggle(master.get_digital(DIGITAL_B));
    leftdoinker.button_toggle(master.get_digital(DIGITAL_RIGHT));


    //COLOR SORTER
    /*if (0.000 < csorthue_val < 30.000) {
    
    }
    */

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  
  }
}
#include "ladybrown.hpp"
#include "subsystems.hpp"

using namespace pros;

// Ldb states
int ladyBrownState = 0;

double ladyBrownMogo = 250;
double ladyBrownAlliance = 212.00;
double ladyBrownLoad1st = 35.00;
double ladyBrownLoad2nd = 45.00;
double ladyBrownPassive = 3.25;
//double ladyBrownLoad1 = 48.00;
//double ladyBrownLoad2 = 36.00;


// Initialize global variables
bool lbPID = false;
double ladyBrownCorrectPosition = ladyBrownLoad;
double ladyBrownCurrentPosition;

// Persistent state variables for PID
double ladyBrownTotalError = 0;
double ladyBrownPrevError = 0;

double ladyBrownPID(double error, double kP, double kI, double kD, double integralThreshold, double maxI) {
	// calculate integral
	if (abs(error) < integralThreshold) {
		ladyBrownTotalError += error;
	}

	if (error > 0) {
		ladyBrownTotalError = std::min(ladyBrownTotalError, maxI);
	} else {
		ladyBrownTotalError = std::max(ladyBrownTotalError, -maxI);
	}

	// calculate derivative
	double derivative = error - ladyBrownPrevError;
	ladyBrownPrevError = error;

	// calculate output
	double speed = (error * kP) + (ladyBrownTotalError * kI) + (derivative * kD);

	// Clamp the output to motor limits
	if (speed > 127) {
		speed = 127;
	} else if (speed < -127) {
		speed = -127;
	}

	return speed;
}

void ladyBrownTask() {
	while (1) {
		// Update current position
		ladyBrownCurrentPosition = ldbrotation.get_angle() / 100;

		// Calculate error
		double lberror = ladyBrownCorrectPosition - ladyBrownCurrentPosition;

		// Apply PID control if enabled
		if (lbPID) {
			ladybrown.move(ladyBrownPID(lberror, 0.75, 0, 0));
		}

		// Delay to prevent CPU overutilization
		pros::delay(10);
	}
}
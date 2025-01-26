#include "ladybrown.hpp"
#include "subsystems.hpp"

using namespace pros;

// Initialize global variables
bool lbPID = false;
double ladyBrownCorrectPosition = 329.00;
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
			ladybrown.move(ladyBrownPID(lberror, -3, -0, -0));
		}

		// Delay to prevent CPU overutilization
		pros::delay(10);
	}
}
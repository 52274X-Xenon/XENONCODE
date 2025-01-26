#include "ladybrown.hpp"
#include "subsystems.hpp"

using namespace pros;

bool lbPID = false;
double ladyBrownCorrectPosition = 329.00;
double ladyBrownCurrentPosition;

double ladyBrownPID(double error, double kP=5, double kI=0, double kD=0, double totalError=0, double prevError=0, double integralThreshold=30, double maxI=500) {
	// calculate integral
	if (abs(error) < integralThreshold)
	{
		totalError += error;
	}

	if (error > 0){
		totalError = std::min(totalError, maxI);
	}
	else{
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

void ladyBrownTask(){
	while(1){
		ladyBrownCurrentPosition = (ldbrotation.get_angle())/100;
		double lberror = (ladyBrownCorrectPosition - ladyBrownCurrentPosition);
		if (lbPID == true){
			ladybrown.move(ladyBrownPID(lberror, -3, -0, -0));
		}
	}
}
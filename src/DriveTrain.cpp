/*
 * DriveTrain.cpp
 *
 *  Created on: Feb 10, 2015
 *      Author: RoboLoCo
 */

#include <DriveTrain.h>

DriveTrain::DriveTrain(CANTalon& Left1, CANTalon& Left2, CANTalon& Right1, CANTalon& Right2, Joystick &Left, Joystick &Right) :
		drive(Left1, Left2, Right1, Right2), left(Left), right(Right) {

}

void DriveTrain::Drive() {
	drive->TankDrive(-left->GetY(), -right->GetY(), false);
}

void DriveTrain::Drive(float pow) {
}

void DriveTrain::Drive(float left, float right) {
}

DriveTrain::~DriveTrain() {
	// TODO Auto-generated destructor stub
}


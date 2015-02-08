/*
 * Lift.cpp
 *
 *  Created on: Feb 8, 2015
 *      Author: RoboLoCo
 */

#include <Lift.h>
#define FACTOR 0.07

Lift::Lift(SpeedController &ChainDriver, DoubleSolenoid &Arm) {
	chainDriver = &ChainDriver;
	arm = &Arm;
	nTotes = 0;
}

void Lift::Open() {
	arm->Set(DoubleSolenoid::kForward);
}

void Lift::Close() {
	arm->Set(DoubleSolenoid::kReverse);
}

void Lift::Hold() {
	chainDriver->Set(0.1 + FACTOR * nTotes);
}

void Lift::Run(float power) {
	chainDriver->Set(-power);
}

Lift::~Lift() {
	// TODO Auto-generated destructor stub
}


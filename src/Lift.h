/*
 * Lift.h
 *
 *  Created on: Feb 8, 2015
 *      Author: RoboLoCo
 */

#include "WPILib.h"

#ifndef SRC_LIFT_H_
#define SRC_LIFT_H_

class Lift {
	SpeedController *chainDriver;
	DoubleSolenoid *arm;
public:
	int nTotes;
	Lift(SpeedController &ChainDriver,DoubleSolenoid &Arm);
	void Open();
	void Close();
	void Hold();
	void Run(float power);
	virtual ~Lift();
};

#endif /* SRC_LIFT_H_ */

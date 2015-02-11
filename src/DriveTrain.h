/*
 * DriveTrain.h
 *
 *  Created on: Feb 10, 2015
 *      Author: RoboLoCo
 */

#ifndef SRC_DRIVETRAIN_H_
#define SRC_DRIVETRAIN_H_

enum DriveState {ACTIVE, DISABLED};
enum ControlState {PRECISE,TURBO};

class DriveTrain {
	RobotDrive drive;
	Joystick *left;
	Joystick *right;
	DriveState driveState;
	ControlState controlState;
public:
	DriveTrain(CANTalon &Left1, CANTalon &Left2, CANTalon &Right1, CANTalon &Right2, Joystick &left, Joystick &right);
	void Drive();
	void Drive(float pow);
	void Drive(float left, float right);
	virtual ~DriveTrain();
};

#endif /* SRC_DRIVETRAIN_H_ */

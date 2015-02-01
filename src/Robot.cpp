#include "WPILib.h"

class Robot: public IterativeRobot {
	LiveWindow *lw;
	Joystick driverLeft{0};
	Joystick driverRight{1};
	Joystick controller{2};
	CANTalon leftF{3};
	CANTalon leftB{4};
	CANTalon rightF{2};
	CANTalon rightB{1};
	TalonSRX lift{0};
	RobotDrive driveTrain{leftF,leftB,rightF,rightB};
	Servo pan{1};
	Servo tilt{2};
	Compressor comp{0};
	DoubleSolenoid cylinder{0,1};
public:
	Robot() {
	}
private:

	void RobotInit() {
		lw = LiveWindow::GetInstance();
	}

	void AutonomousInit() {

	}

	short state = 0;
	void AutonomousPeriodic() {
		/*switch(state) {
		case 0:
			break;
		//	left.Set(1);
		//	right.Set(1);
		}*/
	}

	void TeleopInit() {
		comp.SetClosedLoopControl(true);
		driveTrain.SetExpiration(100);
	}
	short state_driving = 2;
	void TeleopPeriodic() {
		driveTrain.SetSafetyEnabled(false);
		if (driverRight.GetRawButton(1)) {
			state_driving = 2;
		} else {
			state_driving = 1;
		}
		switch(state_driving) {
		default:
		case 0: // Disabled
			driveTrain.TankDrive(.0f,.0f,false);
			break;
		case 1: // Precise Driving
			driveTrain.TankDrive(-driverLeft.GetY()*.5,-driverRight.GetY()*.5,false);
			break;
		case 2: // Turbo Driving
			driveTrain.TankDrive(-driverLeft.GetY(),-driverRight.GetY(),false);
			break;
		}
		pan.Set((controller.GetZ()+1.0)/2);
		tilt.Set((controller.GetThrottle()+1.0)/2);
		if (controller.GetRawButton(5)) {
			cylinder.Set(DoubleSolenoid::kForward);
		} else if (controller.GetRawButton(3)) {
			cylinder.Set(DoubleSolenoid::kReverse);
		} else if (controller.GetRawButton(2)) {
			cylinder.Set(DoubleSolenoid::kOff);
		}
		if(abs(controller.GetY()) > 0.05) {
			lift.Set(controller.GetY());
		} else {
			lift.Set(0);
		}
	}

	void TestPeriodic() {
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);

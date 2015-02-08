#include "WPILib.h"
#include "Lift.h"
#include "math.h"

class Robot: public IterativeRobot {
	LiveWindow *lw;
	Joystick driverLeft { 0 };
	Joystick driverRight { 1 };
	Joystick controller { 2 };
	CANTalon leftF { 3 };
	CANTalon leftB { 4 };
	CANTalon rightF { 2 };
	CANTalon rightB { 1 };
	RobotDrive driveTrain { leftF, leftB, rightF, rightB };
	Servo pan { 1 };
	Servo tilt { 2 };
	PowerDistributionPanel pdp { };
	Compressor comp { };
	BuiltInAccelerometer accel { };

	TalonSRX cd { 0 };
	DoubleSolenoid cylinder { 0, 1 };
	Lift lift { cd, cylinder };

	int raiseLevel { 0 };
	bool bounce = false;
public:
	Robot() {
	}
	;
private:

	void RobotInit() {
		lw = LiveWindow::GetInstance();
		CameraServer::GetInstance()->SetQuality(50);
		CameraServer::GetInstance()->StartAutomaticCapture();
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
	char i;
	void TeleopPeriodic() {
		driveTrain.SetSafetyEnabled(false);
		driveTrain.SetExpiration(200000);
		if (driverRight.GetRawButton(1)) {
			state_driving = 2;
		} else {
			state_driving = 1;
		}
		switch (state_driving) {
		default:
		case 0: // Disabled
			driveTrain.TankDrive(.0f, .0f, false);
			break;
		case 1: // Precise Driving
			driveTrain.TankDrive(-driverLeft.GetY() * .5, -driverRight.GetY() * .5, false);
			break;
		case 2: // Turbo Driving
			driveTrain.TankDrive(-driverLeft.GetY(), -driverRight.GetY(), false);
			break;
		}
		pan.Set((controller.GetZ() + 1.0) / 2);
		tilt.Set((controller.GetThrottle() + 1.0) / 2);

		if (controller.GetRawButton(5)) {
			lift.Open();
		} else if (controller.GetRawButton(3)) {
			lift.Close();
		}

		if ((controller.GetPOV() == 0) && bounce) {
			lift.nTotes++;
			bounce = false;
		} else if ((controller.GetPOV() == 180) && bounce) {
			lift.nTotes--;
			bounce = false;
		} else if (controller.GetPOV() == -1) {
			bounce = true;
		}

		if (fabs(controller.GetY()) > 0.1) {
			lift.Run(controller.GetY());
		} else {
			lift.Hold();
		}

		SmartDashboard::PutNumber("totes", lift.nTotes);
		DisabledPeriodic();
	}

	void DisabledPeriodic() {
		SmartDashboard::PutNumber("gx", accel.GetX());
		SmartDashboard::PutNumber("gy", accel.GetY());
		SmartDashboard::PutNumber("gz", accel.GetZ());
		SmartDashboard::PutNumber("Compressor Current", comp.GetCompressorCurrent() + 0.00001 * i);
		SmartDashboard::PutNumber("Total Current Draw", pdp.GetTotalCurrent() + 0.00001 * i);
		SmartDashboard::PutNumber("Left Drive 1", pdp.GetCurrent(15) + 0.00001 * i);
		SmartDashboard::PutNumber("Left Drive 2", pdp.GetCurrent(14) + 0.00001 * i);
		SmartDashboard::PutNumber("Right Drive 1", pdp.GetCurrent(0) + 0.00001 * i);
		SmartDashboard::PutNumber("Right Drive 2", pdp.GetCurrent(1) + 0.00001 * i);
		SmartDashboard::PutNumber("Lift Motor", pdp.GetCurrent(13) + 0.00001 * i);
		i = (i + 1) % 2;
	}

	void TestPeriodic() {
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);

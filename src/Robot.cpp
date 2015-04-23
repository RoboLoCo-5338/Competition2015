#include "WPILib.h"
#include "Lift.h"
#include "math.h"

class Robot: public IterativeRobot {
	LiveWindow *lw { nullptr };
	Joystick driverLeft { 0 };
	Joystick driverRight { 1 };
	Joystick controller { 2 };
	CANTalon leftF { 3 };
	CANTalon leftB { 4 };
	CANTalon rightF { 2 };
	CANTalon rightB { 1 };
	RobotDrive driveTrain { leftF, leftB, rightF, rightB };
	//Servo pan { 1 };
	//Servo tilt { 2 };
	PowerDistributionPanel pdp { };
	Compressor comp { };
	BuiltInAccelerometer accel { };
	Encoder rightEncoder { 0, 1, false, Encoder::EncodingType::k4X };
	Encoder leftEncoder { 2, 3, true, Encoder::EncodingType::k4X };

	TalonSRX cd { 0 };
	DoubleSolenoid cylinder { 0, 1 };
	Lift lift { cd, cylinder };

	DoubleSolenoid pickup { 2, 3 };

	CANTalon pickupL { 6 };
	CANTalon pickupR { 5 };
	Relay leds { 0, Relay::kForwardOnly };
	SendableChooser autochooser { };

	int raiseLevel { 0 };
	bool bounce { false };
public:
	Robot() {
	}

private:

	void RobotInit() {
		lw = LiveWindow::GetInstance();
		//CameraServer::GetInstance()->SetQuality(50);
		//CameraServer::GetInstance()->StartAutomaticCapture();
		autochooser.AddDefault("No Auto", (void*)0);
		autochooser.AddObject("Drive Straight", (void*)1);
		SmartDashboard::PutData("Autonomous Mode", autochooser);
	}

	short autonum = 0;
	void AutonomousInit() {
		driveTrain.RobotDrive(0.0, 0.0);
		autonum = (int) autochooser.GetSelected();
	}
	void AutonomousPeriodic() {
		UpdateLEDs();
		switch (autonum) {
		default:
		case 0:
			NoAuto();
			break;
		case 1:
			DriveStraight();
			break;
			//case 2:
			//	Auto2();
			//	break;
		}
	}

	short autostate = 0;
	void NoAuto() {
		driveTrain.RobotDrive(0.0, 0.0);
	}
	void DriveStraight() {
		switch (autostate) {
		case 0:
			leftEncoder.GetRaw();
			rightEncoder.GetRaw();
			break;
		case 1:
			if (leftEncoder.GetRaw() > 1000) {
				driveTrain.RobotDrive(0.0,0.0);
				autostate = 2;
			}
			driveTrain.RobotDrive(0.6,0.6);
			break;
		case 2:
			driveTrain.RobotDrive(0.0,0.0);
			break;
		default:
			break;
		}
	}

	void TeleopInit() {
		leds.Set(Relay::kForward);
		comp.SetClosedLoopControl(true);

		driveTrain.SetExpiration(100);
	}
	short state_driving = 2;
	short state_straight = 0;
	char i = 0;
	void TeleopPeriodic() {
		UpdateLEDs();
		driveTrain.SetSafetyEnabled(false);
		driveTrain.SetExpiration(200000);
		if (driverRight.GetRawButton(1)) {
			state_driving = 2;
		} else {
			state_driving = 1;
		}
		if (driverLeft.GetRawButton(1)) {
			state_straight = 1;
		} else {
			state_straight = 2;
		}
		float leftPower, rightPower;
		leftPower = driverLeft.GetY();
		switch (state_straight) {
		default:
		case 0:
			rightPower = driverRight.GetY();
			break;
		case 1:
			rightPower = leftPower;
			break;
		}
		state_driving = controller.GetRawButton(8) ? 0 : state_driving;
		switch (state_driving) {
		default:
		case 0: // Disabled
			driveTrain.TankDrive(.0f, .0f, false);
			break;
		case 1: // Precise Driving
			driveTrain.TankDrive(-leftPower * .5, -rightPower * .5, false);
			break;
		case 2: // Turbo Driving
			driveTrain.TankDrive(-leftPower, -rightPower, false);
			break;
		}
		//pan.Set((controller.GetZ() + 1.0) / 2);
		//tilt.Set((controller.GetThrottle() + 1.0) / 2);

		if (controller.GetRawButton(5)) {
			lift.Open();
		} else if (controller.GetRawButton(3)) {
			lift.Close();
		}
		float throttle = (controller.GetThrottle() - 1.0) * -0.5;
		if (controller.GetRawButton(1)) {
			pickupL.Set(-1.0 * throttle, 0);
			pickupR.Set(1.0 * throttle, 0);
		} else if (controller.GetRawButton(2)) {
			pickupL.Set(1.0 * throttle, 0);
			pickupR.Set(-1.0 * throttle, 0);
		} else if (fabs(controller.GetTwist()) > 0.5) {
			pickupL.Set(controller.GetTwist() * throttle, 0);
			pickupR.Set(controller.GetTwist() * throttle, 0);
		} else {
			pickupL.Set(0);
			pickupR.Set(0);
		}
		if (controller.GetRawButton(4)) {
			pickup.Set(DoubleSolenoid::kForward);
		} else if (controller.GetRawButton(6)) {
			pickup.Set(DoubleSolenoid::kReverse);
		}

		if ((controller.GetPOV() == 0) && bounce) {
			lift.nTotes++;
			bounce = false;
		} else if ((controller.GetPOV() == 180) && bounce) {
			if (lift.nTotes > 0) {
				lift.nTotes--;
			}
			bounce = false;
		} else if ((controller.GetPOV() == 270) && bounce) {
			lift.nTotes = 0;
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
		UpdateDashboard();
	}

	void DisabledInit() {
		//leds.Set(Relay::kOff);
		driveTrain.RobotDrive(0.0, 0.0);
	}

	void DisabledPeriodic() {
		UpdateLEDs();
		UpdateDashboard();
	}
	void UpdateLEDs() {
		if (pdp.GetVoltage() > 11) {
			leds.Set(Relay::kForward);
		} else {
			leds.Set(Relay::kOff);
		}
	}
	void UpdateDashboard() {
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
		SmartDashboard::PutNumber("Right Encoder", rightEncoder.GetRaw());
		SmartDashboard::PutNumber("Left Encoder", leftEncoder.GetRaw());
		SmartDashboard::PutNumber("Right Encoder Rate", rightEncoder.GetRate());
		SmartDashboard::PutNumber("Pitch", CalcPitch());
		SmartDashboard::PutNumber("Roll", CalcYaw());
		i = (i + 1) % 2;
	}

	void TestPeriodic() {
		lw->Run();
	}
	double CalcPitch() {
		return atan(-accel.GetX() / accel.GetZ()) * (180 / M_PI);
	}
	double CalcYaw() {
		return atan(accel.GetY() / (sqrt(pow(accel.GetX(), 2)) - pow(accel.GetZ(), 2))) * (180 / M_PI);
	}
};

START_ROBOT_CLASS(Robot);

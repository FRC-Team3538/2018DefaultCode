//2018-RobotCode
#include <iostream>
#include <memory>
#include <string>
#include "AHRS.h"
#include "WPILib.h"
#include "math.h"
#include "Encoder.h"

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>


class Robot: public frc::IterativeRobot {
	DifferentialDrive Adrive;
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	Joystick Drivestick;
	Joystick OperatorStick;
	VictorSP DriveLeft0;
	VictorSP DriveLeft1;
	VictorSP DriveLeft2;
	VictorSP DriveRight0;
	VictorSP DriveRight1;
	VictorSP DriveRight2;
	VictorSP Dpad1;
	VictorSP Dpad2;
	VictorSP RightStick1;
	VictorSP RightStick2;
	Encoder EncoderLeft;
	Encoder EncoderRight;

	double OutputX, OutputY;
	double OutputX1, OutputY1;
	DigitalInput DiIn8, DiIn9;

	AHRS *ahrs;
	//tells us what state we are in in each auto mode
	int modeState, DriveState, TurnState, ScaleState, NearSwitch, AutoSpot;
	bool AutonOverride, AutoDelayActive;
	int isWaiting = 0;			/////***** Divide this into 2 variables.

	// create pdp variable
	PowerDistributionPanel *pdp = new PowerDistributionPanel();

	//Solenoid's declared
	Solenoid *driveSolenoid = new Solenoid(0);
	Solenoid *XYbutton = new Solenoid(4);
	Solenoid *Bbutton = new Solenoid(3);
	Solenoid *Abutton = new Solenoid(2);
	Solenoid *IntakeButton = new Solenoid(1);

	bool useRightEncoder = false;
	bool driveRightTriggerPrev = false;
	bool driveButtonYPrev = false;
	bool operatorRightTriggerPrev = false;
	bool intakeDeployed = false;
	bool XYDeployed = false;
	bool shooterOn = false;

public:
	Robot() :
			Adrive(DriveLeft0, DriveRight0), Drivestick(0), OperatorStick(1), DriveLeft0(
					0), DriveLeft1(1), DriveLeft2(2), DriveRight0(3), DriveRight1(
					4), DriveRight2(5), Dpad1(8), Dpad2(9), RightStick1(6), RightStick2(
					7), EncoderLeft(0, 1), EncoderRight(2, 3), OutputX(0), OutputY(
					0), OutputX1(0), OutputY1(0), DiIn8(8), DiIn9(9), ahrs(
			NULL), modeState(0), DriveState(0), TurnState(0), ScaleState(0), NearSwitch(
					0), AutoSpot(0), AutonOverride(0), AutoDelayActive(0) {

	}

private:
	void RobotInit() {

		//turn off shifter solenoids
		driveSolenoid->Set(false);

		//disable drive watchdogs
		Adrive.SetSafetyEnabled(false);

		//drive command averaging filter
		OutputX = 0, OutputY = 0;

	}

	void TeleopInit() {
		OutputX = 0, OutputY = 0;
	}

	void RobotPeriodic() {
		//links multiple motors together
		DriveLeft1.Set(DriveLeft0.Get());
		DriveLeft2.Set(DriveLeft0.Get());
		DriveRight1.Set(DriveRight0.Get());
		DriveRight2.Set(DriveRight0.Get());

	}

#define caseDriveDefault 1
#define caseDrive1 2
#define caseDrive2 3
#define caseDrive3 4

	void TeleopPeriodic() {

		double Control_Deadband = 0.10;
		double Drive_Deadband = 0.10;
		double DPadSpeed = 1.0;
		bool RightStickLimit1 = DiIn8.Get();
		bool RightStickLimit2 = DiIn9.Get();

		//high gear & low gear controls
		if (Drivestick.GetRawButton(6))
			driveSolenoid->Set(true);			// High gear press LH bumper
		if (Drivestick.GetRawButton(5))
			driveSolenoid->Set(false);			// Low gear press RH bumper

		//  Rumble code
		//  Read all motor current from PDP and display on drivers station
		double driveCurrent = pdp->GetTotalCurrent();	// Get total current

		// rumble if current to high
		double LHThr = 0.0;		// Define value for rumble
		if (driveCurrent > 125.0)// Rumble if greater than 125 amps motor current
			LHThr = 0.5;
		Joystick::RumbleType Vibrate;				// define Vibrate variable
		Vibrate = Joystick::kLeftRumble;		// set Vibrate to Left
		Drivestick.SetRumble(Vibrate, LHThr);  	// Set Left Rumble to RH Trigger
		Vibrate = Joystick::kRightRumble;		// set vibrate to Right
		Drivestick.SetRumble(Vibrate, LHThr);// Set Right Rumble to RH Trigger

		//drive controls
		double SpeedLinear = Drivestick.GetRawAxis(1) * 1; // get Yaxis(Left stick) value (forward)
		double SpeedRotate = Drivestick.GetRawAxis(4) * -1; // get Xaxis (right stick) value (turn)

		SmartDashboard::PutNumber("YJoystick", SpeedLinear);
		SmartDashboard::PutNumber("XJoystick", SpeedRotate);

		// Set dead band for X and Y axis
		if (fabs(SpeedLinear) < Drive_Deadband)
			SpeedLinear = 0.0;
		if (fabs(SpeedRotate) < Drive_Deadband)
			SpeedRotate = 0.0;

		//slow down direction changes from 1 cycle to 5 ( Brown Out Protection)
		OutputY1 = (0.8 * OutputY1) + (0.2 * OutputY);
		OutputX1 = (0.8 * OutputX1) + (0.2 * OutputX);

		/*
		 * MANIP CODE
		 */

		//A Button to extend (Solenoid On)
		Abutton->Set(OperatorStick.GetRawButton(1));

		//B Button to extend (Solenoid On)
		Bbutton->Set(OperatorStick.GetRawButton(2));

		//if 'Y' button pressed, extend (Solenoid On)
		if (OperatorStick.GetRawButton(4)) {
			XYDeployed = true;
			XYbutton->Set(XYDeployed);
		}

		//else 'X' button pressed, retract (Solenoid Off)
		else if (OperatorStick.GetRawButton(3)) {
			XYDeployed = false;
			XYbutton->Set(XYDeployed);
		}

		//if Left Bumper button pressed, extend (Solenoid On)
		if (OperatorStick.GetRawButton(5)) {
			intakeDeployed = true;
			IntakeButton->Set(intakeDeployed);
		}

		//else Right Bumper pressed, retract (Solenoid Off)
		else if (OperatorStick.GetRawButton(6)) {
			intakeDeployed = false;
			IntakeButton->Set(intakeDeployed);
		}

		//dpad POV stuff
		if (OperatorStick.GetPOV(0) == 0) {
			Dpad1.Set(DPadSpeed);
			Dpad2.Set(DPadSpeed);
		} else if (OperatorStick.GetPOV(0) == 180) {
			Dpad1.Set(-DPadSpeed);
			Dpad2.Set(-DPadSpeed);
		} else {
			Dpad1.Set(0.0);
			Dpad2.Set(0.0);
		}

		double LeftSped = OperatorStick.GetRawAxis(1) * 1; // get Yaxis(Left stick) value (forward)
		double RightSpeed = OperatorStick.GetRawAxis(4) * -1; // get Xaxis value for Right Joystick

		if (fabs(LeftSped) < Control_Deadband) {
			LeftSped = 0.0;
		} else if (LeftSped > Control_Deadband and !RightStickLimit1)
			LeftSped = 0.0;
		else if (LeftSped < Control_Deadband and !RightStickLimit2)
			LeftSped = 0.0;

		if (fabs(RightSpeed) < Control_Deadband) {
			RightSpeed = 0.0;
		} else if (RightSpeed > Control_Deadband and !RightStickLimit1)
			RightSpeed = 0.0;
		else if (RightSpeed < Control_Deadband and !RightStickLimit2)
			RightSpeed = 0.0;

	}

	//--------------Start code for motors------------
	//Set left and right motor speeds
	void motorSpeed(double leftMotor, double rightMotor) {
		DriveLeft0.Set(leftMotor * -1);
		DriveLeft1.Set(leftMotor * -1);
		DriveLeft2.Set(leftMotor * -1);
		DriveRight0.Set(rightMotor);
		DriveRight1.Set(rightMotor);
		DriveRight2.Set(rightMotor);
	}

	//Turn off drive motors
	int stopMotors() {
		//sets motor speeds to zero
		motorSpeed(0, 0);
		return 1;
	}
	//--------------End code for motors

private:

}
;

START_ROBOT_CLASS(Robot)

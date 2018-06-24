//Default Code 2018

#include <iostream>
#include <memory>
#include <string>
#include "math.h"
#include <algorithm>

// And So It Begins...
#include "RJ_RobotMap.h"

#define MAIN_LOOP_PERIOD (0.020)
#define DRIVE_MODES (3) //Number of drive modes

class Robot: public frc::TimedRobot {

	// Robot Hardware Setup
	RJ_RobotMap IO;

	// Built-In Drive code for teleop
	DifferentialDrive Adrive { IO.DriveBase.MotorsLeft, IO.DriveBase.MotorsRight };
	MecanumDrive Mdrive { IO.DriveBase.L2, IO.DriveBase.L1, IO.DriveBase.R1, IO.DriveBase.R2 };
	bool gearState; // For power-breaking feature

	// create pdp variable
	PowerDistributionPanel *pdp = new PowerDistributionPanel();

	// Override Elevator lower limit switch, upper limit switch, and elevator encoder
	bool SensorOverride = false;

	// Drive Input Filter
	float OutputX = 0.0, OutputY = 0.0, OutputZ = 0.0;
	int driveMode = 0;
	int motorVel;
	// This number needs to be changed until the wrist reads 0 at top dead center + is toward the front of the robot

	//Autonomous Variables
	Timer AutonTimer, autoSettleTimer, autoTotalTime;
	std::string autoTarget;
	int autoModeState;  // current step in auto sequence
	double autoHeading; // current gyro heading to maintain

	void RobotInit() {
		//disable drive watchdogs
		Adrive.SetSafetyEnabled(false);
		Mdrive.SetSafetyEnabled(false);

		// Reset Encoders
		IO.DriveBase.EncoderLeft.Reset();
		IO.DriveBase.EncoderRight.Reset();

		// Zeros the NavX Yaw
		IO.DriveBase.ahrs.ZeroYaw();

		// 20ms is the default, but lets enforce it.
		this->SetPeriod(MAIN_LOOP_PERIOD);
	}

	void RobotPeriodic() {

		// Update Smart Dash
		SmartDashboardUpdate();
		//IO.NavXDebugger();

		// Get SmartDash Choosers

		autoTarget = IO.DS.chooseAutoProgram.GetSelected();

		// Disable closed loop control and limit switches
		if (IO.DS.OperatorStick.GetStartButton())
			SensorOverride = false;
		if (IO.DS.OperatorStick.GetBackButton())
			SensorOverride = true;

		//Cycle Drive Modes
		bool BtnStart = IO.DS.DriveStick.GetStartButtonPressed();
		if (BtnStart)
			driveMode++;
		if (driveMode == DRIVE_MODES)
			driveMode = 0;
	}

	void DisabledPeriodic() {
		// NOP
	}

	void TeleopInit() {
		// drive command averaging filter
		OutputX = 0, OutputY = 0, OutputZ = 0;

		// Low Gear by default
		IO.DriveBase.SolenoidShifter.Set(true);
		gearState = true;

		// PCM 2 In By Default
		IO.Manip.Sol_AuxA.Set(false);

		// PCM 2 In By Default
		IO.Manip.Sol_AuxB.Set(false);
	}

	bool bTeleAutoMode = false;

	void TeleopPeriodic() {
		double Control_Deadband = 0.11; // input where the joystick actually starts to move
		double Drive_Deadband = 0.11; // command at which the motors begin to move

		// Drive Control Inputs
		double SpeedLinear = IO.DS.DriveStick.GetY(GenericHID::kLeftHand) * 1; // get Yaxis value (forward)
		double SpeedRotate = IO.DS.DriveStick.GetX(GenericHID::kRightHand) * -1; // get Xaxis value (turn)
		double SpeedStrafe = IO.DS.DriveStick.GetX(GenericHID::kLeftHand) * 1; // get Xaxis value (Strafe)

		if(driveMode == 1)
			SpeedRotate = IO.DS.DriveStick.GetY(GenericHID::kRightHand) * 1; // get Xaxis value (turn)



		// Set dead band for control inputs
		SpeedLinear = deadband(SpeedLinear, Control_Deadband);
		SpeedRotate = deadband(SpeedRotate, Control_Deadband);
		SpeedStrafe = deadband(SpeedStrafe, Control_Deadband);

		// Smoothing algorithm for x^3
		if (SpeedLinear > 0.0)
			SpeedLinear = (1 - Drive_Deadband) * pow(SpeedLinear, 3) + Drive_Deadband;
		else if (SpeedLinear < 0.0)
			SpeedLinear = (1 - Drive_Deadband) * pow(SpeedLinear, 3) - Drive_Deadband;
		else
			SpeedLinear = 0.0;  // added for clarity

		// Smoothing algorithm for x^3
		if (SpeedStrafe > 0.0)
			SpeedStrafe = (1 - Drive_Deadband) * pow(SpeedStrafe, 3) + Drive_Deadband;
		else if (SpeedStrafe < 0.0)
			SpeedStrafe = (1 - Drive_Deadband) * pow(SpeedStrafe, 3) - Drive_Deadband;
		else
			SpeedStrafe = 0.0;  // added for clarity

		// Smoothing algorithm for x^3
		if (SpeedRotate > 0.0)
			SpeedRotate = (1 - Drive_Deadband) * pow(SpeedRotate, 3) + Drive_Deadband;
		else if (SpeedRotate < 0.0)
			SpeedRotate = (1 - Drive_Deadband) * pow(SpeedRotate, 3) - Drive_Deadband;
		else
			SpeedRotate = 0.0; // added for clarity

		// Drive Shifter Controls
		if (IO.DS.DriveStick.GetBumper(frc::GenericHID::kRightHand))
			gearState = false; // High

		if (IO.DS.DriveStick.GetBumper(frc::GenericHID::kLeftHand))
			gearState = true; // Low

		IO.DriveBase.SolenoidShifter.Set(gearState);

		// Moving Average Filter (Previous 5 commands are averaged together.)
		llvm::StringRef sDF = "DriveFilter";
		double df = frc::SmartDashboard::GetNumber(sDF, 0.2);
		frc::SmartDashboard::PutNumber(sDF, df);
		frc::SmartDashboard::SetPersistent(sDF);

		OutputY = (df * OutputY) + ((1.0 - df) * SpeedLinear);
		OutputX = (df * OutputX) + ((1.0 - df) * SpeedRotate);
		OutputZ = (df * OutputZ) + ((1.0 - df) * SpeedStrafe);

		// Drive Code (WPI Built-in)
		switch (driveMode) {
		case 0:
			Adrive.ArcadeDrive(OutputY, OutputX, false);
			break;
		case 1:
			Adrive.TankDrive(OutputY, OutputX, false);
			break;
		case 2:
			Mdrive.DriveCartesian(OutputZ, OutputY, OutputX, 0);
			break;
		}

		//  Rumble code
		//  Read all motor current from PDP and display on drivers station
		//double driveCurrent = pdp->GetTotalCurrent();	// Get total current
		double driveCurrent = pdp->GetTotalCurrent();

		// rumble if current to high
		double RbtThr = 0.0;		// Define value for total rumble current
		if (driveCurrent > 175.0)		// Rumble if greater than 125 amps motor current
			RbtThr = 0.0;

		IO.DS.DriveStick.SetRumble(Joystick::kLeftRumble, RbtThr); // Set Left Rumble to RbtThr
		IO.DS.DriveStick.SetRumble(Joystick::kRightRumble, RbtThr);	// Set Right Rumble to RbtThr

		/*
		 * MANIP CODE
		 */
		//Aux Motors A
		double LTrig = IO.DS.DriveStick.GetTriggerAxis(frc::GenericHID::kLeftHand)
				+ IO.DS.OperatorStick.GetTriggerAxis(frc::GenericHID::kLeftHand);
		double RTrig = IO.DS.DriveStick.GetTriggerAxis(frc::GenericHID::kRightHand)
				+ IO.DS.OperatorStick.GetTriggerAxis(frc::GenericHID::kRightHand);
		LTrig = deadband(LTrig, Control_Deadband);
		RTrig = deadband(RTrig, Control_Deadband);
		IO.Manip.MotorsAuxA.Set(LTrig + RTrig);

		//PCM 1
		bool BtnXBool = IO.DS.DriveStick.GetXButton() + IO.DS.OperatorStick.GetXButton();
		IO.Manip.Sol_AuxA.Set(BtnXBool);

		//PCM 2
		bool BtnYBool = IO.DS.DriveStick.GetYButtonPressed() || IO.DS.OperatorStick.GetYButtonPressed();
		bool PCM2Toggled = IO.Manip.Sol_AuxB.Get();
		if (BtnYBool)
			IO.Manip.Sol_AuxB.Set(!PCM2Toggled);
		//Aux Motors B
		bool BtnABool = IO.DS.DriveStick.GetAButtonPressed() || IO.DS.OperatorStick.GetAButtonPressed();
		bool BtnBBool = IO.DS.DriveStick.GetBButtonPressed() || IO.DS.OperatorStick.GetAButtonPressed();
		if (BtnABool)
			IO.Manip.MotorsAuxB.Set(1);
		if (BtnBBool)
			IO.Manip.MotorsAuxB.Set(0);

		//Moar Talon Testing
		motorVel = IO.Manip.Motor.GetSensorCollection().GetQuadratureVelocity(); //Raw Signal Edge Counts
		double OpJoyRY = IO.DS.OperatorStick.GetY(GenericHID::kRightHand);
		OpJoyRY = deadband( OpJoyRY, Control_Deadband);
		IO.Manip.Motor.Set(OpJoyRY);
	}

	void AutonomousInit() {
		autoModeState = 1;

		// Reset Timers
		AutonTimer.Reset();
		AutonTimer.Start();

		autoSettleTimer.Reset();
		autoSettleTimer.Start();

		autoTotalTime.Reset();
		autoTotalTime.Start();

		// Reset Encoders
		IO.DriveBase.EncoderLeft.Reset();
		IO.DriveBase.EncoderRight.Reset();

		// Turn off drive motors
		IO.DriveBase.MotorsLeft.Set(0);
		IO.DriveBase.MotorsRight.Set(0);

		// Reset the navX heading
		IO.DriveBase.ahrs.ZeroYaw();
		autoHeading = 0.0;

		// Low gear by default
		IO.DriveBase.SolenoidShifter.Set(false);

		// Reset the moving average filters for drive base
		OutputY = 0;
		OutputX = 0;
		OutputZ = 0;

	}

	// Reset all the stuff that needs to be reset at each state
	void autoNextState() {
		autoModeState++;

		AutonTimer.Reset();
		autoSettleTimer.Reset();
		IO.DriveBase.EncoderLeft.Reset();
		IO.DriveBase.EncoderRight.Reset();

		stopMotors();

	}

	void AutonomousPeriodic() {

		// Cross the Line Auto
		if (autoTarget == IO.DS.AutoLine) {
			autoLine();
		}

	}

	/*
	 * AUTO PROGRAM - CROSS THE LINE
	 *
	 * Start robot anywhere.
	 *
	 * The robot will simply cross the line.
	 * This is really just a test program...
	 */
	void autoLine(void) {

		switch (autoModeState) {
		case 1:
			if (autoForward(16.0 * 10))
				autoNextState();
			break;

		default:
			stopMotors();
		}

		return;
	}

// Drivetrain functions
//For auto
	void motorSpeed(double leftMotor, double rightMotor) {

		// Moving Average Filter (slip reduction attempt)
		double cycles = 0.2;
		OutputY = (cycles * OutputY) + (1.0 - cycles) * leftMotor;
		OutputX = (cycles * OutputX) + (1.0 - cycles) * rightMotor;

		IO.DriveBase.MotorsLeft.Set(-OutputY);
		IO.DriveBase.MotorsRight.Set(OutputX);
	}

	int stopMotors() {
		//sets motor speeds to zero
		IO.DriveBase.MotorsLeft.Set(0);
		IO.DriveBase.MotorsRight.Set(0);
		OutputY = 0;
		OutputX = 0;
		OutputZ = 0;
		return 1;
	}

	// Go AutoForward autonomously...

#define KP_LINEAR (0.0311)
#define KI_LINEAR (0.000)
#define KD_LINEAR (0.008)
#define LINEAR_TOLERANCE (1.0)

#define KP_ROTATION (0.0609)
#define KI_ROTATION (0.0000)
#define KD_ROTATION (0.0080)

#define ROTATION_TOLERANCE (10.0)
#define ROTATIONAL_SETTLING_TIME (0.0)
#define ROTATIONAL_MAX_SPEED (0.60)

	double prevError_linear = 0; // Derivative Calculation
	double sumError_linear = 0;  // Integral Calculation

	double prevError_rotation = 0; // Derivative Calculation
	double sumError_rotation = 0;  // Integral Calculation

	int autoForward(double targetDistance, double max_speed, double settle_time) {

		double encoderDistance = getEncoderDistance();

		// P Control
		double error = targetDistance - encoderDistance;        // [Inches]

		// I Control
		if (error < 24) {
			sumError_linear += error / MAIN_LOOP_PERIOD;
		} else {
			sumError_linear = 0;
		}

		// D Control
		double dError = (error - prevError_linear) / MAIN_LOOP_PERIOD; // [Inches/second]
		prevError_linear = error;

		// PID Command
		double driveCommandLinear = error * KP_LINEAR + KI_LINEAR * sumError_linear + KD_LINEAR * dError;

		// limit max drive speed
		driveCommandLinear = absMax(driveCommandLinear, max_speed);

		// Min speed
		driveCommandLinear = absMin(driveCommandLinear, 0.11);

		// Use Gyro to drive straight
		double gyroAngle = IO.DriveBase.ahrs.GetAngle();
		double error_rot = gyroAngle - autoHeading;

		// I Control
		if (error_rot < 15) {
			sumError_rotation += error_rot / MAIN_LOOP_PERIOD;
		} else {
			sumError_rotation = 0;
		}

		// D Control
		double dError_rot = (error_rot - prevError_rotation) / MAIN_LOOP_PERIOD; // [Inches/second]
		prevError_rotation = error_rot;

		double driveCommandRotation = error_rot * KP_ROTATION + KI_ROTATION * sumError_rotation
				+ KD_ROTATION * dError_rot;
		driveCommandRotation = absMax(driveCommandRotation, ROTATIONAL_MAX_SPEED);

		// Do iiiiit!
		motorSpeed(driveCommandLinear - driveCommandRotation, driveCommandLinear + driveCommandRotation);

		// Allow robot to come to a stop after reaching target
		if (abs(error) > LINEAR_TOLERANCE) {
			autoSettleTimer.Reset();

		} else if (autoSettleTimer.Get() > settle_time)
			return 1;

		return 0;
	}

	// Overload for backwards compatibility
	int autoForward(double targetDistance) {
		return autoForward(targetDistance, 1.0, 0.0);
	}

	int autoTurn(float targetYaw, double maxSpeed, double settlingTime) {

		// For linear drive function
		autoHeading = -targetYaw;

		// Use Gyro to drive straight
		double gyroAngle = IO.DriveBase.ahrs.GetAngle();
		double error_rot = gyroAngle - autoHeading;

		// I Control
		if (error_rot < 15) {
			sumError_rotation += error_rot / MAIN_LOOP_PERIOD;
		} else {
			sumError_rotation = 0;
		}

		// D Control
		double dError_rot = (error_rot - prevError_rotation) / MAIN_LOOP_PERIOD; // [Inches/second]
		prevError_rotation = error_rot;

		double driveCommandRotation = error_rot * KP_ROTATION + KI_ROTATION * sumError_rotation
				+ KD_ROTATION * dError_rot;
		driveCommandRotation = absMax(driveCommandRotation, maxSpeed);

		// dooo it!
		motorSpeed(-driveCommandRotation, driveCommandRotation);

		// Allow for the robot to settle into position
		if (abs(error_rot) > ROTATION_TOLERANCE)
			autoSettleTimer.Reset();

		else if (autoSettleTimer.Get() > settlingTime)
			return 1;

		return 0;
	}

	int autoTurn(float targetYaw) {
		return autoTurn(targetYaw, ROTATIONAL_MAX_SPEED, ROTATIONAL_SETTLING_TIME);
	}

	int autoTurn() {
		return autoTurn(autoHeading, ROTATIONAL_MAX_SPEED, ROTATIONAL_SETTLING_TIME);
	}

	int timedDrive(double driveTime, double leftMotorSpeed, double rightMotorSpeed) {

		if (AutonTimer.Get() < driveTime) {

			// Use Gyro to drive straight
			double gyroAngle = IO.DriveBase.ahrs.GetAngle();
			double error_rot = gyroAngle - autoHeading;

			// I Control
			if (error_rot < 15) {
				sumError_rotation += error_rot / MAIN_LOOP_PERIOD;
			} else {
				sumError_rotation = 0;
			}

			// D Control
			double dError_rot = (error_rot - prevError_rotation) / MAIN_LOOP_PERIOD; // [Inches/second]
			prevError_rotation = error_rot;

			double driveCommandRotation = error_rot * KP_ROTATION + KI_ROTATION * sumError_rotation
					+ KD_ROTATION * dError_rot;
			driveCommandRotation = absMax(driveCommandRotation, ROTATIONAL_MAX_SPEED);

			motorSpeed(leftMotorSpeed - driveCommandRotation, rightMotorSpeed + driveCommandRotation);

		} else {

			stopMotors();
			return 1;
		}
		return 0;
	}

	// Gets the encoder distance since last reset
	// Algorithm selected by the dashboard chooser
	double getEncoderDistance() {

		// Inches per second-ish... (No encoder mode)
		double encoderDistance = 0;		// = AutonTimer.Get() * 48.0;

		// If an encoder is available, use it...
		double encoderLeft = IO.DriveBase.EncoderLeft.GetDistance();
		double encoderRight = IO.DriveBase.EncoderRight.GetDistance();

		// Automatically select the larger value (assume one was disconnected)
		if (fabs(encoderLeft) > fabs(encoderRight))
			encoderDistance = encoderLeft;
		else
			encoderDistance = encoderRight;

		return encoderDistance;

	}

	double getEncoderRate() {

		// If an encoder is available, use it...
		double encoderLeft = IO.DriveBase.EncoderLeft.GetRate();
		double encoderRight = IO.DriveBase.EncoderRight.GetRate();

		// Automatically select the larger value (assume one was disconnected)
		if (fabs(encoderLeft) > fabs(encoderRight))
			return encoderLeft;
		else
			return encoderRight;

		return 0;
	}

// Dead band function
// Scales output to accommodate for the loss of the deadband region
	double deadband(double input, double minval) {

		// If less than deadband value, return zero
		if (fabs(input) < minval)
			return 0.0;

		// Transform input so that output has full range [0.0 - 1.0]
		if (input < 0.0)
			return absMax(input * (1 - minval) - minval, 1.0);
		else
			return absMax(input * (1 - minval) + minval, 1.0);

	}

	double cubedControl(double input, double minval) {

		if (input == 0.0)
			return 0.0;

		// Smoothing algorithm for x^3
		if (input > 0.0)
			return (1 - minval) * pow(input, 3) + minval;

		else
			return (1 - minval) * pow(input, 3) - minval;

	}

// Limits absolute value of input
	double absMax(double input, double maxval) {

		// Just in case the max is negative.
		maxval = fabs(maxval);

		// If out of bounds, return max
		if (fabs(input) > maxval) {

			if (input > 0.0)
				return maxval;
			else
				return -maxval;
		}

		// Seems good
		return input;
	}

// Limits absolute value of input
	double absMin(double input, double minval) {

		// Just in case the max is negative.
		minval = fabs(minval);

		// If out of bounds, return max
		if (fabs(input) < minval) {

			if (input > 0.0)
				return minval;
			else
				return -minval;
		}

		// Seems good
		return input;
	}

	void SmartDashboardUpdate() {

		// Motor Outputs
		SmartDashboard::PutNumber("Drive Left (PWM)", IO.DriveBase.MotorsLeft.Get());
		SmartDashboard::PutNumber("Drive Right (PWM)", IO.DriveBase.MotorsRight.Get());

		// Drive Joystick Inputs
		SmartDashboard::PutNumber("Speed Linear", IO.DS.DriveStick.GetY(GenericHID::kLeftHand));
		SmartDashboard::PutNumber("Speed Rotate", IO.DS.DriveStick.GetX(GenericHID::kRightHand) * -1);

		// Auto State
		SmartDashboard::PutString(llvm::StringRef("Auto Target"), llvm::StringRef(autoTarget));
		SmartDashboard::PutNumber("Auto State (#)", autoModeState);
		SmartDashboard::PutNumber("Auto Timer (s)", AutonTimer.Get());
		SmartDashboard::PutNumber("Auto Heading", autoHeading);

		// Drive Encoders
		SmartDashboard::PutNumber("Drive Encoder Left (RAW)", IO.DriveBase.EncoderLeft.GetRaw());
		SmartDashboard::PutNumber("Drive Encoder Left (Inches)", IO.DriveBase.EncoderLeft.GetDistance());

		SmartDashboard::PutNumber("Drive Encoder Right (RAW)", IO.DriveBase.EncoderRight.GetRaw());
		SmartDashboard::PutNumber("Drive Encoder Right (Inch)", IO.DriveBase.EncoderRight.GetDistance());

		// Gyro
		if (&IO.DriveBase.ahrs) {
			SmartDashboard::PutNumber("Gyro Angle", IO.DriveBase.ahrs.GetAngle());
		} else {
			SmartDashboard::PutNumber("Gyro Angle", 999);
		}

		// Sensor Override
		SmartDashboard::PutBoolean("Sensor Override", SensorOverride);

		//Drive Mode
		switch (driveMode) {
		case 0:
			SmartDashboard::PutString(llvm::StringRef("Drive Mode"), llvm::StringRef("Split Arcade"));
			break;
		case 1:
			SmartDashboard::PutString(llvm::StringRef("Drive Mode"), llvm::StringRef("Tanky Tank"));
			break;
		case 2:
			SmartDashboard::PutString(llvm::StringRef("Drive Mode"), llvm::StringRef("Mecanum"));
			break;
		}
		SmartDashboard::PutNumber("Encoder Velocity", ((motorVel * 10)/1));
	}
}
;

START_ROBOT_CLASS(Robot);


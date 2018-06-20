#ifndef _RJ_ROBOTMAP_H_
#define _RJ_ROBOTMAP_H_

#include "WPILib.h"
#include "AHRS.h"

class RJ_RobotMap {

public:

	// Driver's Station
	struct structDS {

		XboxController DriveStick { 0 };
		XboxController OperatorStick { 1 };

		SendableChooser<llvm::StringRef> chooseAutoProgram;
		const llvm::StringRef AutoNone = "1 None";
		const llvm::StringRef AutoLine = "2 Line";

	};
	structDS DS;

	// Drive Base
	struct structDriveBase {
		// Left Motors
		VictorSP L1 { 0 };
		VictorSP L2 { 1 };
		VictorSP L3 { 2 };
		SpeedControllerGroup MotorsLeft { L1, L2, L3 };

		// Right Motors
		VictorSP R1 { 3 };
		VictorSP R2 { 4 };
		VictorSP R3 { 5 };
		SpeedControllerGroup MotorsRight { R1, R2, R3 };

		// Drive Base Encoders
		Encoder EncoderLeft { 0, 1 };
		Encoder EncoderRight { 2, 3 };

		// Shifting Solenoid
		Solenoid SolenoidShifter { 0 };

		// NavX MXP board (Gryo)
		//AHRS *ahrs;
		AHRS ahrs { SPI::Port::kMXP, 200 };

	};
	structDriveBase DriveBase;

	// Default Constructor
	RJ_RobotMap();

	// Send all of the NavX data to the SmartDashboard
	void NavXDebugger();

};

#endif

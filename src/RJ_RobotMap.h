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
		SpeedControllerGroup MotorsLeft { L1, L2 };

		// Right Motors
		VictorSP R1 { 2 };
		VictorSP R2 { 3 };
		SpeedControllerGroup MotorsRight { R1, R2 };

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

	struct structManip {
		VictorSP A1 { 5 };
		VictorSP A2 { 6 };
		VictorSP A3 { 7 };
		SpeedControllerGroup MotorsAuxA { A3, A2 };

		VictorSP B1 { 8 };
		SpeedControllerGroup MotorsAuxB { B1 };

		VictorSP C1 { 9 };

		Solenoid Sol_AuxA { 1 };
		Solenoid Sol_AuxB { 2 };

		Encoder EncoderAuxA { 4, 5 };

		DigitalInput LSA_Pos { 6 };
		DigitalInput LSA_Neg { 7 };
	};
	structManip Manip;

	// Default Constructor
	RJ_RobotMap();

	// Send all of the NavX data to the SmartDashboard
	void NavXDebugger();

};

#endif

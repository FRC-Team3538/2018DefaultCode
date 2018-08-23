#ifndef _RJ_ROBOTMAP_H_
#define _RJ_ROBOTMAP_H_

#include "WPILib.h"
#include "AHRS.h"
#include "ctre/Phoenix.h"

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
		VictorSPX L1 { 0 };
		WPI_TalonSRX L2 { 1 };
		VictorSPX L3 { 2 };
		//SpeedControllerGroup MotorsLeft { L1, L2, L3 };

		// Right Motors
		VictorSPX R1 { 3 };
		WPI_TalonSRX R2 { 4 };
		VictorSPX R3 { 5 };

		VictorSPX R4 { 7 };
		WPI_TalonSRX R5 { 6 };
		VictorSPX R6 { 8 };
		VictorSPX R7 { 10 };
		VictorSPX R8 { 9 };
		//SpeedControllerGroup MotorsRight { R1, R2, R3 };

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

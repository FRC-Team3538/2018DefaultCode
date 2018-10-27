#include "RJ_RobotMap.h"

RJ_RobotMap::RJ_RobotMap() {

	//
	// Drive Base
	//

	// Set Motor directions
	DriveBase.L1.SetInverted(false);
	DriveBase.R1.SetInverted(false);

	DriveBase.L2.Follow(DriveBase.L1);
	DriveBase.R2.Follow(DriveBase.R1);

	// Set Encoder Direction & Scale
	DriveBase.L1.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
	DriveBase.L1.ConfigVelocityMeasurementPeriod(VelocityMeasPeriod::Period_25Ms, 0);
	DriveBase.L1.ConfigVelocityMeasurementWindow(32, 0);
	DriveBase.L1.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 3, 100);

	// Encoder Scale Factor

	// Set Default Gear
	DriveBase.SolenoidShifter.Set(false);

	// Manip Stuff
	Manip.EncoderAuxA.SetReverseDirection(false);

	Manip.EncoderAuxA.SetDistancePerPulse(1);

	//Set Manip Motor Directions
	Manip.A1.SetInverted(false);
	Manip.A2.SetInverted(false);
	Manip.B1.SetInverted(false);
	Manip.B2.SetInverted(false);

	//
	// Smart Dashboard
	//

	//Auto Chooser
	DS.chooseAutoProgram.AddObject(DS.AutoLine, DS.AutoLine);
	DS.chooseAutoProgram.AddDefault(DS.AutoNone, DS.AutoNone);
	frc::SmartDashboard::PutData("Auto OBJ", &DS.chooseAutoProgram);


}

void RJ_RobotMap::NavXDebugger() {
	//if ( !DriveBase.ahrs ) return;

	bool reset_yaw_button_pressed = false;
	if (reset_yaw_button_pressed) {
		DriveBase.ahrs.ZeroYaw();
	}

	//Stock Code for navx
	SmartDashboard::PutBoolean("IMU_Connected", DriveBase.ahrs.IsConnected());
	SmartDashboard::PutNumber("IMU_Yaw", DriveBase.ahrs.GetYaw());
	SmartDashboard::PutNumber("IMU_Pitch", DriveBase.ahrs.GetPitch());
	SmartDashboard::PutNumber("IMU_Roll", DriveBase.ahrs.GetRoll());
	SmartDashboard::PutNumber("IMU_CompassHeading", DriveBase.ahrs.GetCompassHeading());
	SmartDashboard::PutNumber("IMU_Update_Count", DriveBase.ahrs.GetUpdateCount());
	SmartDashboard::PutNumber("IMU_Byte_Count", DriveBase.ahrs.GetByteCount());

	/* These functions are compatible w/the WPI Gyro Class */
	SmartDashboard::PutNumber("IMU_TotalYaw", DriveBase.ahrs.GetAngle());
	SmartDashboard::PutNumber("IMU_YawRateDPS", DriveBase.ahrs.GetRate());

	SmartDashboard::PutNumber("IMU_Accel_X", DriveBase.ahrs.GetWorldLinearAccelX());
	SmartDashboard::PutNumber("IMU_Accel_Y", DriveBase.ahrs.GetWorldLinearAccelY());
	SmartDashboard::PutBoolean("IMU_IsMoving", DriveBase.ahrs.IsMoving());
	SmartDashboard::PutNumber("IMU_Temp_C", DriveBase.ahrs.GetTempC());
	SmartDashboard::PutBoolean("IMU_IsCalibrating", DriveBase.ahrs.IsCalibrating());

	SmartDashboard::PutNumber("Velocity_X", DriveBase.ahrs.GetVelocityX());
	SmartDashboard::PutNumber("Velocity_Y", DriveBase.ahrs.GetVelocityY());
	SmartDashboard::PutNumber("Displacement_X", DriveBase.ahrs.GetDisplacementX());
	SmartDashboard::PutNumber("Displacement_Y", DriveBase.ahrs.GetDisplacementY());

	/* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
	/* NOTE:  These values are not normally necessary, but are made available   */
	/* for advanced users.  Before using this data, please consider whether     */
	/* the processed data (see above) will suit your needs.                     */

	SmartDashboard::PutNumber("RawGyro_X", DriveBase.ahrs.GetRawGyroX());
	SmartDashboard::PutNumber("RawGyro_Y", DriveBase.ahrs.GetRawGyroY());
	SmartDashboard::PutNumber("RawGyro_Z", DriveBase.ahrs.GetRawGyroZ());
	SmartDashboard::PutNumber("RawAccel_X", DriveBase.ahrs.GetRawAccelX());
	SmartDashboard::PutNumber("RawAccel_Y", DriveBase.ahrs.GetRawAccelY());
	SmartDashboard::PutNumber("RawAccel_Z", DriveBase.ahrs.GetRawAccelZ());
	SmartDashboard::PutNumber("RawMag_X", DriveBase.ahrs.GetRawMagX());
	SmartDashboard::PutNumber("RawMag_Y", DriveBase.ahrs.GetRawMagY());
	SmartDashboard::PutNumber("RawMag_Z", DriveBase.ahrs.GetRawMagZ());
	SmartDashboard::PutNumber("IMU_Temp_C", DriveBase.ahrs.GetTempC());
	/* Omnimount Yaw Axis Information                                           */
	/* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
	AHRS::BoardYawAxis yaw_axis = DriveBase.ahrs.GetBoardYawAxis();
	SmartDashboard::PutString("YawAxisDirection", yaw_axis.up ? "Up" : "Down");
	SmartDashboard::PutNumber("YawAxis", yaw_axis.board_axis);

	/* Sensor Board Information                                                 */
	SmartDashboard::PutString("FirmwareVersion", DriveBase.ahrs.GetFirmwareVersion());

	/* Quaternion Data                                                          */
	/* Quaternions are fascinating, and are the most compact representation of  */
	/* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
	/* from the Quaternions.  If interested in motion processing, knowledge of  */
	/* Quaternions is highly recommended.                                       */
	SmartDashboard::PutNumber("QuaternionW", DriveBase.ahrs.GetQuaternionW());
	SmartDashboard::PutNumber("QuaternionX", DriveBase.ahrs.GetQuaternionX());
	SmartDashboard::PutNumber("QuaternionY", DriveBase.ahrs.GetQuaternionY());
	SmartDashboard::PutNumber("QuaternionZ", DriveBase.ahrs.GetQuaternionZ());

}


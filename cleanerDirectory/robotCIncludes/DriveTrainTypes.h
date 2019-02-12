#if !defined(_DriveTrainTypes__H_)
#define _DriveTrainTypes__H_

#ifdef __cplusplus
extern "C" {
#endif


typedef enum TGripperArmType
{
	gripperNone,
	gripperClaw,
	gripperSweeper,

	kNumbOfGripperArmTypes
} TGripperArmType;

typedef enum TDriveTrainType
{
	drvTrainNone,
	drvTrain2WD,									// One drive motor on left and right side of robot.
	drvTrain4WD_ConnectedMotors,	// One motor  on each of left and right side with gears connecting front and rear wheels
	drvTrain4WD,									// Two motors on each of left and right side

	drvTrain4WD_Holonomic,				// Omniwheels Motor at each corner skewed 45 degress
	drvTrain4WD_OmniSides,				// Omniwheels Motor at each of four sides of robot (left, right, front, rear
	drvTrain3WD_Kiwi,							// Omniwheels Motor at each corner of triangular drive

	drvTrain4WD_Mecanum,					// Two meccanum wheel motors on each of left and right side

	kNumbOfDriveTrainTypes
} TDriveTrainType;

typedef enum TMotorQuantityConstants
{
	kMaxMotorsInGripperArm		= 4,
	kMaxMotorsInDriveTrain		= 4
} TMotorQuantityConstants;

typedef struct
{
	// NOTE: Ordered largest size fields to shortest for byte packing
	unsigned short nAxleLength;
	unsigned short nGearRatioA;
	unsigned short nGearRatioB;
	unsigned short nWheelBase;
	unsigned short nWheelDiameter;
	unsigned char  nDriveTrainMotorIndex[kMaxMotorsInDriveTrain];
#if defined(_WINDOWS)
	unsigned char nDriveTrainType;
#else
	TDriveTrainType nDriveTrainType;
#endif
} TDriveTrainConfig;


typedef enum TJoystickDriveType
{
	joyDriveNone,
	joyDriveTank,
	joyDriveArcadeLeft,
	joyDriveArcadeRight,

	kNumbOfJoystickDriveTypes
} TJoystickDriveType;

typedef enum TJoystickAxisScaling
{
	axisScaleNone,
	axisScaleLinear,
	axisScaleExponential,
	axisScaleExponentialExtreme,
	kNumbOfJoystickAxisScalingTypes
} TJoystickAxisScaling;

#ifdef __cplusplus
};
#endif

#endif //#if !defined(_DriveTrainTypes__H_)

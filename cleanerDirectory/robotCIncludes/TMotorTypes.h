#ifndef TMotorTypes__H_
#define TMotorTypes__H_


#ifdef __cplusplus
extern "C" {
#endif

///////////////////////////////////////////////////////////////////////////////////////////////
//
//                              Motor Types
//
//////////////////////////////////////////////////////////////////////////////////////////////

typedef enum TMotorTypes
{
	tmotorNone												=  0,
	tmotorAudio,														// Audio speaker connected to a PWM output pin.
	tmotorServoStandard,
	tmotorServoContinuousRotation,					// Servos modified for continuous rotation

	tmotorVexFlashlight,										// Servo output that controls a LED,

	tmotorVariableIntensityLED,							// Single PWM output that controls a LED,

	tmotorVex269_HBridge,										// Connected to internal H-Bridge
	tmotorVex393_HBridge,										// Connected to internal H-Bridge
	tmotorVex393HighSpeed_HBridge,					// Connected to internal H-Bridge
	tmotorVex393TurboSpeed_HBridge,					// Connected to internal H-Bridge

	tmotorVex269_MC29,											// Connected via VEX motor controller 29
	tmotorVex393_MC29,											// Connected via VEX motor controller 29
	tmotorVex393HighSpeed_MC29,							// Connected via VEX motor controller 29
	tmotorVex393TurboSpeed_MC29,						// Connected via VEX motor controller 29

	tmotorVexIQ,														// VEX IQ motors connected via I2C
	tmotorTSTPA,														// TST PA motors connected via I2C
	tmotorAnalogWrite,

	tmotorNXT,															// LEGO's NXT motor
	tmotorEV3_Large,												// LEGO's EV3 motor - large
	tmotorEV3_Medium,												// LEGO's EV3 motor - small
	tmotorTetrix,														// 12V motor for Tetrix
	tmotorMatrix,														// Matrix Robotics motor

	tmotorInternalHBridgeSinglePWM,
	tmotorInternalHBridgeDualPWM,
	tmotorExternalHBridgeSinglePWM,
	tmotorExternalHBridgeDualPWM,

#if defined(_WINDOWS)

	// Following are obsolete types that preceded the use of boolean variables for 'reverse' and 'PID control'.
	// They are kept around for legacy purposes to support older versions of user programs that may use these values
	// in the "Motor and Sensoe Setup" menu.
	//
	tmotorNormalDeprecated,									// LEGO motors -- with (NXT) and without (RCX) integrated encoder

	tmotorNxtEncoderOpenLoop,								// No   PID speed control
	tmotorNxtEncoderClosedLoop,							// With PID speed control
	tmotorReversed,

	tmotor12VEncoderOpenLoop,								// No   PID speed control
	tmotor12VEncoderClosedLoop,							// With PID speed control
#endif

	tMotorNumbOfMotorTypes,

} TMotorTypes;


#if (bSmartMotorsWithEncoders() != 0) || defined(_WINDOWS)

	#if defined(__GNUC__)
		#define wordAligned __attribute__ ((aligned (2)))
	#else
		#define wordAligned
	#endif

	typedef struct wordAligned TPIDConstants
	{
		unsigned short wordAligned P;                   // Current P parameter used within the regulation
		unsigned short wordAligned I;                   // Current I parameter used within the regulation
		unsigned short wordAligned D;                   // Current D parameter used within the regulation
	} TPIDConstants;

	typedef enum TMotorRegulation
	{
		RegIdle						= 0,
		mtrNoReg					= RegIdle,
		RegSpeed					= 1,
		mtrSpeedReg				= RegSpeed,
	#if defined(NXT) || defined(EV3) ||defined(_WINDOWS)
		RegSync						= 2,									// NXT Only
	#endif
		RegPosition				= 3,									// Move to position and hold for about 0.5 second.
		RegPositionHold		= 4,									// Move to position and continuously hold at that spot
		RegSlaved					= 5,
		RegSyncSpeed			= 6,
		RegSyncPosition		= 7
	} TMotorRegulation;


	#if !defined(NXT) || defined(_WINDOWS)

	typedef struct wordAligned TPIDFields
	{
			unsigned char						nPid_Interval;
			unsigned char						nPid_SlewRate;
			unsigned char						nPid_PositionSlop;
			unsigned char						nPid_Deadband;
			unsigned short wordAligned nPid_MaxCountsPerSecond;
			TPIDConstants						nPIDFactors;
		} TPIDFields;


		typedef struct wordAligned TPidDataPersistentDefaultSetting
		{
			unsigned char						nMotorType;
			TPIDFields wordAligned	nConstants;
		} TPidDataPersistentDefaultSetting;
	#endif


	#if defined(NXT) || defined(_WINDOWS)

		typedef struct TPIDFieldsNXT
		{
			TPIDConstants			nPIDFactorsSpeed;							// Constants to use when regulation a constant speed
			TPIDConstants			nPIDFactorsPosition;					// Constants to use when regulation to a target encoder position
		} TPIDFieldsNXT;

	#endif

	#undef wordAligned
#endif


typedef enum TMotorDriveSide
{
	driveNone,
	driveLeft,
	driveRight,
} TMotorDriveSide;

#ifdef __cplusplus
}
#endif

#endif //#ifndef TMotorTypes__H_

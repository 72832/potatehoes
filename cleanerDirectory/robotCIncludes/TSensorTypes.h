#ifndef TSensorTypes__H_
#define TSensorTypes__H_

///////////////////////////////////////////////////////////////////////////////////////////////
//
//                              Sensor Types
//
//////////////////////////////////////////////////////////////////////////////////////////////

typedef enum TSensorTypes
{
	// *********************   Do not change the order of the first few sensors. It is known to the NXT-G software.  ****************

	sensorNone													= 0,   // RCX returns RAW value. NXT turns off sensor.

#if defined(NXT) || defined(_WINDOWS)
	sensorRawValue											= 0,   // RCX returns RAW value. NXT turns off sensor.
#endif

	sensorTouch 												= 1,   // Legacy sensors common to RCX and NXT bricks

#if defined(NXT) || defined(_WINDOWS)
	sensorTemperature										= 2,   // Legacy sensors common to RCX and NXT bricks
#endif

	sensorReflection										= 3,   // Legacy sensors common to RCX and NXT bricks
	sensorRotation											= 4,   // Legacy sensors common to RCX and NXT bricks

#if defined(NXT) || defined(_WINDOWS) || defined(EV3)
	sensorLightActive 									= 5,   // NXT bricks only
	sensorLightInactive 								= 6,   // NXT bricks only
	sensorSoundDB 											= 7,   // NXT bricks only [Microphone Sensor]
	sensorSoundDBA											= 8,   // NXT bricks only
	sensorCustom												= 9,   // NXT bricks only

	sensorI2CCustom 										= 10,		// Low Speed I2C devices. Same index as NXT-G
	sensorI2CCustom9V 									= 11,		// Low Speed I2C devices with 9V. Same index as NXT-G
	sensorHighSpeed 										= 12,

	// LEGO NXT Color Sensor

	sensorColorNxtFULL									= 13,		// Color "lamp" mode -- i.e. output selected color
	sensorColorNxtRED										= 14,		// Color "lamp" mode -- i.e. output selected color
	sensorColorNxtGREEN									= 15,		// Color "lamp" mode -- i.e. output selected color
	sensorColorNxtBLUE									= 16,		// Color "lamp" mode -- i.e. output selected color
	sensorColorNxtNONE									= 17,		// Ambient light mode.
	sensorColorNxtEXIT									= 18,		// For internal use when going from color or Lamp to no_sensor

#endif

	// *********************   Do not change the above order. It is known to the NXT-G software.  ****************

#if defined(NXT) || defined(_WINDOWS)
	//
	// New NXT types
	//
	sensorI2CCustomFast									= 19,		// I2C
	sensorI2CCustomFast9V								= 20,		// I2C
	sensorI2CCustomFastSkipStates9V			= 21,		// I2C
	sensorI2CCustomFastSkipStates				= 22,		// I2C

	sensorActiveRawValue								= 23,		// New RCX type added for 'active powered' third party sensors

	sensorAnalogActive 									= 24,   // NXT bricks only
	sensorAnalogInactive 								= 25,   // NXT bricks only
	sensorI2CMuxController 							= 26,
	sensorI2CMuxControllerFast 					= 27,

	sensorI2CHiTechnicColor							= 28,		// I2C
	sensorI2CHiTechnicGyro 							= 29,
	sensorI2CHiTechnicAccel							= 30,		// I2C
	sensorI2CHiTechnicCompass						= 31,		// I2C
	sensorHiTechnicTouchMux							= 32,		// Not I2C

	sensorI2CMindsensorsCompass					= 33,		// I2C
	sensorI2CMindsensorsAccel						= 34,		// I2C
	sensorI2CMindsensorsDist 						= 35,		// I2C
	sensorI2CodatexRFID									= 36,		// I2C
	sensorHiTechnicIRSeeker600 					= 37,		// I2C
	sensorHiTechnicIRSeeker1200					= 38,		// I2C
	sensorI2CMindsensorsLineFollow 			= 39,		// I2C
#endif

#if defined(NXT) || defined(EV3) || defined(_WINDOWS)
	sensorSONAR													= 40,		// cm results
	#define sensorSONAR9V sensorSONAR
	sensorI2CLegoTempC									= 41,		// I2C
	sensorI2CLegoTempF 									= 42,		// I2C
#endif

#if defined(NXT) || defined(_WINDOWS)
	sensorHiTechnicMagnetic							= 43,		// Same as analaog light sensor
#endif

#if defined(NXT) || defined(EV3) || defined(_WINDOWS)
	sensorMatrixController							= 44,		// Matrix Motor and Servo Controller (I2C)
#endif

	sensorDigitalIn 										= 45,		// New NXT & VEX type

#if (!defined(NXT) && !defined(EV3)) || defined(_WINDOWS)
	sensorGyro 													= 46,
	sensorAccelerometer									= 47,
	sensorAnalog												= 48,
	sensorPotentiometer									= 49,
	sensorLineFollower 									= 50,		// VEX line following sensor
	sensorDigitalOut 										= 51,		// Sets up a digital I/O pin as output
#endif

#if defined(includeSonarSensorSupportTwoPins) || defined(_WINDOWS)
	sensorSONAR_TwoPins_cm							= 52,		// cm results (centimeter)
	sensorSONAR_TwoPins_inch						= 53,	 // SRP-04 Type -- inch results (inches)
	sensorSONAR_TwoPins_mm							= 54,		// SRP-04 Type -- mm results (millimeter)
	sensorSONAR_TwoPins_raw							= 55,		// SRP-04 Type -- cm results (millimeter)
	sensorSONAR_TwoPins_SecondPort			= 56,		// Internal -- used for second port on two pin SONAR sensors.
#endif

#if defined(AVRCPU) || defined(ArduinoCortex) || defined(PIC32) || defined(STM8CPU) || defined(_WINDOWS)
	sensorMotorPWMCtrlPin								= 57,		// I/O Pin is used by Motor Drivers for H-Bridge
	sensorMotorDirCtrlPin								= 58,		// I/O Pin is used by Motor Drivers for H-Bridge
	sensorServoCtrlPin									= 59,		// I/O Pin is used by Motor Drivers for Servos
	sensorUARTCtrlPin										= 60,		// I/O Pin is used by UART
	sensorI2CCtrlPin										= 61,		//
	sensorSPICtrlPin										= 62,		//
	sensorAudioSpeaker									= 63,		//

	sensorInternalButton								= 64,		// I/O Pin is used for a integrated button
	sensorStatusLED											= 65,		// I/O Pin is used for a status LED display
	sensorBatteryMonitor								= 66,		// Analog I/O Pin is used for a status LED display
#endif

#if defined(bHasI2CPort) ||  defined(_WINDOWS)
	sensorQuadEncoderOnI2CPort					= 67,		// Quadrature Encoder connected to I2C Port
	sensorLineFollowArrayOnI2CPort			= 68,		// Array of line following sensors connected to I2C Port
	sensorCustomDaisyChainedI2C					= 69,		// Custom user defined sensor daisy chained on I2C Port
	sensorSpare2OnI2CPort								= 70,		// Spare Type 2 connected to I2C Port
	sensorSpare3OnI2CPort								= 71,		// Spare Type 3 connected to I2C Port
	sensorSpare4OnI2CPort								= 72,		// Spare Type 4 connected to I2C Port
#endif

	sensorVirtualCompass								= 73,
	sensorLEDtoVCC											= 74,
	sensorDigitalHighImpedance 					= 75,		// Sets up a digital I/O pin as high impedance

#if defined(includeSonarSensorSupportOnePin) ||  defined(_WINDOWS)
	sensorSONAR_Ping_cm									= 76,		// Parallax Ping - cm results (centimeters)	[single I/O port]
	sensorSONAR_Ping_inch								= 77,		// Parallax Ping - inch results (inches)			[single I/O port]
	sensorSONAR_Ping_mm									= 78,		// Parallax Ping - cm results (millimeter)		[single I/O port]
	sensorSONAR_Ping_raw								= 79,		// Parallax Ping - cm results (millimeter)		[single I/O port]
#endif
#if defined(includeEncoderSensorSupport) || defined(_WINDOWS)
	sensorQuadEncoder										= 80,
#endif
#if (defined(includeEncoderSensorSupport) && defined(includeSonarSensorSupportTwoPins)) || defined(_WINDOWS)
	sensorQuadEncoderSecondPort					= 81,		// Internal -- used for second port on quadrature sensors.
#endif
#if defined(NXT) || defined(_WINDOWS)
	sensorCustom9V											= 82,
#endif

#if defined(VexIQ) || defined(_WINDOWS)
	sensorVexIQ_Distance								= 83,
	sensorVexIQ_ColorNotActive					= 84,
	sensorVexIQ_ColorHue								= 85,
	sensorVexIQ_ColorGrayscale					= 86,
	sensorVexIQ_Color12Color						= 87,
	sensorVexIQ_Touch										= 88,
	sensorVexIQ_Gyro										= 89,
	sensorVexIQ_LED											= 90,
	sensorVexIQ_Motor										= 91,
#endif

#if defined(EV3) || defined(_WINDOWS)
	sensorEV3_Touch                     = 92,
	sensorEV3_Color                     = 93,
	sensorEV3_Ultrasonic								= 94,
	sensorEV3_Gyro											= 95,
	sensorEV3_IRSensor  								= 96,
	sensorEV3_EnergyMeter               = 97,
	sensorEV3_GenericI2C                = 98,
	sensorEV3_Terminal									= 99,			// LINUX console terminal?
#endif

#if defined(EV3) || defined(_WINDOWS)
	sensorSONAR_Inch										= 100,		// Inch results
#endif

} TSensorTypes;

//#define kLastBuiltinNxtSensor ((TSensorTypes)(sensorHighSpeed - 1))



///////////////////////////////////////////////////////////////////////////////////////////////
//
//                              Sensor Modes
//
//////////////////////////////////////////////////////////////////////////////////////////////

typedef enum TSensorModes
{
	//NXT Sensors (in EV3 Mode)
  modeNone																= 0,

  modeKeepExisting     										= -1,
  LEV3_KeepMode       										= modeKeepExisting,

	modeNXTTouch														= 0,
	modeNXTBump															= 1,

	modeNXTUltrasonic_Cm          					= 0,
	modeNXTUltrasonic_Inch									= 1,

	modeNXTLight_Reflected									= 0,
	modeNXTlight_Ambient										= 1,

	modeNXTSound_DB													= 0,
	modeNXTSound_DBA												= 1,

	modeNXTColor_Reflected									= 0,
	modeNXTColor_Ambient										= 1,
	modeNXTColor_Color     									= 2,
	modeNXTColor_Green 											= 3,
	modeNXTColor_Blue												= 4,
	modeNXTColor_Raw												= 5,

	modeNXTTemperature_C										= 0,
	modeNXTTemperature_F										= 1,

	//EV3 Motors
  modeEV3Large_Motor_Degree								= 0,
  modeEV3Large_Motor_Rotation							= 1,
  modeEV3Large_Motor_Power								= 2,

  modeEV3Medium_Motor_Degree							= 0,
  modeEV3Medium_Motor_Rotation						= 1,
  modeEV3Medium_Motor_Power								= 2,

  //EV3 Sensors
  modeEV3Touch														= 0,
  modeEV3Bump															= 1,

  modeEV3Color_Reflected									= 0,
  modeEV3Color_Ambient										= 1,
  modeEV3Color_Color											= 2,
  modeEV3Color_Reflected_Raw							= 3,
  modeEV3Color_RGB_Raw										= 4,
  modeEV3Color_Calibration								= 5,

  modeEV3Ultrasonic_Cm										= 0,
  modeEV3Ultrasonic_Inch									= 1,
  modeEV3Ultrasonic_Listen								= 2,
  modeEV3Ultrasonic_SI_Cm									= 3,
  modeEV3Ultrasonic_SI_Inch								= 4,
  modeEV3Ultrasonic_DC_Cm									= 5,
  modeEV3Ultrasonic_DC_Inch								= 6,

  modeEV3Gyro_Angle												= 0,
  modeEV3Gyro_Rate												= 1,
  modeEV3Gyro_Fast												= 2,
  modeEV3Gyro_RateAndAngle								= 3,
  modeEV3Gyro_Calibration									= 4,

  modeEV3IR_Proximity											= 0,
  modeEV3IR_Seeker												= 1,
  modeEV3IR_Remote												= 2,
  modeEV3IR_Remote_Advanced								= 3,
  modeEV3IR_Calibration										= 5,

  //EV3 "Energy Meter"
  modeEnergyMeter_Voltage_In							= 0,
  modeEnergyMeter_Amps_In									= 1,
  modeEnergyMeter_Voltage_Out							= 2,
  modeEnergyMeter_Amps_Out								= 3,
  modeEnergyMeter_Joule										= 4,
  modeEnergyMeter_Watts_In								= 5,
  modeEnergyMeter_Watts_Out								= 6,
  modeEnergyMeter_All											= 7,

  // Modes for the I2C sensors
  modeEV3I2C_Normal                       = 0,
  modeEV3I2C_9V                           = 2,

  //Legacy Modes
	modeRaw																	= (unsigned char)  (0 << 5), // 0x00
	modeBoolean       											= (unsigned char)  (1 << 5), // 0x20
	modeEdgeCount     											= (unsigned char)  (2 << 5), // 0x40
	modePulseCount    											= (unsigned char)  (3 << 5), // 0x60
	modePercentage    											= (unsigned char)  (4 << 5), // 0x80
	modeTemperatureC  											= (unsigned char)  (5 << 5), // 0xA0
	modeTemperatureF  											= (unsigned char)  (6 << 5), // 0xC0
	modeRotation      											= (unsigned char)  (7 << 5), // 0xE0
	modeDummy         											= (unsigned char)  0xFF
} TSensorModes;

#endif

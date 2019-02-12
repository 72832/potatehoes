#ifndef opcodeDefinitions__H_
#define opcodeDefinitions__H_

#ifdef __cplusplus
	extern "C" {
#endif


#include "TSensorTypes.h"
#include "TSounds.h"


//
// Do not change the order of the following enum declares. This info is known by assembler
// code and is hard coded.
//
typedef enum TArithOffsets
{
	arithOffsetAddTo          = 0, // Must be consecutive opcodes
	arithOffsetMinusTo        = 1,
	arithOffsetTimesTo        = 2,
	arithOffsetDivideTo       = 3,
	arithOffsetLast
} TArithOffsets;


typedef enum TSystemFunctions
{
	sysFuncHogProcessor 									=  0,  // Hog all processor CPU time. No switching to other runnable tasks
	sysFuncFreeProcessor									=  1,  // Release "Hog all processor CPU time"
	sysFuncSwitchContext									=  2,  // Finish timeslice. Switch to other runnable tasks if available.
	sysFuncMemCpy 												=  3,  // Standard "memcpy"  function
	sysFuncMemSet 												=  4,  // Standard "memset"  function
	sysFuncMemCmp 												=  5,  // Standard "memcmp"  function
#if defined(hasMemMoveSupport) || defined(_WINDOWS)
	sysFuncMemMove 												=  6,  // Standard "memmove"  function
#endif
	sysFuncMemCpyVariableInitialization		=  7,  // Modified "memcpy"  function. First parm is 'call by reference' variable.

#if defined(hasStringSupport) || defined(_WINDOWS)
	sysFuncStrCat 												=  8,  // Standard "strcat"  function
	sysFuncStrNCat												=  9,  // Standard "strncat" function
	sysFuncStrCmp 												= 10,  // Standard "strcmp"  function
	sysFuncStrNCmp												= 11,  // Standard "strncmp" function
	sysFuncStrLen 												= 12,  // Standard "strlen"  function
	sysFuncStrIndex 											= 13,  // String index function
	sysFuncStrSubstring 									= 14,  // String 'substring' function
	sysFuncStrDelete											= 15,  // String 'delete substring' function
	sysFuncStrTrim												= 16,  // Trim string function
	sysFuncStrNTrim 											= 17,  // Trim string function
	sysFuncStrCatSingleChar               = 18,  // Concatenate a single char to end of string
	sysFuncStrNCatSingleChar              = 19,  // Concatenate a single char to end of string
	sysFuncStrCpy 												= 20,  // Standard "strcpy"  function
	sysFuncStrNCpy												= 21,  // Standard "strncpy" function
	sysFuncGetFileName										= 22,
#endif

#if !defined(VEX) || defined(_WINDOWS)
	sysFuncSetSuspendResumeTask 					= 23,  // Set task into suspended state or note
	sysFuncGetTaskState 									= 24,  // Get task state
#endif

#if defined(TETRIX_CAPABLE) || defined(_WINDOWS)
	sysFuncDefineSensorPortHubs 					= 25,
#endif

#if defined(NXT) || defined(_WINDOWS)
	sysFuncDrawNXTMenu										= 26,
	sysFuncValidIcon											= 27,
	sysFuncGetFirmwareDate								= 28,
#endif

#if defined(NXT) || defined(_WINDOWS)
	sysFuncI2CGetSensorInfo 							= 29,  // Nxt only. Gets copy of I2C sensor handler block
	sysFuncI2CClearSensorInfo 						= 30,  // NXT Only. Clears I2C sensor handler block statistics
	sysFuncI2CSensorReply 								= 31,  // Nxt only. Gets copy of last I2C reply data from sensor
	sysFuncI2CSensorConfig								= 32,  // NXT only. Configures data for I2C sensor background polling.
	sysFuncI2CGetDriverState							= 33,  // NXT only. Gets the I2C driver state.
#endif

#if defined(NXT) || defined(EV3) || defined(_WINDOWS)
	sysFuncGetValidColorSensor						= 34,
	sysFuncGetColorSensorValues						= 35,
	sysFuncGetColorSensorCounts						= 36,
#endif

#if defined(NXT) || defined(_WINDOWS)
	sysFuncIOMapRead											= 37,  // Nxt only. Read IOMap.
	sysFuncIOMapWrite											= 38,  // Nxt only. Write IOMap.
	sysFuncGetStandardIconInfo						= 39,  // Nxt only. Get info on standard builtin ICONS
	sysFuncDisplayStandardIcon						= 40,  // Nxt only. Display one of the standard builtin ICONS
#endif

  //UNUSED          											= 41,
	//UNUSED          											= 42,
	//UNUSED          											= 43,
	//UNUSED          											= 44,

	//sysFuncPlaySound											= 45,
	//sysFuncClearSound											= 46,
	//sysFuncPlaySoundFileVariableName			= 47,
	//sysFuncPlaySoundFileEmbeddedFileName	= 48,
	//sysFuncPlayMusicalNote								= 49,


#if defined(hasUserSerialPorts) || defined(_WINDOWS)
	sysFuncConfigureSerialPort						= 50,  // Define configuration type of user accessible UART ports
	sysFuncSerialSetBaudRate							= 51,
	sysFuncSerialGetChar									= 52,
	sysFuncSerialSendChar									= 53,
	sysFuncSerialGetXmitComplete					= 54,

	#if (defined(hasUserSerialPorts) && !defined(VEX)) || defined(_WINDOWS)
		sysFuncSerialGetConfig							= 55,
	#endif

#endif

#if !defined(VEX) || defined(_WINDOWS)
	sysFuncGetTaskPriority								= 56,
	sysFuncSetTaskPriority								= 57,
	sysFuncStartTaskWithPriority					= 58,
#endif

	sysFuncClearTimer											= 59,

#if defined(hasAlphaNumericLCDDisplayViaUART) || defined(_WINDOWS)
	//
	// For Alphanumeric Displays LCD & Button Control
	//
	sysFuncLCDPutNextChar									= 60,  // Display a single char on the next LCD
	sysFuncLCDPutNextShort								= 61,  // Display a short 16-bit variable on the LCD
	sysFuncLCDPutNextConstString					= 62,  // Display a const char string (i.e. stored in opcode) on the LCD
	sysFuncLCDClearLCDLine								= 63,
	sysFuncLCDPutCenteredConstString			= 64,
	sysFuncLCDDisplayProgramName					= 65,
	sysFuncLCDPosition										= 66,
#if defined(hasStringSupport) || defined(_WINDOWS)
	sysFuncLCDPutNextString               = 67,
	sysFuncLCDPutCenteredString           = 68,
#endif
#endif

#if (defined(hasTranscendentalSupport) && defined(useFloats)) || defined(_WINDOWS)
	sysFuncPowerOf												= 69,  // 'pow' power of transcendental function. NOTE: it has two paramters so can't fit in standard transcendental implmentation
	sysFuncAtan2													= 70,  // 'atan2' transcendental function. NOTE: it has two paramters so can't fit in standard transcendental implmentation
#endif

#if defined(hasOptionalUploadSupport) || defined(_WINDOWS)
	sysFuncBulkPropertiesRead 						= 71,
	sysFuncBulkPropertiesWrite 						= 72,
#endif

#if defined(bHasFlashFileSystem) || defined(_WINDOWS)
	sysFuncValidFileIndex									= 73,
#endif

#if defined(bHasGetSensorRawADValue) || defined(_WINDOWS)
	sysFuncGetRawADValue									= 74,
#endif

#if (defined(bHasFlashFileSystem) && !defined(VEX)) || defined(_WINDOWS)
	sysFuncStartupProgramIndex						= 75,
	sysFuncIdleTimeProgramIndex						= 76,
	sysFuncGetFileType										= 77,
#endif

#if defined(bHasFlashFileSystemDelete) || defined(_WINDOWS)
	sysFuncDeleteFile											= 78,
#endif

#if defined(hasJoystickDownloadMessageOpcodes) || defined(_WINDOWS)
	sysFuncGetJoysticks 									= 79,
#endif

#if defined(bHasSemaphoreSupport) || defined(_WINDOWS)
	sysFuncSemaphoreInitialize            = 80,
	sysFuncSemaphoreLock									= 81,
	sysFuncSemaphoreUnlock								= 82,
#endif

#if defined(bHasSoftwareControlledPowerOnOff) || defined(_WINDOWS)
	sysFuncPowerOFF												= 83,
#endif

#if defined(bHasShiftOutOpcode) || defined(_WINDOWS)
	sysFuncShiftIn												= 84,
	sysFuncShiftOut												= 85,
	sysFuncShiftInConstParms							= 86,
	sysFuncShiftOutConstParms							= 87,
#endif

	sysFuncDefineMotorIntrinsic						= 88,

#if defined(bAssociateMotorWithEncoderSensor) || defined(_WINDOWS)
	sysFuncMapEncoderToMotor							= 89,  // Define mapping between motor and associated quadrature encoder (i.e. "write" mapping)
	sysFuncGetEncoderForMotor             = 90,	 // Get sensor port for encoder associated with motor							 (i.e. "read" mapping)
#endif

#if ((defined(bMotorsWithEncoders) || defined(bAssociateMotorWithEncoderSensor)) && defined(bHasEncoderVelocity)) || defined(_WINDOWS)
	sysFuncGetEncoderVelocity							= 91,
#endif

#if ((defined(bMotorsWithEncoders) || defined(bAssociateMotorWithEncoderSensor)) && defined(bHasEncoderTimeStamp)) || defined(_WINDOWS)
	sysFuncGetEncoderAndTimeStampForMotor	= 92,
#endif

#if defined(NXT)  || defined(_WINDOWS)
	sysFuncMotorRampingParms							= 93,
#endif

#if ((bSmartMotorsWithEncoders() != 0) && !(defined(NXT) || defined(EV3))) || defined(_WINDOWS)
	sysFuncMotorMoveToPositionAbsolute		= 94,
	sysFuncMotorMoveToPositionRelative		= 95,
	sysFuncMotorDefinePidSettings					= 96,
	sysFuncMotorResetPidSettingsToDefaults= 97,
	sysFuncMotorSlave											= 98,
  sysFuncMotorResetEncoder              = 99,
  sysFuncMotorGetEncoder                = 100,
  sysFuncMotorPositionComplete          = 101,
	sysFuncMotorGetSlewSpeed							= 102,
	//sysFuncMotorDriveSynchronizedCancel		= xx,
	//sysFuncMotorDriveSynchronizedCancel		= xx,
	//sysFuncMotorDriveStraight							= xx,
	//sysFuncMotorDriveSynchronizedToPosition	= xx,
	//sysFuncMotorDriveStraightToPosition		= xx,
#endif

#if defined(bHasBulkClearMotorFunction) || defined(_WINDOWS)
	sysFuncClearMotors										= 103,				// Function clears a list of motor indices to 'tmotorNone'
#endif

#if defined(bHasBulkClearSensorFunction) || defined(_WINDOWS)
	sysFuncClearSensors										= 104,				// Function clears a list of sensor indices to 'sensorNone'
#endif

#if defined(bHasRandomFunctions) || defined(_WINDOWS)
	sysFuncRandomWord											= 105,
	sysFuncRandomLong											= 106,
	sysFuncSRandWord											= 107,
	sysFuncSRandLong											= 108,
#endif

#if defined(bHasDaisyChainedI2C) || defined(_WINDOWS)
	sysFuncMapSensorToI2CAddress					= 109,
	sysFuncGetSensorToI2CAddress					= 110,
#endif

#if defined(bHasI2CPort) || defined(_WINDOWS)
	sysFuncSendI2cMsg 										= 111,
	sysFuncReadI2cReply 									= 112,
  sysFuncVexCortexI2cStatistics					= 113,
  sysFuncVexCortexI2cErrorPegs					= 114,
#endif

#if !defined(VEX) || defined(_WINDOWS)
	// Used by "Communications Test link" Dialog
	sysFuncCheckTransmitMsgVariableLength = 115,
	sysFuncCheckReceiveMsgVariableLength  = 116,
#endif

#if defined(hasDriveTrainConfiguration) || defined(_WINDOWS)
	//
	// Drive Train Control
	//
	sysFuncConfigureDriveTrainMotors			= 117,
	sysFuncConfigureDriveTrainWheels			= 118,
	sysFuncConfigureGripperArm						= 119,
	sysFuncGetDriveTrainConfig						= 120,
	sysFuncGetGripperArmTypeAndMotors			= 121,
#endif

#if defined(bHasSourceTypeSensorMode) || defined(_WINDOWS)
	sysFuncSetSensorTypeAndModeConstants	= 122,
	sysFuncSetSensorTypeAndModeVariables	= 123,
#endif

	//Unused															= 124

#if defined(hasJoystickAxisConfiguration) || defined(_WINDOWS)
	sysFuncConfigureGameController				= 125,
	sysFuncGetDriveTrainJoystickValues		= 126,
#endif

#if defined(VexIQ) || defined(_WINDOWS)
	sysFuncSetVexIqUserDebugWindow				= 127,
	sysFuncGetVexIqGetDeviceInfo					= 128,
	sysFuncVexIqDrawText									= 129,
	sysFuncVexIqDrawTextLine							= 130,
#endif

#if defined(hasCamera) || defined(_WINDOWS)
	//
	// Camera Sensor Control
	//
	sysFuncCameraFunctions								= 131,
	sysFuncSetCameraConfig								= 132,
	sysFuncGetCameraConfig								= 133,
	sysFuncGetTrackingParms 							= 134,
#endif

#if defined(TETRIX_CAPABLE) || defined(_WINDOWS)
  sysFuncNxtMuxStatistic								= 135,
#endif

  syfFuncLast
} TSystemFunctions;


#define kDefaultTaskPriority 7              // Timeslice scheduler priority for tasks. 0 is lowest. 255 is highest

#if defined(bHasDatalog)

	typedef enum TDatalogionFunctions
	{
		funcDatalogAddPoint					      = 0,
		funcDatalogAddPointWithTimeStamp,
		//funcDatalogDefineDataPointName,
		funcDatalogClear,
		funcDatalogDefineSeries,
		funcDatalogClearSeriesDefinition,
		funcDatalogGroupStart,
		funcDatalogGroupEnd,
		funcDatalogGroupCancel,
		funcDatalogPollingResume,
		funcDatalogPollingPause,
#if defined(bHasDatalogSave) || defined(_WINDOWS)
		funcDatalogSaveNext,
#endif
	} TDatalogionFunctions;

#endif

//////////////////////////////////////////////////////////////////////////////////////////
//
//                          Definition of Firmware Builtin Variable Parameters
//
//////////////////////////////////////////////////////////////////////////////////////////

typedef enum TOffsetConstants
{
	parmOffsetSByte																			= 0,
	parmOffsetUByte																			= 1,
	parmOffsetSShort																		= 2,
	parmOffsetUShort																		= 3,
	parmOffsetSLong																			= 4,
	parmOffsetULong																			= 5,
	parmOffsetFloat																			= 6
} TOffsetConstants;

typedef enum TOpCodeSourceTypes
{
	parmSourceConstant																	= 0x00, //
	parmSourceUnused01																	= 0x01, //

	parmSourceUnused02							 										= 0x02,
	parmSourceExternMemoryRef														= 0x03, // Contains references to 'native' extern memory variables

	// Direct global variable access.
	parmSourceGlobalSByteVar														= 0x04, // Global variables
	parmSourceGlobalUByteVar														= parmSourceGlobalSByteVar + parmOffsetUByte,											// 0x05
	parmSourceGlobalSShortVar														= parmSourceGlobalSByteVar + parmOffsetSShort,										// 0x06
	parmSourceGlobalUShortVar														= parmSourceGlobalSByteVar + parmOffsetUShort,										// 0x07
	parmSourceGlobalSLongVar														= parmSourceGlobalSByteVar + parmOffsetSLong, 										// 0x08
	parmSourceGlobalULongVar														= parmSourceGlobalSByteVar + parmOffsetULong, 										// 0x09
	parmSourceGlobalFloatVar														= parmSourceGlobalSByteVar + parmOffsetFloat,											// 0x0A

	parmSourceIndexedGlobalSByteAndGlobalShortIndexVar	= 0x0B, // indirect global result of constant + globalVar   (index by 1-bytes)
	parmSourceIndexedGlobalUByteAndGlobalShortIndexVar	= parmSourceIndexedGlobalSByteAndGlobalShortIndexVar + parmOffsetUByte, // 0x0C indirect global result of constant + globalVar   (index by 1-bytes)
	parmSourceIndexedGlobalSShortAndGlobalShortIndexVar	= parmSourceIndexedGlobalSByteAndGlobalShortIndexVar + parmOffsetSShort,// 0x0D indirect global result of constant + globalVar   (index by 2-bytes)
	parmSourceIndexedGlobalUShortAndGlobalShortIndexVar	= parmSourceIndexedGlobalSByteAndGlobalShortIndexVar + parmOffsetUShort,// 0x0E indirect global result of constant + globalVar   (index by 2-bytes)
	parmSourceIndexedGlobalSLongAndGlobalShortIndexVar	= parmSourceIndexedGlobalSByteAndGlobalShortIndexVar + parmOffsetSLong,	// 0x0F indirect global result of constant + globalVar*2 (index by 4-bytes)
	parmSourceIndexedGlobalULongAndGlobalShortIndexVar	= parmSourceIndexedGlobalSByteAndGlobalShortIndexVar + parmOffsetULong, // 0x10 indirect global result of constant + globalVar*2 (index by 4-bytes)
	parmSourceIndexedGlobalFloatAndGlobalShortIndexVar	= parmSourceIndexedGlobalSByteAndGlobalShortIndexVar + parmOffsetFloat, // 0x11 indirect global result of constant + globalVar*2 (index by 4-bytes)

	parmSourceIndexedGlobalSByteAndGlobalLongIndexVar		= 0x12, // indirect global result of constant + globalVar   (index by 1-bytes)
	parmSourceIndexedGlobalUByteAndGlobalLongIndexVar		= parmSourceIndexedGlobalSByteAndGlobalLongIndexVar + parmOffsetUByte, // 0x13 indirect global result of constant + globalVar   (index by 1-bytes)
	parmSourceIndexedGlobalSShortAndGlobalLongIndexVar	= parmSourceIndexedGlobalSByteAndGlobalLongIndexVar + parmOffsetSShort,// 0x14 indirect global result of constant + globalVar   (index by 2-bytes)
	parmSourceIndexedGlobalUShortAndGlobalLongIndexVar	= parmSourceIndexedGlobalSByteAndGlobalLongIndexVar + parmOffsetUShort,// 0x15 indirect global result of constant + globalVar   (index by 2-bytes)
	parmSourceIndexedGlobalSLongAndGlobalLongIndexVar		= parmSourceIndexedGlobalSByteAndGlobalLongIndexVar + parmOffsetSLong, // 0x16 indirect global result of constant + globalVar*2 (index by 4-bytes)
	parmSourceIndexedGlobalULongAndGlobalLongIndexVar		= parmSourceIndexedGlobalSByteAndGlobalLongIndexVar + parmOffsetULong, // 0x17 indirect global result of constant + globalVar*2 (index by 4-bytes)
	parmSourceIndexedGlobalFloatAndGlobalLongIndexVar		= parmSourceIndexedGlobalSByteAndGlobalLongIndexVar + parmOffsetFloat, // 0x18 indirect global result of constant + globalVar*2 (index by 4-bytes)

	parmSourceGlobalVarPointerToSByte										= 0x19, // Pointer to a specific variable type
	parmSourceGlobalVarPointerToUByte										= parmSourceGlobalVarPointerToSByte + parmOffsetUByte,						// 0x1A
	parmSourceGlobalVarPointerToSShort									= parmSourceGlobalVarPointerToSByte + parmOffsetSShort,						// 0x1B
	parmSourceGlobalVarPointerToUShort									= parmSourceGlobalVarPointerToSByte + parmOffsetUShort,						// 0x1C
	parmSourceGlobalVarPointerToSLong										= parmSourceGlobalVarPointerToSByte + parmOffsetSLong,						// 0x1D
	parmSourceGlobalVarPointerToULong										= parmSourceGlobalVarPointerToSByte + parmOffsetULong,						// 0x1E
	parmSourceGlobalVarPointerToFloat										= parmSourceGlobalVarPointerToSByte + parmOffsetFloat,						// 0x1F

	parmSourceGlobalVarPointerToSBytePlusOffset					= 0x20, // Pointer (1-byte) and offset (1-byte) to a specific variable type
	parmSourceGlobalVarPointerToUBytePlusOffset					= parmSourceGlobalVarPointerToSBytePlusOffset + parmOffsetUByte,	// 0x21
	parmSourceGlobalVarPointerToSShortPlusOffset				= parmSourceGlobalVarPointerToSBytePlusOffset + parmOffsetSShort,	// 0x22
	parmSourceGlobalVarPointerToUShortPlusOffset				= parmSourceGlobalVarPointerToSBytePlusOffset + parmOffsetUShort,	// 0x23
	parmSourceGlobalVarPointerToSLongPlusOffset					= parmSourceGlobalVarPointerToSBytePlusOffset + parmOffsetSLong,	// 0x24
	parmSourceGlobalVarPointerToULongPlusOffset					= parmSourceGlobalVarPointerToSBytePlusOffset + parmOffsetULong,	// 0x25
	parmSourceGlobalVarPointerToFloatPlusOffset					= parmSourceGlobalVarPointerToSBytePlusOffset + parmOffsetFloat,	// 0x26

	parmSourceStackSByteVar															= 0x27, // Stack variables
	parmSourceStackUByteVar															= parmSourceStackSByteVar + parmOffsetUByte,											// 0x28
	parmSourceStackSShortVar														= parmSourceStackSByteVar + parmOffsetSShort,											// 0x29
	parmSourceStackUShortVar														= parmSourceStackSByteVar + parmOffsetUShort,											// 0x2A
	parmSourceStackSLongVar															= parmSourceStackSByteVar + parmOffsetSLong,											// 0x2B
	parmSourceStackULongVar															= parmSourceStackSByteVar + parmOffsetULong,											// 0x2C
	parmSourceStackFloatVar															= parmSourceStackSByteVar + parmOffsetFloat,											// 0x2D

	parmSourceIndexedStackSByteAndStackShortIndexVar		= 0x2E, // indirect global result of constant + globalVar   (index by 1-bytes)
	parmSourceIndexedStackUByteAndStackShortIndexVar		= parmSourceIndexedStackSByteAndStackShortIndexVar + parmOffsetUByte,  // 0x2F indirect global result of constant + globalVar   (index by 1-bytes)
	parmSourceIndexedStackSShortAndStackShortIndexVar		= parmSourceIndexedStackSByteAndStackShortIndexVar + parmOffsetSShort, // 0x30 indirect global result of constant + globalVar   (index by 2-bytes)
	parmSourceIndexedStackUShortAndStackShortIndexVar		= parmSourceIndexedStackSByteAndStackShortIndexVar + parmOffsetUShort, // 0x31 indirect global result of constant + globalVar   (index by 2-bytes)
	parmSourceIndexedStackSLongAndStackShortIndexVar		= parmSourceIndexedStackSByteAndStackShortIndexVar + parmOffsetSLong,  // 0x32 indirect global result of constant + globalVar*2 (index by 4-bytes)
	parmSourceIndexedStackULongAndStackShortIndexVar		= parmSourceIndexedStackSByteAndStackShortIndexVar + parmOffsetULong,  // 0x33 indirect global result of constant + globalVar*2 (index by 4-bytes)
	parmSourceIndexedStackFloatAndStackShortIndexVar		= parmSourceIndexedStackSByteAndStackShortIndexVar + parmOffsetFloat,  // 0x34 indirect global result of constant + globalVar*2 (index by 4-bytes)

	parmSourceIndexedStackSByteAndStackLongIndexVar			= 0x35, // indirect global result of constant + globalVar   (index by 1-bytes)
	parmSourceIndexedStackUByteAndStackLongIndexVar			= parmSourceIndexedStackSByteAndStackLongIndexVar + parmOffsetUByte,  // 0x36 indirect global result of constant + globalVar   (index by 1-bytes)
	parmSourceIndexedStackSShortAndStackLongIndexVar		= parmSourceIndexedStackSByteAndStackLongIndexVar + parmOffsetSShort, // 0x37 indirect global result of constant + globalVar   (index by 2-bytes)
	parmSourceIndexedStackUShortAndStackLongIndexVar		= parmSourceIndexedStackSByteAndStackLongIndexVar + parmOffsetUShort, // 0x38 indirect global result of constant + globalVar   (index by 2-bytes)
	parmSourceIndexedStackSLongAndStackLongIndexVar			= parmSourceIndexedStackSByteAndStackLongIndexVar + parmOffsetSLong,  // 0x39 indirect global result of constant + globalVar*2 (index by 4-bytes)
	parmSourceIndexedStackULongAndStackLongIndexVar			= parmSourceIndexedStackSByteAndStackLongIndexVar + parmOffsetULong,  // 0x3A indirect global result of constant + globalVar*2 (index by 4-bytes)
	parmSourceIndexedStackFloatAndStackLongIndexVar			= parmSourceIndexedStackSByteAndStackLongIndexVar + parmOffsetFloat,  // 0x3B indirect global result of constant + globalVar*2 (index by 4-bytes)

	parmSourceStackVarPointerToSByte										= 0x3C, // Pointer to a specific variable type
	parmSourceStackVarPointerToUByte										= parmSourceStackVarPointerToSByte + parmOffsetUByte,  					// 0x3D
	parmSourceStackVarPointerToSShort										= parmSourceStackVarPointerToSByte + parmOffsetSShort, 					// 0x3E
	parmSourceStackVarPointerToUShort										= parmSourceStackVarPointerToSByte + parmOffsetUShort, 					// 0x3F
	parmSourceStackVarPointerToSLong										= parmSourceStackVarPointerToSByte + parmOffsetSLong,  					// 0x40
	parmSourceStackVarPointerToULong										= parmSourceStackVarPointerToSByte + parmOffsetULong,  					// 0x41
	parmSourceStackVarPointerToFloat										= parmSourceStackVarPointerToSByte + parmOffsetFloat,  					// 0x42

	parmSourceStackVarPointerToSBytePlusOffset					= 0x43, // Pointer (1-byte) and offset (1-byte) to a specific variable type
	parmSourceStackVarPointerToUBytePlusOffset					= parmSourceStackVarPointerToSBytePlusOffset + parmOffsetUByte,  // 0x44
	parmSourceStackVarPointerToSShortPlusOffset					= parmSourceStackVarPointerToSBytePlusOffset + parmOffsetSShort, // 0x45
	parmSourceStackVarPointerToUShortPlusOffset					= parmSourceStackVarPointerToSBytePlusOffset + parmOffsetUShort, // 0x46
	parmSourceStackVarPointerToSLongPlusOffset					= parmSourceStackVarPointerToSBytePlusOffset + parmOffsetSLong,  // 0x47
	parmSourceStackVarPointerToULongPlusOffset					= parmSourceStackVarPointerToSBytePlusOffset + parmOffsetULong,  // 0x48
	parmSourceStackVarPointerToFloatPlusOffset					= parmSourceStackVarPointerToSBytePlusOffset + parmOffsetFloat,  // 0x49

	parmSourceFunctionReturnValueSByte									= 0x4A,
	parmSourceFunctionReturnValueUByte									= parmSourceFunctionReturnValueSByte + parmOffsetUByte,						// 0x4B
	parmSourceFunctionReturnValueSShort									= parmSourceFunctionReturnValueSByte + parmOffsetSShort,  				// 0x4C
	parmSourceFunctionReturnValueUShort									= parmSourceFunctionReturnValueSByte + parmOffsetUShort,  				// 0x4D
	parmSourceFunctionReturnValueSLong									= parmSourceFunctionReturnValueSByte + parmOffsetSLong,						// 0x4E
	parmSourceFunctionReturnValueULong									= parmSourceFunctionReturnValueSByte + parmOffsetULong,						// 0x4F
	parmSourceFunctionReturnValueFloat									= parmSourceFunctionReturnValueSByte + parmOffsetFloat,						// 0x50

	parmSourceConstantVarSByte													= 0x51,
	parmSourceConstantVarUByte													= parmSourceConstantVarSByte + parmOffsetUByte,										// 0x52
	parmSourceConstantVarSShort													= parmSourceConstantVarSByte + parmOffsetSShort,  								// 0x53
	parmSourceConstantVarUShort													= parmSourceConstantVarSByte + parmOffsetUShort,  								// 0x54
	parmSourceConstantVarSLong													= parmSourceConstantVarSByte + parmOffsetSLong,										// 0x55
	parmSourceConstantVarULong													= parmSourceConstantVarSByte + parmOffsetULong,										// 0x56
	parmSourceConstantVarFloat													= parmSourceConstantVarSByte + parmOffsetFloat,										// 0x57

	parmSourceIndexedGlobalSByteAndStackShortIndexVar		= 0x58,
	parmSourceIndexedGlobalUByteAndStackShortIndexVar		= parmSourceIndexedGlobalSByteAndStackShortIndexVar + parmOffsetUByte,	// 0x59
	parmSourceIndexedGlobalSShortAndStackShortIndexVar	= parmSourceIndexedGlobalSByteAndStackShortIndexVar + parmOffsetSShort,	// 0x5A
	parmSourceIndexedGlobalUShortAndStackShortIndexVar	= parmSourceIndexedGlobalSByteAndStackShortIndexVar + parmOffsetUShort,	// 0x5B
	parmSourceIndexedGlobalSLongAndStackShortIndexVar		= parmSourceIndexedGlobalSByteAndStackShortIndexVar + parmOffsetSLong,	// 0x5C
	parmSourceIndexedGlobalULongAndStackShortIndexVar		= parmSourceIndexedGlobalSByteAndStackShortIndexVar + parmOffsetULong,	// 0x5D
	parmSourceIndexedGlobalFloatAndStackShortIndexVar		= parmSourceIndexedGlobalSByteAndStackShortIndexVar + parmOffsetFloat,	// 0x5E

	parmSourceIndexedGlobalSByteAndStackLongIndexVar		= 0x5F,
	parmSourceIndexedGlobalUByteAndStackLongIndexVar		= parmSourceIndexedGlobalSByteAndStackLongIndexVar + parmOffsetUByte,		// 0x60
	parmSourceIndexedGlobalSShortAndStackLongIndexVar		= parmSourceIndexedGlobalSByteAndStackLongIndexVar + parmOffsetSShort,	// 0x61
	parmSourceIndexedGlobalUShortAndStackLongIndexVar		= parmSourceIndexedGlobalSByteAndStackLongIndexVar + parmOffsetUShort,	// 0x62
	parmSourceIndexedGlobalSLongAndStackLongIndexVar		= parmSourceIndexedGlobalSByteAndStackLongIndexVar + parmOffsetSLong,		// 0x63
	parmSourceIndexedGlobalULongAndStackLongIndexVar		= parmSourceIndexedGlobalSByteAndStackLongIndexVar + parmOffsetULong,		// 0x64
	parmSourceIndexedGlobalFloatAndStackLongIndexVar		= parmSourceIndexedGlobalSByteAndStackLongIndexVar + parmOffsetFloat,		// 0x65

	// end of consecutive "memory" parms

	//
	//////////////////////////////////////////////////////////
	//
	parmSourceBad																					,
	parmSourceMax																				= parmSourceBad

	//
	// Max value is 22 				=    				= 0x3F (6 bits to fit in filed of conditional branch instruction
	//
#if defined(_WINDOWS)
	,
	parmSourceUndefined																	= 0xFF,

#endif
} TOpCodeSourceTypes;

#define parmSourceBadUndefined						((TOpCodeSourceTypes)(unsigned char) 0xFF)


#if defined(ARMCPU)
	typedef unsigned int TOpCodeSourceParmTypes;
#else
	typedef TOpCodeSourceTypes TOpCodeSourceParmTypes;
#endif


typedef enum TSystemParmTypes
{
	//
	// Read only parameters to interrogate operating system
	//
	kSystemFirmwareVersion                      =   0,    // Do not change order. Want consistentcy across different versions
	kSystemPlatformType													=   1,    // Do not change order. Want consistentcy across different versions

	kSystemStartOfFlashFileHeader								=   2,
	kSystemDebugTaskMode                        =   3,

	kSystemProgramNumb                          =   4,
	kSystemCurrentTask                          =   5,
#if defined(bHasReadBatteryVoltage) || defined(_WINDOWS)
	kSystemImmediateBatteryLevel                =   6,
#endif

	//
	// Customize operating system behaviour
	//
	kSystemOpcodesPerTimeslice                  =   7,
	kSystemClockMinutes                         =   8,

#if defined(hasLCDBacklight) || defined(_WINDOWS)
	kSystemLCDBacklight                         =   9,
#endif

#if defined(hasLCDButtons) || defined(_WINDOWS)
	kSystemLCDButtons                           =  10,
#endif

#if defined(hasAlphaNumericLCDDisplayViaUART) || defined(_WINDOWS)
	kSystemLCDRefreshRate                       =  11,
#endif

#if defined(AVRCPU) || defined(_WINDOWS)
	kSystemLCDRefreshCycles                     =  12,
#endif

	kSystemStartOfGlobalVariables								=  13,

#if  defined(bHasFlashFileSystem) || defined(_WINDOWS)
	kSystemStartOfFlashFileSystem								=  14,
#endif

#if defined(bHasFastSonarSensor) || defined(_WINDOWS)
	kSystemFastSonarSensor                      =  15,
#endif

	kSystemSysTime                              =  16,
	kSystemTaskStackAddress                     =  17,   // start memory location of the array of task stacks
	kSystemPgmTime                              =  18,
	kSystemTaskSchedulingPriority               =  19,
	kSystemClearVariablesOnPgmStart							=  20,

	// Unused																		=  21,
	// Unused																		=  22,
	// Unused																		=  23,
	// Unused																		=  24,
	// Unused																		=  25,

#if defined(bHasSoftwareControlledPowerOnOffTiming) || defined(_WINDOWS)
	kSystemPowerDownDelay                       =  26,
	kSystemPowerDownDelayDefault                =  27,
#endif

#if defined(useFloats) || defined(_WINDOWS)
	kSystemFloatConversionException							=  28,
#endif

	//
	// Sound Parameters
	//
#if (bHasSoundVolume != 0) || defined(_WINDOWS)
	kSystemVolume                               =  29,  // Volume control. Applies to PlayNote opcode only..Valid for a "session"
#endif

#if (bHasSoundSupport != 0) || defined(_WINDOWS)
	kSystemBOOLSoundPlaying                     =  30,  // Bool variable to indicate if a sound is currently playing
	kSystemPlaySounds                           =  31,
	kSystemSoundQueueHasFreeEntries             =  32,  // Able to queue the next sound request
#endif

#if defined(bHasRemoteButtons) || defined(_WINDOWS)
	kSystemRemoteButton                         =  33,
#endif

#if (defined(bHasStandardBatteryMonitoring) && defined(bHasSoftwareControlledPowerOnOff) && !defined(NXT)) || defined(_WINDOWS)
	kSystemShutdownVoltage                      =  34,
#endif

#if defined(bHasStandardBatteryMonitoring) || defined(_WINDOWS)
	kSystemAvgBatteryLevel                      =  35,
#endif

#if defined(bHasLCDContrast) || defined(_WINDOWS)
	kSystemLCDDefaultContrast                   =  36,  // Value stored in non-volatile memory and used on power up.
	kSystemLCDContrast													=  37,  // Contrast level for the LCD
#endif

#if  defined(bHasFloatDuringInactivePWM) || defined(_WINDOWS)
	kSystemFloatDuringInactiveMotorPWM          =  39,
#endif

	kSystemSendVariableLengthTestMessage        =  39,
	kSystemReplyVariableLengthTestMessage       =  40,

#if defined(VEX2) || defined(VexIQ) || defined(_WINDOWS)
	kSystemCurrCortexProgramType								=  41,
	kSystemNextCortexProgramType								=  42,
#endif

#if ((bHasSoundSupport != 0) && (bHasSoundDriverOnOff != 0)) || defined(_WINDOWS)
	kSystemSoundDriverOnOff                     =  43,
#endif

#if defined(VEX) || defined(_WINDOWS)
	kSystemIFIPwmOwner                          =  44,
	kSystemVEXAnalogChannelCount                =  45,
	kSystemIFIUserCmd                           =  46,
	kSystemVEXDurationAutonomous                =  47,
	kSystemVEXDurationUserControl               =  48,
#endif

#if defined(VEX) || defined(VEX2) || defined(_WINDOWS)
	kSystemIFIPWMOverride                       =  49,		// Behaviour when transmitter or USB Cable (VEX2) is not-connected
	kSystemVEXRCReceiveState                    =  50,  //  Bit map of active transmitters
	kSystemIFIMasterVersion                     =  51,
	kSystemBackupBatteryLevel                   =  52,
	kSystemVEXNEXTActive                        =  53,
	kSystemIFISPIMsgCounts                      =  54,
	kSystemIFIRobotDisabled                     =  55,
	kSystemIFIAutonomousMode                    =  56,
	kSystemIFIAutonomousModeBySlave             =  57,
#endif

#if (defined(bHasI2CPort) && defined(VEX2)) || defined(_WINDOWS)
	kSystemIncludeVelocityInVexCortexI2C				=  58,
#endif

#if defined(bHasDatalog) || defined(_WINDOWS)
	kSystemDatalogBufferSize                    =  59, // Number of entries in datalog
	kSystemDatalogStatus      									=  60, // status and number of series
	kSystemNumbOfDatalogBytesAvailToRead				=  61,
	kSystemNumbOfDatalogResets									=  62,
	kSystemNumbOfDatalogOverruns								=  63,
#endif

#if defined(NXT) || defined(EV3) || defined(_WINDOWS)
	kSystemNxtSynchMotors                       =  64,  // Bitmap of the syncrhonized motors on the NXT.
	kSystemNxtSynchSlaveSpeedRatio              =  65,  // Bitmap of the syncrhonized motors on the NXT.
	kSystemColorSensorWhiteThreshold            =  66,  // R,G,B thresholds for detecting "white" color with LEGO color sensor
	kSystemNxtI2CCheckACKs                      =  67,  // Check all I2C ACKs or only first ACK in message
	kSystemNxtI2CRetries                        =  68,  // Number of attempts to be made to send a I2c sensor message
#endif

#if defined(NXT) || defined(_WINDOWS)
	kSystemAvgBackgroundTime                    =  69,
	kSystemAvgInterpreterTime                   =  70,

	kSystemBTCurrentStreamIndex                 =  71,
	kSystemBluetoothStatus                      =  72,
	kSystemBTSkipPasswordPrompt                 =  73,  // A Boolean variable to indicate whether BT should use default pswd without prompting
	kSystemBluetoothMode                        =  74,
	kSystemBluetoothCmdStatus                   =  75,  // Returns the status of the last bluetooth command
	kSystemBTOptimizePerformance								=  76,  // Optimize Bluetooth performance
	kSystemBTStreamSwitchDelay                  =  77,  // Delay when switching from DATA mode to CMD mode to allow buffers to fully transmit.
	kSystemBluetoothBusy                        =  78,
	kSystemBluetoothDebug                       =  79,
	kSystemBTHasProgressSounds                  =  80,
	kSystemBluetoothLastCmd                     =  81,
	kSystemBTVisible                            =  82,  // A Boolean variable to indicate whether BT is enabled
	kSystemBTEnabled                            =  83,  // A Boolean variable to indicate whether BT is visible

	kSystemNxtLCDStatusDisplay                  =  84,

	kSystemMaxRegulatedSpeedCountsPerSecondNxt  =  85,  // Used by NXT PID speed regulation on NXT motors.
	kSystemPidUpdateIntervalInMsecsNxt          =  86,  // Update interval used by NXT PID speed regulation on motors.

	kSystemHS_Mode                              =  87,  // High Speed RS-485 Mode
	kSystemHS_Status                            =  88,  // High Speed RS-485 Status (R/O)
	kSystemNxtButtonTransition                  =  89,  // Reports button pressed transitions. Only report once
	kSystemVolumeDefault                        =  90,  // Volume control. Applies to PlayNote opcode only..Valid for a "session"
	kSystemTraceInternalBluetooth               =  91,  // Traces internal Bluetooth Activity
	kSystemTraceInternalHighSpeed               =  92,  // Traces internal High Speed Link Activity
	kSystemTraceInternalDirectCommands          =  93,  // Traces internal Fantom Direct Commands Activity
	kSystemTraceInternalOpcodeMessages          =  94,  // Traces internal VM Opcode Messages Activity
	kSystemNXTGMessagingCompatability           =  95,
	kSystemNoPowerDownOnACAdaptor               =  96,
	kSystemUltrasonicFilterCount                =  97,
	kSystemNxtAvailFlash                        =  98,  // Amount of unused flash (in 100s of bytes) in the NXT file system.
	kSystemNxtButtonTask                        =  99,  // This selects task to run when a button is pushed
	kSystemNxtExitClicks                        = 100,  // When 'nNxtButtonTask' is not -1, defines the number of 'exit' button clicks to terminate program

	kSystemNxtHideDataFile                      = 101,  // A Boolean variable to indicate whether DATA files should be hidden on the NXT GUI
	kSystemNxtRechargable                       = 102,  // A Boolean variable to indicate whether rechargeable battery is used on NXT
	kSystemNxtButtonPressed                     = 103,  // This contains the index of the currently pressed button  (0..3). No button is 0xFF.
	kSystemNxtMaxDataFileCount                  = 104,  // Maximum number of "dataNNNN.rdt" files on the NXT
	kSystemNxtMaxDataFileSize                   = 105,  // Maximum size (in K bytes) for all "dataNNNN.rdt" file storage on the NXT
	kSystemMotorBeingTraced                     = 106,
	kSystemLowVoltageBatteryCountLimits         = 107,
	kSystemVirtualMotorChanges                  = 108,
	kSystemVirtualSensorTypeChanges             = 109,
	kSystemVirtualSensorModeChanges             = 110,
#endif

#if defined(TETRIX_CAPABLE) || defined(_WINDOWS)
	kSystemAvgBatteryLevelExternal              = 111,  // External battery voltage for MUX controllers -- average value
	kSystemBatteryLevelExternal                 = 112,  // External battery voltage for MUX controllers -- immediate value
	kSystemLeaveServosEnabledOnProgramStop			= 113,
	kSystemPidUpdateIntervalInMsecs12V          = 114,  // Update interval used by NXT PID speed regulation on motors.
	kSystemFtcHubUpdateInterval                 = 115,  // Minimum update interval (in msec) for servo/motor controller
	kSystemFtcServoUseFastI2C                   = 116,  // Use ROBOTC fast I2C messaging for servo controller
	kSystemMaxRegulatedSpeedCountsPerSecond12V  = 117,  // Used by NXT PID speed regulation on 12V motors.
	kSystemBatteryShutdownExternal              = 118,  // Low voltage battery shutdown threshold for servo controller
#endif

#if defined(bHasI2CPort) || defined(_WINDOWS)
	kSystemNumbI2CBytesReady                    = 119,
	kSystemI2CStatus                            = 120,
#endif


#if defined(hasWatchDogTimerFunctions) || defined(_WINDOWS)
	kSystemWatchdogTimeoutInterval							= 121,
	kSystemResetFromWatchdogTimeout							= 122,
#endif

#if defined(bHasMessageStatistics) || defined(_WINDOWS)
	//
	// System Messaging Error Statistics
	//
	kSystemTotalMessagesRcvd										= 123,
	kSystemTotalPartialMessagesRcvd							= 124,
	kSystemTotalCharsRcv												= 125,
	kSystemTotalMessagesSent										= 126,
	kSystemTotalInterCharTimeouts								= 127,
	kSystemTotalBadChars												= 128,
	kSystemTotalDroppedMessagesSent							= 129,
#endif //#if defined(bHasMessageStatistics)

#if defined(hasLCDMenuFunctions)
	kSystemUserWantsExitButton									= 130,
#endif

#if defined(VexIQ) || defined(_WINDOWS)
	kSystemVexIQSchedulerSliceSize							= 131,
	kSystemVexIQActiveProgramSlot								= 132,

	kSystemVexIQUserTaskCycles									= 133,
	kSystemVexIQInitTaskCycles									= 134,
	kSystemVexIQIdleTaskCycles									= 135,
	kSystemVexIQRawTime													= 136,
	kSystemVexIQSystemTime											= 137,
	kSystemVexIQUserTaskTime										= 138,
	kSystemVexIQUserControlsLCDButtons					= 139,
	kSystemVexIQUserTaskStartTick								= 140,
	kSystemVexIQUserTaskCurrTimeSliceTotalTicks	= 141,
	kSystemVexIQUserTaskCurrTimeSliceIncrementalTicks	= 142,
	kSystemVexIQChooserState										= 143,
	kSystemVexIQI2CSafeSendMessageTime					= 144,
	kSystemVexIQI2CForceInlineWait							= 145,
	kSystemVexIQUseSmartRadioFeatures						= 146,
	kSystemVexIQSupportsSmartRadio							= 147,
#endif

	kSystemLastParmIndex																				// Gets the next sequential address
} TSystemParmTypes;

#define kSizeOfSystemParameters (kSystemLastParmIndex)  // total size for arrays

typedef enum TSystemExternParms
{
#if defined(NXT) || defined(_WINDOWS)
	kExternUiIOMAPAddress                       =  0,
	kExternBluetoothContactTableAddress         =  1,
	kExternBluetoothConnectionTableAddress      =  2,
	kExternSensorIOMAPAddress                   =  3,
	kExternMotorIOMAPAddress                    =  4,
	kExternBrickData                            =  5,
#endif

#if defined(VEX) || defined(VEX2) || defined(_WINDOWS)
	kExternIFITxData                            =  6,
	kExternIFIRxData                            =  7,
#endif

#if defined(hasCamera) || defined(_WINDOWS)
	kExternCMUCAMTracking                       =  8,
#endif

	kExternLastExternIndex                  // Gets the next sequential address
} TSystemExternParms;

#if defined(VexIQ) || defined(_WINDOWS)

typedef enum TVexIqOpcodeFunctions
{
	vexIqFcn_getI2CStatus															= 0,
	vexIqFcn_StartI2CDeviceBytesWrite,
	vexIqFcn_StartI2CDeviceBytesRead,
	vexIqFcn_StoreI2CDeviceBytesReadFromPortBuffer,

	vexIqFcn_BumperSwitchValue,

	vexIqFcn_LedCDevicePressed,
	vexIqFcn_LedCDeviceSetColor,
	vexIqFcn_LedCDeviceSet12Color,
	vexIqFcn_LedCDeviceSetHue,
	vexIqFcn_LedCDeviceSetBrightness,
	vexIqFcn_LedCDeviceSetFade,
	vexIqFcn_LedCDeviceSetBlinkMode,
	vexIqFcn_LedCDeviceGetRed,
	vexIqFcn_LedCDeviceGetGreen,
	vexIqFcn_LedCDeviceGetBlue,

	vexIqFcn_GyroZeroAngle,
	vexIqFcn_GyroGetTotalDegrees,
	vexIqFcn_GyroGetDegrees,
	vexIqFcn_GyroGetRate,
	vexIqFcn_GyroGetRange,
	vexIqFcn_GyroSetRange,
	vexIqFcn_GyroGetTotalDegreesFloat,
	vexIqFcn_GyroGetDegreesFloat,
	vexIqFcn_GyroGetRateFloat,
	vexIqFcn_GyroStartCalibration,
	vexIqFcn_GyroReadCalibrationFlag,
	vexIqFcn_GyroReadOverRangeFlag,

	vexIqFcn_GetDistanceValue,
	vexIqFcn_GetDistanceStrongest,
	vexIqFcn_GetDistanceSecondStrongest,
	vexIqFcn_GetDistanceMostReflective,
	vexIqFcn_GetDistanceMinRange,
	vexIqFcn_GetDistanceMaxRange,
	vexIqFcn_GetDistanceAdvanced,
	vexIqFcn_SetDistanceSNRThreshold,
	vexIqFcn_SetDistanceBrightnessThreshold,
	vexIqFcn_SetDistanceFilterFactor,
	vexIqFcn_SetDistanceTransmitPower,
	vexIqFcn_SetDistanceMinRange,
	vexIqFcn_SetDistanceMaxRange,
	vexIqFcn_SetDistanceObjectFilterMode,

	vexIqFcn_ColorSetType,  			//Set the Color Type
	vexIqFcn_ColorGetType,				//Get the current Color Type
	vexIqFcn_ColorGetProximityRawValue,
	vexIqFcn_ColorGetProximityEventOccurance, //Get the value of the proximity detector
	vexIqFcn_ColorGetProximityEventValue,			//Get the Boolean Value of the Proximity Sensor
	vexIqFcn_ColorSetProximityEventValue, 		//Set the distance to trigger the flag for the proximity detector
	vexIqFcn_GetColorValue,				//Get the current "Value" - This is dependent on the "type"
	vexIqFcn_GetColorRaw,					//Return the raw 32-bit color
	vexIqFcn_GetColorHue,					//Return the "hue" 8-bit color value
	vexIqFcn_GetColorRawRed,			//Get the raw value of Red
	vexIqFcn_GetColorRawGreen,		//Get the raw value of Green
	vexIqFcn_GetColorRawBlue,			//Get the raw value of Blue
	vexIqFcn_ColorGetSaturation,  //Return the "saturation" 8-bit value
	vexIqFcn_ColorGetInfraredValue,
	vexIqFcn_GetColor12,					//Return the "12-Color" enum color
	vexIqFcn_GetColorGray,				//Return the Grayscale Color for Line Tracking
	vexIqFcn_GetColorInfo,				//Return a structure to debug the color sensor

  vexIqFcn_GetDataPath,         //Return a char to indicate the current data path
} TVexIqOpcodeFunctions;
#endif

#if defined(bHasUserMessaging) || defined(_WINDOWS)

// can expand this to support more features, like maybe having addressable user messages
typedef enum TUsrMsgOpcodeFunctions
{

  //////////////////////////////////////////////////////
  //  External communication opcodes
  //////////////////////////////////////////////////////////

  usgMsgFcn_RecieveUserMsg = 0x10,
  usgMsgFcn_RequestUsrMsg,

  //////////////////////////////////////////////////////
  //  user program opcodes
  //////////////////////////////////////////////////////////

  usgMsgFcn_ReadUserMsg = 0x20,
  usgMsgFcn_SendUserMsg,
  usgMsgFcn_UserMsgAvailable,
  usrMsgFcn_UserMsgSize,
//  usrMsgFcn_GetSendBuffer,
} TUsrMsgOpcodeFunctions;
#endif


#if defined(hasMotorOpcodes)
typedef enum TMotorOpTypes
{
	mtrOpSetBrakeMode = 0,
	mtrOpGetBrakeMode = 1,
	mtrOpResetEncoder = 2,
	mtrOpGetEncoder = 3,
	mtrOpGetEncoderTargetAbsolute = 4,
	mtrOpSetMirroredFlag = 5,
	mtrOpGetMirroredFlag = 6,
  mtrOpSetPositionAndVelocityRelative = 7,
  mtrOpSetPositionAndVelocityAbsolute = 8,
	mtrOpSetVelocity = 9,
	mtrOpGetVelocity = 10,

#if defined(VexIQ) || defined(_WINDOWS)
	mtrOpSetServoPosition = 11,
	mtrOpSetCurrentLimit = 12,
	mtrOpGetCurrentLimit = 13,
	mtrOpGetImmediateCurrent = 14,
	mtrOpRequestPWMScaleUpdate = 15,
	mtrOpGetOverTemp = 16,
	mtrOpGetCurrentLimitFlag = 17,
	mtrOpGetZeroVelocity = 18,
	mtrOpGetZeroPosition = 19,
  mtrOpGetMotorEncoderUnits = 20,
  mtrOpSetMotorEncoderUnits = 21,
#endif

#if defined(bFirmwareStoresMotorDriveSide) || defined(_WINDOWS)
	mtrOpDriveSideList = 22,
	mtrOpGetDriveTrainMoving = 23,
#endif

#if defined(EV3) || defined(_WINDOWS)
  mtrOpGetMotorStatus = 24,
#endif
} TMotorOpTypes;
#endif

#if defined(EV3) || defined(_WINDOWS)

typedef enum TEV3OpcodeFunctions
{
  ev3ShellSort,

  ev3SensorReset,
	ev3GetSensorDataset,
  ev3GetSensorDatasetSize,

	ev3SetSensorAutoID,
	ev3ResetAllSensorAutoID,
	ev3SetSensorDevCon,
	ev3WaitForAllSensorsReady,

  ev3OpenMailbox,
	ev3CloseMailbox,
  ev3GetMailboxName,
	ev3GetMailboxReady,
  ev3ReadMailbox,
  ev3WriteMailbox,

  ev3Microseconds,

  ev3Fcn_TouchSensorGetState,
  ev3Fcn_TouchSensorGetBumped,

  ev3Fcn_ColorSensorGetAmbient,
  ev3Fcn_ColorSensorGetRGBChannels,
  ev3Fcn_ColorSensorGetRawRGBChannels,
  ev3Fcn_ColorSensorGetName,
  ev3Fcn_ColorSensorGetReflected,
  ev3Fcn_ColorSensorGetSaturation,
  ev3Fcn_ColorSensorGetHue,

  ev3Fcn_GyroReset,
  ev3Fcn_GyroGetRateHeading,
  ev3Fcn_GyroGetDegrees,
  ev3Fcn_GyroGetHeading,
  ev3Fcn_GyroGetRate,

  ev3Fcn_USGetDistance,

  ev3Fcn_IRGetDistance,
  ev3Fcn_IRGetBeaconDirection,
  ev3Fcn_IRGetBeaconChannelDirection,
  ev3Fcn_IRGetBeaconStrength,
  ev3Fcn_IRGetBeaconChannelStrength,
  ev3Fcn_IRGetBeaconAllChannels,
  ev3Fcn_IRRemoteGetButtons,
  ev3Fcn_IRRemoteGetChannelButtons,
  ev3Fcn_IRRemoteGetAllChannelsButtons,

  ev3Fcn_MotorSyncTime,
  ev3Fcn_MotorSyncEncoder,
  ev3Fcn_MotorSync,

  ev3Fcn_MotorRPM,

  // File ops, functions live in EV3FileSystemDriver.c
  ev3Fcn_FileOpen,
  ev3Fcn_FileDelete,
  ev3Fcn_FileClose,
  ev3Fcn_FileWriteData,
  ev3Fcn_FileWriteLong,
  ev3Fcn_FileWriteShort,
  ev3Fcn_FileWriteChar,
  ev3Fcn_FileWriteFloat,
  ev3Fcn_FileReadData,
  ev3Fcn_FileReadLong,
  ev3Fcn_FileReadShort,
  ev3Fcn_FileReadChar,
  ev3Fcn_FileReadFloat,

  ev3Fcn_DatalogOpen,
  ev3Fcn_DatalogClose,
  ev3Fcn_DatalogFlush,
  ev3Fcn_DatalogAddChar,
  ev3Fcn_DatalogAddShort,
  ev3Fcn_DatalogAddLong,
  ev3Fcn_DatalogAddFloat,
//  ev3Fcn_DatalogAddString,

  ev3Fcn_SensorGetConnType,
  ev3CommandOne,
  ev3CommandTwo,
  ev3CommandThree,
} TEV3OpcodeFunctions;

#endif

#if defined (LINUX)
// These are handled in Linux_OpcodeHandler.c
typedef enum TLinuxOpcodeFunctions
{
	// Misc ops, functions live in Linux_Misc.c
	linuxFcn_Microseconds,
	linuxFcn_ShellSort,

	// File ops, functions live in Linux_FileIO.c
	linuxFcn_FileOpen,
	linuxFcn_FileDelete,
	linuxFcn_FileClose,
	linuxFcn_FileWriteData,
	linuxFcn_FileWriteLong,
	linuxFcn_FileWriteShort,
	linuxFcn_FileWriteChar,
	linuxFcn_FileWriteFloat,
	linuxFcn_FileReadData,
	linuxFcn_FileReadLong,
	linuxFcn_FileReadShort,
	linuxFcn_FileReadChar,
	linuxFcn_FileReadFloat,

	// Datalogging ops, functions live in Linux_Datalog.c
	linuxFcn_DatalogOpen,
	linuxFcn_DatalogClose,
	linuxFcn_DatalogFlush,
	linuxFcn_DatalogAddChar,
	linuxFcn_DatalogAddShort,
	linuxFcn_DatalogAddLong,
	linuxFcn_DatalogAddFloat,
} TLinuxOpcodeFunctions;
#endif // LINUX

#if defined (RPI)
// These are handled in RPI_OpcodeHandler.c
typedef enum TRPiOpcodeFunctions
{
	rpiFcn_SetPinIO,
	rpiFcn_Noop,
} TRPiOpcodeFunctions;
#endif // RPI

#if defined (includeRVWLocationSensorSupport)
// These are handled in GPSSensor_OpcodeHandler.c
typedef enum TGPSSensorOpcodeFunctions
{
	gpsSensorFcn_GetGPSStr,
	gpsSensorFcn_GetGPSFullStr,

	// The following sub-opcodes were used for testing.
	///*
	//gpsSensorFcn_GetLatitude,
	//gpsSensorFcn_GetLongitude,
	//gpsSensorFcn_GetElevation,
	//gpsSensorFcn_GetHeading,
	//gpsSensorFcn_GetHasStrength,
	//gpsSensorFcn_GetStrength,
	//gpsSensorFcn_GetTime,
    //gpsSensorFcn_GetMagnitude,
	//*/
} TGPSSensorOpcodeFunctions;
#endif // RPI

//////////////////////////////////////////////////////////////////////////////////////////
//
//                           Clear Command Flags
//
//////////////////////////////////////////////////////////////////////////////////////////

//
// Define individual bits that can be "OR-ed" together for the "clear All" opcode
//
typedef enum TClearCommandAction
{
	actOnTimers       = 0x01,   // Resets the internal timers to zero value
	actOnSensors      = 0x02,   // Sets the value of all sensors to zero (including NXT motor encoder values
	actOnVariables    = 0x04,   // Clears all variables to zero
	actOnEvents       = 0x10,   // Undefines all events
	actOnBreakpoints  = 0x20,   // Undefines all breakpoints
	actOnMotors       = 0x40,   // Stop all motors
	actOnFlashFileResetProgram    = 0x0100,   // This will cause interpreter to reset the RAM pointers to program locations in flash
															// it is not intended for user use and will screw up running programs and likely cause
															// VM to crash if used. It is for internal ROBOTC IDE use only.
#if defined(bHasDatalog)
	actOnDatalog      = 0x80,   // Zeros the datalog size. (i.e. size 1)
#endif
	//actOnFRCRelays    = 0x80,   // Clears all the FRC Relays
	actOnNone         = 0x00    // Do nothing
} TClearCommandAction;

#if defined(NXT)
//////////////////////////////////////////////////////////////////////////////////////////
//
//                           NXT Misecellaneous Functions
//
//////////////////////////////////////////////////////////////////////////////////////////

enum TMiscellaneousFunctions
{
	miscMemSet            = 0,
	miscMemCpy            = 1,

};

#endif


//////////////////////////////////////////////////////////////////////////////////////////
//
//                           NXT Trig Functions
//
//////////////////////////////////////////////////////////////////////////////////////////

typedef enum TTranscendentals
{
	//
	// Do not shift these constants. Some are known to Robolab.
	//
#if defined(hasTranscendentalSupport) && defined(useFloats) || defined(_WINDOWS)
	mathSine                =  0,
	mathCosine              =  1,
	mathArcTan              =  2,

	mathSineDegrees         =  3,
	mathCosDegrees          =  4,
	mathSqrtWord            =  5,
	mathSqrtLong            =  6,
	mathSqrtFloat           =  7,
#endif

#if defined(useFloats) || defined(_WINDOWS)
	mathAbsFloat            =  8,
	mathSgnFloat            =  9,
#endif

 	mathAbsWord             = 10,

#if defined(useLongs) || defined(_WINDOWS)
	mathAbsLong             = 11,
	mathSgnLong             = 12,
#endif

	mathSgnWord             = 13,

#if defined(hasTranscendentalSupport) || defined(_WINDOWS)
	mathExp                 = 14,
#endif

	mathNegateByte          = 24,
	mathNegateWord          = 25,

#if defined(useLongs) || defined(_WINDOWS)
	mathNegateLong          = 26,
#endif

	mathArcSin              = 28,
	mathArcCos              = 29,
	mathDegreesToRadians    = 30,
	mathRadiansToDegrees    = 31,

	mathLog                 = 32,
	mathLog10               = 33,

	mathCeil								= 34,
	mathFloor								= 35,
	mathRound								= 36,

	mathLast

} TTranscendentals;

typedef enum TOpcodeSubTypes
{
	opcdSubTypeResultSByte						= 0,
	opcdSubTypeResultUByte						= 1,

	opcdSubTypeResultSShort						= 2,
	opcdSubTypeResultUShort						= 3,

	opcdSubTypeResultLong						  = 4,
	opcdSubTypeResultLong_LongParm	  = 5,
} TOpcodeSubTypes;


//////////////////////////////////////////////////////////////////////////////////////////
//
//                              Definition of ROBOTC VM Opcodes
//
//////////////////////////////////////////////////////////////////////////////////////////

#if defined(_WINDOWS)
	#define opEnumType (ubyte)
#else
	#define opEnumType (sbyte) (ubyte)
#endif

typedef enum TVMOpcode
{
	opcdNoop																= opEnumType	0x00, // Could be reused
	opcdDebugGetStatus											= opEnumType	0x01,
	opcdDebugSetProgramCounter							= opEnumType	0x02,
	opcdDebugSetBreakPoint									= opEnumType	0x03,
	opcdDebugClearException									= opEnumType	0x04,
	opcdDebugSuspendResumeStep							= opEnumType	0x05,
	opcdClearAllEvents											= opEnumType	0x06,
#if defined(hasMotorOpcodes)
	opcdMotorFunctions											= opEnumType	0x07,
#endif

	opcdAddToSourceValue										= opEnumType	0x08, // Must be consecutive opcodes
	opcdMinusToSourceValue									= opEnumType (opcdAddToSourceValue + (ubyte) arithOffsetMinusTo),
	opcdTimesToSourceValue									= opEnumType (opcdAddToSourceValue + (ubyte) arithOffsetTimesTo),
	opcdDivideToSourceValue									= opEnumType (opcdAddToSourceValue + (ubyte) arithOffsetDivideTo),
	opcdShiftLeftTo													= opEnumType  0x0C,
	opcdShiftRightTo												= opEnumType  0x0D,
	opcdModuloTo														= opEnumType  0x0E,
	opcdBitComplement												= opEnumType  0x0F,

	opcdAlive																= opEnumType	0x10,
#if defined(VexIQ) || defined(_WINDOWS)
	opcdVexIqFunctions											= opEnumType	0x11,
#endif
	opcdAssignSourceValue										= opEnumType	0x12,
	opcdOrTo																= opEnumType	0x13,
	opcdMsgDownloadCommands									= opEnumType	0x14,		// Used to download various messages and commands to ROBOTC VM
	opcdMsgUploadCommands										= opEnumType	0x15,		// Used to upload various messages to ROBOTC IDE
	opcdAndTo																= opEnumType	0x16,
	opcdCallSub															= opEnumType	0x17,

	opcdAddToLong														= opEnumType	0x18, // Must be consecutive opcodes and op % 8 == 0
	opcdMinusToLong													= opEnumType (opcdAddToLong + (ubyte) arithOffsetMinusTo),
	opcdTimesToLong													= opEnumType (opcdAddToLong + (ubyte) arithOffsetTimesTo),
	opcdDivideToLong												= opEnumType (opcdAddToLong + (ubyte) arithOffsetDivideTo),
	opcdXorTo																= opEnumType  0x1C,
	opcdCallSubExpanded											= opEnumType	0x1D,
#if defined(EV3) || defined(_WINDOWS)
	opcdEV3_ROBOTC													= opEnumType  0x1E,			// Manually implemented EV3 opcodes
	opcdEV3_LEGO														= opEnumType  0x1F,			// Auto generated access to LEGO's EV3 opcodes.
#endif

	opcdTestAndBranchUShort									= opEnumType  0x20,
	opcdTestAndBranchSShort									= opEnumType	0x21,
	opcdTestAndBranchSByteVar								= opEnumType	0x22,
	opcdTestAndBranchUByteVar								= opEnumType	0x23,
	opcdTestAndBranchSLong									= opEnumType	0x24,
	opcdTestAndBranchULong									= opEnumType	0x25,
	opcdTestAndBranchString									= opEnumType	0x26,
	opcdTestAndBranchFloat									= opEnumType	0x27,

	opcdAssignLongConstantWord							= opEnumType	0x28,
	opcdAssignLongConstant32Bit							= opEnumType	0x29,
	opcdArrayBounds													= opEnumType	0x2A,
	opcdArrayBoundsComplicated							= opEnumType	0x2B,
	//Unused																= opEnumType	0x2C, // cannot be used for message opcode. Conflict, and length wrong
	//Unused																= opEnumType	0x2D, // cannot be used for message opcode. Conflict, and length wrong
	opcdMiscellaneousFunctions							= opEnumType	0x2E,
	opcdArrayBounds16Bits										= opEnumType	0x2F,

	opcdAddToGenericUnsignedByteValue				= opEnumType	0x30,
	opcdMinusToGenericUnsignedByteValue			= opEnumType (opcdAddToGenericUnsignedByteValue + (ubyte) arithOffsetMinusTo),
	opcdTimesToGenericUnsignedByteValue			= opEnumType (opcdAddToGenericUnsignedByteValue + (ubyte) arithOffsetTimesTo),
	opcdDivideToGenericUnsignedByteValue		= opEnumType (opcdAddToGenericUnsignedByteValue + (ubyte) arithOffsetDivideTo),
	opcdMultiRobotNtwking										= opEnumType	0x34,
	opcdAssignLong													= opEnumType	0x35,
	opcdAssignFloat													= opEnumType	0x36,
	opcdSetPropertyConstantIndexConstantValue = opEnumType	0x37,

	opcdAddToFloat													= opEnumType	0x38, // Must be consecutive opcodes
	opcdMinusToFloat												= opEnumType (opcdAddToFloat + (ubyte) arithOffsetMinusTo),
	opcdTimesToFloat												= opEnumType (opcdAddToFloat + (ubyte) arithOffsetTimesTo),
	opcdDivideToFloat												= opEnumType (opcdAddToFloat + (ubyte) arithOffsetDivideTo),
	//Unused																= opEnumType	0x3C,
	//Unused																= opEnumType	0x3D,
	//Unused																= opEnumType	0x3E,
	//Unused																= opEnumType	0x3F,

	opcdExitEventCheck											= opEnumType	0x40,
	opcdStartEventMonitorShort							= opEnumType	0x41,
	opcdStartEventMonitorLong								= opEnumType	0x42,
	opcdSetEventDefinition									= opEnumType	0x43,
	opcdWaitTimer1MSecLong									= opEnumType	0x44,
	opcdWaitTimer1MSecShort									= opEnumType	0x45,
	opcdWaitTimer10MSec											= opEnumType	0x46,
	opcdSetException												= opEnumType	0x47,

	opcdAddToGenericSignedByteValue					= opEnumType	0x48, // Must be consecutive opcodes
	opcdMinusToGenericSignedByteValue				= opEnumType (opcdAddToGenericSignedByteValue + (ubyte) arithOffsetMinusTo),
	opcdTimesToGenericSignedByteValue				= opEnumType (opcdAddToGenericSignedByteValue + (ubyte) arithOffsetTimesTo),
	opcdDivideToGenericSignedByteValue			= opEnumType (opcdAddToGenericSignedByteValue + (ubyte) arithOffsetDivideTo),
	opcdGetPropertyConstantIndexToByte			= opEnumType	0x4C,
	opcdGetPropertyConstantIndexToShort			= opEnumType	0x4D,
	opcdGetPropertyConstantIndexToLong			= opEnumType	0x4E,
	opcdGetPropertyConstantIndexToFloat			= opEnumType	0x4F,

	opcdInvokeBootloader										= opEnumType	0x50,
	opcdSoundFunctions											= opEnumType	0x51,
	opcdSendI2CMsg													= opEnumType	0x52,
	opcdReadI2CMsg													= opEnumType	0x53,
	opcdSendI2CMsgWithReturn								= opEnumType	0x54,
	opcdReadI2CMsgWithReturn								= opEnumType	0x55,// don't use 0x55 as an opcode that can be sent via message or it will screw up recognition of message headers
	opcdSetPropertyVariableIndex						= opEnumType	0x56,
	opcdSetPropertyConstantIndex						= opEnumType	0x57,

	opcdSwitchByteCase											= opEnumType	0x58,
	opcdSwitchIndexTableNear								= opEnumType	0x59,
	opcdSwitchIndexTableFar									= opEnumType	0x5A,
	opcdSwitch															= opEnumType	0x5B,
	opcdGetPropertyVariableIndexToByte			= opEnumType	0x5C,
	opcdGetPropertyVariableIndexToShort			= opEnumType	0x5D,
	opcdGetPropertyVariableIndexToLong			= opEnumType	0x5E,
	opcdGetPropertyVariableIndexToFloat			= opEnumType	0x5F,

	opDrawMenuOnGraphicsLCDDisplay					= opEnumType	0x60,
	opcdClearItems													= opEnumType	0x61,
	opcdStopAllTasks												= opEnumType	0x62,
	opcdStartTask														= opEnumType	0x63,
	opcdStopTask														= opEnumType	0x64,
	opcdDirectEvent													= opEnumType	0x65,
	opcdAssert															= opEnumType	0x66,
	opcdSystemFunctions											= opEnumType	0x67,

	opcdAddToLong32BitConstant							= opEnumType	0x68,	// Must be consecutive opcodes and op % 8 == 0
	opcdMinusToLong32BitConstant						= opEnumType (opcdAddToLong32BitConstant + (ubyte) arithOffsetMinusTo),
	opcdTimesToLong32BitConstant						= opEnumType (opcdAddToLong32BitConstant + (ubyte) arithOffsetTimesTo),
	opcdDivideToLong32BitConstant						= opEnumType (opcdAddToLong32BitConstant + (ubyte) arithOffsetDivideTo),
	opcdAddToStackSShortConstant						= opEnumType	0x6C,	// Must be consecutive opcodes and op % 4 == 0
	opcdMinusToStackSShortConstant					= opEnumType (opcdAddToStackSShortConstant + (ubyte) arithOffsetMinusTo),
	opcdTimesToStackSShortConstant					= opEnumType (opcdAddToStackSShortConstant + (ubyte) arithOffsetTimesTo),
	opcdDivideToStackSShortConstant					= opEnumType (opcdAddToStackSShortConstant + (ubyte) arithOffsetDivideTo),

	opcdAddToStackSLong16BitSignedConstant			= opEnumType	0x70,	// Must be consecutive opcodes and op % 84 == 0
	opcdMinusToStackSLong16BitSignedConstant		= opEnumType (opcdAddToStackSLong16BitSignedConstant + (ubyte) arithOffsetMinusTo),
	opcdTimesToStackSLong16BitSignedConstant		= opEnumType (opcdAddToStackSLong16BitSignedConstant + (ubyte) arithOffsetTimesTo),
	opcdDivideToStackSLong16BitSignedConstant		= opEnumType (opcdAddToStackSLong16BitSignedConstant + (ubyte) arithOffsetDivideTo),
	opcdAddToGlobalSLong16BitSignedConstant			= opEnumType	0x74,	// Must be consecutive opcodes and op % 8 == 0
	opcdMinusToGlobalSLong16BitSignedConstant		= opEnumType (opcdAddToGlobalSLong16BitSignedConstant + (ubyte) arithOffsetMinusTo),
	opcdTimesToGlobalSLong16BitSignedConstant		= opEnumType (opcdAddToGlobalSLong16BitSignedConstant + (ubyte) arithOffsetTimesTo),
	opcdDivideToGlobalSLong16BitSignedConstant	= opEnumType (opcdAddToGlobalSLong16BitSignedConstant + (ubyte) arithOffsetDivideTo),


	opcdBranchFar														= opEnumType	0x78,
	opcdBranchNear													= opEnumType	0x79,
	opcdAssignLongConstantByte							= opEnumType	0x7A,
	opcdReturn															= opEnumType	0x7B,
	opcdTranscendentalFunctions							= opEnumType	0x7C,
	opcdAssignRamVariableAddress						= opEnumType	0x7D,	// Store in "dest" operand the RAM address of "srce" operand
	opcdDebugStreamFcns											= opEnumType	0x7E,
	opcdFloatToLong													= opEnumType	0x7F,

	opcdAddToStackSShort										= opEnumType	0x80,
	opcdMinusToStackSShort									= opEnumType	(opcdAddToStackSShort + (ubyte) arithOffsetMinusTo),
	opcdTimesToStackSShort									= opEnumType	(opcdAddToStackSShort + (ubyte) arithOffsetTimesTo),
	opcdDivideToStackSShort									= opEnumType	(opcdAddToStackSShort + (ubyte) arithOffsetDivideTo),
	opcdAddToStackSLong											= opEnumType	0x84,
	opcdMinusToStackSLong										= opEnumType	(opcdAddToStackSLong + (ubyte) arithOffsetMinusTo),
	opcdTimesToStackSLong										= opEnumType	(opcdAddToStackSLong + (ubyte) arithOffsetTimesTo),
	opcdDivideToStackSLong									= opEnumType	(opcdAddToStackSLong + (ubyte) arithOffsetDivideTo),

	opcdAddToGlobalSShortConstant						= opEnumType	0x88,	// Must be consecutive opcodes and op % 8 == 0
	opcdMinusToGlobalSShortConstant					= opEnumType (opcdAddToGlobalSShortConstant + (ubyte) arithOffsetMinusTo),
	opcdTimesToGlobalSShortConstant					= opEnumType (opcdAddToGlobalSShortConstant + (ubyte) arithOffsetTimesTo),
	opcdDivideToGlobalSShortConstant				= opEnumType (opcdAddToGlobalSShortConstant + (ubyte) arithOffsetDivideTo),
	//Unused																= opEnumType	0x8C,
	//Unused																= opEnumType	0x8D,
	//Unused																= opEnumType	0x8E,
	//Unused																= opEnumType	0x8F,

	opcdFloatAddTo32BitFloatConstant				= opEnumType	0x90,
	opcdFloatMinusTo32BitFloatConstant			= opEnumType  0x91,
	opcdFloatTimesTo32BitFloatConstant			= opEnumType  0x92,
	opcdFloatDivideTo32BitFloatConstant			= opEnumType  0x93,
	//Unused																= opEnumType	0x94,
	//Unused																= opEnumType	0x95,
	//Unused																= opEnumType	0x96,
	//Unused																= opEnumType	0x97,

	opcdDrawOnGraphicsLCDDisplay						= opEnumType	0x98,
	opcdAssignSourceByteValue								= opEnumType	0x99,
	opcdUShortToFloat												= opEnumType	0x9A,
	opcdSignalEvent													= opEnumType	0x9B,
	opcdResetEvent													= opEnumType	0x9C,
	opcdSetMessage	 												= opEnumType	0x9D,
	opcdSendMessageNoParm										= opEnumType	0x9E,
	opcdSendMessageWithParm 								= opEnumType	0x9F,

	opcdAddToShortVar												= opEnumType	0xA0, // Must be consecutive opcodes and op % 8 == 0
	opcdMinusToShortVar											= opEnumType (opcdAddToShortVar + (ubyte) arithOffsetMinusTo),
	opcdTimesToShortVar											= opEnumType (opcdAddToShortVar + (ubyte) arithOffsetTimesTo),
	opcdDivideToShortVar										= opEnumType (opcdAddToShortVar + (ubyte) arithOffsetDivideTo),
	opcdFloatToByte													= opEnumType	0xA4,
	opcdUShortToLong												= opEnumType	0xA5,
	opcdSShortToLong												= opEnumType	0xA6,
	opcdLongToByte													= opEnumType	0xA7,

	opcdTrinaryAddShort											= opEnumType	0xA8,
	opcdTrinaryMinusShort										= opEnumType	0xA9,
	opcdTrinaryTimesShort										= opEnumType	0xAA,
	opcdDatalogFunctions										= opEnumType	0xAB,
	opcdSShortToFloat												= opEnumType	0xAC,
	opcdLongToFloat													= opEnumType	0xAD,
	opcdFloatToShort												= opEnumType	0xAE,
	opcdLongToShort													= opEnumType	0xAF,

	opcdBreakpoint													= opEnumType	0xB0,
	//Unused																	= opEnumType	0xB1,
	//Unused																	= opEnumType	0xB2,
	opcdBTLinkIO														= opEnumType	0xB3,
	opcdNXTMessageIO												= opEnumType	0xB4,
	opcdFileIO															= opEnumType	0xB5,
	opcdHSLinkIO														= opEnumType	0xB6,
	opcdStringOps														= opEnumType	0xB7,

	//Unused																= opEnumType	0xB8,
	//Unused																= opEnumType	0xB9,
	//Unused																= opEnumType  0xBA,
	//Unused																= opEnumType  0xBB,
	//Unused																= opEnumType	0xBC,
	opcdTrinaryAddLong											= opEnumType	0xBD,
	opcdTrinaryMinusLong										= opEnumType	0xBE,
	opcdTrinaryTimesLong										= opEnumType	0xBF,

	opcdAssignConstantOneSByte							= opEnumType	0xC0,
	opcdAssignConstantOneUByte							= opEnumType	0xC1,
	opcdAssignConstantTwoByteSWord					= opEnumType	0xC2,
	opcdAssignGlobalSShort									= opEnumType	0xC3,
	opcdAssignGlobalSLong										= opEnumType	0xC4,
	opcdAssignStackSShort										= opEnumType	0xC5,
	opcdAssignStackSLong										= opEnumType	0xC6,
	//Unused																= opEnumType	0xC7,

	opcdAssignGlobalSShortConstant					= opEnumType	0xC8,
	opcdAssignGlobalSLong16BitSignedConstant= opEnumType	0xC9,
	opcdAssignStackSShortConstant						= opEnumType	0xCA,
	opcdAssignStackSLong16BitSignedConstant	= opEnumType	0xCB,
	//Unused																= opEnumType	0xCC,
	//Unused																= opEnumType	0xCD,
	//Unused																= opEnumType	0xCE,
	//Unused																= opEnumType	0xCF,

#if defined(bHasUserMessaging) || defined(_WINDOWS)
  opcdUsrMsgFunctions                     = opEnumType  0xD0,
#endif
#if defined (LINUX) || defined(_WINDOWS)
	opcdLinuxFunctions											= opEnumType	0xD1,
#endif
#if defined (RPI) || defined(_WINDOWS)
	opcdRPiFunctions												= opEnumType	0xD2,
#endif
	//Unused																= opEnumType	0xD3,
	//Unused																= opEnumType	0xD4,
	//Unused																= opEnumType	0xD5,
	//Unused																= opEnumType	0xD6,
	//Unused																= opEnumType	0xD7,

#if defined(includeRVWLocationSensorSupport)
  opcdGPSSensorFunctions                  = opEnumType  0xD8,
#endif
	//Unused																= opEnumType	0xD9,
	//Unused																= opEnumType	0xDA,
	//Unused																= opEnumType	0xDB,
	//Unused																= opEnumType	0xDC,
	//Unused																= opEnumType	0xDD,
	//Unused																= opEnumType	0xDE,
	//Unused																= opEnumType	0xDF,

	//Unused																= opEnumType	0xE0,
	//Unused																= opEnumType	0xE1,
	//Unused																= opEnumType	0xE2,
	//Unused																= opEnumType	0xE3,
	//Unused																= opEnumType	0xE4,
	//Unused																= opEnumType	0xE5,
	//Unused						 										= opEnumType	0xE6,
	//Unused																= opEnumType	0xE7,

	//Unused																= opEnumType	0xE8,
	//Unused																= opEnumType	0xE9,
	//Unused																= opEnumType	0xEA,
	//Unused																= opEnumType	0xEB,
	//Unused																= opEnumType	0xEC,
	//Unused																= opEnumType	0xED,
	//Unused																= opEnumType	0xEE,
	//Unused																= opEnumType	0xEF,

	//Unused																= opEnumType	0xF0,
	//Unused																= opEnumType	0xF1,
	//Unused																= opEnumType	0xF2,
	//Unused																= opEnumType	0xF3,
	//Unused																= opEnumType	0xF4,
	//Unused																= opEnumType	0xF5,
	//Unused																= opEnumType	0xF6,
	//Unused																= opEnumType	0xF7,

	//Unused																= opEnumType	0xF8,
	//Unused																= opEnumType	0xF9,
	//Unused																= opEnumType	0xFA,
	//Unused																= opEnumType	0xFB,
	//Unused																= opEnumType	0xFC,
	//Unused																= opEnumType	0xFD,
	//Unused																= opEnumType	0xFE,
	opcdErasedFlash													= opEnumType	0xFF	 // Erased flash is 0xFF. Catch this case.


#if defined(_WINDOWS)
	,
	opcdBAD																	= 0x0100, // Not really used. Could be reallocated!!!

	//
	// Virtual opcodes thaqt are expanded into "system functions" or number type specific opcodes. They are not "real" opcodes
	// which must be in the range of 0..255
	//
	opcdAssign,
	opcdNegate,											// Unary Op
	opcdAddTo,
	opcdMinusTo,
	opcdTimesTo,
	opcdDivideTo,

#endif
} TVMOpcode;

///////////////////////////////////////////////////////////////////////////////////////////////
//
//                              Upload Message Types
//
//////////////////////////////////////////////////////////////////////////////////////////////

typedef enum TUploadMsgTypes
{
	uploadBad														= 0,
	uploadSensors												= 1,
	uploadMotors												= 2,
	uploadTimers												= 3,
	upload4ByteMemoryValue							= 4,
	upload4BytePropertyValue						= 5,

	uploadGetDataBytesRAM								= 6,
	uploadGetDataBytesFlash							= 7,
	uploadGlobalVariables								= 8,
	uploadLocalVariables								= 9,

#if defined(bHasEvents) || defined(_WINDOWS)
	uploadEventMap											= 10,
	uploadEvent													= 11,
#endif // #if defined(bHasEvents) || defined(_WINDOWS)

#if defined(bHasLCDGraphicsDisplay) || defined(_WINDOWS)
	uploadPixelLCD											= 12,
#endif

#if defined(hasAlphaNumericLCDDisplayViaUART) || defined(_WINDOWS)
	uploadAlphaNumericLCD								= 13,
#endif

#if defined(bHasDatalog) || defined(_WINDOWS)
	uploadDatalog												= 14,
	uploadReadDatalogStream							= 15,
#endif

#if ((bSmartMotorsWithEncoders() != 0) && defined(NXT)) || defined(_WINDOWS)
	uploadPIDSettings										= 16,
#endif

#if (defined(hasDebugStreamSupport) && !defined(NXT)) || defined(_WINDOWS)
	uploadReadDebugStream								= 17,
#endif



#if defined(VexIQ) || defined(_WINDOWS)
	uploadVexIqDeviceTable							= 18,		// Applies to VEX IQ only
	uploadVexIqDeviceStatistics					= 19,		// Applies to VEX IQ only
#endif

#if defined(hasJoystickUploadMessageOpcodes) || defined(_WINDOWS)
	uploadJoysticksVexCortex 						= 20,		// Applies to VEX Cortex only
	uploadJoysticksVexIQ 								= 21,		// Applies to VEX IQ only
#endif
} TUploadMsgTypes;


typedef enum TDownloadMsgTypes
{
	downloadBad											= 0,
	writeDataBytesFlash							= 1, // write flash memory
	writeDataBytesRAM								= 2, // write RAM   memory

#if defined(hasJoystickDownloadMessageOpcodes) || defined(_WINDOWS)
	downloadJoysticks								= 3,
#endif

#if ((bSmartMotorsWithEncoders() != 0) && defined(NXT)) || defined(_WINDOWS)
	downloadPIDSettings							= 4,
#endif

#if defined(bHasClearFlashOpcodes) || defined(_WINDOWS)
	clearDataBytesFlash							= 5,
#endif
} TDownloadMsgTypes;


#define kUploadCommandMessageLength 8
#define kWriteFlashOverhead					8


typedef enum TUploadMotorFields
{
	upldMtrType											=	 1,								// Bit mask for data sent to PC to include "motor type"
	upldMtrEncoder									= (1 << 1),					// Bit mask for data sent to PC to include "motor encoder"
	upldMtrEncoderTimeStamp					= (1 << 2),					// Bit mask for data sent to PC to include "motor encoder timestamp"
	upldMtrEncoderVelocity					= (1 << 3),					// Bit mask for data sent to PC to include "motor encoder velocity"  (unique to VEX Cortex encoders)
	upldMtrPidCalcVars							= (1 << 4),					// Bit mask for data sent to PC to include "PID calculation Temp Variables"
	upldMtrPidCycles								= (1 << 5),					// Bit mask for data sent to PC to include "PID cycle count"
	upldMtrPowerAfterSlew						= (1 << 6),					// Bit mask for data sent to PC to include "motor Power after Slew"
	upldMtrPowerPWM									= (1 << 7),					// Bit mask for data sent to PC to include "motor Power PWM"
	upldMtrLimits										= (1 << 8),					// Bit mask for data sent to PC to include "motor Deadband" and "motor Power Limit"
	upldMtrPidFactors_Period				= (1 << 9),					// Bit mask for data sent to PC to include "PID calculation period"
	upldMtrPidFactors_Constants			= (1 << 10),				// Bit mask for data sent to PC to include "PID constants"
	upldMtrPidRegulationState				= (1 << 11),				// Bit mask for data sent to PC to include PID regulation state
	upldMtrEncoderTarget						= (1 << 12),				// Bit mask for data sent to PC to include "motor encoder target"
	upldMtrCurrentLimit							= (1 << 13),				// Bit mask for data sent to PC to include "motor encoder target"
	upldMtrTargetReached						= (1 << 14)					// Bit mask for data sent to PC to include "motor encoder target"
} TUploadMotorFields;


///////////////////////////////////////////////////////////////////////////////////////////////
//
//																	Sound Opcode Sub Functions
//
//////////////////////////////////////////////////////////////////////////////////////////////

#if (bHasSoundSupport != 0)

	typedef enum TSoundSubFunctions
	{
		soundSubTypePlaySound,
		soundSubTypeClearSound,
		soundSubTypePlayToneVarDurationVar,
		soundSubTypePlayMusicalNote,
		soundSubTypePlaySoundFileVariableName,
		soundSubTypePlayToneImmediate,
		soundSubTypePlayRepetitiveSound,
		soundSubTypePlaySoundFileEmbeddedFileName,
		soundSubTypePlayToneRaw,
	} TSoundSubFunctions;
#endif



//////////////////////////////////////////////////////////////////////////////////////////
//
//                              Definition of Event Constants
//
//////////////////////////////////////////////////////////////////////////////////////////

typedef enum TEventSensorType
{
	EST_SENSOR_1                = 0,
	EST_SENSOR_2,
	EST_SENSOR_3,
	EST_TIMER_1                 = 3,
	EST_TIMER_2,
	EST_TIMER_3,
	EST_TIMER_4,
	//EST_LAST_IR_MSG             = 7,
	//EST_COUNTER_1               = 8,
	//EST_COUNTER_2,
	//EST_COUNTER_3,
	EST_USER_EVENT_0            = 11,
	EST_USER_EVENT_1,
	EST_USER_EVENT_2,
	EST_USER_EVENT_3,
	EST_USER_EVENT_4            = 15,

	EST_VIRTUAL_MOTOR           = 16,// New.
	EST_VIRTUAL_SENSOR          = 17,
	EST_WAIT_FOR_MSG            = 18,
	EST_INFRARED_STATUS         = 19,

	EST_SENSORLastValid         = EST_INFRARED_STATUS,
	EST_SENSOR_UNUSED           = 20 // internal state. Indicates no eventType for this entry
} TEventSensorType;

#define kNumbOfEventSensorTypes (EST_SENSORLastValid + 1)

//
// Global "define" to disable / enable event handling
//

typedef enum TEventTypes
{
	eventTypePressed                  = 0,
	eventTypeReleased                 = 1,
	eventTypePeriod                   = 2,
	eventTypeTransition               = 3,

	eventType4                        = 4,   // Unused
	eventType5                        = 5,   // Unused
	eventType6                        = 6,   // Unused

	eventTypeVirtualMotorChange       = 4,   // New for virtual devices. Same as 'eventType4'
	eventTypeVirtualSensorDefinition  = 5,   // New for virtual devices. Same as 'eventType5'

	eventTypeExceedChangeRate         = 7,
	eventTypeEnterLow                 = 8,
	eventTypeEnterNormal              = 9,
	eventTypeEnterHigh                = 10,
	eventTypeClick                    = 11,
	eventTypeDoubleClick              = 12,
	eventTypeInfraredIdle             = 13,  // New
	eventTypeMailbox                  = 14,
	//unused 15
	eventTypeReset                    = 16 // this is reset command to clear
} TEventTypes;

typedef enum TEventTypeMasks
{
	eventTypeMaskPressed            = (1), //(1 << eventTypePressed),
	eventTypeMaskReleased           = (1 << eventTypeReleased),
	eventTypeMaskPeriod             = (1 << eventTypePeriod),
	eventTypeMaskTransition         = (1 << eventTypeTransition),
	//unused 4
	//unused 5
	//unused 6
	eventTypeMaskExceedChangeRate   = (1 << eventTypeExceedChangeRate),
	eventTypeMaskEnterLow           = (1 << eventTypeEnterLow),
	eventTypeMaskEnterNormal        = (1 << eventTypeEnterNormal),
	eventTypeMaskEnterHigh          = (1 << eventTypeEnterHigh),
	eventTypeMaskClick              = (1 << eventTypeClick),
	eventTypeMaskDoubleClick        = (1 << eventTypeDoubleClick),
	//unused 13
	eventTypeMaskMailbox            = ((uword) 1 << eventTypeMailbox)
	//unused 15
} TEventTypeMasks;


typedef enum  TEventStates			// constants are 'getValue' parameter. Do not change order
{
	eventStateBelowLowerThreshold,
	eventStateBetweenThresholds,
	eventStateAboveUpperThreshold,
	eventStateUndetermined,              // Not yet figured out or 'message' type
	//
	// For NQC Compatability
	//
	ES_BELOW_LOWER      = eventStateBelowLowerThreshold,
	ES_BETWEEN          = eventStateBetweenThresholds,
	ES_ABOVE_UPPER      = eventStateAboveUpperThreshold,
	ES_UNDETERMINED     = eventStateUndetermined
} TEventStates;


///////////////////////////////////////////////////////////////////////////////////////////////
//
//                               Multi-Robot Networking
//
//////////////////////////////////////////////////////////////////////////////////////////////

#if defined(bHasMultRobotNetworking) || defined(_WINDOWS)

typedef enum TMultRobotMessageOps
{
	//
	// Do not shift these constants.
	//
	ntwkingOpStatus    							=  1,   // Gets the status of the multi-robot messaging system.

	ntwkingOpSendMessage 						=  2,   // Send message. Fails if all buffers are used.
	ntwkingOpSendMessageWithWait    =  3,   // Send message. If necessary wait for an idle buffer.
	ntwkingOpReadyToSend						=	 4,   // Used to check whether idle buffer is available

	ntwkingOpGetMessage   					=  5,
	ntwkingOpGetReceiveMsgSize			=  6,   // Retrieve size of next message in queue. Zero if no message.
	ntwkingOpMessageAvail						=  7,   // Returns true if message is available.

	ntwkingOpResetNetworkDefinition =  8,   // Clears the definition of the network.
	ntwkingSetNtwkConfiguration			=  9,   // Used by "Motors and Sensors Setup" to configure a network

	ntwkingOpGetMyNodeIndex         = 10,   // Retrieves the node index of this robot in the list of network nodes.

	ntwkingOpStartupFinished				= 11,   // Indicates whether the multi-robot network system has finished startup
} TMultRobotMessageOps;

typedef enum TNtwkingStatus
{
	mrSuccess									= 0,
	mrBusy										= 1,
	mrFailed									= 2
} TNtwkingStatus;

#endif //#if defined(bHasMultRobotNetworking)


#if !defined(VexIqProgramTypes)
#define VexIqProgramTypes

typedef enum TProgramTypes
{
	NO_RADIO          = 0,     //Autonomous
	RADIO_REQUIRED    = 1,     //Operator Controlled
	UTILITY           = 2,     //PC Controlled
	GAME              = 3      //No time update
} TProgramTypes;
#define kVexIqRobotcFlag   0x80   // When set indicates program type of ROBOTC
#endif
///////////////////////////////////////////////////////////////////////////////////////////////
//
//                              Exception Types
//
//////////////////////////////////////////////////////////////////////////////////////////////

typedef enum TVMException
{
	//
	// Don't mess with the numeric values without also changing the constants in assembler files.
	//
	exceptionNone																								= 0,
	exceptionSensorOutOfRange																		= 1,
	exceptionMotorOutOfRange            												= 2,
	exceptionTimerOutOfRange            												= 3,
	exceptionEventOutOfRange            												= 4,
	exceptionVariableOutOfRange         												= 5,
	exceptionSetPropertyIndexOutOfRange		                			= 6,
	exceptionArrayOutOfRange            												= 7,

	exceptionGetPropertyIndexOutOfRange                 				= 8,

	exceptionCallSubStackOverflow              									= 9,
	exceptionCallSubStackUnderflow             									= 10,
	exceptionVariablesStackOverflow              								= 11,
	exceptionVariablesStackUnderflow             								= 12,

	exceptionReadOnlyParm               												= 13,

	exceptionPCOutOfRange               												= 14,

	exceptionInvalidSubroutine          												= 15,
	exceptionInvalidSubroutineReturn    												= 16,

	exceptionAssertFailure              												= 17,
	exceptionParmOutOfRangeRead         												= 18,
	exceptionSystemParmReadOutOfRange   												= 19,
	exceptionValueOutOfRange            												= 20,
	exceptionUartParmOutOfRange         												= 21,
	exceptionSoundParmOutOfRange        												= 22,
	exceptionTaskOutOfRange             												= 23,
	exceptionInvalidLengthLongMessage   												= 24,

	exceptionInvalidBreakpoint          												= 25,
	exceptionWordReadOfLongParm         												= 26,
	exceptionFloatNotSupported          												= 27,
	exceptionLongNotSupported           												= 28,
	exceptionBreakpoint                 												= 29,
	exceptionSystemParmWriteOutOfRange  												= 31,
	exceptionSystemParmExtOutOfRange    												= 32,

#if defined(NXT) || defined(_WINDOWS)
	exceptionBTRawModeError             												= 33,
#endif
#if defined(ARMCPU) || defined(_WINDOWS)
	exceptionServoOutOfRange            												= 34,
#endif

#if defined(ARMCPU) || defined(_WINDOWS)
	exceptionFloatToShortOutOfRange     												= 35,
	exceptionFloatToLongOutOfRange      												= 36,
#endif

	exceptionInvalidAnalogPort          												= 37,

	exceptionIllegalFloatCalculation    												= 38,  // Float Calculation results in "NAN" (Not a Number) result
	exceptionInternalError              												= 39,
	exceptionParmOutOfRangeWrite        												= 40,  // Bad Opcode Source
	exceptionInvalidPConSubReturn       												= 41,
	exceptionInvalidInternalQueues															= 42,
	exceptionFromNodeIsNotInNtwk																= 43,
	exceptionReceivedNtwkMessageTooLarge												= 44,
	exceptionPlatformDoesNotAssociateMotorWithEncoder 					= 45,
	exceptionNoSensorPortAssociatedForMotor											= 46,

	exceptionOpcodeNotSupportedInEmulationForNXT								= 47,
	exceptionOpcodeNotSupportedInEmulationForEV3								= 48,
	exceptionOpcodeNotSupportedInEmulationForVexPIC							= 49,
	exceptionOpcodeNotSupportedInEmulationForVexCortex					= 50,
	exceptionOpcodeNotSupportedInEmulationForVexIQ							= 51,
	exceptionOpcodeNotSupportedInEmulationForTSTPA							= 52,
	exceptionOpcodeNotSupportedInEmulationForArduino						= 53,

	exceptionVariableNotSupportedInEmulationForNXT							= 54,
	exceptionVariableNotSupportedInEmulationForVexPIC						= 55,
	exceptionVariableNotSupportedInEmulationForVexCortex				= 56,
	exceptionVariableNotSupportedInEmulationForVexIQ						= 57,
	exceptionVariableNotSupportedInEmulationForArduino					= 58,
	exceptionVariableNotSupportedInEmulationForEV3							= 59,
	exceptionVariableNotSupportedInEmulationForTSTPA						= 60,

	exceptionStringIndexOutOfRange      												= 61,
	exceptionServoDisabled              												= 62,
	exceptionMotorDisabled																			= 63,
	exceptionInvalidSensorPortForMotor  												= 64,
	exceptionInvalidSystemFunction															= 65,
	exceptionExecutionInstructionOutOfRange											= 66,
	exceptionCannotUpdateSlaveSync      												= 67,
#if defined(bHasSemaphoreSupport) || defined(_WINDOWS)
	exceptionSemaphoreNotOwnedForUnlock													= 68,
	exceptionSemaphoreTooManyLocksByOneTask											= 69,
	exceptionTooManySemaphoresDefined														= 70,
#endif
	exceptionPlatformDoesNotSupportMotorWithEncoder							= 71,
	exceptionIllegalSprintfFormatSpecification									= 72,
	exceptionIllegalSprintfFloatFormatSpecification							= 73,			// "%f" format must be of form "%N.Nf" or "%Nf" or "%f"
	exceptionEmulatorComPortNotConfigured												= 74,
	exceptionInvalidOpcode              												= 75,
	exceptionShiftValueOutOfRange       												= 76,
	exceptionPropertyNotSupportedDuringEmulation								= 77,
	exceptionSScanfVarArgIsInvalidAddress												= 78,
	exceptionMemCpyArgIsInvalidAddress													= 79,

	exceptionInvalidFunctionForPlatform													= 80,
	exceptionInvalidFunctionForSensorConfiguration							= 81,

	exceptionDatalogIndexError          												= 82,
	exceptionDatalogRawNotImplemented   												= 83,
	exceptionDatalogOverflow            												= 84,
	exceptionInvalidDatalogSourceType   												= 85,

	exceptionInvalidDriveTrainOrGripperConfiguration						= 86,
	exceptionMotorHardwareNotDetectedOnDevicePort								= 87,
	exceptionSensorHardwareNotDetectedOnDevicePort							= 88,

	exceptionInvalidSensorRamBufferAssignment										= 89,
	exceptionMoreThanSevenVarArgsToSScanf												= 90,

	exceptionDivideByZero																				= 91,

	exceptionInvalidTaskState					   												= 92,

	exceptionInvalidParmSource																	= 93,

	exceptionInvalidLCDPosition																	= 94,
	exceptionSensorDataNotReady																	= 95,

	exceptionSensorPortBadConfiguration                         = 96,

	exceptionRealAddressOutOfRange      												= 97,
	exceptionDeprecatedOpcodeNoLongerSupported									= 98,

	exceptionNestedDatalogGroupRequests						  						= 99,
	exceptionNestedDatalogUnmatchedGrouping											= 100,
	exceptionDatalogInternalError           										= 101,

	exceptionUserException1																			= 102,
	exceptionUserException2,
	exceptionUserException3,

	exceptionSScanfFormatError																	= 105,

	exceptionUnknown                    												= opEnumType 255
} TVMException;


#ifdef __cplusplus
	}
#endif

#endif

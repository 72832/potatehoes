////////////////////////////////////////////////////////////////////////////////////////////////
//
//                       Intrinsic Brick Instruction Implementations
//
// This file contains the detailed implementation code and definitions for
// RobotC intrinsics
//
////////////////////////////////////////////////////////////////////////////////////////////////

#include "ProgramType.h"
#include "PropertyTypes.h"
#include "DriveTrainTypes.h"
#include "EnumTypes.h"

#if defined(hasUserSerialPorts)
  #include "SerialPortEnums.h"
#endif

#if defined(bFirmwareStoresMotorType) || (bSmartMotorsWithEncoders() != 0) || defined(_WINDOWS)
  #include "TMotorTypes.h"
#endif

#if defined(bHasI2CPort) || defined(_WINDOWS)
	#include "I2CTypedefs.h"
#endif

typedef signed char byte;
typedef unsigned char ubyte;
typedef signed char sbyte;

typedef short word;

#define NULL ((void *) 0)


//////////////////////////////////////////////////////////////////////////////////////////
//
// Definition to 'ROBOTC' compiler of unique built-in variables used by the byte-code interpreter
//
//////////////////////////////////////////////////////////////////////////////////////////

//
// Syntax Example,
//    'intrinsic variable'      -- Compiler directive to define an internal "intrinsic" variable "array".
//    'globalVar'               -- the variable name known to the compiler.
//    'propertyVar'              -- the type of variable/parameter
//    'kNumbOfGlobalVariablesInBytes'  -- the size of the 'array' for these variables. If this is not specified
//                                 then a non-array variable is being used. e.g. 'clock', or 'message'
//                                 and can be referred to by any value
//
//    Item can now be referenced in 'ROBOTC' code with 'globalVar[23]', 'globalVar[i]', ... format
//

//
//    Note: There is an internal compiler 'kludge' so than 'random' is referred to as 'random(int)'
//          using round instead of curly brackets. Can't declare random as a 'function' because
//          assignment is allowed to set the random seed. The trick is to let 'random' be a variable
//          but it is referenced with round instead of curly brackets.
//

///////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                              Task execution Control
//
///////////////////////////////////////////////////////////////////////////////////////////////////////

intrinsic void hogCPU()             asm(opcdSystemFunctions, byte(sysFuncHogProcessor));
intrinsic void releaseCPU()         asm(opcdSystemFunctions, byte(sysFuncFreeProcessor));
intrinsic void abortTimeslice()     asm(opcdSystemFunctions, byte(sysFuncSwitchContext));

///////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                memset and memcpy
//
///////////////////////////////////////////////////////////////////////////////////////////////////////

intrinsic void memcpy(void *pToBuffer, const void *pFromBuffer, const short nNumbOfBytes)
                      asm(opcdSystemFunctions, byte(sysFuncMemCpy),
                            variableRefVoidPtr(pToBuffer),
                            variableRefVoidPtr(pFromBuffer),
                            variable(nNumbOfBytes));

intrinsic void memset(void *pToBuffer, const char nValue, const short nNumbOfBytes)
                      asm(opcdSystemFunctions, byte(sysFuncMemSet),
                            variableRefVoidPtr(pToBuffer),
                            variable(nValue),
                            variable(nNumbOfBytes));

#if defined(hasMemMoveSupport)

intrinsic void memmove(void *pToBuffer, const void *pFromBuffer, const short nNumbOfBytes)
                      asm(opcdSystemFunctions, byte(sysFuncMemMove),
                            variableRefVoidPtr(pToBuffer),
                            variableRefVoidPtr(pFromBuffer),
                            variable(nNumbOfBytes));
#endif

#if defined(hasMemCmpSupport)

intrinsic short memcmp(const void *pToBuffer, const void *pFromBuffer, const short nNumbOfBytes)
                      asm(opcdSystemFunctions, byte(sysFuncMemCmp),
                            variableRefVoidPtr(pToBuffer),
                            variableRefVoidPtr(pFromBuffer),
                            variable(nNumbOfBytes),
														functionReturn);
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                "string" functions on byte arrays
//
///////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(hasStringSupport)

intrinsic void strcpy(char *pToBuffer, const char *pFromBuffer)
                      asm(opcdSystemFunctions, byte(sysFuncStrCpy),
                          variableRefCharPtr(pToBuffer),
                          variableRefCharPtr(pFromBuffer));

intrinsic void strcpy(string &pToBuffer, const char *pFromBuffer)
                      asm(opcdSystemFunctions, byte(sysFuncStrNCpy),
                          variableRefString(pToBuffer),
                          variableRefCharPtr(pFromBuffer),
                          constantValue(20));   // 'string' variable is 20 bytes long

intrinsic void strncpy(char *pToBuffer, const char *pFromBuffer, const short nMaxBufferSize)
                      asm(opcdSystemFunctions, byte(sysFuncStrNCpy),
                          variableRefCharPtr(pToBuffer),
                          variableRefCharPtr(pFromBuffer),
                          variable(nMaxBufferSize));

intrinsic void strcat(string &sToString, const char *pFromBuffer)
                      asm(opcdSystemFunctions, byte(sysFuncStrNCat),
                          variableRefString(sToString),
                          variableRefCharPtr(pFromBuffer),
                          constantValue(20));   // 'string' variable is 20 bytes long

intrinsic void strcat(char *pToBuffer, const char *pFromBuffer)
                      asm(opcdSystemFunctions, byte(sysFuncStrCat),
                          variableRefCharPtr(pToBuffer),
                          variableRefCharPtr(pFromBuffer));

intrinsic void strncat(char *pToBuffer, const char *pFromBuffer, const short nMaxBufferSize)
                      asm(opcdSystemFunctions,
                          byte(sysFuncStrNCat),
                          variableRefCharPtr(pToBuffer),
                          variableRefCharPtr(pFromBuffer),
                          variable(nMaxBufferSize));

intrinsic void strcatSingleChar(char &pToBuffer, const char cSingleChar)
                      asm(opcdSystemFunctions, byte(sysFuncStrCatSingleChar),
                          variableRefRAM(pToBuffer),
                          variable(cSingleChar));

intrinsic void strncatSingleChar(char *pToBuffer, const char cSingleChar, const short nMaxBufferSize)
                      asm(opcdSystemFunctions, byte(sysFuncStrNCatSingleChar),
                          variableRefCharPtr(pToBuffer),
                          variable(cSingleChar),
                          variable(nMaxBufferSize));

intrinsic short strcmp(const char *pString1, const char *pString2)
                      asm(opcdSystemFunctions, byte(sysFuncStrCmp),
                          variableRefCharPtr(pString1),
                          variableRefCharPtr(pString2),
                          functionReturn);

intrinsic short strncmp(const char *pString1, const char *pString2, const short nMaxBufferSize)
                      asm(opcdSystemFunctions, byte(sysFuncStrNCmp),
                          variableRefCharPtr(pString1),
                          variableRefCharPtr(pString2),
                          variable(nMaxBufferSize),
                          functionReturn);

intrinsic char strIndex(const string &sString, const short nIndex)
                      asm(opcdSystemFunctions, byte(sysFuncStrIndex),
                          variableRefString(sString),
                          variable(nIndex),
                          functionReturn);


intrinsic void strTrim(string &sString)
                      asm(opcdSystemFunctions, byte(sysFuncStrNTrim),
                          variableRefString(sString),
													byte(parmSourceConstant), byte(20), byte(0));

intrinsic void strTrim(char *pString)
                      asm(opcdSystemFunctions, byte(sysFuncStrTrim),
                          variableRefCharPtr(pString));

intrinsic void strTrimN(char *pString, const int nMaxBufferSize)
                      asm(opcdSystemFunctions, byte(sysFuncStrNTrim),
                          variableRefCharPtr(pString),
													variable(nMaxBufferSize));

#endif


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                          Miscellaneous Variables and Functions
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(bHasRandomFunctions)
	intrinsic short rand(void)												asm(opcdSystemFunctions, byte(sysFuncRandomWord), functionReturn);
	intrinsic void srand(const short nSeedValue)			asm(opcdSystemFunctions, byte(sysFuncSRandWord),  variable(nSeedValue));
	#if defined(useLongs)
		intrinsic long randlong(void)										asm(opcdSystemFunctions, byte(sysFuncRandomLong), functionReturn);
		intrinsic void srand(const long nSeedValue)			asm(opcdSystemFunctions, byte(sysFuncSRandLong),  variableRefLong(nSeedValue));
	#endif
#else
	#define srand(seed)																random[0] = seed
	#define rand()																		random[0x7FFF]
#endif

intrinsic unsigned int function(random,							propertyRandom,  0x08000);

////////////////////////////////////////////////////////////////////////////////////////////////
//
//                  Program and Task Management Handling Opcodes
//
////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(bHasReadBatteryVoltage)
  intrinsic word propertyIndex(nImmediateBatteryLevel,					propertySystem, kSystemImmediateBatteryLevel);
#endif

#if defined(bHasStandardBatteryMonitoring)

  intrinsic word propertyIndex(nAvgBatteryLevel,								propertySystem, kSystemAvgBatteryLevel);

	#if defined(VEX2)
	  intrinsic word propertyIndex(BackupBatteryLevel,						propertySystem, kSystemBackupBatteryLevel);
	#endif

	#if (defined(bHasSoftwareControlledPowerOnOff) && !defined(VEX2) && !defined(NXT) && !defined(EV3))
		intrinsic word propertyIndex(nShutdownVoltage,              propertySystem, kSystemShutdownVoltage);
	#endif


	#if defined(NXT)
		intrinsic bool propertyIndex(bNxtRechargable,								propertySystem, kSystemNxtRechargable);
		intrinsic bool propertyIndex(bNoPowerDownOnACAdaptor,				propertySystem, kSystemNoPowerDownOnACAdaptor);
		intrinsic word propertyIndex(LowVoltageBatteryCountLimits,	propertySystem, kSystemLowVoltageBatteryCountLimits);
  #endif

#endif

#if defined(bHasSoftwareControlledPowerOnOff) //|| defined(Simulator)
  intrinsic void powerOff()                                   asm(opcdSystemFunctions, byte(sysFuncPowerOFF));
#endif

#if defined(bHasSoftwareControlledPowerOnOffTiming) //|| defined(Simulator)
	intrinsic word propertyIndex(nPowerDownDelayMinutes,        propertySystem, kSystemPowerDownDelay);
	intrinsic word propertyIndex(nPowerDownDelayMinutesDefault, propertySystem, kSystemPowerDownDelayDefault);
#endif

#if (defined(bHasI2CPort) && defined(VEX2))
  intrinsic word deprecated propertyIndex(bUseVexI2CEncoderVelocity,			propertySystem, kSystemIncludeVelocityInVexCortexI2C);
#endif



const int kLowPriority      		= 0;    // lowest priority
#undef kDefaultTaskPriority
const int kDefaultTaskPriority  = 7;    // default priority
const int kHighPriority     		= 255;  // highest priority
intrinsic word propertyIndex(nSchedulePriority,           propertySystem, kSystemTaskSchedulingPriority);

intrinsic const word propertyIndex(version,               propertySystem, kSystemFirmwareVersion);

#if defined(VEX) || defined(VEX2)
	intrinsic const word propertyIndex(nVexMasterVersion,		propertySystem, kSystemIFIMasterVersion);
#endif


// Read only operating system parameters

intrinsic const word propertyIndex(nCurrentTask,          propertySystem, kSystemCurrentTask);
intrinsic word propertyIndex(nOpcodesPerTimeslice,        propertySystem, kSystemOpcodesPerTimeslice);

#if defined(VexIQ)

	intrinsic word propertyIndex(nVexIQSchedulerSliceSize,    								propertySystem, kSystemVexIQSchedulerSliceSize);
	intrinsic word propertyIndex(nVexIQI2CSafeSendMessageTime,    						propertySystem, kSystemVexIQI2CSafeSendMessageTime);
	intrinsic bool propertyIndex(bVexIQI2CForceInlineWait,    								propertySystem, kSystemVexIQI2CForceInlineWait);
	intrinsic bool propertyIndex(bVexIQUseSmartRadioFeatures,    							propertySystem, kSystemVexIQUseSmartRadioFeatures);
	intrinsic const bool propertyIndex(bVexIQSupportsSmartRadio,    					propertySystem, kSystemVexIQSupportsSmartRadio);

	intrinsic const word propertyIndex(nVexIQActiveProgramSlot,								propertySystem, kSystemVexIQActiveProgramSlot);

	intrinsic const unsigned long propertyIndex(nVexIQIdleTaskCycles,					propertySystem, kSystemVexIQIdleTaskCycles);
	intrinsic const unsigned long propertyIndex(nVexIQUserTaskCycles,					propertySystem, kSystemVexIQUserTaskCycles);
	intrinsic const unsigned long propertyIndex(nVexIQInitTaskCycles,					propertySystem, kSystemVexIQInitTaskCycles);
	intrinsic const unsigned long propertyIndex(nVexIQSystemTime,							propertySystem, kSystemVexIQSystemTime);
	intrinsic const unsigned long propertyIndex(nVexIQUserTaskTime,						propertySystem, kSystemVexIQUserTaskTime);

	intrinsic const unsigned long propertyIndex(nVexIQRawTime,								propertySystem, kSystemVexIQRawTime);
	intrinsic const short propertyIndex(nVexIQUserTaskStartTick,							propertySystem, kSystemVexIQUserTaskStartTick);
	intrinsic const short propertyIndex(nVexIQUserTaskCurrTimeSliceTotalTicks,propertySystem, kSystemVexIQUserTaskCurrTimeSliceTotalTicks);
	intrinsic const short propertyIndex(nVexIQUserTaskCurrTimeIncrementalTicks,	propertySystem, kSystemVexIQUserTaskCurrTimeSliceIncrementalTicks);
	typedef enum
	{
		stateinitializeMenu,
		stateStartupDelay,
		stateWaitForButtonNone,
		stateWaitForButtonPress,
		stateWaitForUpDownRelease,
		stateWaitForSelectRelease,
	} TChooserStates;
	intrinsic const TChooserStates propertyIndex(vexIQChooserState,						propertySystem, kSystemVexIQChooserState);

	intrinsic bool propertyIndex(bUserControlsLCDButtons,											propertySystem, kSystemVexIQUserControlsLCDButtons);

#endif


#if defined(VEX)
	intrinsic const short propertyIndex(pStartOfFlashFileHdr, propertySystem, kSystemStartOfFlashFileHeader);
#else
	intrinsic const long  propertyIndex(pStartOfFlashFileHdr, propertySystem, kSystemStartOfFlashFileHeader);
#endif

#if defined(NXT)
	intrinsic void getFirmwareBuildDate(string &sDate)			asm(opcdSystemFunctions, byte(sysFuncGetFirmwareDate), variableRefString(sDate));
	intrinsic word propertyIndex(nMaxDataFiles,							propertySystem, kSystemNxtMaxDataFileCount);
	intrinsic word propertyIndex(nMaxDataFileSize,					propertySystem, kSystemNxtMaxDataFileSize);
	intrinsic word propertyIndex(bHideDataFiles,						propertySystem, kSystemNxtHideDataFile);

	intrinsic word propertyIndex(nNxtButtonTask,            propertySystem, kSystemNxtButtonTask);
	intrinsic word propertyIndex(nNxtExitClicks,            propertySystem, kSystemNxtExitClicks);

	intrinsic TNxtButtons propertyIndex(nNxtButtonPressed,  propertySystem, kSystemNxtButtonPressed); 	intrinsic TNxtButtons propertyIndex(nNxtButtonTransition,propertySystem,kSystemNxtButtonTransition);

	intrinsic word propertyIndex(nI2CRetries,               propertySystem, kSystemNxtI2CRetries);
	intrinsic bool propertyIndex(bI2CCheckAcks,             propertySystem, kSystemNxtI2CCheckACKs);
	intrinsic word propertyIndex(nAvailFlash,               propertySystem, kSystemNxtAvailFlash);
#endif

#if defined(VEX)
  intrinsic ubyte propertyIndex(nVEXDurationAutonomous,     propertySystem, kSystemVEXDurationAutonomous);
  intrinsic ubyte propertyIndex(nVEXDurationUserControl,    propertySystem, kSystemVEXDurationUserControl);

  typedef enum
  {
    vrNoXmiters           = 0,    // No transmitters connected
    vrXmit1               = 1,    // Transmitter 1 connected
    vrXmit2               = 2,    // Transmitter 2 connected
    vrXmit1and2           = 3,    // Transmitter 1 & 2 connected
    vrNoCompetitionSwitch = 4,
    vrBit4                = 8,
  } TVexReceiverState;
  intrinsic const TVexReceiverState propertyIndex(nVexRCReceiveState,   propertySystem, kSystemVEXRCReceiveState);

#elif defined(VEX2)

  typedef enum
  {
    vrNoXmiters           = 0,        // No transmitters connected
    vrXmit1               = 0x01,     //                          1 == Transmitter 1 connected
    vrXmit2               = 0x02,     //                          1 == Transmitter 2 connected
    vrBit2                = 0x04,     // Unused
    vrCompetitionSwitch   = 0x08,     // 0 == No Comp Switch      1 == Competition Switch attached.
		vrResetSlave					= 0x10,			// Unused
		vrGameController			= 0x20,     // 0 == Legacy75MHz,        1 == Game Controller
		vrAutonomousMode      = 0x40,     // 0 == Driver Control,     1 == Autonomous Mode
		vrDisabled      			= 0x80,     // 0 == Enabled             1 == Disabled.
  } TVexReceiverState;
  intrinsic const TVexReceiverState propertyIndex(nVexRCReceiveState,   propertySystem, kSystemVEXRCReceiveState);
#endif

#if defined(VEX) || defined(VEX2)
  intrinsic const bool propertyIndex(bVEXNETActive,   propertySystem, kSystemVEXNEXTActive);

  #define nVexPwmOwner        nIfiPwmOwner         // Backward compatibility with earlier VEX only
  #define nVexSPIMsgCounts    nIfiSPIMsgCounts     // Backward compatibility with earlier VEX only
  #define bVexAutonomousMode  bIfiAutonomousMode   // Backward compatibility with earlier VEX only

	intrinsic bool propertyIndex(bIfiPwmOverride,											propertySystem, kSystemIFIPWMOverride);
	intrinsic const word propertyIndex(nIfiSPIMsgCounts,							propertySystem, kSystemIFISPIMsgCounts);
  intrinsic const bool propertyIndex(bIfiRobotDisabled,							propertySystem, kSystemIFIRobotDisabled);

#endif

#if defined(VEX)
  intrinsic word propertyIndex(nIfiPwmOwner,                propertySystem, kSystemIFIPwmOwner);
  typedef enum
  {
    UserVEXBlackEye           = 0x01,  // Shuts off the EYE on VEX
    UserAutonomous            = 0x02,  // Enables Autonomous Robot Operation
    UserPwmOverride           = 0x04,  // Enables User to control PWMs as well when transmitter is off
    UserLatchVEXAutonDuration = 0x08,  // when set 'nVEXDurationAutonomous'  is latched as autonomous interval data
    UserLatchVEXUserDuration  = 0x10,  // when set 'nVEXDurationUserControl' is latched as  the interval to terminate the match
  } TUserCmds;
  intrinsic const TUserCmds propertyIndex(nIfiUserCmd,							propertySystem, kSystemIFIUserCmd);
  intrinsic bool propertyIndex(bIfiAutonomousMode,									propertySystem, kSystemIFIAutonomousMode);

	#define setVEXBlackEye(nNewValue) \
		do\
		{\
			if (nNewValue)\
				nIfiUserCmd |= UserVEXBlackEye;\
			else\
				nIfiUserCmd &= ~UserVEXBlackEye;\
		} while(false)
	#define getVEXBlackEye()										((nIfiUserCmd & UserVEXBlackEye) != 0)

#elif defined(VEX2)

  intrinsic const bool propertyIndex(bIfiAutonomousMode,						propertySystem, kSystemIFIAutonomousMode);
  intrinsic bool propertyIndex(bIfiAutonomousModeBySlave,						propertySystem, kSystemIFIAutonomousModeBySlave);
  intrinsic TProgramType propertyIndex(nCurrCortexProgramType,		  propertySystem, kSystemCurrCortexProgramType);
  intrinsic TProgramType propertyIndex(nNextCortexProgramType,		  propertySystem, kSystemNextCortexProgramType);
#endif


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//       Semaphore Support
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(bHasSemaphoreSupport)

	typedef struct
	{
	  ubyte nOwningTask;
	  ubyte nLockCount;
	} TSemaphore;

  intrinsic void semaphoreInitialize(TSemaphore &nSemaphore)
                asm(opcdSystemFunctions, byte(sysFuncSemaphoreInitialize), variable(nSemaphore));

  intrinsic void semaphoreLock(TSemaphore &nSemaphore, const int waitTime = 0x7FFF)
                asm(opcdSystemFunctions, byte(sysFuncSemaphoreLock), variable(nSemaphore), variable(waitTime));

  intrinsic void semaphoreUnlock(TSemaphore &nSemaphore)
                asm(opcdSystemFunctions, byte(sysFuncSemaphoreUnlock), variable(nSemaphore));

  #define getSemaphoreTaskOwner(nSemaphore) nSemaphore.nOwningTask

  #define bDoesTaskOwnSemaphore(nSemaphore) (nSemaphore.nOwningTask == (ubyte) nCurrentTask)

#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//       Interface to Optional Drive Train and Joystick COnfiguration
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(hasDriveTrainConfiguration)

intrinsic void setDriveTrainMotors(compileConst TDriveTrainType nItsDriveTrainType, compileConst ubyte motorIndex0, compileConst ubyte motorIndex1, compileConst ubyte motorIndex2, compileConst ubyte motorIndex3)
          asm(opcdSystemFunctions, byte(sysFuncConfigureDriveTrainMotors), variableCompileConstByte(nItsDriveTrainType),
          variableCompileConstByte(motorIndex0), variableCompileConstByte(motorIndex1), variableCompileConstByte(motorIndex2), variableCompileConstByte(motorIndex3));

intrinsic void setDriveTrainWheels(compileConst short nAxleLength, compileConst short nGearRatioA, compileConst short nGearRatioB, compileConst short nWheelBase, compileConst short nWheelDiameter)
          asm(opcdSystemFunctions, byte(sysFuncConfigureDriveTrainWheels), variableCompileConstWord(nAxleLength),
          variableCompileConstWord(nGearRatioA), variableCompileConstWord(nGearRatioB), variableCompileConstWord(nWheelBase), variableCompileConstWord(nWheelDiameter));

intrinsic void getDriveTrainConfiguration(TDriveTrainConfig &nDriveTrain)
          asm(opcdSystemFunctions, byte(sysFuncGetDriveTrainConfig), variableRefRAM(nDriveTrain));

intrinsic void setGripperArm(compileConst TGripperArmType nItsGripperArmType, compileConst ubyte motorIndex0, compileConst ubyte motorIndex1, compileConst ubyte motorIndex2, compileConst ubyte motorIndex3)
          asm(opcdSystemFunctions, byte(sysFuncConfigureGripperArm), variableCompileConstByte(nItsGripperArmType),
          variableCompileConstByte(motorIndex0), variableCompileConstByte(motorIndex1), variableCompileConstByte(motorIndex2), variableCompileConstByte(motorIndex3));

intrinsic void getGripperArmConfiguration(TGripperArmType &nGripperArmType, short &nMotorIndex_0, short &nMotorIndex_1, short &nMotorIndex_2, short &nMotorIndex_3)
          asm(opcdSystemFunctions, byte(sysFuncGetGripperArmTypeAndMotors), variableRefRAM(nGripperArmType),
          variableRefWord(nMotorIndex_0), variableRefWord(nMotorIndex_1), variableRefWord(nMotorIndex_2), variableRefWord(nMotorIndex_3));

          #endif

#if defined(hasJoystickAxisConfiguration)

intrinsic void setJoystickDrive(compileConst TJoystickDriveType nJoystickDriveType, compileConst TJoystickAxisScaling nJoystickAxisScaling)
          asm(opcdSystemFunctions, byte(sysFuncConfigureGameController), variableCompileConstByte(nJoystickDriveType), variableCompileConstByte(nJoystickAxisScaling));

intrinsic void getDriveTrainJoysticks(short &nValueAxis_0, short &nValueAxis_1)
          asm(opcdSystemFunctions, byte(sysFuncGetDriveTrainJoystickValues), variableRefWord(nValueAxis_0), variableRefWord(nValueAxis_1));

#endif


#if defined(VexIQ)

typedef enum
{
	dvcStatusNone				= 0,
	dvcStatusActive			= 1,
} TDeviceStatus;


//Device Table Sensor Types
typedef enum TVexIQDeviceTypes
{
	vexIQ_SensorNone            = 0,
	vexIQ_SensorMOTOR           = 2,
	vexIQ_SensorLED             = 3,
	vexIQ_SensorRGB             = 4,
	vexIQ_SensorBUMPER          = 5,
	vexIQ_SensorGYRO				    = 6,
	vexIQ_SensorSONAR		        = 7,
	//#define NO_SENSOR                     0
	//#define MOTOR_SENSOR                  2
	//#define LED_SENSOR                    3
	//#define RGB_SENSOR                    4
	//#define BUMPER_SENSOR                 5
	//#define GYRO_SENSOR				            6
	//#define SONAR_SENSOR		              7
} TVexIQDeviceTypes;

intrinsic void getVexIqDeviceInfo(short nDeviceIndex, TVexIQDeviceTypes &nDeviceType, TDeviceStatus &nDeviceStatus, short &nDeviceVersion)
 					asm(opcdSystemFunctions, byte(sysFuncGetVexIqGetDeviceInfo), variable(nDeviceIndex), variableRefByte(nDeviceType), variableRefByte(nDeviceStatus), variableRefWord(nDeviceVersion));

#endif


#if defined(bHasFlashFileSystem)

	#if !defined(VEX) && !defined(NXT) && !defined(EV3)
	  intrinsic short getStartupProgramIndex()
	                asm(opcdSystemFunctions, byte(sysFuncStartupProgramIndex), functionReturn);
	  intrinsic short getIdleTimeProgramIndex()
	                asm(opcdSystemFunctions, byte(sysFuncIdleTimeProgramIndex), functionReturn);
	#endif

	intrinsic short bValidFile(const int nFileIndex)
                asm(opcdSystemFunctions, byte(sysFuncValidFileIndex), variable(nFileIndex), functionReturn);

	#if !(defined(VEX))

		#if defined(bHasFlashFileSystemDelete)
			intrinsic void deleteFile(const int nFileIndex)
											asm(opcdSystemFunctions, byte(sysFuncDeleteFile), variable(nFileIndex));
		#endif

		intrinsic TFileExtensionTypes getFileType(const int nFileIndex)
		                asm(opcdSystemFunctions, byte(sysFuncGetFileType), variable(nFileIndex), functionReturn);

		#if defined(hasStringSupport)
			intrinsic void getFileName(const int nFileIndex, string &sFileName)
											asm(opcdSystemFunctions, byte(sysFuncGetFileName), variable(nFileIndex), variableRefString(sFileName));
		#endif

  #endif

#endif

#if defined(hasUserSerialPorts)

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                       Optional User Control of Serial Port Two (VEX and ????)
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef enum TUARTs
{
  UART0         = 0,
  UART1         = 1,
  UART2         = 2,
  UART3         = 3,

  uartOne       = 1,
  uartTwo       = 2,
  uartThree     = 3,
} TUARTs;


intrinsic void configureSerialPort(const TUARTs nPort, TSerialPortMode nMode) asm(opcdSystemFunctions, byte(sysFuncConfigureSerialPort),
																																					variable(nPort),
																																					variable(nMode));

intrinsic void setBaudRate(const TUARTs nPort, TBaudRate nBaudRate)asm(opcdSystemFunctions, byte(sysFuncSerialSetBaudRate),
																																					variable(nPort),
																																					variable(nBaudRate));

#if defined(VEX2) || defined(_WINDOWS)
  intrinsic TSerialPortMode getUartConfig(const TUARTs nPort)asm(opcdSystemFunctions, byte(sysFuncSerialGetConfig),
																																					variable(nPort),
																																					functionReturn);
#endif

intrinsic short getChar(const TUARTs nPort)                        asm(opcdSystemFunctions, byte(sysFuncSerialGetChar),
																																					variable(nPort),
																																					functionReturn);

intrinsic void sendChar(const TUARTs nPort, short nChar)           asm(opcdSystemFunctions, byte(sysFuncSerialSendChar),
																																					variable(nPort),
																																					variable(nChar));

intrinsic short bXmitComplete(const TUARTs nPort)                  asm(opcdSystemFunctions, byte(sysFuncSerialGetXmitComplete),
																																					variable(nPort),
																																					functionReturn);
#endif //#if defined(hasUserSerialPorts)

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//       Interface to Optional IFI LCD Display and Buttons
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(hasLCDButtons) || defined(_WINDOWS)

	enum TControllerButtons
	{
		kButtonNone     = 0x00,
#if defined(VexIQ)
		kButtonUp				= 0x01,
		kButtonDown			= 0x04,
		kButtonSelect		= 0x02,
		kButtonExit			= 0x08,
#else
		kButtonLeft     = 0x01,
		kButtonCenter   = 0x02,
		kButtonRight    = 0x04,
		kButtonExit     = 0x08,
		kButtonExit2    = 0x10,
#endif
	};
	intrinsic const TControllerButtons propertyIndex(nLCDButtons, propertySystem, kSystemLCDButtons);

#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//       Interface to Optional IFI LCD Display and Buttons
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(VexIQ)

intrinsic void setUserDebugWindow(bool bSetting) asm(opcdSystemFunctions, byte(sysFuncSetVexIqUserDebugWindow), variable(bSetting));

//intrinsic void drawUserText(int nPixelRow, int nPixelColumn, char *pFormatSpec, ...)
//                      asm(opcdSystemFunctions, byte(sysFuncVexIqDrawText),
//                      variable(nPixelRow), variable(nPixelColumn),
//                      variableRefCharPtr(pFormatSpec), varArgListOfLongs);
//intrinsic void drawUserTextLine(int nTextLine, char *pFormatSpec, ...)
//                      asm(opcdSystemFunctions, byte(sysFuncVexIqDrawTextLine),
//                      variable(nTextLine),
//                      variableRefCharPtr(pFormatSpec), varArgListOfLongs);

#define drawUserTextLine displayTextLine
#define drawUserText displayStringAt
#endif

#if defined(hasAlphaNumericLCDDisplayViaUART)

intrinsic word propertyIndex(nLCDRefreshRate,             propertySystem, kSystemLCDRefreshRate);

#if defined(hasLCDBacklight)
	intrinsic bool propertyIndex(bLCDBacklight,             propertySystem, kSystemLCDBacklight);
#endif

intrinsic void clearLCDLine(const int nLine)
                asm(opcdSystemFunctions, byte(sysFuncLCDClearLCDLine), variable(nLine));

intrinsic void displayLCDPos(const int nLine, const int nPos)
							 asm(opcdSystemFunctions, byte(sysFuncLCDPosition), variable(nLine), variable(nPos));
intrinsic void setLCDPosition(const int nLine, const int nPos)
							 asm(opcdSystemFunctions, byte(sysFuncLCDPosition), variable(nLine), variable(nPos));

intrinsic void displayFileName(const int nLine, const int nSlot)
                asm(opcdSystemFunctions, byte(sysFuncLCDDisplayProgramName), variable(nLine), variable(nSlot));

intrinsic void displayLCDChar(const int nLine, const int nPos, const int cChar)
                asm(opcdSystemFunctions, byte(sysFuncLCDPosition), variable(nLine), variable(nPos),
								    opcdSystemFunctions, byte(sysFuncLCDPutNextChar), variable(cChar));
intrinsic void displayLCDNumber(const int nLine, const int nPos, const int nValue, const int nPrecision = -1)
                asm(opcdSystemFunctions, byte(sysFuncLCDPosition), variable(nLine), variable(nPos),
								    opcdSystemFunctions, byte(sysFuncLCDPutNextShort), variable(nPrecision), variable(nValue));
intrinsic void displayLCDString(const int nLine, const int nPos, compileConst string &sString)
							  asm(opcdSystemFunctions, byte(sysFuncLCDPosition), variable(nLine), variable(nPos),
								    opcdSystemFunctions, byte(sysFuncLCDPutNextConstString), stringConstant(sString));

intrinsic void displayLCDCenteredString(const int nLine, compileConst string &sString)
							 asm(opcdSystemFunctions, byte(sysFuncLCDPutCenteredConstString),
							      variable(nLine), stringConstant(sString));

intrinsic void displayNextLCDChar(const int cChar)
							 asm(opcdSystemFunctions, byte(sysFuncLCDPutNextChar), variable(cChar));
intrinsic void displayNextLCDNumber(const int nValue, const int nPrecision = 0x40)
							 asm(opcdSystemFunctions, byte(sysFuncLCDPutNextShort), variable(nPrecision), variable(nValue));
intrinsic void displayNextLCDString(compileConst string sString)
							 asm(opcdSystemFunctions, byte(sysFuncLCDPutNextConstString), stringConstant(sString));
intrinsic void displayNextLCDString(compileConst char *pzString)
							 asm(opcdSystemFunctions, byte(sysFuncLCDPutNextConstString), stringConstant(pzString));

#if defined(hasStringSupport)
intrinsic void displayLCDString(const int nLine, const int nPos, string &sString)
							  asm(opcdSystemFunctions, byte(sysFuncLCDPosition), variable(nLine), variable(nPos),
								    opcdSystemFunctions, byte(sysFuncLCDPutNextString), variableRefString(sString));
intrinsic void displayLCDString(const int nLine, const int nPos, const char *pzString)
							  asm(opcdSystemFunctions, byte(sysFuncLCDPosition), variable(nLine), variable(nPos),
								    opcdSystemFunctions, byte(sysFuncLCDPutNextString), variableRefCharPtr(pzString));

intrinsic void displayNextLCDString(string &sString)
							 asm(opcdSystemFunctions, byte(sysFuncLCDPutNextString), variableRefString(sString));
intrinsic void displayNextLCDString(const char *pzString)
							 asm(opcdSystemFunctions, byte(sysFuncLCDPutNextString), variableRefCharPtr(pzString));

intrinsic void displayLCDCenteredString(const int nLine, string &sString)
							 asm(opcdSystemFunctions, byte(sysFuncLCDPutCenteredString),
							      variable(nLine), variableRefString(sString));
intrinsic void displayLCDCenteredString(const int nLine, const char *pzString)
							 asm(opcdSystemFunctions, byte(sysFuncLCDPutCenteredString),
							      variable(nLine), variableRefCharPtr(pzString));
#endif

#endif //#if defined(hasAlphaNumericLCDDisplayViaUART)

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//       Interface to VEX RF Functions (standard VEX and Playstation Controller)
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////



#if defined(VEX) || defined(VEX2) || defined(VexIQ)

typedef enum TVexJoysticks
	{
#if defined(VexIQ)
		ChA               = 0,    // Channel 1
		ChB               = 1,    // Channel 2
		ChC               = 2,    // Channel 3
		ChD               = 3,    // Channel 4

#elif defined(VEX) | defined(VEX2)
	//Channels
		Ch1               = 0,    // Channel 1
		Ch2               = 1,    // Channel 2
		Ch3               = 2,    // Channel 3
		Ch4               = 3,    // Channel 4
		Ch5               = 4,    // Channel 5
		Ch6               = 5,    // Channel 6

	// Second receiver / transmitter can also be (optionall) connected
		Ch1Xmtr2          = 6,
		Ch2Xmtr2          = 7,
		Ch3Xmtr2          = 8,
		Ch4Xmtr2          = 9,
		Ch5Xmtr2          = 10,
		Ch6Xmtr2          = 11,
#endif

		// Button Bit Maps

#if defined(VexIQ)		//All Buttons

		BtnEUp			= 14,
		BtnEDown		= BtnEUp +  1,
		BtnFUp			= BtnEUp +  2,
		BtnFDown		= BtnEUp +  3,
		BtnLUp			= BtnEUp +  4,
		BtnLDown		= BtnEUp + 	5,
		BtnRUp			= BtnEUp + 	6,
		BtnRDown		= BtnEUp +  7,

#elif defined(VEX) | defined(VEX2)

		Btn5D				= 14,
		Btn5U				= Btn5D +  1,
		Btn6D				= Btn5D +  2,
		Btn6U				= Btn5D +  3,

#if defined(VEX2)
		Btn8D				= Btn5D +  4,
		Btn8L				= Btn5D +  5,
		Btn8U				= Btn5D +  6,
		Btn8R				= Btn5D +  7,

		Btn7D				= Btn5D +  8,
		Btn7L				= Btn5D +  9,
		Btn7U				= Btn5D + 10,
		Btn7R				= Btn5D + 11,
#endif

		Btn5DXmtr2				= 26,
		Btn5UXmtr2				= Btn5DXmtr2 +  1,
		Btn6DXmtr2				= Btn5DXmtr2 +  2,
		Btn6UXmtr2				= Btn5DXmtr2 +  3,

#if defined(VEX2)
		Btn8DXmtr2				= Btn5DXmtr2 +  4,
		Btn8LXmtr2				= Btn5DXmtr2 +  5,
		Btn8UXmtr2				= Btn5DXmtr2 +  6,
		Btn8RXmtr2				= Btn5DXmtr2 +  7,

		Btn7DXmtr2				= Btn5DXmtr2 +  8,
		Btn7LXmtr2				= Btn5DXmtr2 +  9,
		Btn7UXmtr2				= Btn5DXmtr2 + 10,
		Btn7RXmtr2				= Btn5DXmtr2 + 11,

		AccelX       = 38,
		AccelY       = AccelX + 1,
		AccelZ       = AccelX + 2,

		AccelXXmtr2       = 41,
		AccelYXmtr2       = AccelXXmtr2 + 1,
		AccelZXmtr2       = AccelXXmtr2 + 2,
#endif

#endif //All Button EndIf
    kNumbOfVexRFIndices,
	} TVexJoysticks;

const int kNumbOfVexRFJoysticks   = 14;   // deprecated

intrinsic const word property(vexRT,     propertyIFIRadioControl,        kNumbOfVexRFIndices, TVexJoysticks);
#define getJoystickValue(channel) vexRT[channel]

#define vexJSRightH 		Ch1
#define vexJSRightV 		Ch2
#define vexJSLeftV 			Ch3
#define vexJSLeftH 			Ch4
#define vexJSChannel5 	Ch5
#define vexJSChannel6 	Ch6

#endif


///////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                  NXT LCD Display Opcodes
//
///////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(bHasLCDContrast)
  intrinsic short propertyIndex(nLCDContrast,           propertySystem, kSystemLCDContrast);
	intrinsic short propertyIndex(nLCDContrastDefault,    propertySystem, kSystemLCDDefaultContrast);
#endif

#if defined(bHasLCDGraphicsDisplay)

#if defined(NXT)
  intrinsic bool propertyIndex(bNxtLCDStatusDisplay,    propertySystem, kSystemNxtLCDStatusDisplay);
#endif

#if defined(bHasRemoteButtons) || defined(_WINDOWS)
  intrinsic short propertyIndex(nRemoteButton,					propertySystem, kSystemRemoteButton);
#endif

intrinsic void eraseDisplay()                       asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawEraseScreen));

#if defined(VexIQ)
intrinsic void eraseUserScreenArea()                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawEraseUserScreenArea));
#endif

intrinsic void getLcdScreenSize(const short &X, const short &Y)
								asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawGetScreenSize), variableRefWord(X), variableRefWord(Y));

intrinsic void displayString(const int nLineNumber, char *pChar, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawStringVarArgs),
                                variable(nLineNumber),
                                variableRefCharPtr(pChar),
                                varArgListOfLongs);

intrinsic void displayString(const int nLineNumber, const string sFormatString, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawTextLineVarArgs),
                                variable(nLineNumber),
                                variableRefString(sFormatString),
                                varArgListOfLongs);

intrinsic void displayTextLine(const int nLineNumber, char *pChar, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawTextLineVarArgs),
                                variable(nLineNumber),
                                variableRefCharPtr(pChar),
                                varArgListOfLongs);

intrinsic void displayTextLine(const int nLineNumber, const string sFormatString, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawTextLineVarArgs),
                                variable(nLineNumber),
                                variableRefString(sFormatString),
                                varArgListOfLongs);

#if defined(VexIQ) || defined(EV3)

intrinsic void displayInverseString(const int nLineNumber, char *pChar, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawInverseTextLineVarArgs),
                                variable(nLineNumber),
                                variableRefCharPtr(pChar),
                                varArgListOfLongs);

intrinsic void displayInverseString(const int nLineNumber, const string sFormatString, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawInverseTextLineVarArgs),
                                variable(nLineNumber),
                                variableRefString(sFormatString),
                                varArgListOfLongs);

intrinsic void displayInverseTextLine(const int nLineNumber, char *pChar, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawInverseTextLineVarArgs),
                                variable(nLineNumber),
                                variableRefCharPtr(pChar),
                                varArgListOfLongs);

intrinsic void displayInverseTextLine(const int nLineNumber, const string sFormatString, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawInverseTextLineVarArgs),
                                variable(nLineNumber),
                                variableRefString(sFormatString),
                                varArgListOfLongs);
#endif

intrinsic void displayBigTextLine(const int nLineNumber, char *pChar, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawBigTextLineVarArgs),
                                variable(nLineNumber),
                                variableRefCharPtr(pChar),
                                varArgListOfLongs);

intrinsic void displayBigTextLine(const int nLineNumber, const string sFormatString, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawBigTextLineVarArgs),
                                variable(nLineNumber),
                                variableRefString(sFormatString),
                                varArgListOfLongs);

#define displayClearTextLine(nLineNumber)     displayTextLine(nLineNumber, "")

intrinsic void displayCenteredTextLine(const int nLineNumber, char *pChar, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawCenteredTextLineVarArgs),
                                variable(nLineNumber),
                                variableRefCharPtr(pChar),
                                varArgListOfLongs);

intrinsic void displayCenteredTextLine(const int nLineNumber, const string sFormatString, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawCenteredTextLineVarArgs),
                                variable(nLineNumber),
                                variableRefString(sFormatString),
                                varArgListOfLongs);

intrinsic void displayCenteredBigTextLine(const int nLineNumber, const char *pChar, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawCenteredBigTextLineVarArgs),
                                variable(nLineNumber),
                                variableRefCharPtr(pChar),
                                varArgListOfLongs);

intrinsic void displayCenteredBigTextLine(const int nLineNumber, const string sFormatString, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawCenteredBigTextLineVarArgs),
                                variable(nLineNumber),
                                variableRefString(sFormatString),
                                varArgListOfLongs);

#define drawTextAt displayStringAt

intrinsic void displayStringAt(const int xPos, const int yPos, char *pChar, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawStringAtCoordVarArgs),
                                variable(xPos), variable(yPos),
                                variableRefCharPtr(pChar),
                                varArgListOfLongs);

intrinsic void displayStringAt(const int xPos, const int yPos, const string sFormatString, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawStringAtCoordVarArgs),
                                variable(xPos), variable(yPos),
                                variableRefString(sFormatString),
                                varArgListOfLongs);

intrinsic void displayBigStringAt(const int xPos, const int yPos, const char *pChar, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawBigStringAtCoordVarArgs),
                                variable(xPos), variable(yPos),
                                variableRefCharPtr(pChar),
                                varArgListOfLongs);

intrinsic void displayBigStringAt(const int xPos, const int yPos, const string sFormatString, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawBigStringAtCoordVarArgs),
                                variable(xPos), variable(yPos),
                                variableRefString(sFormatString),
                                varArgListOfLongs);

#if defined(VexIQ) || defined(EV3)

intrinsic void displayInverseStringAt(const int xPos, const int yPos, char *pChar, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawInverseStringAtCoordVarArgs),
                                variable(xPos), variable(yPos),
                                variableRefCharPtr(pChar),
                                varArgListOfLongs);

intrinsic void displayInverseStringAt(const int xPos, const int yPos, const string sFormatString, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawInverseStringAtCoordVarArgs),
                                variable(xPos), variable(yPos),
                                variableRefString(sFormatString),
                                varArgListOfLongs);

intrinsic void displayInverseBigStringAt(const int xPos, const int yPos, const char *pChar, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawInverseBigStringAtCoordVarArgs),
                                variable(xPos), variable(yPos),
                                variableRefCharPtr(pChar),
                                varArgListOfLongs);

intrinsic void displayInverseBigStringAt(const int xPos, const int yPos, const string sFormatString, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawInverseBigStringAtCoordVarArgs),
                                variable(xPos), variable(yPos),
                                variableRefString(sFormatString),
                                varArgListOfLongs);
#endif

intrinsic void scrollText(const char *pChar, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawScrollTextVarArgs),
                                variableRefCharPtr(pChar),
                                varArgListOfLongs);

intrinsic void scrollText(const string sFormatString, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawScrollTextVarArgs),
                                variableRefString(sFormatString),
                                varArgListOfLongs);

intrinsic bool getPixel(const int xPos, const int yPos)    asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawGetPixel), variable(xPos), variable(yPos), functionReturn);
intrinsic void setPixel(const int xPos, const int yPos)    asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawSetPixel), variable(xPos), variable(yPos));
intrinsic void clearPixel(const int xPos, const int yPos)  asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawClearPixel), variable(xPos), variable(yPos));
intrinsic void invertPixel(const int xPos, const int yPos) asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawInvertPixel), variable(xPos), variable(yPos));

intrinsic void drawLine(const int xPos, const int yPos, const int xPosTo, const int yPosTo)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawLine),  variable(xPos), variable(yPos), variable(xPosTo), variable(yPosTo));

intrinsic void eraseLine(const int xPos, const int yPos, const int xPosTo, const int yPosTo)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawEraseLine),  variable(xPos), variable(yPos), variable(xPosTo), variable(yPosTo));

intrinsic void invertLine(const int xPos, const int yPos, const int xPosTo, const int yPosTo)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawInvertLine),  variable(xPos), variable(yPos), variable(xPosTo), variable(yPosTo));

#if defined(NXT)
intrinsic void displayRICFile(const int nleft, const int nBottom, const char * pzFileName)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawRICFile),
                                        variable(nleft), variable(nBottom),
                                        variableRefFNExists(pzFileName));
#endif



#if defined(NXT) || defined(EV3)

	// NOTE: Parameters are 'left/top'  and not 'center of circle' as in intrinsics below

	#define drawCircle(Left, Top, Diameter) drawEllipse(Left, Top, Left + Diameter - 1, Top - Diameter + 1)

#else

	// NOTE: Parameters are 'center of circle' and not 'left/top' as in macro above

	intrinsic void drawCircle(const int CenterX, const int CenterY, const int Radius)
	              asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawCircle),  variable(CenterX), variable(CenterY), variable(Radius));
	intrinsic void fillCircle(const int CenterX, const int CenterY, const int Radius)
	              asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawFillCircle),  variable(CenterX), variable(CenterY), variable(Radius));
	intrinsic void eraseCircle(const int CenterX, const int CenterY, const int Radius)
	              asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawEraseCircle),  variable(CenterX), variable(CenterY), variable(Radius));
	intrinsic void drawInvertCircle(const int CenterX, const int CenterY, const int Radius)
	              asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawInvertCircle),  variable(CenterX), variable(CenterY), variable(Radius));

#endif

intrinsic void drawEllipse(const int Left, const int Top, const int Right, const int Bottom)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawEllipse),  variable(Left), variable(Top), variable(Right), variable(Bottom));
intrinsic void fillEllipse(const int Left, const int Top, const int Right, const int Bottom)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawFillEllipse),  variable(Left), variable(Top), variable(Right), variable(Bottom));
intrinsic void eraseEllipse(const int Left, const int Top, const int Right, const int Bottom)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawEraseEllipse),  variable(Left), variable(Top), variable(Right), variable(Bottom));
intrinsic void drawInvertEllipse(const int Left, const int Top, const int Right, const int Bottom)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawInvertEllipse),  variable(Left), variable(Top), variable(Right), variable(Bottom));

intrinsic void drawRect(const int Left, const int Top, const int Right, const int Bottom)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawRect),  variable(Left), variable(Top), variable(Right), variable(Bottom));
intrinsic void drawInvertRect(const int Left, const int Top, const int Right, const int Bottom)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawInvertRect),  variable(Left), variable(Top), variable(Right), variable(Bottom));
intrinsic void eraseRect(const int Left, const int Top, const int Right, const int Bottom)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawEraseRect), variable(Left), variable(Top), variable(Right), variable(Bottom));
intrinsic void fillRect(const int Left, const int Top, const int Right, const int Bottom)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawFillRect), variable(Left), variable(Top), variable(Right), variable(Bottom));

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//				Intrinsic Functions for Drawing Text Menus on Controllers with Graphical LCD Screens
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(hasLCDMenuFunctions)

	intrinsic void menuInitialize()														asm(opDrawMenuOnGraphicsLCDDisplay, byte(menuOpInitialize));
	intrinsic void menuSetTitle(const char *pTitleText)				asm(opDrawMenuOnGraphicsLCDDisplay, byte(menuOpSetTitle), variable(pTitleText));
	intrinsic void menuSetProgramName(const char *pProgramName)  asm(opDrawMenuOnGraphicsLCDDisplay, byte(menuOpSetProgramName), variable(pProgramName));

	intrinsic void menuRepaint()															asm(opDrawMenuOnGraphicsLCDDisplay, byte(menuOpRepaint));
	intrinsic void menuScroll(bool bScrollDown)								asm(opDrawMenuOnGraphicsLCDDisplay, byte(menuOpScroll), variable(bScrollDown));
	intrinsic ubyte menuGetSelection()												asm(opDrawMenuOnGraphicsLCDDisplay, byte(menuOpGetSelection), functionReturn);
	intrinsic void menuAddItem(ubyte nMenuID, const char *pMenuText)asm(opDrawMenuOnGraphicsLCDDisplay, byte(menuOpAddItem), variable(nMenuID), variable(pMenuText));
	intrinsic void menuUpdateCommandText(ubyte nMenuID, const char *pMenuText)asm(opDrawMenuOnGraphicsLCDDisplay, byte(menuOpUpdateCommandText), variable(nMenuID), variable(pMenuText));
	intrinsic bool menuIsCommandVisible(ubyte nMenuID, const char *pMenuText)asm(opDrawMenuOnGraphicsLCDDisplay, byte(menuOpIsCommandVisible), variable(nMenuID), functionReturn);

	//intrinsic void drawTextCenteredInUserScreenArea(char *pText) asm(opDrawMenuOnGraphicsLCDDisplay, byte(menuOpDrawTextCenteredInUserScreenArea), variable(pText));
	intrinsic void setInitialProgramRunningScreen()						asm(opDrawMenuOnGraphicsLCDDisplay, byte(menuOpSetInitialProgramRunningScreen));

  intrinsic bool propertyIndex(bUserWantsExitButton, propertySystem, kSystemUserWantsExitButton);
#endif // #if defined(hasLCDMenuFunctions)

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//								Deprecated versions for "NXT". Replacement does not have leading 'nxt'
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#if defined(NXT)

intrinsic void deprecated nxtDisplayString(const int nLineNumber, char *pChar, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawStringVarArgs),
                                variable(nLineNumber),
                                variableRefCharPtr(pChar),
                                varArgListOfLongs);

intrinsic void deprecated nxtDisplayString(const int nLineNumber, const string sFormatString, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawTextLineVarArgs),
                                variable(nLineNumber),
                                variableRefString(sFormatString),
                                varArgListOfLongs);

intrinsic void deprecated nxtDisplayTextLine(const int nLineNumber, char *pChar, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawTextLineVarArgs),
                                variable(nLineNumber),
                                variableRefCharPtr(pChar),
                                varArgListOfLongs);

intrinsic void deprecated nxtDisplayTextLine(const int nLineNumber, const string sFormatString, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawTextLineVarArgs),
                                variable(nLineNumber),
                                variableRefString(sFormatString),
                                varArgListOfLongs);

intrinsic void deprecated nxtDisplayBigTextLine(const int nLineNumber, char *pChar, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawBigTextLineVarArgs),
                                variable(nLineNumber),
                                variableRefCharPtr(pChar),
                                varArgListOfLongs);

intrinsic void deprecated nxtDisplayBigTextLine(const int nLineNumber, const string sFormatString, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawBigTextLineVarArgs),
                                variable(nLineNumber),
                                variableRefString(sFormatString),
                                varArgListOfLongs);

#define nxtDisplayClearTextLine(nLineNumber)     nxtDisplayTextLine(nLineNumber, "")

intrinsic void deprecated nxtDisplayCenteredTextLine(const int nLineNumber, char *pChar, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawCenteredTextLineVarArgs),
                                variable(nLineNumber),
                                variableRefCharPtr(pChar),
                                varArgListOfLongs);

intrinsic void deprecated nxtDisplayCenteredTextLine(const int nLineNumber, const string sFormatString, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawCenteredTextLineVarArgs),
                                variable(nLineNumber),
                                variableRefString(sFormatString),
                                varArgListOfLongs);

intrinsic void deprecated nxtDisplayCenteredBigTextLine(const int nLineNumber, const char *pChar, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawCenteredBigTextLineVarArgs),
                                variable(nLineNumber),
                                variableRefCharPtr(pChar),
                                varArgListOfLongs);

intrinsic void deprecated nxtDisplayCenteredBigTextLine(const int nLineNumber, const string sFormatString, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawCenteredBigTextLineVarArgs),
                                variable(nLineNumber),
                                variableRefString(sFormatString),
                                varArgListOfLongs);

intrinsic void deprecated nxtDisplayStringAt(const int xPos, const int yPos, char *pChar, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawStringAtCoordVarArgs),
                                variable(xPos), variable(yPos),
                                variableRefCharPtr(pChar),
                                varArgListOfLongs);

intrinsic void deprecated nxtDisplayStringAt(const int xPos, const int yPos, const string sFormatString, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawStringAtCoordVarArgs),
                                variable(xPos), variable(yPos),
                                variableRefString(sFormatString),
                                varArgListOfLongs);

intrinsic void deprecated nxtDisplayBigStringAt(const int xPos, const int yPos, const char *pChar, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawBigStringAtCoordVarArgs),
                                variable(xPos), variable(yPos),
                                variableRefCharPtr(pChar),
                                varArgListOfLongs);

intrinsic void deprecated nxtDisplayBigStringAt(const int xPos, const int yPos, const string sFormatString, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawBigStringAtCoordVarArgs),
                                variable(xPos), variable(yPos),
                                variableRefString(sFormatString),
                                varArgListOfLongs);

intrinsic void deprecated nxtScrollText(const char *pChar, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawScrollTextVarArgs),
                                variableRefCharPtr(pChar),
                                varArgListOfLongs);

intrinsic void deprecated nxtScrollText(const string sFormatString, ...)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawScrollTextVarArgs),
                                variableRefString(sFormatString),
                                varArgListOfLongs);


intrinsic void deprecated nxtSetPixel(const int xPos, const int yPos)    asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawSetPixel), variable(xPos), variable(yPos));
intrinsic void deprecated nxtClearPixel(const int xPos, const int yPos)  asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawClearPixel), variable(xPos), variable(yPos));

intrinsic void deprecated nxtDrawLine(const int xPos, const int yPos, const int xPosTo, const int yPosTo)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawLine),  variable(xPos), variable(yPos), variable(xPosTo), variable(yPosTo));

intrinsic void deprecated nxtEraseLine(const int xPos, const int yPos, const int xPosTo, const int yPosTo)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawEraseLine),  variable(xPos), variable(yPos), variable(xPosTo), variable(yPosTo));

intrinsic void deprecated nxtInvertLine(const int xPos, const int yPos, const int xPosTo, const int yPosTo)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawInvertLine),  variable(xPos), variable(yPos), variable(xPosTo), variable(yPosTo));

intrinsic void deprecated nxtDisplayRICFile(const int nleft, const int nBottom, const char * pzFileName)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawRICFile),
                                        variable(nleft), variable(nBottom),
                                        variableRefFNExists(pzFileName));

intrinsic void deprecated nxtDrawEllipse(const int Left, const int Top, const int Right, const int Bottom)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawEllipse),  variable(Left), variable(Top), variable(Right), variable(Bottom));

#define nxtDrawCircle(Left, Top, Diameter) nxtDrawEllipse(Left, Top, Left + Diameter - 1, Top - Diameter + 1)

intrinsic void deprecated nxtFillEllipse(const int Left, const int Top, const int Right, const int Bottom)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawFillEllipse),  variable(Left), variable(Top), variable(Right), variable(Bottom));
intrinsic void deprecated nxtEraseEllipse(const int Left, const int Top, const int Right, const int Bottom)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawEraseEllipse),  variable(Left), variable(Top), variable(Right), variable(Bottom));
intrinsic void deprecated nxtDrawInvertEllipse(const int Left, const int Top, const int Right, const int Bottom)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawInvertEllipse),  variable(Left), variable(Top), variable(Right), variable(Bottom));

intrinsic void deprecated nxtDrawRect(const int Left, const int Top, const int Right, const int Bottom)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawRect),  variable(Left), variable(Top), variable(Right), variable(Bottom));
intrinsic void deprecated nxtDrawInvertRect(const int Left, const int Top, const int Right, const int Bottom)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawInvertRect),  variable(Left), variable(Top), variable(Right), variable(Bottom));
intrinsic void deprecated nxtEraseRect(const int Left, const int Top, const int Right, const int Bottom)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawEraseRect), variable(Left), variable(Top), variable(Right), variable(Bottom));
intrinsic void deprecated nxtFillRect(const int Left, const int Top, const int Right, const int Bottom)
                asm(opcdDrawOnGraphicsLCDDisplay, byte(displayDrawFillRect), variable(Left), variable(Top), variable(Right), variable(Bottom));

#endif //#if defined(NXT)
/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//									end of Deprecated versions for "NXT". Replacement does not have leading 'nxt'
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
#endif


#if defined(AVRCPU)
  intrinsic uword propertyIndex(nLCDRefreshCycles, propertySystem,    kSystemLCDRefreshCycles);
#endif

////////////////////////////////////////////////////////////////////////////////////////////////
//
//                         Timer Handling Opcodes and Variables
//
////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(useLongs)
	intrinsic const unsigned long propertyIndex(nSysTime,     propertySystem,        kSystemSysTime);
	intrinsic const unsigned long propertyIndex(nPgmTime,     propertySystem,        kSystemPgmTime);
#else
	intrinsic const unsigned short propertyIndex(nSysTime,    propertySystem,        kSystemSysTime);
	intrinsic const unsigned short propertyIndex(nPgmTime,    propertySystem,        kSystemPgmTime);
#endif

intrinsic short propertyIndex(nClockMinutes,        propertySystem,        kSystemClockMinutes);

enum TTimers
{
  T1          = 0,
  timer1      = 0,
  T2          = 1,
  timer2      = 1,
  T3          = 2,
  timer3      = 2,
  T4          = 3,
  timer4      = 3
};

#define getTimerValue(theTimer) time1[theTimer]

#if defined(useLongs)
	intrinsic long property(time1,                    property1MsecTimer,    kNumbOfTimers, TTimers);  //   1 milli-second tick timer
	intrinsic long deprecated property(time10,        property10MsecTimer,   kNumbOfTimers, TTimers);  //  10 milli-second tick timer
	intrinsic long deprecated property(time100,       property100MsecTimer,  kNumbOfTimers, TTimers);  // 100 milli-second tick timer
#else
	intrinsic short property(time1,                   property1MsecTimer,    kNumbOfTimers, TTimers);  //   1 milli-second tick timer
	intrinsic short deprecated property(time10,       property10MsecTimer,   kNumbOfTimers, TTimers);  //  10 milli-second tick timer
	intrinsic short deprecated property(time100,      property100MsecTimer,  kNumbOfTimers, TTimers);  // 100 milli-second tick timer
#endif

intrinsic void clearTimer(TTimers theTimer)								asm(opcdSystemFunctions, sysFuncClearTimer, variable(theTimer));


#if defined(useLongs)
	intrinsic void wait1Msec(const long nMSec)              	asm(opcdWaitTimer1MSecLong,  variable(nMSec));
	intrinsic void sleep(const long nMSec)                  	asm(opcdWaitTimer1MSecLong,  variable(nMSec));
	intrinsic void delay(const long nMSec)										asm(opcdWaitTimer1MSecLong,  variable(nMSec));
	intrinsic void deprecated wait10Msec(const int nTenMSec)  asm(opcdWaitTimer10MSec, 		 variable(nTenMSec));
#endif

#if (kSizeOfRAMMemoryPtrInHW != 4)
	intrinsic void wait1Msec(const short nMSec)             	asm(opcdWaitTimer1MSecShort, variable(nMSec));
	intrinsic void sleep(const short nMSec)                 	asm(opcdWaitTimer1MSecShort, variable(nMSec));
	intrinsic void wait10Msec(const int nTenMSec)  						asm(opcdWaitTimer10MSec, 		 variable(nTenMSec));
#endif

#define waitUntil(condition) while(!(condition)) sleep(5)

////////////////////////////////////////////////////////////////////////////////////////////////
//
//                         Sound Handling Opcodes
//
////////////////////////////////////////////////////////////////////////////////////////////////

// Sound playback variables

#if (bHasSoundSupport != 0)
	intrinsic void playSound(TSounds sound)                   asm(opcdSoundFunctions, byte(soundSubTypePlaySound), variable(sound));
	intrinsic const bool propertyIndex(bSoundActive,          propertySystem, kSystemBOOLSoundPlaying);
	intrinsic bool propertyIndex(bPlaySounds,                 propertySystem, kSystemPlaySounds);
	intrinsic const bool propertyIndex(bSoundQueueAvailable,  propertySystem, kSystemSoundQueueHasFreeEntries);
	intrinsic void clearSounds()                              asm(opcdSoundFunctions, byte(soundSubTypeClearSound));

  #define muteSound()                  setPlaySounds(false)
  #define unmuteSound()                setPlaySounds(true)

	#if defined(bHasSoundsInRomChip)
	  intrinsic void playRepetitiveSound(TSounds sound,  const int durationIn10MsecTicks)
	                asm(opcdSoundFunctions, byte(soundSubTypePlayRepetitiveSound), variable(sound), variable(durationIn10MsecTicks));
  #endif

	#if defined(bHasPlayMusicalNotes)
		typedef enum TMusicalNotes
		{
			noteC    		=  0,
		  noteCSharp  =  1,
		  noteD    		=  2,
		  noteDSharp  =  3,
		  noteE    		=  4,
		  noteF    		=  5,
		  noteFSharp  =  6,
		  noteG    		=  7,
		  noteGSharp  =  8,
		  noteA    		=  9,
		  noteASharp  =  10,
		  noteB    		=  11
		} TMusicalNotes;

		typedef enum TMusicalOctaves
		{
			octave1			= 0,
			octave2			= 1,
			octave3			= 2,
		} TMusicalOctaves;

		intrinsic void playNote(const TMusicalNotes nNote, const TMusicalOctaves nOctave, const int durationIn10MsecTicks)
										asm(opcdSoundFunctions, byte(soundSubTypePlayMusicalNote), variable(nNote), variable(nOctave), variable(durationIn10MsecTicks));
		intrinsic void playNoteRaw(const int nNote)
										asm(opcdSoundFunctions, byte(soundSubTypePlayToneRaw), variable(nNote));

	#endif

  #if !defined(VexIQ)

		intrinsic void playTone(const int frequency = 440,  const int durationIn10MsecTicks = 100)
	                asm(opcdSoundFunctions, byte(soundSubTypePlayToneVarDurationVar), variable(frequency), variable(durationIn10MsecTicks));
		intrinsic void playImmediateTone(const int frequency, const int durationIn10MsecTicks)
									asm(opcdSoundFunctions,  byte(soundSubTypePlayToneImmediate), variable(frequency), variable(durationIn10MsecTicks));
	#endif


  #if (bHasSoundFiles != 0)
	  //intrinsic void playSoundFile(const string sFileName)     asm(opcdSoundFunctions, soundSubTypePlaySoundFileVariableName, variableRefFNExists(sFileName));
		intrinsic void playSoundFile(const char *pFileName)      asm(opcdSoundFunctions, soundSubTypePlaySoundFileVariableName, variableRefFNExists(pFileName));
	#endif

  #if (bHasSoundVolume != 0)
	  const int kMaxVolumeLevel = 4;
	  intrinsic word propertyIndex(nVolume,                     propertySystem, kSystemVolume);
	#endif

	#if (bHasSoundDriverOnOff != 0)
	  intrinsic bool propertyIndex(bHasSoundDriver,             propertySystem, kSystemSoundDriverOnOff);
	#endif

#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                               Sensor Handling Opcodes and Variables
//
///////////////////////////////////////////////////////////////////////////////////////////////////////

#if (1)

  // the actual 'enum' values for 'tSensors' are automatically defined internally by the ROBOTC compiler. Each platform
  // and circuit board type has its own set of "enum names" and it was becoming too hard to manage them in
  // this file. So they are automatically configured by the compiler which has better utilities for managing
  // differences between platform types.

  typedef enum tSensors;

#else

	#if defined(NXT)

		typedef enum
		{
		  S1    = 0,
		  S2    = 1,
		  S3    = 2,

		  S4    = 3,    // virtual sensors for external MUXes
		  S5,
		  S6
		} tSensors;

	#elif defined(VEX) || defined(STM3210E) || defined(Generic)

		typedef enum
		{
		  in1    =  0,
		  in2    =  1,
		  in3    =  2,
		  in4    =  3,
		  in5    =  4,
		  in6    =  5,
		  in7    =  6,
		  in8    =  7,
		  in9    =  8,
		  in10   =  9,
		  in11   = 10,
		  in12   = 11,
		  in13   = 12,
		  in14   = 13,
		  in15   = 14,
		  in16   = 15
		} tSensors;

	#elif defined(ArduinoSmallMemory)

		typedef enum
		{
		  //
		  // Analog I/O Ports: light, line follower, gyro, accelerometer,
		  //
		  anlg0     =  0,
		  anlg1     =  1,
		  anlg2     =  2,
		  anlg3     =  3,
		  anlg4     =  4,
		  anlg5     =  5,

		  dgtl0,
		  dgtl1,
		  dgtl2,
		  dgtl3,
		  dgtl4,
		  dgtl5,
		  dgtl6,
		  dgtl7,
		  dgtl8,
		  dgtl9,
		  dgtl10,
		  dgtl11,
		  dgtl12,
		  dgtl13,
		} tSensors;

	#elif defined(ArduinoMega)

		typedef enum
		{
		  //
		  // Analog I/O Ports, light, line follower, gyro, accelerometer,
		  //
		  anlg0     =  0,
		  anlg1     =  1,
		  anlg2     =  2,
		  anlg3     =  3,
		  anlg4     =  4,
		  anlg5     =  5,
		  anlg6     =  6,
		  anlg7     =  7,
		  anlg8     =  8,
		  anlg9     =  9,
		  anlg10    =  10,
		  anlg11    =  11,
		  anlg12    =  12,
		  anlg13    =  13,
		  anlg14    =  14,
		  anlg15    =  15,

		  dgtl0     =  16,
		  dgtl1     =  17,
		  dgtl2     =  18,
		  dgtl3     =  19,
		  dgtl4     =  20,
		  dgtl5     =  21,
		  dgtl6     =  22,
		  dgtl7     =  23,
		  dgtl8     =  24,
		  dgtl9     =  25,
		  dgtl10    =  26,
		  dgtl11    =  27,
		  dgtl12    =  28,
		  dgtl13    =  29,
		  dgtl14    =  30,
		  dgtl15    =  31,
		  dgtl16    =  32,
		  dgtl17    =  33,
		  dgtl18    =  34,
		  dgtl19    =  35,
		  dgtl20    =  36,
		  dgtl21    =  37,
		  dgtl22    =  38,
		  dgtl23    =  39,
		  dgtl24    =  40,
		  dgtl25    =  41,
		  dgtl26    =  42,
		  dgtl27    =  43,
		  dgtl28    =  44,
		  dgtl29    =  45,
		  dgtl30    =  46,
		  dgtl31    =  47,
		  dgtl32    =  48,
		  dgtl33    =  49,
		  dgtl34    =  50,
		  dgtl35    =  51,
		  dgtl36    =  52,
		  dgtl37    =  53,
		  dgtl38    =  54,
		  dgtl39    =  55,
		  dgtl40    =  56,
		  dgtl41    =  57,
		  dgtl42    =  58,
		  dgtl43    =  59,
		  dgtl44    =  60,
		  dgtl45    =  61,
		  dgtl46    =  62,
		  dgtl47    =  63,
		  dgtl48    =  64,
		  dgtl49    =  65,
		  dgtl50    =  66,
		  dgtl51    =  67,
		  dgtl52    =  68,
		  dgtl53    =  69,
		} tSensors;

	#elif defined(VEX2)

		typedef enum
		{
		  //
		  // Analog I/O Ports: light, line follower, gyro, accelerometer,
		  //
		  in1    =  0,
		  in2    =  1,
		  in3    =  2,
		  in4    =  3,
		  in5    =  4,
		  in6    =  5,
		  in7    =  6,
		  in8    =  7,

		  //
		  // Digital I/O Ports: Touch sensors, sonar, etc
		  //
		  dgtl1  = 8,
		  dgtl2  = 9,
		  dgtl3  = 10,
		  dgtl4  = 11,
		  dgtl5  = 12,
		  dgtl6  = 13,
		  dgtl7  = 14,
		  dgtl8  = 15,
		  dgtl9  = 16,
		  dgtl10 = 17,
		  dgtl11 = 18,
		  dgtl12 = 19,

		} tSensors;

	#else

	  typedef short tSensors;

	#endif

#endif

#if defined(NXT)
  //
	// LEGO RGB Color Sensor support
	//
  intrinsic short validColorSensor(const tSensors nPort)     asm(opcdSystemFunctions, sysFuncGetValidColorSensor, variable(nPort), functionReturn);

  //
	// Use for "sensorColorFull" sensors to copy the values for each of the four colors (none, R, G, B)
	// Input parameter is used to select "ADRaw", "Raw" (calibrated by sensor), and final per cent value
	//

	typedef enum
	{
	  BLACKCOLOR   = 1,
	  BLUECOLOR    = 2,
	  GREENCOLOR   = 3,
	  YELLOWCOLOR  = 4,
	  REDCOLOR     = 5,
	  WHITECOLOR   = 6
	} TColors;

  typedef enum
	{
		colorAtoD				= 0,
		colorRaw        = 1,
		colorValue      = 2
	} TColorDataType;

  intrinsic void getColorSensorData(const tSensors nPort, TColorDataType nDataType, short *pData)
                      asm(opcdSystemFunctions, sysFuncGetColorSensorValues, variable(nPort), variable(nDataType), variableRefRAM(pData));

	typedef struct
	{
		long nDisconnects;						// Incremented once when sensor detection (see nWaitForConnect) fails after sensor has
																	// been connected.
		long nSoftwareResets;         // Another pin on sensor indicates internal error condition. Incremented once per erro.
		long nCRCMismatch;            // 50 bytes are uploaded from sensor on startup. A CRC-code is used to detect errors.
																	// Peg count incremented once for each time failure is detected.
		long nWaitForConnect;					// Counts the number of times the sensor is not detected. There's an I/O pin on the
																	// sensor cable indicating whether sensor is working OK. This peg count is incremented
																	// every millisecond when not connected when it is supposed to be connected. Usually
																	// triggered by removing the sensor in mid-operation.
		long nReadCount;              // Not an error! A count of the number of times data is read from the sensor.
	} TColorPegCounts;

  intrinsic void getColorSensorPegCounts(const tSensors nPort, TColorPegCounts *pData)
                      asm(opcdSystemFunctions, sysFuncGetColorSensorCounts, variable(nPort), variableRefRAM(pData));


#endif

#if defined(bHasFastSonarSensor)
  intrinsic bool propertyIndex(bFastSonarSensor,   propertySystem, kSystemFastSonarSensor);
#endif

intrinsic int property(SensorValue,              propertySensor,         kNumbOfTotalSensors, tSensors);

#if defined(EV3)
	intrinsic float property(SensorValueFloat,      propertySensorFloat,    kNumbOfTotalSensors, tSensors);
#endif

intrinsic const bool property(SensorBoolean,      propertySensorBoolean,  kNumbOfTotalSensors, tSensors);
intrinsic TSensorTypes property(SensorType,       propertySensorType,     kNumbOfTotalSensors, tSensors);

#if defined(bHasGetSensorRawADValue)
  intrinsic short getSensorRawADValue(const tSensors nPort)     asm(opcdSystemFunctions, sysFuncGetRawADValue, variable(nPort), functionReturn);
#endif

#if defined(bHasSourceTypeSensorMode)
	intrinsic TSensorModes property(SensorMode,       propertySensorMode,     kNumbOfTotalSensors, tSensors);
#endif

#if defined(NXT)
	intrinsic TSensorModes property(DigitalPinDirection,	propertyDigitalPinDirection,	kNumbOfTotalSensors, tSensors);
	intrinsic TSensorModes property(DigitalPinValue,			propertyDigitalPinValue,			kNumbOfTotalSensors, tSensors);
#endif

#if defined(bHasSourceTypeSensorRawValue)
	intrinsic word property(SensorRaw,                propertySensorRaw,      kNumbOfTotalSensors, tSensors);
#endif

#if defined(bHasSensorBiasSettings)
	intrinsic word property(SensorBias,               propertySensorBias,     kNumbOfTotalSensors, tSensors);
	intrinsic word property(SensorSmallBias,          propertySensorSmallBias,kNumbOfTotalSensors, tSensors);
	intrinsic word property(SensorScale,              propertySensorScale,    kNumbOfTotalSensors, tSensors);
	intrinsic word property(SensorFullCount,          propertySensorFullCount,kNumbOfTotalSensors, tSensors);
#endif

#if defined(VEX)
intrinsic word property(ifiInterruptPorts,          propertyIfiInterruptPort, kNumbOfTotalSensors, tSensors);
intrinsic int propertyIndex(nAnalogChannelCount,    propertySystem,           kSystemVEXAnalogChannelCount);
#endif


#if defined(NXT)
intrinsic word propertyIndex(nVirtualSensorTypeChanges,   propertySystem, kSystemVirtualSensorTypeChanges);
intrinsic word propertyIndex(nVirtualSensorModeChanges,   propertySystem, kSystemVirtualSensorModeChanges);
#endif


#define setSensorType(nSensorX, nType) SensorType[nSensorX] = nType
#define setSensorMode(nSensorX, nMode) SensorMode[nSensorX] = nMode

#if defined(bHasShiftOutOpcode) || defined(_WINDOWS)
	intrinsic void shiftOut(compileConst tSensors dataPin, compileConst tSensors clockPin, compileConst bool bBitOrderLSBFirst, const ubyte nValue, const int nDelay)
	  	asm(opcdSystemFunctions, sysFuncShiftOutConstParms, variableCompileConstByte(dataPin), variableCompileConstByte(clockPin), variableCompileConstByte(bBitOrderLSBFirst), variable(nValue), variable(nDelay));
	intrinsic void shiftOut(const tSensors dataPin, const tSensors clockPin, const bool bBitOrderLSBFirst, const ubyte nValue, const int nDelay)
	  	asm(opcdSystemFunctions, sysFuncShiftOut, variable(dataPin), variable(clockPin), variable(bBitOrderLSBFirst), variable(nValue), variable(nDelay));
	intrinsic void shiftIn(compileConst tSensors dataPin, compileConst tSensors clockPin, compileConst bool bBitOrderLSBFirst, ubyte &nValue, const int nDelay)
	  	asm(opcdSystemFunctions, sysFuncShiftInConstParms, variableCompileConstByte(dataPin), variableCompileConstByte(clockPin), variableCompileConstByte(bBitOrderLSBFirst), variableRefByte(nValue), variable(nDelay));
	intrinsic void shiftIn(const tSensors dataPin, const tSensors clockPin, const bool bBitOrderLSBFirst, const ubyte &nValue, int nDelay)
	  	asm(opcdSystemFunctions, sysFuncShiftIn, variable(dataPin), variable(clockPin), variable(bBitOrderLSBFirst), variableRefByte(nValue), variable(nDelay));
#endif

////////////////////////////////////////////////////////////////////////////////////////////////
//
//                         Motor Handling Opcodes
//
////////////////////////////////////////////////////////////////////////////////////////////////

#if (1)

  // the actual 'enum' values for 'tMotor' are automatically defined by the ROBOTC compiler. Each platform
  // and circuit board type has its own set of "enum names" and it was becoming too hard to manage them in
  // this file. So they are automatically configured by the compiler which has better utilities for managing
  // differences between platform types.

  typedef enum tMotor;

#else

	// The old definitions are temporarily maintained here as well until the new implementation is confirmed
  // to be working well!

  #if defined(NXT) || defined(EV3)

		typedef enum
		{
		  motorA					= 0,
		  motorB					= 1,
		  motorC					= 2,
		  motorD					= 3,
			//
			// HiTechnic Motor Controllers.
			//    - Each controller supports two motors
			//    = Up to four controllers per sensor port.
			//
			//motorD					=  3,
		  //motorE,

#if defined(NXT)
			mtr_S1_C1_1     =  3,
#elif defined(EV3)
			mtr_S1_C1_1     =  4,
#endif
			mtr_S1_C1_2,
		 	mtr_S1_C2_1,
		  mtr_S1_C2_2,
		  mtr_S1_C3_1,
		  mtr_S1_C3_2,
		  mtr_S1_C4_1,
		  mtr_S1_C4_2,

		  mtr_S2_C1_1,
		  mtr_S2_C1_2,
		 	mtr_S2_C2_1,
		  mtr_S2_C2_2,
		  mtr_S2_C3_1,
		  mtr_S2_C3_2,
		  mtr_S2_C4_1,
		  mtr_S2_C4_2,

		  mtr_S3_C1_1,
		  mtr_S3_C1_2,
		 	mtr_S3_C2_1,
		  mtr_S3_C2_2,
		  mtr_S3_C3_1,
		  mtr_S3_C3_2,
		  mtr_S3_C4_1,
		  mtr_S3_C4_2,

		  mtr_S4_C1_1,
		  mtr_S4_C1_2,
		 	mtr_S4_C2_1,
		  mtr_S4_C2_2,
		  mtr_S4_C3_1,
		  mtr_S4_C3_2,
		  mtr_S4_C4_1,
		  mtr_S4_C4_2,
		} tMotor;

	#elif defined(VEX) || defined(VEX2) || defined(VexIQ)

		typedef enum// Specifies a single motor
		{
		  port1         = 0,
		  port2         = 1,
		  port3         = 2,
		  port4         = 3,
		  port5         = 4,
		  port6         = 5,
		  port7         = 6,
		  port8         = 7,
		#if defined(VEX2)
		  port9         = 8,
		  port10        = 9,
		#endif
		#if defined(VexIQ)
		  port11        = 10,
		  port12        = 11,
		#endif
		} tMotor;

	#elif defined(AVRCPU) || defined(Generic)

		typedef enum// Specifies a single motor
		{
		  motor0        = 0,
		  motor1        = 1,
		  motor2        = 2,
		  motor3        = 3,
		  motor4        = 4,
		  motor5        = 5,
		} tMotor;

	#endif

#endif

intrinsic word property(motor,												propertyMotorPower, kNumbOfTotalMotors, tMotor);
#define motorPowerSigned motor

#if defined(bFirmwareStoresMotorType)
	intrinsic word property(motorType,									propertyMotorType,   kNumbOfTotalMotors, tMotor);
#endif

#if defined(bMotorMirroring)
	intrinsic bool property(bMotorReflected,            propertyMotorReflectDirection, kNumbOfTotalMotors, tMotor);
	#define bMotorFlippedMode bMotorReflected
#endif

#if defined(bFirmwareStoresMotorDriveSide)
	intrinsic TMotorDriveSide property(nMotorDriveSide, propertyMotorDriveSide, kNumbOfTotalMotors, tMotor);
#endif

#if defined(bFirmwareStoresMotorDriveSide) && defined(hasMotorOpcodes)

#if defined(NXT)
	typedef tMotor TMotorList[3];
#else
	typedef tMotor TMotorList[4];
	intrinsic void getMotorsWithDriveSideType(TMotorDriveSide nType, TMotorList &nMotorList)
																	asm(opcdMotorFunctions, byte(mtrOpDriveSideList), variable(nType), variableRefRAM(nMotorList));
#endif


#endif

#if defined(VexIQ)
	//
	// Unique VEX IQ intrinsic functions
	//
	//Legacy
	#define setMotorStrength setMotorCurrentLimit
	#define getMotorStrength getMotorCurrentLimit
	#define getMotorRunning getMotorZeroVelocity

	intrinsic void setMotorCurrentLimit(tMotor nMotorIndex, unsigned short nCurrent)						asm(opcdMotorFunctions, byte(mtrOpSetCurrentLimit), 						variable(nMotorIndex), variable(nCurrent));
	intrinsic unsigned long getMotorCurrentLimit(tMotor nMotorIndex)														asm(opcdMotorFunctions, byte(mtrOpGetCurrentLimit), 						variable(nMotorIndex), functionReturn);
	intrinsic unsigned long	getMotorCurrent(tMotor nMotorIndex)																	asm(opcdMotorFunctions, byte(mtrOpGetImmediateCurrent),			 		variable(nMotorIndex), functionReturn);

	intrinsic void 	updateMotorPWMScale(tMotor nMotorIndex)															asm(opcdMotorFunctions, byte(mtrOpRequestPWMScaleUpdate), variable(nMotorIndex));
	intrinsic bool 	getMotorOverTemp(tMotor nMotorIndex)																asm(opcdMotorFunctions, byte(mtrOpGetOverTemp), 								variable(nMotorIndex), functionReturn);
	intrinsic bool 	getMotorCurrentLimitFlag(tMotor nMotorIndex)												asm(opcdMotorFunctions, byte(mtrOpGetCurrentLimitFlag), 				variable(nMotorIndex), functionReturn);
	intrinsic bool 	getMotorZeroVelocity(tMotor nMotorIndex)														asm(opcdMotorFunctions, byte(mtrOpGetZeroVelocity), 						variable(nMotorIndex), functionReturn);
	intrinsic bool 	getMotorZeroPosition(tMotor nMotorIndex)														asm(opcdMotorFunctions, byte(mtrOpGetZeroPosition), 						variable(nMotorIndex), functionReturn);
#endif

#if defined(VexIQ) || defined(EV3)
	//
	// Motor Control intrinsic functions that are likely to apply to other new platforms with smart motors
	//
	typedef enum TMotorBrakeModes
	{
		motorCoast = 0,
		motorBrake = 1,
		#if defined(VexIQ)
		  motorHold = 2,
		#endif
	} TMotorBrakeModes;

	#if defined(VexIQ)
		typedef enum
		{
			encoderDegrees   = 0,
			encoderRotations = 1,
			encoderCounts    = 2,   //Raw Encoder Count Mode
		} TMotorEncoderUnitModes;

		intrinsic float getServoEncoder(tMotor nMotorIndex)																	asm(opcdMotorFunctions, byte(mtrOpGetEncoder), 									variable(nMotorIndex), functionReturn);
		intrinsic void 	setServoTarget(tMotor nMotorIndex, float nPosition) 								asm(opcdMotorFunctions, byte(mtrOpSetServoPosition), 						variable(nMotorIndex), variable(nPosition));
		intrinsic void setMotorEncoderUnits(TMotorEncoderUnitModes nEncoderUnitMode)				asm(opcdMotorFunctions, byte(mtrOpSetMotorEncoderUnits),				variable(nEncoderUnitMode));
		intrinsic TMotorEncoderUnitModes 	getMotorEncoderUnits()														asm(opcdMotorFunctions, byte(mtrOpGetMotorEncoderUnits),				functionReturn);
	#endif

	//EV3 Specific Motor Opcodes Here
	#if defined(EV3)
		intrinsic bool 	getMotorRunning(tMotor nMotorIndex)														asm(opcdMotorFunctions, byte(mtrOpGetMotorStatus), 									variable(nMotorIndex), functionReturn);
	#endif

	//Helper functions/commands
	#if defined (EV3)
		#define getMotorMoving getMotorRunning
		#define waitUntilMotorStop(nMotorIndex) do {sleep(100); while(getMotorRunning(nMotorIndex)) sleep(1);} while(false)
	#elif defined(VexIQ)
		//Motor Speed Delays
		#define getMotorMoving(nMotorIndex) !getMotorZeroVelocity(nMotorIndex)
		#define waitUntilMotorStop(nMotorIndex) do {sleep(100); while(!getMotorZeroVelocity(nMotorIndex)) sleep(1);} while(false)
		//Position Based Delays
		#define getMotorMoveComplete(nMotorIndex) !getMotorZeroPosition(nMotorIndex)
		#define waitUntilMotorMoveComplete(nMotorIndex) do {sleep(100); while(!getMotorZeroPosition(nMotorIndex)) sleep(1);} while(false)
	#endif


	intrinsic void 	setMotorReversed(tMotor nMotorIndex, bool bReversed)								asm(opcdMotorFunctions, byte(mtrOpSetMirroredFlag), 								variable(nMotorIndex), variable(bReversed));
	intrinsic bool 	getMotorReversed(tMotor nMotorIndex)																asm(opcdMotorFunctions, byte(mtrOpGetMirroredFlag), 								variable(nMotorIndex), functionReturn);

	intrinsic void 	moveMotorTarget(tMotor nMotorIndex, float nPosition, short nSpeed) 	asm(opcdMotorFunctions, byte(mtrOpSetPositionAndVelocityRelative),	variable(nMotorIndex), variable(nPosition), variable(nSpeed));
	intrinsic void 	setMotorTarget(tMotor nMotorIndex, float nPosition, short nSpeed) 	asm(opcdMotorFunctions, byte(mtrOpSetPositionAndVelocityAbsolute),	variable(nMotorIndex), variable(nPosition), variable(nSpeed));
	intrinsic float	getMotorTarget(tMotor nMotorIndex) 																	asm(opcdMotorFunctions, byte(mtrOpGetEncoderTargetAbsolute), 				variable(nMotorIndex), functionReturn);
	intrinsic bool	getMotorDriveTrainMoving() 																					asm(opcdMotorFunctions, byte(mtrOpGetDriveTrainMoving), functionReturn);

	intrinsic void setMotorBrakeMode(tMotor nMotorIndex, TMotorBrakeModes nBrakeMode)		asm(opcdMotorFunctions, byte(mtrOpSetBrakeMode), 										variable(nMotorIndex), variable(nBrakeMode));
	intrinsic TMotorBrakeModes 	getMotorBrakeMode(tMotor nMotorIndex)										asm(opcdMotorFunctions, byte(mtrOpGetBrakeMode), 										variable(nMotorIndex), functionReturn);

	intrinsic void 	setMotorSpeed(tMotor nMotorIndex, int nSpeed)												asm(opcdMotorFunctions, byte(mtrOpSetVelocity), 										variable(nMotorIndex), variable(nSpeed));
	intrinsic short	getMotorSpeed(tMotor nMotorIndex)																		asm(opcdMotorFunctions, byte(mtrOpGetVelocity), 										variable(nMotorIndex), functionReturn);

	intrinsic float getMotorEncoder(tMotor nMotorIndex)																	asm(opcdMotorFunctions, byte(mtrOpGetEncoder), 											variable(nMotorIndex), functionReturn);
	intrinsic void 	resetMotorEncoder(tMotor nMotorIndex)																asm(opcdMotorFunctions, byte(mtrOpResetEncoder), 										variable(nMotorIndex));

#endif

#if defined(bAssociateMotorWithEncoderSensor)
	intrinsic void mapEncoderToMotor(compileConst tMotor nMotorIndex, compileConst tSensors nSensorIndex)
								 asm(opcdSystemFunctions, byte(sysFuncMapEncoderToMotor), variableCompileConstByte(nMotorIndex), variableCompileConstByte(nSensorIndex));
	intrinsic tSensors getEncoderForMotor(tMotor nMotorIndex)
								 asm(opcdSystemFunctions, byte(sysFuncGetEncoderForMotor), variable(nMotorIndex), functionReturn);
#endif

#if defined(bAssociateMotorWithEncoderSensor) || defined(bMotorsWithEncoders)
  intrinsic long property(nMotorEncoder,                  propertyMotorEncoder, kNumbOfTotalMotors, tMotor);

	#if !defined(EV3)
  intrinsic const long property(nMotorEncoderRaw,         propertyMotorEncoderRaw, kNumbOfTotalMotors, tMotor);
	#endif

	#if defined(bHasEncoderTimeStamp)
  	intrinsic void getEncoderAndTimeStamp(tMotor nMotor, long &nEncoder, unsigned long &nTimeStamp)
  							asm(opcdSystemFunctions, byte(sysFuncGetEncoderAndTimeStampForMotor), variable(nMotor), variableRefLong(nEncoder), variableRefLong(nTimeStamp));
	#endif
#endif

#if defined(NXT)
  intrinsic const short property(motorPWMLevel,               propertyMotorPWM,                   kNumbOfTotalMotors, tMotor);
#endif

#if (bSmartMotorsWithEncoders() != 0) && !defined(NXT) && !defined(EV3)
	intrinsic const short property(motorPWMLevel,								propertyMotorPWM,										kNumbOfTotalMotors, tMotor);

  intrinsic const short property(mtrPid_ErrorP,								propertyPid_ErrorP,									kNumbOfTotalMotors, tMotor);
  intrinsic const long  property(mtrPid_ErrorI,								propertyPid_ErrorI,									kNumbOfTotalMotors, tMotor);
  intrinsic const short property(mtrPid_ErrorD,								propertyPid_ErrorD,									kNumbOfTotalMotors, tMotor);
  //intrinsic const long  property(mtrPid_SyncAdjust,					propertyPid_SyncAdjusts,						kNumbOfTotalMotors, tMotor);
  //intrinsic const short property(mtrPid_SyncError,					propertyPid_SyncError,							kNumbOfTotalMotors, tMotor);
  //intrinsic const short property(mtrPid_SyncDerivative,			propertyPid_SyncDerivative,					kNumbOfTotalMotors, tMotor);
  //intrinsic const long  property(mtrPid_SyncIntegral,				propertyPid_SyncIntegral,						kNumbOfTotalMotors, tMotor);
  intrinsic const long  property(mtrPid_StartEncoder,					propertyPid_StartEncoderCount,			kNumbOfTotalMotors, tMotor);

  intrinsic short property(mtrPid_Period,											propertyPid_Period,									kNumbOfTotalMotors, tMotor);
  intrinsic short property(mtrPid_kP,													propertyPid_kP,											kNumbOfTotalMotors, tMotor);
  intrinsic short property(mtrPid_kI,													propertyPid_kI,											kNumbOfTotalMotors, tMotor);
  intrinsic short property(mtrPid_kD,													propertyPid_kD,											kNumbOfTotalMotors, tMotor);
  intrinsic short property(mtrPid_Deadband,										propertyPid_Deadband,								kNumbOfTotalMotors, tMotor);
  //intrinsic short property(mtrPid_Slop,											propertyPid_Slop,										kNumbOfTotalMotors, tMotor);

  //intrinsic void driveStraight(tMotor nLeftMotor, tMotor nRightMotor, int nSpeed)
  //							asm(opcdSystemFunctions, byte(sysFuncMotorDriveStraight), variable(nLeftMotor), variable(nRightMotor), variable(nSpeed));
  //intrinsic void driveSynchronized(tMotor nLeftMotor, tMotor nRightMotor, int nLeftSpeed, int nRightSpeed)
  //							asm(opcdSystemFunctions, byte(sysFuncMotorDriveSynchronized), variable(nLeftMotor), variable(nRightMotor), variable(nLeftSpeed), variable(nRightSpeed));
  //intrinsic void driveSynchronizedCancel(tMotor nMotor)
  //							asm(opcdSystemFunctions, byte(sysFuncMotorDriveSynchronizedCancel), variable(nMotor));

  intrinsic void setMotorTarget(tMotor nMotor, long nEncoderCountTarget = 360, int nMaxSpeedToUse = 63, bool bHoldAtEnd = true) // Move to absolute encoder position
  							asm(opcdSystemFunctions, byte(sysFuncMotorMoveToPositionAbsolute), variable(nMotor), variable(nEncoderCountTarget), variable(nMaxSpeedToUse), variable(bHoldAtEnd));

  intrinsic void moveMotorTarget(tMotor nMotor, long nEncoderCountTarget = 360, int nMaxSpeedToUse = 63, bool bHoldAtEnd = true) // Move to a relative encoder position
  							asm(opcdSystemFunctions, byte(sysFuncMotorMoveToPositionRelative), variable(nMotor), variable(nEncoderCountTarget), variable(nMaxSpeedToUse), variable(bHoldAtEnd));

  intrinsic short getMotorSlewSpeed(tMotor nMotor)
  							asm(opcdSystemFunctions, byte(sysFuncMotorGetSlewSpeed), variable(nMotor), functionReturn);

  intrinsic void resetMotorEncoder(tMotor nMotor)  				asm(opcdSystemFunctions, byte(sysFuncMotorResetEncoder), variable(nMotor));
  intrinsic long getMotorEncoder(tMotor nMotor)  					asm(opcdSystemFunctions, byte(sysFuncMotorGetEncoder), variable(nMotor), functionReturn);
  intrinsic short getMotorVelocity(tMotor nMotor)  				asm(opcdSystemFunctions, byte(sysFuncGetEncoderVelocity), variable(nMotor), functionReturn);
  intrinsic bool getMotorTargetCompleted(tMotor nMotor)  	asm(opcdSystemFunctions, byte(sysFuncMotorPositionComplete), variable(nMotor), functionReturn);

  #define waitUntilMotorStop(nMotorIndex) do{sleep(100); while(!getMotorTargetCompleted(nMotorIndex)) sleep(5);} while(false)


	intrinsic void slaveMotor(tMotor nSlaveMotor, tMotor nMasterMotor)
  							asm(opcdSystemFunctions, byte(sysFuncMotorSlave), variable(nSlaveMotor), variable(nMasterMotor));
#endif

#if  defined(bHasFloatDuringInactivePWM)
	intrinsic bool propertyIndex(bFloatDuringInactiveMotorPWM,propertySystem, kSystemFloatDuringInactiveMotorPWM);
#endif

#if defined(NXT)
	intrinsic word propertyIndex(nVirtualMotorChanges,        propertySystem, kSystemVirtualMotorChanges);
#endif

#if defined(bHasServoSpecificSourceTypes)

  typedef enum TServoIndex;

	const int kNumbOfServos = 4 * 6 * 6;

	intrinsic const short property(ServoType,								  				propertyServoType,				    kNumbOfServos, TServoIndex);
	intrinsic const short property(ServoValue,												propertyServoPosition,				kNumbOfServos, TServoIndex);
	intrinsic short property(servoChangeRate,							    				propertyServoRateOfChange,		kNumbOfServos, TServoIndex);
	intrinsic short property(servoMinPos,															propertyServoMinPos,					kNumbOfServos, TServoIndex);
	intrinsic short property(servoMaxPos,															propertyServoMaxPos,					kNumbOfServos, TServoIndex);
	intrinsic short property(servo,																		propertyServoTarget,					kNumbOfRealServos, TServoIndex);
  #define servoTarget  servo  // Retain for legacy purposes.

#endif

#if defined(Tetrix) && !defined(EV3)

	intrinsic word propertyIndex(nPidUpdateInterval12V,										propertySystem, kSystemPidUpdateIntervalInMsecs12V);
  intrinsic word propertyIndex(muxUpdateInterval,					  						propertySystem, kSystemFtcHubUpdateInterval);

	intrinsic word propertyIndex(nMaxRegulatedSpeed12V,										propertySystem, kSystemMaxRegulatedSpeedCountsPerSecond12V);
  intrinsic word propertyIndex(muxLowBatteryShutdown,										propertySystem, kSystemBatteryShutdownExternal);
  intrinsic const short propertyIndex(externalBatteryAvg,								propertySystem, kSystemAvgBatteryLevelExternal);
	intrinsic const short propertyIndex(externalBattery,									propertySystem, kSystemBatteryLevelExternal);
	intrinsic bool propertyIndex(bSystemLeaveServosEnabledOnProgramStop,	propertySystem, kSystemLeaveServosEnabledOnProgramStop);

#endif

#if defined(bMotorsWithCurrentLimiting)
	intrinsic short property(nMotorCurrentLimit, propertyMotorCurrentLimit, kNumbOfTotalMotors, tMotor);
#endif

#if (bSmartMotorsWithEncoders() != 0))

	intrinsic TMotorRegulation property(nMotorPIDSpeedCtrl, propertyMotorRegulation, kNumbOfTotalMotors, tMotor);

#endif

#if defined(NXT)

	intrinsic long property(nMotorEncoderTarget, propertyMotorEncoderTargetRelative, kNumbOfTotalMotors, tMotor);

	typedef enum
	{
		runStateIdle         = 0x00,  // published IOMAP value. do not change
		runStateRampUp       = 0x10,  // published IOMAP value. do not change
		runStateRunning      = 0x20,  // published IOMAP value. do not change
		runStateRampDown     = 0x40,  // published IOMAP value. do not change
		runStateHoldPosition = 0x01,   // Dick's add. Can be any value.
	} TNxtRunState;

	intrinsic TNxtRunState property(nMotorRunState,         propertyMotorRunState, kNumbOfTotalMotors, tMotor);


	intrinsic void changeSpeedRampingParms(short nMotorIndex,
																					short nMinimumRampStartUpSpeed,
																					short nRampStartupChangeRate,
																					short nMinimumRampDownSpeed,
																					short nRampDownChangeRate)
											asm(opcdSystemFunctions, sysFuncMotorRampingParms,
																					variable(nMotorIndex),
																					variable(nMinimumRampStartUpSpeed),
																					variable(nRampStartupChangeRate),
																					variable(nMinimumRampDownSpeed),
																					variable(nRampDownChangeRate));

	intrinsic word propertyIndex(nMaxRegulatedSpeedNxt,     propertySystem, kSystemMaxRegulatedSpeedCountsPerSecondNxt);
	intrinsic word propertyIndex(nPidUpdateInterval,				propertySystem, kSystemPidUpdateIntervalInMsecsNxt);
	#define nPidUpdateIntervalNxt nPidUpdateInterval
	intrinsic TSynchedMotors propertyIndex(nSyncedMotors,   propertySystem, kSystemNxtSynchMotors);
	intrinsic word propertyIndex(nSyncedTurnRatio,          propertySystem, kSystemNxtSynchSlaveSpeedRatio);

	intrinsic tMotor propertyIndex(nMotorBeingTrace,   			propertySystem, kSystemMotorBeingTraced);

#endif // #if defined(NXT)


#if defined(hasWatchDogTimerFunctions)
	intrinsic int propertyIndex(nWatchdogTimeoutInterval,		propertySystem, kSystemWatchdogTimeoutInterval);
	intrinsic const bool propertyIndex(bResetFromWatchdogTimeout,	propertySystem, kSystemResetFromWatchdogTimeout);
#endif
///////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                            Memory Management and Procedure / Task Management Opcodes
//
///////////////////////////////////////////////////////////////////////////////////////////////////////

#if !defined(NXT)
	intrinsic word propertyIndex(programNumber,                   propertySystem, kSystemProgramNumb);
	#define setProgramNumber(nNumber) programNumber = nNumber
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                            Data Logging
//
///////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(bHasDatalog)
	intrinsic void UploadDatalog(compileConst int nSize)
                                      asm(opcdMsgUploadCommands, uploadDatalog, variableCompileConstWord(nSize), 0, 0, 0, 0);

  #define nNumbOfDatalogSeries (nDatalogStatus & 0x0FF)
	#define bDatalogPolling        (nDatalogStatus & 0x100 ? true : false)
	intrinsic const word propertyIndex(nDatalogBufferSize,    					propertySystem, kSystemDatalogBufferSize);
	intrinsic const word propertyIndex(nDatalogStatus,     				      propertySystem, kSystemDatalogStatus);
	intrinsic const word propertyIndex(nNumbOfDatalogBytesAvailToRead,  propertySystem, kSystemNumbOfDatalogBytesAvailToRead);
	intrinsic const word propertyIndex(nNumbOfDatalogResets,     				propertySystem, kSystemNumbOfDatalogResets);
	intrinsic const word propertyIndex(nNumbOfDatalogOverruns,     			propertySystem, kSystemNumbOfDatalogOverruns);

	intrinsic void datalogAddValue(const int nDataSeries, const int nDataValue)  asm(opcdDatalogFunctions, byte(funcDatalogAddPoint), variable(nDataSeries), variable(nDataValue));
	intrinsic void datalogAddValueWithTimeStamp(const int nDataSeries, const int nDataValue)  asm(opcdDatalogFunctions, byte(funcDatalogAddPointWithTimeStamp), variable(nDataSeries), variable(nDataValue));
	//intrinsic void datalogDefineDataPointName(const int nDataSeries, const string &sDataName)
	//																													asm(opcdDatalogFunctions, byte(funcDatalogDefineDataPointName), variable(nDataSeries), variableRefString(sDataName));
	intrinsic void datalogClear()															asm(opcdDatalogFunctions, byte(funcDatalogClear));

	intrinsic void datalogDataGroupStart()										asm(opcdDatalogFunctions, byte(funcDatalogGroupStart));
	intrinsic void datalogDataGroupEnd()											asm(opcdDatalogFunctions, byte(funcDatalogGroupEnd));

	intrinsic void datalogBackgroundPollingPause()						asm(opcdDatalogFunctions, byte(funcDatalogPollingPause));
	intrinsic void datalogBackgroundPollingResume()						asm(opcdDatalogFunctions, byte(funcDatalogPollingResume));

	#if defined(bHasDatalogSave)
	// No longer supported.
		intrinsic deprecated void SaveNxtDatalog()							asm(opcdDatalogFunctions, byte(funcDatalogSaveNext));
	#endif

#endif //#if defined(bHasDatalog)

///////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Procedure / Task Flow Control
//
///////////////////////////////////////////////////////////////////////////////////////////////////////

intrinsic void callSub(compileConst int number)       asm(opcdCallSub, variableCompileConstByte(number));

intrinsic void startTask(void TaskID)									asm(opcdStartTask, taskNameConstant(TaskID));
#if !defined(VEX)
  intrinsic void startTask(void TaskID, const short nTaskPriority) asm(opcdSystemFunctions, byte(sysFuncStartTaskWithPriority), taskNameVariable(TaskID), variable(nTaskPriority));
#endif
intrinsic void stopTask(void TaskID)									asm(opcdStopTask,  taskNameConstant(TaskID));
intrinsic void stopAllTasks()                         asm(opcdStopAllTasks);

#if !defined(VEX)
	intrinsic void suspendTask(void TaskID)							asm(opcdSystemFunctions, byte(sysFuncSetSuspendResumeTask), taskNameVariable(TaskID), byte(parmSourceConstant), word(1));
	intrinsic void resumeTask(void TaskID)							asm(opcdSystemFunctions, byte(sysFuncSetSuspendResumeTask), taskNameVariable(TaskID), byte(parmSourceConstant), word(0));

  enum TTaskStates
  {
    taskStateUninitialized                    = 0,
    taskStateStopped                          = 1,
    taskStateRunning                          = 2,
    taskStateWaitingForTimer                  = 3,
    taskStateSuspendedWasRunning              = 4,
    taskStateSuspendedWasWaiting              = 5,
    taskStateBreakpointStop                   = 6,
    taskStateWaitingForSemaphore              = 7,
    taskStateSuspendedWasWaitingForSemaphore  = 8
  };


  intrinsic TTaskStates getTaskState(void TaskID)				    asm(opcdSystemFunctions, byte(sysFuncGetTaskState),    taskNameVariable(TaskID), functionReturn);
  intrinsic void setTaskPriority(void TaskID, const short nTaskPriority)
																											asm(opcdSystemFunctions, byte(sysFuncSetTaskPriority), taskNameVariable(TaskID), variable(nTaskPriority));
	intrinsic short getTaskPriority(void TaskID)				asm(opcdSystemFunctions, byte(sysFuncGetTaskPriority), taskNameVariable(TaskID), functionReturn);
#endif

intrinsic void clearAll(compileConst TClearCommandAction nCommandFlags)
                asm(opcdClearItems, variableCompileConstWord(nCommandFlags));

#if defined(NXT) || defined(VEX2) || defined(VexIQ)
intrinsic void EndTimeSlice()                                 asm(opcdSystemFunctions, byte(sysFuncSwitchContext));
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(NXT) || defined(VEX2) || defined(VexIQ)
  intrinsic void alive()                                                asm(opcdAlive);
#elif defined(EV3)
	#define alive()
#endif

intrinsic void noOp()                                             asm(opcdNoop);

///////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                                  Debugging Opcodes
//
///////////////////////////////////////////////////////////////////////////////////////////////////////

intrinsic void setBreakpoint(compileConst ubyte breakNumb, compileConst ubyte memoryIndex, compileConst word nOffsetInSub)
                asm(opcdDebugSetBreakPoint, variableCompileConstByte(breakNumb), variableCompileConstByte(memoryIndex), variableCompileConstWord(nOffsetInSub));

intrinsic void assert()                                   asm(opcdAssert);

intrinsic void setException(TVMException nExceptionType)  asm(byte(opcdSetException), variable(nExceptionType));

intrinsic void setUserExceptionType1()          					asm(byte(opcdSetException), constantValue(exceptionUserException1));
intrinsic void setUserExceptionType2()          					asm(byte(opcdSetException), constantValue(exceptionUserException2));
intrinsic void setUserExceptionType3()          					asm(byte(opcdSetException), constantValue(exceptionUserException3));


#if defined(_DEBUG)
#define ASSERT(assertVar) {if (!(assertVar)) assert();}
#else
#define ASSERT(assertVar) {}
#endif

#define VERIFY(assertVar) {if (!(assertVar)) assert(); else {}}

#if defined(hasDebugStreamSupport)

	intrinsic void clearDebugStream()     asm(opcdDebugStreamFcns, byte(debugStreamClear));
	intrinsic void writeDebugStream(const string sFormatString, ...)
	                asm(opcdDebugStreamFcns, debugStreamFormat, variableRefString(sFormatString),  varArgListOfLongs);
	intrinsic void writeDebugStream(const char *pzFormatString, ...)
	                asm(opcdDebugStreamFcns, debugStreamFormat, variableRefCharPtr(pzFormatString),  varArgListOfLongs);
	intrinsic void writeDebugStreamLine(const string sFormatString, ...)
	                asm(opcdDebugStreamFcns, debugStreamFormatWithNewLine, variableRefString(sFormatString),  varArgListOfLongs);
	intrinsic void writeDebugStreamLine(const char *pzFormatString, ...)
	                asm(opcdDebugStreamFcns, debugStreamFormatWithNewLine, variableRefCharPtr(pzFormatString),  varArgListOfLongs);
  //intrinsic void writeDebugStream(const string sString) asm(opcdStringOps, byte(strOpDebugStream), variableRefString(sString));
  intrinsic short getAvailSpaceInDebugStream() asm(opcdDebugStreamFcns, byte(debugStreamGetAvailableSpace), functionReturn);

#endif //#if defined(hasDebugStreamSupport)



intrinsic const word propertyIndex(nDebugTaskMode,        propertySystem, kSystemDebugTaskMode);

////////////////////////////////////////////////////////////////////////////////////////////////
//
//                Event Handling Variables and Opcodes
//
////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(bHasEvents)  || defined(_WINDOWS)
intrinsic word property(eventType,               propertyEventType,				kNumbOfEvents);
intrinsic word property(EST_SENSOR,              propertyEvent,						kNumbOfEvents);
intrinsic word property(eventCounts,             propertyEventCounts,			kNumbOfEvents);
intrinsic word property(eventState,              propertyEventState,			kNumbOfEvents);
intrinsic word property(taskEvents,              propertyTaskEvents,    kNumbOfTasks+1);
intrinsic word property(eventClicks,             propertyClickCounter,		kNumbOfEvents);
intrinsic word property(eventUpper,              propertyUpperThreshold,	kNumbOfEvents);
intrinsic word property(eventDuration,           propertyDuration,				kNumbOfEvents);
intrinsic word property(eventLower,              propertyLowerThreshold,	kNumbOfEvents);
intrinsic word property(eventHysteris,           propertyHysteresis,			kNumbOfEvents);

intrinsic void DirectEvent(const int nEventMask)						asm(opcdDirectEvent, variable(nEventMask));
intrinsic void SignalEvent(compileConst byte nEventNumber)  asm(opcdSignalEvent, variableCompileConstByte(nEventNumber));
intrinsic void ResetEvent(compileConst byte nEventNumber)   asm(opcdResetEvent,  variableCompileConstByte(nEventNumber));
intrinsic void ClearAllEvents()															asm(opcdClearAllEvents);

#define EVENT_MASK(nEvent) (1 << nEvent)
#define nThisTaskEvents   taskEvents[10]

intrinsic void SetPriority(compileConst int nTaskNumber, compileConst int nPriority);

intrinsic void CalibrateEvent(compileConst byte nEventNumber, compileConst byte lowerThreshold, compileConst byte upperThreshold, compileConst byte hysteresis);

////intrinsic void SetEvent(compileConst byte nEvent, compileConst TEventSensorType nSensor, compileConst TEventTypes nType);

// Note use of 'overloads' in 'SetEvent' inline procs

///inline    void SetEvent(compileConst byte nEvent, compileConst TTimers          nTimer,  compileConst TEventTypes nType);
//inline    void SetEvent(compileConst byte nEvent, compileConst tSensors         nSensor, compileConst TEventTypes nType);
////inline    void SetEvent(compileConst byte nEvent, compileConst int              nSensor, compileConst TEventTypes nType);

intrinsic void DirectEvent(compileConst int nEventMask);
intrinsic void SignalEvent(compileConst int nEventNumber);
intrinsic void ResetEvent(compileConst int nEventNumber);
////inline void ClearEvent(compileConst int nEventNumber)          { SetEvent(nEventNumber, EST_SENSOR_UNUSED, eventTypeReset); }

//
// Several overloads for the single procedure. Allows users to easily set the 'TEventSensorType' for various types of variables
//
intrinsic void SetEvent(compileConst byte nEvent, compileConst TEventSensorType nSensor, compileConst TEventTypes nType)
                asm(opcdSetEventDefinition, variableCompileConstByte(nEvent), variableCompileConstByte(nSensor), variableCompileConstByte(nType));

#endif //#if defined(bHasEvents)


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                         Message and Serial Link Handling Opcodes
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(bUserMessagesAllowed)

  intrinsic word propertyIndex(message,          propertyMessage,       0);
  intrinsic word property(messageParm,           propertyMessage,       3);

  #define bQueuedMsgAvailable()    (cCmdMessageGetSize() > 0)

  intrinsic void sendMessage(const short nMessageID)              asm(opcdSendMessageNoParm, variable(nMessageID));
  intrinsic void sendMessageWithParm(short nMessageID, short nParm1 = 0, short nParm2 = 0)
                  asm(opcdSendMessageWithParm, variable(nMessageID), variable(nParm1), variable(nParm2));
  #define ClearMessage()                             message = 0

#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                              NXT I2C Messaging Functions
//
///////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(NXT)

	typedef enum
	{
		NO_ERR                  =  0,
		STAT_COMM_PENDING       =  32,
		ERR_COMM_CHAN_NOT_READY = -32,  // Already defined but valid here. Specified channel/connection not configured or busy.
		ERR_COMM_BUS_ERR        = -35,
		ERR_COMM_CHAN_INVALID		= -33,
	} TI2CStatusLegacy;

	typedef enum
	{
		i2cStatusNoError         =  NO_ERR,
		i2cStatusPending					= STAT_COMM_PENDING,
		i2cStatusFailed						= ERR_COMM_BUS_ERR,
		i2cStatusBadConfig				= ERR_COMM_CHAN_INVALID,
	} TI2CStatus;



	intrinsic word property(nI2CBytesReady,     propertyLegoI2CBytesReady,      kNumbOfRealSensors, tSensors);
	intrinsic TI2CStatus property(nI2CStatus,   propertyLegoI2CMessagingStatus, kNumbOfRealSensors, tSensors);

	intrinsic void sendI2CMsg(const tSensors nPort, const char *pSendMsg, const int nReplySize)
									asm(opcdSendI2CMsg, variable(nPort), variableRefCharPtr(pSendMsg), variable(nReplySize));
	intrinsic void readI2CReply(const tSensors nPort, char *pReplyBytes, const int nBytesToRead)
									asm(opcdReadI2CMsg, variable(nPort), variableRefCharPtr(pReplyBytes), variable(nBytesToRead));


#elif defined(EV3)

	enum TI2CStatus
	{
	        i2cStatusNoError         =  0,  // No error
	        i2cStatusPending,               // Transaction is being processed
	        i2cStatusFailed,                // The transaction has failed
	        i2cStatusBadConfig,             // The port has not been configured properly
	        i2cStatusStopped,               // A transaction has completed successfully.
	        i2cStatusStartTransfer = 8,     // After the initial write call, the I2C sub system returns this to indicate that is initiating the transaction
	};

	intrinsic word property(nI2CBytesReady,     propertyLegoI2CBytesReady,      kNumbOfRealSensors, tSensors);
	intrinsic TI2CStatus property(nI2CStatus,   propertyLegoI2CMessagingStatus, kNumbOfRealSensors, tSensors);

	intrinsic TI2CStatus sendI2CMsg(const tSensors nPort, const char *pSendMsg, const int nReplySize)
									asm(opcdSendI2CMsgWithReturn, variable(nPort), variable(pSendMsg), variable(nReplySize), functionReturn);
	intrinsic TI2CStatus readI2CReply(const tSensors nPort, char *pReplyBytes, const int nBytesToRead)
									asm(opcdReadI2CMsgWithReturn, variable(nPort), variable(pReplyBytes), variable(nBytesToRead), functionReturn);


#elif defined(bHasI2CPort)

	typedef enum _i2cStateStatus {
		i2cStatusUndefined = 0,
		i2cStatusStart     = 1,
		i2cStatusAddress   = 2,
		i2cStatusSend      = 3,
		i2cStatusReceive   = 4,
		i2cStatusDone      = 5,
		i2cStatusError     = 6
	} TI2CStatus;

	intrinsic word propertyIndex(nI2CBytesReady,		propertySystem, kSystemNumbI2CBytesReady);
	intrinsic TI2CStatus propertyIndex(nI2CStatus, 	propertySystem, kSystemI2CStatus);

	intrinsic void sendI2CMsg(const char *pSendMsg, const int nReplySize)
										asm(opcdSystemFunctions, byte(sysFuncSendI2cMsg), variable(pSendMsg), variable(nReplySize));
	intrinsic void readI2CReply(char *pReplyBytes, const int nBytesToRead)
										asm(opcdSystemFunctions, byte(sysFuncReadI2cReply), variable(pReplyBytes), variable(nBytesToRead));
	intrinsic void getI2CStatistics(TI2cStatistics *pStatistics, int nDataSize) asm(opcdSystemFunctions, byte(sysFuncVexCortexI2cStatistics),
																																						variable(pStatistics), variable(nDataSize));
	intrinsic void getI2CErrorPegs(TSpuriousInterruptPegCounts *pPegCounts, int nDataSize) asm(opcdSystemFunctions, byte(sysFuncVexCortexI2cErrorPegs),
																																						variable(pPegCounts), variable(nDataSize));

#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                            Transcendental / Trig / Math Opcodes
//
///////////////////////////////////////////////////////////////////////////////////////////////////////


#if defined(hasTranscendentalSupport)

intrinsic float sin(const float fRadians)							asm(opcdTranscendentalFunctions, functionReturn, variableRefFloat(fRadians), byte(mathSine));
intrinsic float cos(const float fRadians)							asm(opcdTranscendentalFunctions, functionReturn, variableRefFloat(fRadians), byte(mathCosine));
intrinsic float asin(const float Sine)								asm(opcdTranscendentalFunctions, functionReturn, variableRefFloat(Sine),     byte(mathArcSin));
intrinsic float acos(const float Cosine)							asm(opcdTranscendentalFunctions, functionReturn, variableRefFloat(Cosine),   byte(mathArcCos));
intrinsic float atan(const float Tangent)							asm(opcdTranscendentalFunctions, functionReturn, variableRefFloat(Tangent),  byte(mathArcTan));
intrinsic float atan2(const float Y, const float X)		asm(opcdSystemFunctions, byte(sysFuncAtan2),functionReturn, variableRefFloat(Y),  variableRefFloat(X));
intrinsic float degreesToRadians(const int nDegrees)		asm(opcdTranscendentalFunctions, functionReturn, variable(nDegrees),   byte(mathDegreesToRadians));
intrinsic short radiansToDegrees(const float fRadians)asm(opcdTranscendentalFunctions, functionReturn, variableRefFloat(fRadians), byte(mathRadiansToDegrees));
intrinsic float sinDegrees(const float nDegrees)			asm(opcdTranscendentalFunctions, functionReturn, variableRefFloat(nDegrees),   byte(mathSineDegrees));
intrinsic float cosDegrees(const float nDegrees)			asm(opcdTranscendentalFunctions, functionReturn, variableRefFloat(nDegrees),   byte(mathCosDegrees));

// Standard "C" uses "fabs" and not "abs" for float parameters.
intrinsic deprecated float  abs(const float input)    asm(opcdTranscendentalFunctions, functionReturn, variable(input),  byte(mathAbsFloat));
intrinsic          float fabs(const float input)      asm(opcdTranscendentalFunctions, functionReturn, variable(input),  byte(mathAbsFloat));

intrinsic unsigned short  abs(const short input)			asm(opcdTranscendentalFunctions, functionReturn, variable(input),  byte(mathAbsWord));
intrinsic unsigned long   abs(const long input)  			asm(opcdTranscendentalFunctions, functionReturn, variable(input),  byte(mathAbsLong));

intrinsic float sqrt(const short input)           		asm(opcdTranscendentalFunctions, functionReturn, variable(input),  byte(mathSqrtWord));
intrinsic float sqrt(const long input)          			asm(opcdTranscendentalFunctions, functionReturn, variable(input),  byte(mathSqrtLong));
intrinsic float sqrt(const float input)         			asm(opcdTranscendentalFunctions, functionReturn, variableRefFloat(input), byte(mathSqrtFloat));

intrinsic short sgn(const short input)								asm(opcdTranscendentalFunctions, functionReturn, variable(input),   byte(mathSgnWord));
intrinsic short sgn(const long input)         			  asm(opcdTranscendentalFunctions, functionReturn, variable(input),  byte(mathSgnLong));
intrinsic short sgn(const float input)        			  asm(opcdTranscendentalFunctions, functionReturn, variable(input), byte(mathSgnFloat));

intrinsic float exp(const float input)        			  asm(opcdTranscendentalFunctions, functionReturn, variableRefFloat(input),  byte(mathExp));
intrinsic float log(const float input)        			  asm(opcdTranscendentalFunctions, functionReturn, variableRefFloat(input),  byte(mathLog));
intrinsic float log10(const float input)      			  asm(opcdTranscendentalFunctions, functionReturn, variableRefFloat(input),  byte(mathLog10));

intrinsic float ceil(const float input)         			asm(opcdTranscendentalFunctions, functionReturn, variableRefFloat(input),  byte(mathCeil));
intrinsic float floor(const float input)        			asm(opcdTranscendentalFunctions, functionReturn, variableRefFloat(input),  byte(mathFloor));
intrinsic long  round(const float input)        			asm(opcdTranscendentalFunctions, functionReturn, variableRefFloat(input),  byte(mathRound));

intrinsic float pow(const float base, const float exponent) asm(opcdSystemFunctions, byte(sysFuncPowerOf),
																								functionReturn, variableRefFloat(base),  variableRefFloat(exponent));

const float PI = 3.14159265358979323846264338327950288419716939937510;

intrinsic bool propertyIndex(bFloatConversionErrors,							propertySystem, kSystemFloatConversionException);
intrinsic bool propertyIndex(bClearVariablesOnPgmStart,           propertySystem, kSystemClearVariablesOnPgmStart);

#else

	#if defined(VEX)

		#define abs(X) ((X < 0) ? -X : X)

	#endif
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                     String Manipulation Functions
//
///////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(hasStringSupport)

intrinsic void stringAssign(    string &sDest, const char   &nSrce) asm(opcdStringOps, strOpAssignString,variableRefString(sDest), variableRefByte(nSrce));
intrinsic void stringAssign(    string &sDest, const short  &nSrce) asm(opcdStringOps, strOpAssignByte,  variableRefString(sDest), variableRefWord(nSrce));
intrinsic void stringAssign(    string &sDest, const long   &nSrce) asm(opcdStringOps, strOpAssignShort, variableRefString(sDest), variableRefLong(nSrce));
intrinsic void stringAssign(    string &sDest, const float  &nSrce) asm(opcdStringOps, strOpAssignLong,  variableRefString(sDest), variableRefFloat(nSrce));
intrinsic void stringAssign(    string &sDest, const string &sSrce) asm(opcdStringOps, strOpAssignFloat, variableRefString(sDest), variableRefString(sSrce));
intrinsic void stringAssign(    string &sDest, const char  *pzSrce) asm(opcdStringOps, strOpAssignFloat, variableRefString(sDest), variableRefCharPtr(pzSrce));

intrinsic void stringConcatenate(string &sDest, const string &sSrce)asm(opcdStringOps, strOpConcatenateString, variableRefString(sDest), variableRefString(sSrce));
intrinsic void stringConcatenate(string &sDest, const char  *pzSrce)asm(opcdStringOps, strOpConcatenateString, variableRefString(sDest), variableRefCharPtr(pzSrce));
intrinsic void stringConcatenate(string &sDest, const byte   nSrce) asm(opcdStringOps, strOpConcatenateByte,   variableRefString(sDest), variable(nSrce));

intrinsic char stringGetChar(const string &sSrce, const int nIndex)	asm(opcdStringOps, strOpGetChar, functionReturn, variableRefString(sSrce), variable(nIndex));
intrinsic char * stringGetAddress(const string &sSrce)							asm(opcdStringOps, strOpGetAddress,   variableRefString(sSrce), functionReturn);

intrinsic void  reverseChars(const string &sSrce)			              asm(opcdStringOps, strOpReverseInPlace,  variableRefString(sSrce));
intrinsic void  reverseChars(char *pSrce)														asm(opcdStringOps, strOpReverseInPlace,  variableRefCharPtr(pSrce));
intrinsic long  atoi(const string &sSrce)			                      asm(opcdStringOps, strOp_atoi_long,  functionReturn, variableRefString(sSrce));
intrinsic long  atoi(char *pzSrce)			                            asm(opcdStringOps, strOp_atoi_long,  functionReturn, variableRefCharPtr(pzSrce));

intrinsic float atof(const string &sSrce)			                      asm(opcdStringOps, strOp_atof,  functionReturn, variableRefString(sSrce));
intrinsic float atof(char *pzSrce)			                            asm(opcdStringOps, strOp_atof,  functionReturn, variableRefCharPtr(pzSrce));

intrinsic void stringDelete(string &sDest, const int nIndex, const int nSize)	asm(opcdStringOps, strOpDelete, variableRefString(sDest), variable(nIndex), variable(nSize));

intrinsic short stringFind(const string &sSrce, const string sFindString)				asm(opcdStringOps, strOpFind,   variableRefString(sSrce), functionReturn, variableRefString(sFindString));
intrinsic short stringFind(const string &sSrce, const char *pzFindString)				asm(opcdStringOps, strOpFind,   variableRefString(sSrce), functionReturn, variableRefCharPtr(pzFindString));
intrinsic short stringFind(const char *sSrce, const string sFindString)					asm(opcdStringOps, strOpFind,   variableRefCharPtr(sSrce), functionReturn, variableRefString(sFindString));
intrinsic short stringFind(const char *sSrce, const char *pzFindString)					asm(opcdStringOps, strOpFind,   variableRefCharPtr(sSrce), functionReturn, variableRefCharPtr(pzFindString));

intrinsic short strlen(const string &sString)																		asm(opcdStringOps, strOp_strlen, variableRefString(sString), functionReturn);
intrinsic short strlen(const char   *pString)																		asm(opcdStringOps, strOp_strlen, variableRefCharPtr(pString), functionReturn);

intrinsic void stringFromChars(string &sToString, const string &sFromChars)		asm(opcdStringOps, strOpAssignString, variableRefString(sToString), variableRefString(sFromChars));
intrinsic void stringFromChars(string &sToString, const char   *pFromChars)		asm(opcdStringOps, strOpAssignCharPointer, variableRefString(sToString), variableRefCharPtr(pFromChars));

intrinsic void snprintf(char *ppDest, const short nDestLength, const string &sFormatString, ...)
                asm(opcdStringOps, strOp_snprintf,
                                variableRefCharPtr(ppDest),
                                variable(nDestLength),
                                variableRefString(sFormatString),
                                varArgListOfLongs);

intrinsic void snprintf(char *ppDest, const short nDestLength, const char *pzFormatString, ...)
                asm(opcdStringOps, strOp_snprintf,
                                variableRefCharPtr(ppDest),
                                variable(nDestLength),
                                variableRefCharPtr(pzFormatString),
                                varArgListOfLongs);

intrinsic void stringFormat(string sDest, const string &sFormatString, ...)
                asm(opcdStringOps, strOp_snprintf,
                								variableRefString(sDest),
                                byte(parmSourceConstant), byte(20), byte(0),    // constant value 20. Which is size of 'string' variable.
                                variableRefString(sFormatString),
                                varArgListOfLongs);

intrinsic void stringFormat(string sDest, const char *pzFormatString, ...)
                asm(opcdStringOps, strOp_snprintf,
                								variableRefString(sDest),
                                byte(parmSourceConstant), byte(20), byte(0),    // constant value 20. Which is size of 'string' variable.
                                variableRefCharPtr(pzFormatString),
                                varArgListOfLongs);

intrinsic void sprintf(string &sDest, const char *pzFormatString, ...)
                asm(opcdStringOps, strOp_snprintf,  variableRefString(sDest),
                                byte(parmSourceConstant), byte(20), byte(0),    // constant value 20. Which is size of 'string' variable.
                                variableRefCharPtr(pzFormatString),
                                varArgListOfLongs);

intrinsic void sprintf(char *pzCharsString, const char *pzFormatString, ...)
                asm(opcdStringOps, strOp_sprintf,
                                variableRefCharPtr(pzCharsString),
                                variableRefCharPtr(pzFormatString),
                                varArgListOfLongs);

#if defined(has_sscanf_Support)
intrinsic short sscanf(const string &sSource, const string &sFormatString, ...)
                 asm(opcdStringOps, strOp_sscanf,
                                  functionReturn,
                                  variableRefString(sSource),
                                  variableRefString(sFormatString),
                                  varArgListOfPointers);
intrinsic short sscanf(const char *pzSource, const char *pzFormatString, ...)
                asm(opcdStringOps, strOp_sscanf,
                                functionReturn,
                                variableRefCharPtr(pzSource),
                                variableRefCharPtr(pzFormatString),
                                varArgListOfPointers);
#endif

#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                 NXT File I/O Opcodes
//
///////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(NXT)


typedef short TFileHandle;

intrinsic void OpenRead(   TFileHandle &hFileHandle, TFileIOResult &nIoResult, const string &sFileName, short  &nFileSize) asm(opcdFileIO, ioOpenReadVariableFN,  variableRefWord(nIoResult), variableRefWord(hFileHandle), variableRefString(sFileName), variableRefWord(nFileSize));
intrinsic void OpenRead(   TFileHandle &hFileHandle, TFileIOResult &nIoResult, const char   *pzFileName, short  &nFileSize) asm(opcdFileIO, ioOpenReadVariableFN,  variableRefWord(nIoResult), variableRefWord(hFileHandle), variableRefCharPtr(pzFileName), variableRefWord(nFileSize));
intrinsic void OpenWrite(  TFileHandle &hFileHandle, TFileIOResult &nIoResult, const string &sFileName, short  &nFileSize) asm(opcdFileIO, ioOpenWriteVariableFN, variableRefWord(nIoResult), variableRefWord(hFileHandle), variableRefString(sFileName), variableRefWord(nFileSize));
intrinsic void OpenWrite(  TFileHandle &hFileHandle, TFileIOResult &nIoResult, const char   *pzFileName, short  &nFileSize) asm(opcdFileIO, ioOpenWriteVariableFN, variableRefWord(nIoResult), variableRefWord(hFileHandle), variableRefCharPtr(pzFileName), variableRefWord(nFileSize));

intrinsic void Close(      TFileHandle hFileHandle, TFileIOResult &nIoResult)                asm(opcdFileIO, ioClose,            variableRefWord(nIoResult),  variable(hFileHandle));
intrinsic void CloseAllHandles(TFileIOResult &nIoResult)                                     asm(opcdFileIO, ioCloseAllFileHandles, variableRefWord(nIoResult));

intrinsic void Delete(const string &sFileName,    TFileIOResult &nIoResult)             asm(opcdFileIO, ioDelete,            variableRefWord(nIoResult), variableRefString(sFileName));
intrinsic void Delete(const char   *pzFileName,   TFileIOResult &nIoResult)             asm(opcdFileIO, ioDelete,            variableRefWord(nIoResult), variableRefCharPtr(pzFileName));

intrinsic void Rename(const string &sFileName, TFileIOResult &nIoResult, const string &sOriginalFileName) asm(opcdFileIO, ioRename,variableRefWord(nIoResult), variableRefString(sFileName), variableRefString(sOriginalFileName));
intrinsic void Rename(const char  *pzFileName, TFileIOResult &nIoResult, const char   *pzOriginalFileName)asm(opcdFileIO, ioRename,variableRefWord(nIoResult), variableRefCharPtr(pzFileName), variableRefCharPtr(pzOriginalFileName));

intrinsic void ReadByte(   const TFileHandle hFileHandle, TFileIOResult &nIoResult, byte  &nParm)   asm(opcdFileIO, ioReadByte,       variableRefWord(nIoResult), variableRefWord(hFileHandle), variableRefByte(nParm));
intrinsic void ReadShort(  const TFileHandle hFileHandle, TFileIOResult &nIoResult, word  &nParm)   asm(opcdFileIO, ioReadShort,      variableRefWord(nIoResult), variable(hFileHandle), variableRefWord(nParm));
intrinsic void ReadLong(   const TFileHandle hFileHandle, TFileIOResult &nIoResult, long  &nParm)   asm(opcdFileIO, ioReadLong,       variableRefWord(nIoResult), variable(hFileHandle), variableRefLong(nParm));
intrinsic void ReadFloat(  const TFileHandle hFileHandle, TFileIOResult &nIoResult, float &fParm)   asm(opcdFileIO, ioReadFloat,      variableRefWord(nIoResult), variable(hFileHandle), variableRefFloat(fParm));
// "ReadString" -- for future implementation

intrinsic void WriteByte(  const TFileHandle hFileHandle, TFileIOResult &nIoResult, const byte   nParm)  asm(opcdFileIO, ioWriteByte, variableRefWord(nIoResult), variable(hFileHandle), variableRefByte(nParm));
intrinsic void WriteShort( const TFileHandle hFileHandle, TFileIOResult &nIoResult, const word   nParm)  asm(opcdFileIO, ioWriteShort,variableRefWord(nIoResult), variable(hFileHandle), variableRefWord(nParm));
intrinsic void WriteLong(  const TFileHandle hFileHandle, TFileIOResult &nIoResult, const long   nParm)  asm(opcdFileIO, ioWriteLong, variableRefWord(nIoResult), variable(hFileHandle), variableRefLong(nParm));
intrinsic void WriteFloat( const TFileHandle hFileHandle, TFileIOResult &nIoResult, const float  fParm)  asm(opcdFileIO, ioWriteFloat,variableRefWord(nIoResult), variable(hFileHandle), variableRefFloat(fParm));
intrinsic void WriteString(const TFileHandle hFileHandle, TFileIOResult &nIoResult, compileConst string sParm)
                asm(opcdFileIO, ioWriteStringConst, variableRefWord(nIoResult), variable(hFileHandle), stringConstant(sParm));
intrinsic void WriteString(const TFileHandle hFileHandle, TFileIOResult &nIoResult, const char *pzParm)
                asm(opcdFileIO, ioWriteStringVar, variableRefWord(nIoResult), variable(hFileHandle), variableRefCharPtr(pzParm));
intrinsic void WriteText(const TFileHandle &hFileHandle, TFileIOResult &nIoResult, string sParm)
                asm(opcdFileIO, ioWriteText, variableRefWord(nIoResult), variable(hFileHandle), variableRefString(sParm));
intrinsic void WriteText(const TFileHandle &hFileHandle, TFileIOResult &nIoResult, const char *pzParm)
                asm(opcdFileIO, ioWriteText, variableRefWord(nIoResult), variable(hFileHandle), variableRefCharPtr(pzParm));

intrinsic void FindFirstFile(TFileHandle &hFileHandle, TFileIOResult &nIoResult, const string sSearch, string &sFileName, short &nFilesize)
                asm(opcdFileIO, ioFindFirstFile,   variableRefWord(nIoResult), variableRefWord(hFileHandle), variableRefString(sSearch), variableRefString(sFileName), variableRefWord(nFilesize));
intrinsic void FindFirstFile(TFileHandle &hFileHandle, TFileIOResult &nIoResult, const char  *pzSearch, string &sFileName, short &nFilesize)
                asm(opcdFileIO, ioFindFirstFile,   variableRefWord(nIoResult), variableRefWord(hFileHandle), variableRefCharPtr(pzSearch), variableRefString(sFileName), variableRefWord(nFilesize));

intrinsic void FindNextFile( const TFileHandle hFileHandle, TFileIOResult &nIoResult, string &sFileName, short &nFilesize)
                asm(opcdFileIO, ioFindNextFile,    variableRefWord(nIoResult), variable(hFileHandle), variableRefString(sFileName), variableRefWord(nFilesize));

#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                      NXT Bluetooth Functions
//
///////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(NXT)

//
// Interface between Bluecore commands and other modules
//
typedef enum
{
  SENDFILE          = 0,
  SEARCH            = 1,    // Internal use only!
  STOPSEARCH        = 2,    // Internal use only!
  CONNECT           = 3,
  DISCONNECT        = 4,
  DISCONNECTALL     = 5,
  REMOVEDEVICE      = 6,    //
  VISIBILITY        = 7,
  SETCMDMODE        = 8,    // Internal use only!
  OPENSTREAM        = 9,
  SENDDATA          = 10,   // Internal use only!
  FACTORYRESET      = 11,
  BTON              = 12,
  BTOFF             = 13,
  SETBTNAME         = 14,
  EXTREAD           = 15,   // Internal use only!
  PinReq            = 16,   // Internal use only!
  CONNECTREQ        = 17,   // Internal use only!
  //
  // Robolab/ROBOTC additions beyond standard firmware
  //
  //
  // Robolab/RobotC additions beyond standard firmware. Some of these generate immediate replies and some need to wait
  //
  IDLE              = 18,
  LinkQualityReq    = 19,     // Requests quality of link from Bluecore
  opSetRawDataMode  = 20,     // Sets Bluecore into Raw Data mode
  opSetDefaultPIN   = 21,     // Sets default Passcode. Immediate command. Bluecore interaction not required.
  opGetDefaultPIN   = 22,     // Retrieves default Passcode. Immediate command. Bluecore interaction not required.
  opSetSessionPIN   = 23,     // Sets session Passcode. Immediate command. Initialized to "default" passcode on power up
  opGetSessionPIN   = 24,     // Retrieves session Passcode. Immediate command. Bluecore interaction not required.
  opResetSessionPIN = 25,     // Resets session Passcode to default passcode.
  opGetFriendlyName = 26,     // Gets the Bluetooth "Friendly Name" String.
  opGetBTAddress    = 27,
  opGetMyContactInfo = 28,    // Gets info on selected "My Contacts" list.
} TBTCommands;


const int kSizeOfBtName              = 16 + 1;  // Add 1 for zero termination char
const int kSizeOfBrickName           =  8 + 1;  // Add 1 for zero termination char
const int kSizeOfClassOfDevice       =  4;
const int kSizeOfBtPincode           = 16;
const int kSizeOfBtAddr              =  6;   // NOTE, NXT firmware uses 7 bytes!!

typedef ubyte TBTAddress[kSizeOfBtAddr];

typedef enum
{
  BT_STATE_VISIBLE            = 0x01,         // RW - BT visible
  BT_STATE_CONNECTED          = 0x02,         // RW - BT connected to something
  BT_STATE_OFF                = 0x04,         // RW - BT power off
  BT_ERROR_ATTENTION          = 0x08,         // W  - BT error attention
  BT_STATE_PORT_OPEN          = 0x10,         // R  - BT Port open for receiving connection. New. Moved from IOMapComm
  BT_CONNECT_REQUEST          = 0x40,         // RW - BT get connect accept in progress
  BT_PIN_REQUEST              = 0x80          // RW - BT get pin code
} TBluetoothStateX;


intrinsic TBluetoothStateX  propertyIndex(nBluetoothState,    propertySystem, kSystemBluetoothStatus);

intrinsic const bool     propertyIndex(bBTBusy,               propertySystem, kSystemBluetoothBusy);
intrinsic const bool     propertyIndex(bBTVisble,             propertySystem, kSystemBTVisible);
intrinsic const bool     propertyIndex(bBTRawMode,            propertySystem, kSystemBluetoothMode);
intrinsic bool           propertyIndex(bBTDebugTrace,         propertySystem, kSystemBluetoothDebug);
intrinsic bool           propertyIndex(bBTSkipPswdPrompt,     propertySystem, kSystemBTSkipPasswordPrompt);
intrinsic bool           propertyIndex(bBTHasProgressSounds,  propertySystem, kSystemBTHasProgressSounds);
intrinsic const TFileIOResult  propertyIndex(nBluetoothCmdStatus,   propertySystem, kSystemBluetoothCmdStatus);
intrinsic bool           propertyIndex(bBTOptimizePerformance,propertySystem, kSystemBTOptimizePerformance);
intrinsic uword          propertyIndex(nBTStreamSwitchDelay,	propertySystem, kSystemBTStreamSwitchDelay);


intrinsic TBTCommands    propertyIndex(nLastBTCommand,        propertySystem, kSystemBluetoothLastCmd);
intrinsic const short    propertyIndex(nBTCurrentStreamIndex, propertySystem, kSystemBTCurrentStreamIndex);
//intrinsic const short    variable(nBTLinkQuality,             parmSourceNxtLinkQuality, 4);


#define BTOpcdFcn(function)  opcdBTLinkIO, byte(function),  functionReturn            // NOTE, word result and not byte!!!!

intrinsic TFileIOResult btFactoryReset()                                        asm(BTOpcdFcn(FACTORYRESET));
intrinsic TFileIOResult setBluetoothRawDataMode()                               asm(BTOpcdFcn(opSetRawDataMode));

intrinsic TFileIOResult setBluetoothOn()                                        asm(BTOpcdFcn(BTON));
intrinsic TFileIOResult setBluetoothOff()                                       asm(BTOpcdFcn(BTOFF));
intrinsic TFileIOResult setBluetoothVisibility(const bool bBluetoothVisible)    asm(BTOpcdFcn(VISIBILITY), variable(bBluetoothVisible));
intrinsic TFileIOResult btRequestLinkQuality(const int nPort)                   asm(BTOpcdFcn(LinkQualityReq), variable(nPort));

intrinsic TFileIOResult btSearch()                                              asm(BTOpcdFcn(SEARCH));
intrinsic TFileIOResult btStopSearch()                                          asm(BTOpcdFcn(STOPSEARCH));

typedef enum TBtDeviceStatus
{
  btDeviceStatusSlotEmpty           = (ubyte) 0x00,
  btDeviceStatusFoundViaSearch      = (ubyte) 0x01, // A new device has been added to the table in the most recent BT search.
  btDeviceStatusPairedByBluecore    = (ubyte) 0x02, // The device is known to the internal device list in the Bluecore module.

  btDeviceStatusNameLookedUp        = (ubyte) 0x40, // At least one successful lookup of device name over BT since device powered on
  btDeviceStatusAWAY                = (ubyte) 0x80, // A known device did not respond to the last "Search" command. It may be powered off?

  //
  // Dick's add. Twmporary fields to hold search results
  //
  btDeviceStatusProcessedNameByUI   = (ubyte) 0x04, // Used during searching to indicate device has been processing
  btDeviceStatusProcessedByUI       = (ubyte) 0x08, // Used during searching to indicate device has been processing

} TBtDeviceStatus;

typedef ubyte tBTAddress[6];
typedef struct
{
  string            sFriendlyName;
  tBTAddress        nAddress;
  TBtDeviceStatus   nDeviceStatus;
} TMyContactsEntry;
//typedef TMyContactsEntry *pTMyContactsEntry;

const int kMaxMyContacts = 30;

intrinsic void btGetMyContactsEntry(const int nIndexInList, TMyContactsEntry &nInfo)
            asm(opcdBTLinkIO, byte(opGetMyContactInfo),  variable(nIndexInList), variableRefRAM(nInfo));
intrinsic TFileIOResult btConnect(const int nPort, const string &sFriendlyName)  asm(BTOpcdFcn(CONNECT), variable(nPort), variableRefString(sFriendlyName));
intrinsic TFileIOResult btDisconnect(const int nPort)                           asm(BTOpcdFcn(DISCONNECT), variable(nPort));
intrinsic TFileIOResult btDisconnectAll()                                       asm(BTOpcdFcn(DISCONNECTALL));
intrinsic TFileIOResult btRemoveDevice(string sFriendlyName)                    asm(BTOpcdFcn(REMOVEDEVICE), variableRefString(sFriendlyName));
intrinsic TFileIOResult setFriendlyName(string sFriendlyName)                   asm(BTOpcdFcn(SETBTNAME), variableRefString(sFriendlyName));

intrinsic TFileIOResult transferFile(const int nPort, const string &sFileName)     asm(BTOpcdFcn(SENDFILE), variable(nPort), variableRefString(sFileName));

intrinsic void setDefaultPIN(const string passCode)                            asm(opcdBTLinkIO, byte(opSetDefaultPIN),  variableRefString(passCode));
intrinsic void getDefaultPIN(string passCode)                                  asm(opcdBTLinkIO, byte(opGetDefaultPIN),  variableRefString(passCode));
intrinsic void setSessionPIN(const string passCode)                            asm(opcdBTLinkIO, byte(opSetSessionPIN),  variableRefString(passCode));
intrinsic void getSessionPIN(string passCode)                                  asm(opcdBTLinkIO, byte(opGetSessionPIN),  variableRefString(passCode));
intrinsic void resetSessionPIN()                                                asm(opcdBTLinkIO, byte(opResetSessionPIN));
intrinsic void getFriendlyName(string &sName)                                   asm(opcdBTLinkIO, byte(opGetFriendlyName), variableRefString(sName));

intrinsic void getBTAddress(TBTAddress &btAddr)                                 asm(opcdBTLinkIO, byte(opGetBTAddress), variableRefRAM(btAddr));
intrinsic TFileIOResult btSendMessage(const int nPort, const int nXmitLength, const byte &nXmitBuffer, const bool bWaitForReply)
                                                                                asm(BTOpcdFcn(SENDDATA),
                                                                                             variable(nPort),
                                                                                             variable(nXmitLength),
                                                                                             variableRefByte(nXmitBuffer),
                                                                                             variable(bWaitForReply));

///////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                              NXT Bluetooth Messaging Functions
//
///////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

#if defined(NXT) || defined(EV3)

//
// The NXT has ten message queues. Need to match the queue that we are polling on with the queue that
// has been selected in the "TETRIX Controller Station" program on the PC.
//
typedef enum TMailboxIDs
{
	mailbox1    =  0,
	mailbox2    =  1,
	mailbox3    =  2,
	mailbox4    =  3,
	mailbox5    =  4,
	mailbox6    =  5,
	mailbox7    =  6,
	mailbox8    =  7,
	mailbox9    =  8,
	mailbox10   =  9,

	//
	// Internal queues
	//
	mailbox11   =  10,
	mailbox12   =  11,
	mailbox13   =  12,
	mailbox14   =  13,
	mailbox15   =  14,
	mailbox16   =  15,
	mailbox17   =  16,
	mailbox18   =  17,
	mailbox19   =  18,
	mailbox20   =  19,
} TMailboxIDs;


#endif

#if defined(NXT)

const TMailboxIDs kDefaultBTQueue = mailbox1;

#define constZero()     byte(parmSourceConstant), byte(0), byte(0)
#define nxtMsgOp(subType)  opcdNXTMessageIO, byte(subType), functionReturn

intrinsic short cCmdMessageGetSize(const TMailboxIDs nQueueID = kDefaultBTQueue)
    asm(nxtMsgOp(msgOpSizeForQueue),    variable(nQueueID));
intrinsic TFileIOResult cCmdBTCheckStatus(short nStream)    asm(nxtMsgOp(msgOpBluetoothStatus), variable(nStream));
intrinsic TFileIOResult cCmdBTPurgeRcvBuffer()              asm(nxtMsgOp(msgOpPurgeReadBuffer));

//
//   Bluettoth Messaging Functions
//
intrinsic TFileIOResult nxtWriteRawBluetooth(const short nStream, const ubyte *pData, const short nLength)
    asm(nxtMsgOp(msgOpBluetoothWriteRawToPort),  constZero(),       variableRefCharPtr(pData), variable(nLength),       variable(nStream));

intrinsic TFileIOResult nxtWriteRawBluetooth(const ubyte *pData, const short nLength)
    asm(nxtMsgOp(msgOpBluetoothWriteRawNoPort),  constZero(),        variableRefCharPtr(pData), variable(nLength),       constZero());

intrinsic short nxtReadRawBluetooth(ubyte *pData, const int nMaxBufferSize)
    asm(nxtMsgOp(msgOpBluetoothReadRaw),         constZero(),        variableRefCharPtr(pData), variable(nMaxBufferSize), constZero());

intrinsic TFileIOResult cCmdMessageWriteToBluetooth(const ubyte *pData, const short nLength, const TMailboxIDs nQueueID = kDefaultBTQueue)
    asm(nxtMsgOp(msgOpMessageWriteNoPort),       variable(nQueueID), variableRefCharPtr(pData), variable(nLength),       constZero());

intrinsic TFileIOResult cCmdMessageWriteToBluetooth(const short nStream, const ubyte *pData, const short nLength, const TMailboxIDs nQueueID = kDefaultBTQueue)
    asm(nxtMsgOp(msgOpMessageWriteToPort),       variable(nQueueID), variableRefCharPtr(pData), variable(nLength),       variable(nStream));

intrinsic TFileIOResult cCmdMessageAddToQueue(const TMailboxIDs nQueueID, const ubyte *pData, const short nLength)
    asm(nxtMsgOp(msgOpMessageAddQueue),          variable(nQueueID), variableRefCharPtr(pData), variable(nLength),       constZero());

intrinsic TFileIOResult cCmdMessageRead(ubyte *pData, const short nLengthToRead, const TMailboxIDs nQueueID = kDefaultBTQueue)
    asm(nxtMsgOp(msgOpMessageReadFromQueue),     variable(nQueueID), variableRefCharPtr(pData), variable(nLengthToRead), constZero());


    ///////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                              NXT High Speed RS-485 Functions
//
///////////////////////////////////////////////////////////////////////////////////////////////////////
//
intrinsic TFileIOResult nxtWriteRawHS(const char *pData, const short nLength, const short nAddress = -1)
    asm(nxtMsgOp(msgOpHSWrite),                variableRefCharPtr(pData), variable(nLength), variable(nAddress));

intrinsic TFileIOResult nxtWriteCharRawHS(const ubyte nSingleChar, const short nAddress = -1)
    asm(nxtMsgOp(msgOpHSWriteSingleChar),      variable(nSingleChar), variable(nAddress));

intrinsic short nxtReadRawHS(ubyte *pData, const int nMaxBufferSize)
    asm(nxtMsgOp(msgOpHSRead),                 variableRefCharPtr(pData), variable(nMaxBufferSize));

const long kDefaultBaudRate = 921600;
intrinsic TFileIOResult nxtSetHSBaudRate(const long nBaudRate = kDefaultBaudRate)
    asm(nxtMsgOp(msgOpHSBaudRate),             variableRefLong(nBaudRate));

intrinsic short nxtGetAvailHSBytes()
    asm(nxtMsgOp(msgOpHSReadAvailBytes));

#define nxtEnableHSPort()         SensorType[S4] = sensorHighSpeed
#define nxtDisableHSPort()        SensorType[S4] = sensorNone

intrinsic THiSpeed_Mode propertyIndex(nxtHS_Mode,            propertySystem, kSystemHS_Mode);
intrinsic const THighSpeedState propertyIndex(nxtHS_Status,  propertySystem, kSystemHS_Status);

#undef nxtMsgOp

#endif //#if defined(NXT)



///////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                  NXT IO Map Read/Write
//
// Generic I/O Map read/write
//
///////////////////////////////////////////////////////////////////////////////////////////////////////


#if defined(NXT)
intrinsic void nxtReadIOMap(const string sIOMapName, short &nIoResult, char *pBuffer, const int nOffset, const int nNumbOfBytes)

                asm(opcdSystemFunctions, byte(sysFuncIOMapRead),
                                          variableRefString(sIOMapName),
                                          variable(nIoResult),
                                          variableRefCharPtr(pBuffer),
                                          variable(nOffset), variable(nNumbOfBytes));

intrinsic void nxtWriteIOMap(const string sIOMapName, int &nIoResult, const byte *pBuffer, const int nOffset, const int nNumbOfBytes)

                asm(opcdSystemFunctions, byte(sysFuncIOMapWrite),
                                          variableRefString(sIOMapName),
                                          variable(nIoResult),
                                          variableRefCharPtr(pBuffer),
                                          variable(nOffset), variable(nNumbOfBytes));

#endif


///////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                             Multi-Robot Messaging Functions
//
///////////////////////////////////////////////////////////////////////////////////////////////////////



#if defined(bHasMultRobotNetworking)

typedef enum TNodeIndex
{
  nDummyNode        = 0,
} TNodeIndex;


#define mrMsgOp(subType)  opcdMultiRobotNtwking, byte(subType), functionReturn

intrinsic word getMyNodeIndex()
    asm(mrMsgOp(ntwkingOpGetMyNodeIndex));

intrinsic bool bMsgAvailable()
    asm(mrMsgOp(ntwkingOpMessageAvail));

intrinsic TNtwkingStatus getNetworkingStartupFinished()
    asm(mrMsgOp(ntwkingOpStartupFinished));

intrinsic TNtwkingStatus getNetworkingStatus()
    asm(mrMsgOp(ntwkingOpStatus));

intrinsic TNtwkingStatus sendMessage(const TNodeIndex nNodeIndex, const ubyte *pData, const short nLength)
    asm(mrMsgOp(ntwkingOpSendMessage),  variable(nNodeIndex), variableRefCharPtr(pData), variable(nLength));

intrinsic TNtwkingStatus sendMessageWithWait(const TNodeIndex nNodeIndex, const ubyte *pData, const short nLength)
    asm(mrMsgOp(ntwkingOpSendMessageWithWait),  variable(nNodeIndex), variableRefCharPtr(pData), variable(nLength));

intrinsic TNtwkingStatus getMessage(TNodeIndex &nNodeIndex, ubyte *pData, const short nMaxLength,  short &nActualLength)
    asm(mrMsgOp(ntwkingOpGetMessage),  variableRefWord(nNodeIndex), variableRefCharPtr(pData), variable(nMaxLength), variableRefWord(nActualLength));

intrinsic TNtwkingStatus resetNetworkDefinition()
    asm(mrMsgOp(ntwkingOpResetNetworkDefinition));


#endif // defined(bHasMultRobotNetworking)

#if defined(hasJoystickDownloadMessageOpcodes)

	typedef struct
	{
		short   joy1_x1;           // -128 to +127
		short   joy1_y1;           // -128 to +127
		short   joy1_x2;           // -128 to +127
		short   joy1_y2;           // -128 to +127
		short   joy1_Buttons;      // Bit map for 12-buttons
		short   joy1_TopHat;       // value -1 = not pressed, otherwise 0 to 7 for selected "octant".
	} TPCJoystick;

	intrinsic void getPCJoystickSettings(TPCJoystick &joystick)				asm(opcdSystemFunctions, byte(sysFuncGetJoysticks), variableRefRAM(joystick));
#endif

#if defined(bHasUserMessaging) && defined(VexIQ_SR)
intrinsic bool userMessageAvailable() asm(opcdUsrMsgFunctions, byte(usgMsgFcn_UserMsgAvailable), functionReturn);
//intrinsic short userMessageSize() asm(opcdVexIqFunctions, byte(usrMsgFcn_UserMsgSize), functionReturn);

intrinsic bool readUserMessage(ubyte *pUserMessage) asm(opcdUsrMsgFunctions, byte(usgMsgFcn_ReadUserMsg), variable(pUserMessage), functionReturn);
intrinsic void sendUserMessage(const ubyte *pUserMessage) asm(opcdUsrMsgFunctions, byte(usgMsgFcn_SendUserMsg), variable(pUserMessage));
#endif

#if defined(VexIQ) && defined(VexIQ_SR)

	typedef enum
	{
	  DataPathNone				= (char) 0,
	  DataPathUsb					= (char) 1,
	  DataPathSmartRadio	= (char) 2
	} TVexIqDataPath;

	intrinsic TVexIqDataPath vexIqGetDataPath() asm(opcdVexIqFunctions, byte(vexIqFcn_GetDataPath), functionReturn);
#endif

#if defined(_Target_VirtWorld_)
	intrinsic short getGPSString(char *pString)									asm(opcdGPSSensorFunctions, byte(gpsSensorFcn_GetGPSStr), variableRefCharPtr(pString), functionReturn);
	intrinsic short getGPSFullString(char *pString)									asm(opcdGPSSensorFunctions, byte(gpsSensorFcn_GetGPSFullStr), variableRefCharPtr(pString), functionReturn);
#endif

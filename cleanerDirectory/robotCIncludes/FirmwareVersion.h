#ifndef __FirmwareVersion_
#define __FirmwareVersion_

#define kLastCompatibleVersionFirmware	(1056)
#define kLastCompatibleVersionIDE				(456)

#define kFirmwareVersion  			(1056)
#define kRobotCVersionNumeric		456											// Used as numeric in a preprocessor "#define" symbol created by the ROBOTC Compiler.
#define kRobotCVersionChar			"4.56"

#if defined(_WINDOWS)
	#define pzFirmwareVersion				_T("1056") 						// Used in building default file name
	#define kRobotCVersion					_T(kRobotCVersionChar)
#else
	#define pzFirmwareVersion				"1056" 								// Used in building default file name
	#define kRobotCVersion					kRobotCVersionChar
#endif

#define kLatestVEX_PIC_MasterFirmwareVersion					10

#define kLatestVEX_Cortex_MasterFirmwareVersion				((4 << 8) | ( 25 << 0))		// 4.25
#define kLatestVEX_Joystick_FirmwareVersion						((4 << 8) | ( 25 << 0))		// 4.25

#define kLatestVexIqBrain_FirmwareVersion							((2 << 8) | ( 0 << 0))		// 2.00  // Update this when brain firmware is upgraded
#define kLatestVexIqController_FirmwareVersion				((1 << 8) | ( 5 << 0))		// 1.05

#if defined(_WINDOWS)
	#define sLatestVexNet_1_5FirmwareFile								_T("WIFI_MASTER_V")		_T("13_1")		_T(".BIN")
	#define sLatestVEX_Cortex_MasterFirmwareVersion			_T("CORTEX_V")    		_T("4_25")		_T(".BIN")
	#define sLatestVEX_Joystick_FirmwareVersion					_T("JOY_V")						_T("4_25")		_T(".BIN")
	#define	sLatestEV3Kernel_FimrwareVersion						_T("107X")
#endif

#endif

#if  !defined(_TCamera_H__)
#define _TCamera_H__

#if defined(ROBOTC)
	#pragma systemFile
#endif



///////////////////////////////////////////////////////////////////////////////////////////////
//
//                              Camera Definitions
//
//////////////////////////////////////////////////////////////////////////////////////////////



typedef enum TCameraControl
{
	camStart            = 0, // Start camera. It normally is disabled and inactive
	camStop             = 1, // Stop camera
	camUseDefaultConfig = 2,
	camSetParameters    = 3
} TCameraControl;


/*
typedef enum
{
	camSendBasicRegisters     = 0,
	camSetTrackingWindow      = 1,
	camSetViewWindow          = 2
} TCameraSetupCommands;
*/

typedef enum TResolution
{
	kFullFrame                    = 0,
	kHalfFrame                    = 1,
	kQuarterFrame                 = 2
} TResolution;

typedef struct TCameraConfig
{
	//
	// Fields requiring CMUCAM commands
	//
	ubyte Resolution;
	ubyte DownSample;
	ubyte StabilizationTime;
	//
	// Fields set with image sensor control registers
	//
	ubyte RGBColorSpace;
	ubyte AGCEnable;
	ubyte AutoWhiteBalance;
	ubyte AECDisable;
	ubyte FluoresecentFilter;
} TCameraConfig;


typedef struct TBasicRegisters
{
	//
	// Used to set up ColorSpace, AGC, WhiteBalance, Auto Exposure, Fluorescent Filter
	//
	ubyte ComA;     //  AGC, AWB, RGB/YCrCb
	ubyte ComB;     //  Enables auto controls
	ubyte ComI;     //  AEC
	ubyte ComJ;     //  Fluorescent Filter
} TBasicRegisters;

typedef struct TCameraViewWindow
{
	ubyte left;
	ubyte top;
	ubyte right;
	ubyte bottom;
} TCameraViewWindow;

typedef struct TCameraTrackingControl
{
	ubyte Red_Min;
	ubyte Red_Max;
	ubyte Green_Min;
	ubyte Green_Max;
	ubyte Blue_Min;
	ubyte Blue_Max;
	ubyte NoiseFilter;
	ubyte TrackInverted;
} TCameraTrackingControl;

typedef struct TServoControl
{
	ubyte PanRangeFar;
	ubyte PanRangeNear;
	ubyte PanStep;
	ubyte TiltRangeFar;
	ubyte TiltRangeNear;
	ubyte TiltStep;
} TServoControl;


typedef struct TAdvancedRegisters
{
	ubyte AGCGain;
	ubyte BlueGain;
	ubyte RedGain;
	ubyte Saturation;
	ubyte Brightness;
	ubyte Exposure;
	ubyte Sharpness;
	ubyte WhiteBalanceBlue;
	ubyte WhiteBalanceRed;
} TAdvancedRegisters;



// CMUCAM Camera Tracking Packet Structure

typedef struct TTrackingData
{
	ubyte horizontal;
	ubyte vertical;
	ubyte left;
	ubyte top;
	ubyte right;
	ubyte bottom;
	ubyte pixels;
	ubyte confidence;
}	TTrackingData;


typedef enum TCameraState
{
	//
	// Do not change entries without adjusting the array that follows in the code!!
	//
	camMsgDisabled                        = 0,
	camMsgInitialize                      = 1,
	camStateInitialize,
	camStateInitializeWait,
	camStateInitializeGotAttention,

	camStateInitializeWaitForRawModeReset,
	camStateInitializeSendRawMode,
	camStateInitializeWaitForRawMode,
	camStateLoadCameraRegistersStart,
	camStateLoadCameraRegistersWaitAck,

	camStateLoadCameraRegisters,
	camStateSetResolution,
	camStateSetResolutionWaitAck,
	camStateSetDownSample,
	camStateSetDownSampleWaitAck,

	camStateLoadNoiseFilter,
	camStateLoadNoiseFilterWaitAck,
	camStateLoadTrackColor,
	camStateLoadTrackColorWaitAck,
	camStateTracking,

	camStateStartShutdown,
	camStateShutdownGotAttention,
	camStateShutdownPowerOffCamera,
	camStateShutdownPowerOffCameraWaitAck,

	kNumbOfCameraStates
} TCameraState;

#if defined(ROBOTC)
	#include <Includes/TCameraResults.h>
	#include "cameraIntrinsics.h"
#endif

#endif

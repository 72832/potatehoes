#ifndef LoadBuildOptions__H_
#define	LoadBuildOptions__H_

#if  !(defined(MainIncludes__H_) || defined(RobotCIncludes__H_)) && !defined(_WINDOWS)
	#error "'LoadBuildOptions.h' should only be '#included' from 'mainIncludes.h' file"
#endif

#ifdef __cplusplus
extern "C" {
#endif

//#define	debugLowSpeed  1				// I2C Debug Trace Enabled
#define	debugLowSpeed  0					// I2C Debug Trace Disabled

//#define	debugMotors  1					// Motor Debug Trace Enabled
#define	debugMotors  0						// Motor Debug Trace Disabled

// NOTE: Combinining below in a single logical expression "blew up" the ROBOTC Compiler's limit of 15 <ops> in a sub-expression. Each '||' is a <op>.

#if defined(NXT) || defined(EV3)
#elif defined(VEX2) || defined(VEX) || defined(VexIQ) 
#elif defined(TSTPA)
#elif defined(Arduino) || defined(ArduinoSmallMemory) || defined(ArduinoMega) || defined(ArduinoDue) || defined(ArduinoCortex)
#elif defined(Generic) || defined(Simulator) || defined(_WINDOWS)
#elif defined(STM8) || defined(CHIPKITMAX)
#elif defined(STM3210E) || defined(SX256)
#elif defined(RPI)
#else
	 #error("Platform type for compile is illegal")
#endif


//////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                             Toggles to Control Optional Features
//
//////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(_WINDOWS)
	#include "LoadBuildOptionsWindows.h"
#elif defined(NXT)
	#include "LoadBuildOptionsNXT.h"
#elif defined(EV3)
	#include "LoadBuildOptionsLegoEV3.h"
#elif defined(VEX)
	#include "LoadBuildOptionsVexPIC.h"
#elif defined(VEX2)
	#include "LoadBuildOptionsVexCortex.h"
#elif defined(VexIQ)
	#include "LoadBuildOptionsVexIQ.h"
#elif defined(TSTPA)
	#include "LoadBuildOptionsTSTPA.h"
#elif defined(ArduinoMega)
	#include "LoadBuildOptionsArduinoMega.h"
#elif defined(Generic)
	#include "LoadBuildOptionsArduinoMega.h"
#elif defined(ArduinoCortex)
	#include "LoadBuildOptionsArduinoCortex.h"
#elif defined(ArduinoSmallMemory)
	#include "LoadBuildOptionsArduinoSmallMemory.h"
#elif defined(CHIPKITMAX)
	#include "LoadBuildOptionsArduinoChipKitMax.h"
#elif defined(STM8)
	#include "LoadBuildOptionsArduinoSTM8.h"
#elif defined(RPI)
  #include "LoadBuildOptionsRPi.h"
#else
	#error "Missing preprocessor variable for platform type"
#endif

#if defined(_Target_VirtWorld_) || defined(_WINDOWS)
	#define includeRVWLocationSensorSupport
#else
	#undef includeRVWLocationSensorSupport
#endif

#if !defined (optionalDebugSupport)
	#if defined(optionalBreakpoints)
		#warning "Inconsistent macro definitions"
	#endif
	#undef	optionalBreakpoints
#endif

#if defined(hasHarvardArchitecture)
	#if defined(useParmForCurrOpPtr)
		#error "'useParmForCurrOpPtr' cannot be defined for Harvard architecture CPUs!"
	#endif
	#undef	useParmForCurrOpPtr
#endif
//
// Flag to indicate whether serial link communications can be customized. Only relevant to the RCX
// platform.
//
#undef	bSerialLinkCustomizable

//////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                      Handle dependencies between options
//
//    1. Some options always required for windows
//    2. Some options invalid without 100X firmware version
//
//////////////////////////////////////////////////////////////////////////////////////////////////////

//
// Never support floats and longs without 100X firmware!
//
#if (bDebugAddressAlignment)
	#define	validateWordAddress(pAddress) pAddress = (word *) ((unsigned int) pAddress & 0xFFFFFFFE)
	#define	validateLongAddress(pAddress) pAddress = (long *) ((unsigned int) pAddress & 0xFFFFFFFC)
#else
	#define	validateWordAddress(pAddress)
	#define	validateLongAddress(pAddress)
#endif


#if defined(useFloats)
	#define	useFloatsOrLongs
#elif defined(useLongs)
	#define	useFloatsOrLongs
#else
	#undef	useFloatsOrLongs
#endif


#if defined(bHasEvents)
	#define	kNumbOfEvents									((unsigned char) 16)
#else
	#undef	kNumbOfEvents
	#undef	kNumbOfVirtualSensors
	#undef	kNumbOfVirtualMotors

	#define	kNumbOfEvents									((ubyte) 0)
	#define	kNumbOfVirtualSensors         ((ubyte) 0)
	#define	kNumbOfVirtualMotors          ((ubyte) 0)
#endif

#define	kNumbOfTotalSensors           ((unsigned char) (kNumbOfRealSensors + kNumbOfVirtualSensors))

#define	kNumbOfTotalMotors            ((unsigned char) (kNumbOfRealMotors + kNumbOfVirtualMotors))

#define	kNumbOfTimers									((unsigned char) 4)

#if defined(_WINDOWS)
	#define kNumbOfSerialPortsForArrays 6
#else
	#define kNumbOfSerialPortsForArrays getNumbOfSerialPorts()
#endif
#ifdef __cplusplus
}
#endif


#define sensorUploadSizeShortX			0
#define sensorUploadSizeLongX				1
#define sensorUploadSizeFloatX			2

#endif //#ifndef LoadBuildOptions__H_

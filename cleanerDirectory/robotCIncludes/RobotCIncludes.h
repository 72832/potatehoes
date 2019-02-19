#if !defined(RobotCIncludes__H_)
#define RobotCIncludes__H_

#include "LoadBuildOptions.h"

const bool true  = (bool) 1;
const bool false = (bool) 0;

typedef unsigned char  ubyte;
typedef   signed char  sbyte;
typedef unsigned short uword; // unsigned variables are not supported. Everything is signed!! But allow definition


#ifndef INLINE
	#define INLINE inline
#endif

#include "OpcodeDefinitions.h"

#if defined(NXT)
	#include "NXTFileIODefs.h"
	#include "NxtSynchMotors.h"
#endif


#if defined(hasStringSupport)
	#include "stringOpcodes.h"
#endif
#if defined(bHasLCDGraphicsDisplay) || defined(hasDebugStreamSupport)
	#include "GraphicLCD.h"
#endif

#include "RobotCIntrinsics.c"

#if defined(VexIQ)
	#include "VexIqRobotCIntrinsics.h"
#elif defined(EV3)
	#include "EV3Intrinsics.h" //THIS MUST BE BEFORE EV3RobotCIntrinsics.h!
	#include "EV3RobotCIntrinsics.h"
#endif

#endif

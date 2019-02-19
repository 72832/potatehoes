//----------------------------------
//	- Leagcy Natural Language Switcher
//----------------------------------

#pragma systemFile            	// eliminates warning for "unreferenced" functions

// Choose Virtual World or Physical Robot:
#if defined(_Target_VirtWorld_)
	// Virtual World:  Now check which platform type:
	#if defined(VEX2)
		#include "NatLang_Legacy_CORTEX_VW.c"
	#else
		##error "Legacy Natural Language is not supported for this platform type (VIRTUAL WORLDS)."
	#endif

#elif defined(_Target_Robot_)
	// Physical Robot:  Now check which platform type:
	#if defined(VEX2)
		#include "NatLang_Legacy_CORTEX.c"
	#else
		#error "Legacy Natural Language is not supported for this platform type."
	#endif

#elif defined(_Target_Emulator_)
	#error "Legacy Natural Language is not supported for 'Emulator' target.  Please select either Physical Robot or Virtual World"

#else
	#error "Legacy Natural Language is not supported for this unknown debug target.  Please select either Physical Robot or Virtual World"

#endif

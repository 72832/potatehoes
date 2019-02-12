/*----------------------------------*\
|*	- Natural Language Switcher -	 |
\*----------------------------------*/

#pragma systemFile            	// eliminates warning for "unreferenced" functions

// Choose Virtual World or Physical Robot:
#if defined(_Target_VirtWorld_)
	// Virtual World:  Now check which platform type:
	#if defined(NXT)
		#include "NatLang_NXT_VW.c"
	#elif defined(EV3)
		#include "NatLang_EV3_VW.c"
	#elif defined(VEX2)
		#include "NatLang_CORTEX_VW.c"
	#elif defined(VexIQ)
		#include "NatLang_IQ_VW.c"
	#else
		##error "Natural Language is not supported for this platform type (VIRTUAL WORLDS)."
	#endif

#elif defined(_Target_Robot_)
	// Physical Robot:  Now check which platform type:
	#if defined(NXT)
		#include "NatLang_NXT.c"
	#elif defined(EV3)
		#include "NatLang_EV3.c"
	#elif defined(VexIQ)
		#include "NatLang_IQ.c"
	#elif defined(VEX2)
		#include "NatLang_CORTEX.c"
	#else
		#error "Natural Language is not supported for this platform type."
	#endif

#elif defined(_Target_Emulator_)
	#if defined(NXT)
		#include "NatLang_NXT.c"
	#elif defined(EV3)
		#include "NatLang_EV3.c"
	#elif defined(VexIQ)
		#include "NatLang_IQ.c"
	#elif defined(VEX2)
		#include "NatLang_CORTEX.c"
	#else
		#error "Natural Language is not supported for this platform type."
	#endif
	//#error "Natural Language is not supported for 'Emulator' target.  Please select either Physical Robot or Virtual World"

#else
	#error "Natural Language is not supported for this unknown debug target.  Please select either Physical Robot or Virtual World"

#endif

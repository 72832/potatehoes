#ifndef _GraphcisLCD__H_
#define _GraphcisLCD__H_

typedef enum TGraphicDisplayCommands
{
	displayDrawEraseScreen                       =  0,
	displayDrawSetPixel                          =  1,
	displayDrawClearPixel                        =  2,
	displayDrawLine                              =  3,
	displayDrawInvertEllipse                     =  4,
	displayDrawInvertRect                        =  5,
	displayDrawRect                              =  6,
	displayDrawEraseRect                         =  7,
	displayDrawEllipse                           =  8,
	displayDrawGetPixel                          =  9,
	displayDrawFillEllipse                       = 10,
	displayDrawEraseEllipse                      = 11,
	displayDrawFillRect                          = 12,
	displayDrawRICFile                           = 13,
	displayDrawTextLineVarArgs                   = 14,
	displayDrawBigTextLineVarArgs                = 15,
	displayDrawCenteredTextLineVarArgs           = 16,
	displayDrawCenteredBigTextLineVarArgs        = 17,
	displayDrawStringVarArgs                     = 18,
	displayDrawStringAtCoordVarArgs              = 19,
	displayDrawBigStringAtCoordVarArgs           = 20,  // Use Big Font
	displayDrawScrollTextVarArgs                 = 21,
 	displayDrawEraseLine                         = 25,
	displayDrawInvertLine                        = 26,
	displayDrawGetScreenSize                     = 27,
	displayDrawInverseStringAtCoordVarArgs			 = 28,
	displayDrawInverseBigStringAtCoordVarArgs		 = 29,
	displayDrawInverseStringVarArgs              = 30,
	displayDrawInverseTextLineVarArgs            = 31,

	displayDrawCircle                            = 32,
	displayDrawInvertCircle                      = 33,
	displayDrawFillCircle                        = 34,
	displayDrawEraseCircle                       = 35,

	displayDrawEraseUserScreenArea               = 36,
	displayDrawInvertPixel                       = 37,

	displayDraw_EV3DrawPicture									 = 38,		// EV3 Only

} TGraphicDisplayCommands;

//
// Commands for menu support on the Graphics LCD
//
#if defined(hasLCDMenuFunctions)

typedef enum TGraphicDisplayMenuCommands
{
	menuOpInitialize,
	menuOpSetProgramName,
	menuOpSetTitle,
	menuOpRepaint,
	menuOpScroll,
	menuOpGetSelection,
	menuOpAddItem,
	menuOpUpdateCommandText,
	menuOpIsCommandVisible,

	// Place here for now
	menuOpDrawTextCenteredInUserScreenArea,
	menuOpSetInitialProgramRunningScreen
} TGraphicDisplayMenuCommands;

#endif

#endif

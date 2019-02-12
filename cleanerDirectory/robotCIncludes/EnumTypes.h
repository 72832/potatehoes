#if !defined(__EnumTypes_H_)
#define __EnumTypes_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//							Various Enum Types Jointly used by ROBOTC VM Firmware and ROBOTC IDE
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef enum TFileExtensionTypes
{
	ftUnknown           = 0,
	ftNone              = 0,

	ftExecutable        = 1,

	ftSound             = 2,
	ftWAV				= 3,

	ftText              = 4,
	ftData              = 5,
	ftIcon              = 6,

	ftMIDI				= 7,

	ftMenuCommand		= 8,
	ftSystemCommand		= 9,

	ftLast
} TFileExtensionTypes;


#endif

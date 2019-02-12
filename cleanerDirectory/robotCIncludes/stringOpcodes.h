#ifndef _stringOpcodes__H_
#define _stringOpcodes__H_

///////////////////////////////////////////////////////////////////////////////////////////////
//
//                               String Opcode Functions
//
///////////////////////////////////////////////////////////////////////////////////////////////

//
// String I/O Command Types
//
typedef enum TStringSubOps
{
	strOpAssignString														=  0,				// Assign string variable to string variable
	strOpAssignCharPointer											=  1,				// Assign char pointer    to string variable

	strOpAssignByte															=  2,				// Type Conversion -- byte	numeric variable/value to string
	strOpAssignShort														=  3,				// Type Conversion -- short numeric variable/value to string
	strOpAssignLong															=  4,				// Type Conversion -- long	numeric variable/value to string
	strOpAssignFloat														=  5,				// Type Conversion -- float numeric variable/value to string

	strOpConcatenateString											=  6,				// Concatenate string variable to string variable
	strOpConcatenateCharPointer									=  7,				// Concatenate char pointer    to string variable
	strOpConcatenateByte												=  8,				// Concatenate single char     to string variable

#if defined(has_sscanf_Support)
	strOp_sscanf																=  9,				// Standard 'sscanf' function'
#endif

	strOpSubString															= 10,
	strOpFind																		= 11,
	strOp_strlen																= 12,

	strOp_sprintf																= 13,
	strOp_snprintf															= 14,

	strOpGetChar																= 15,
	strOpGetAddress															= 16,

	strOp_atoi_long															= 17,
	strOp_atoi_short														= 18,
	strOp_atof    															= 19,

	strOpDelete																	= 20,

	strOpReverseInPlace													= 21,

	strOpBad,

} TStringSubOps;

#define kStringSize 20


typedef enum TDebugStreamSubOps
{
	debugStreamGetAvailableSpace								= 0,				// Debug stream support
	debugStreamFormat														= 1,				// Debug stream support
	debugStreamFormatWithNewLine								= 2,				// Debug stream support
	debugStreamClear														= 3,				// Debug stream support
} TDebugStreamSubOps;

#endif

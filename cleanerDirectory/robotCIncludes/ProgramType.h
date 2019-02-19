#if !defined(_programType__H_)
#define _programType__H_


#ifdef __cplusplus
extern "C" {
#endif


typedef enum TProgramType
{
	pgmTypeNotYetAssigned               = 0xFF, //VEX Cortex   +   VEX PIC
	pgmTypeStandaloneWithWiFi           = 1, //VEX Cortex
	pgmTypeStandaloneWithUSBCable       = 2, //VEX Cortex   +   VEX PIC
	pgmTypeCompetition									= 3, //                 VEX PIC
	pgmTypeCompetitionAutonomous				= 4, //                 VEX PIC
	pgmTypeCompetitionUserControl				= 5, //                 VEX PIC
} TProgramType;

#ifdef __cplusplus
};
#endif


#endif

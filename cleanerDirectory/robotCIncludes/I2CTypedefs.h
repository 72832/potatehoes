#if !defined(I2C_Typedefs__H_)

#define I2C_Typedefs__H_

#define kMaxI2cDevices 12

	typedef enum
  {
    i2cStatesWaitForCompletionSTART,
    
    i2cStatesWaitForCompletionAddressForSend,
    i2cStatesWaitForCompletionSendByte,
    
    i2cStatesWaitForCompletionAddressForRcv,
    i2cStatesWaitForCompletionRcvByteFirst,
    i2cStatesWaitForCompletionRcvByteMore,
    i2cStatesWaitForCompletionRcvByteLast,
    
    i2cStatesMessageComplete,
    i2cStatesMessageFailed,
    
    kNumbOfI2CStates,
  } TI2cMsgStates;

	typedef struct
	{
		unsigned long		nNumbOfSuccessfullPolls;
		unsigned long		nCumulativefFailedPolls;
		unsigned long		nNumbOfFailedPolls;
  } TI2CPegCounts;

	typedef struct
	{
		unsigned long		nTotalAddressResets;						// Total number of address resets
		bool						bI2CNeverResponded;							// 'true' indicates that I2C hardware has never responded
		unsigned short	nNumbOfI2CPortsDetected; 				// Number of daisy chained encoders found
		unsigned long		nTotalIdlePollingCycles;				// Total number of polling cycles with no active devices
		unsigned long		nTotalPollingCycles;						// Total number of device polling cycles
		unsigned long		nTotalI2cMessagesSent;					// Total number of I2C messages sent
		unsigned long		nTotalACK_Failures;						  // Total number of ACK failures during I2C message transmission
		unsigned long		nTotalArbitrationLost;					// Total number of totally unexpected "Arbitration Lost" interrupts
		unsigned long		nTotalDataOverflow;					    // Total number of data overflow errors on rcv or transmit data
		unsigned long		nTotal10MsecTimeouts; 					// Total number of 10 msec transaction timeouts
		unsigned long   nTotalUnexpectedError;			    // Total number of unexpected I2C error type interrupts
		unsigned long   nTotalInvalidStatesEncountered;	// Total number of invalid 'I2C state machine' values found during event processing
		unsigned long   nTotalLateInterruptsRecovered;	// Total number of excessive delays before processing I2C char Received interrupt. Recovered from this situation
    unsigned long		nTotalTimeouts;						      // Total number of timeouts (message took too long) causing message failure
    unsigned long   nI2cStateAtTimeout[kNumbOfI2CStates + 1];//Peg count for I2C state when timeout occurred
		unsigned long   nMissedXmitBufferReadyInterrupt;// Total number of missed interrupts
    unsigned long		nTotalI2cMessagesSuccessful;		// Total number of successful messages
		unsigned long		nTotal1MillisecondTicks;				// Total number of 1-millisecond ticks

    unsigned long   nPegCountMsgDurationTimerTicks[13];// Total number of I2C messages that finished with 'NN' intermediate ticks of 1-msec background task

		unsigned long   nTotalMicroseconds;
		unsigned long   nAverageMicroSecPerMessage;
		unsigned long   nStartMicroseconds;
    TI2CPegCounts   nPegCounts[kMaxI2cDevices];
	} TI2cStatistics;

  typedef struct
  {
    unsigned int nTotalSpuriousCount;
    unsigned int nUnknown;
    unsigned int n0x00000000;
    unsigned int n0x00000040;
    unsigned int n0x20008000;
    unsigned int n0x00030000;
    unsigned int n0x00030084;
    unsigned int n0x00030044;
    unsigned int n0x00000084;
    unsigned int n0x00030041;
  } TSpuriousInterruptPegCounts;

  #endif //#if !defined(I2C_Typedefs__H_)

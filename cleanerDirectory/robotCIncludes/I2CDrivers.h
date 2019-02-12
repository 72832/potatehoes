#if !defined(__I2C_Drivers__)
#define __I2C_Drivers__

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                    Configure Internal Firmware Device Driver
//
// ROBOTC firmware uses an internal background polling mechanism to continuously retrieve the values from the
// digital sensors that it has a built-in driver. This file provides the functionality to expose this interface
// to end user application programs.
//
//
// Analog Sensors:
// ==============
// Working with an analog sensor on the NXT is straightforward. The NXT firmware is continuously performing a
// background task that does an analog-to-digital conversion on the sensor. Every analog sensor is polled once every
// three milliseconds. When a user application program accesses the sensor value it immediately retrieves the last
// polled value; it does not wait 0.003 seconds for a new A-to-D conversion.
//
// It is easy to build complete support for all analog sensor types in the NXT firmware. For the LEGO supplied
// sensors, the internal firmware may do an optional conversion step to take the raw analog value (a number in the
// range 0..1023) and rescale it – e.g. as a percentage or a temperature reading.
//
// For new custom analog sensors, the raw analog value can always be obtained from the firmware.
//
//
// Digital Sensors:
// ===============
// Digital I2C sensors on the NXT are more complicated.
//
// * Analog sensors have a single value. Digital sensors may contain several values (e.g. an accelerometer with
//   three axes).
//
// * A single common A-to-D conversion obtains the raw value of any analog sensor. Digital sensors obtain values
//   via a “I2C message” to read the values from the smart digital sensor. Each sensor type requires a unique
//   message for reading.
//
// * Digital sensors typically require an initialization I2C message to configure the sensor.
//
// * If you poll some digital sensors too fast (e.g. the ultrasonic sensor) it will return erroneous results.
//
// * Some digital sensors change their values very slowly. There is not a need to read the values at a faster
//   rate (e.g. the Codatex RFID sensor should not be polled faster than about every 0.075 seconds.
//
// * Some digital sensors need a periodic I2C message. Otherwise they may enter a low-power mode where they are
//   no longer properly functioning.
//
// * Some of the newer sensors have many registers that need to be polled. This can take 10 to 15 milliseconds
//   (at about one byte per 1.2 milliseconds) to retrieve the sensor value(s) using standard (slow) speed I2C
//   communications.
//
// Historically most custom sensors have been supported by writing application code invoked by a user program
// to retrieve the sensor’s values. Generally the code retrieves the values as inline execution which delays
// the rest of the user’s application code.
//
// ROBOTC firmware uses an internal background polling mechanism to continuously retrieve the values from the
// digital sensors that it has a built-in driver. This file provides the functionality to expose this interface
// to end user application programs.
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                Configure I2C Background Device Driver
//
// Filling in the following data structure and passing it to the ROBOTC firmware will provide a data definition to
// the firmware that is used to implement the background polling. The data includes:
//
// 1. An I2C initialization message that is sent to the sensor to configure it. This mesage is sent on startup
//    and every time there is an I2C messaging error in communications with the sensor.
//
// 2. The delay interval between when the initializaiton message is sent and the driver can start polling the sensor.
//
// 3. The starting register (i.e. "address") within the I2C sensor of the data to be polled and the number of
//    consecutive registers to poll.
//
// 4. The period to use for a polling cycle.
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


typedef enum TI2CDataFormat
{
	fmtCustom             = 0,   // Always use this value. Other values are for internal ROBOTC use.
	fmtSingleByte         = 1,
	fmtTwoBytes           = 2,
	fmtSonar              = 3,
	fmtAccel              = 4,
	fmtI2CTemperature     = 5,
} TI2CDataFormat;


#define kMaxInitMessageBytes 8

typedef enum TI2CSignallingSpeed
{
	i2cFastClockAndSkipStates    = 0,     // faster clock and skip wait states
	i2cFastClock,                         // Faster clock (0x0C vs 0x20 period)
	i2cSlowClock                          // Ultrasonic compatible
} TI2CSignallingSpeed;

typedef struct TI2CSensorSetupData
{
	ubyte 							 	nInitializaionMsgSize; 				// Size of initialization message. Zero indicates no initialization message needed
	ubyte 								nInitializationMsg[kMaxInitMessageBytes];  // Message used to initialize / configure the sensor.
	ubyte 								nInitializationSettlingTime;	// Delay after initialization message before first poll
	ubyte 								nPollFrequencyInMSec;					// Delay between polling requires.
	ubyte 								nPollingStartRegisterIndex; 	// Frist register in sensor to be polled (e.g. SONAR is 0x42)
	ubyte 								nPollSizeInBytes;							// Number of bytes that should be polled from sensor (range 1..16). Zero indicate no polling.
	TI2CDataFormat				nDataFormat;									// Internal driver use. Should always be zero.
	TI2CSignallingSpeed 	nI2CSpeed;
	short 								nUninitializedValue;			 		// Value to return for sensor value before first poll has been completed
} TI2CSensorSetupData;



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                   Retrieve the Last Poll Results from the I2C Sensor
//
// Once an I2C sensor has been configured as above, then the user application program can easily request a copy of
// the latest polling results. The struct "TI2CHandlerInfo" contains this information and more -- some of the extra
// information is internal states used by the firmware.
//
// There are three fields of interest:
//
// nNumbOfInits                     is a count of the number of initialization messages sent to the sensor. With a
//                                  perfectly working sensor, this count should be 1. But the sensor is re-initialized
//                                  every time there is an I2C messaging failure so it also can be used to calculate the
//                                  error rate on the sensor link.
//
// nNumbOfPolls                     a counter of the number of polling messages sent to the sensor.
//
// nReplyBuffer                     a buffer containing the last reply successfully received from the sensor.
//
// The ROBOTC NXT firmware provides two interfaces for getting a copy of this data:
//   1. Get a copy of the "reply data" only.
//   2. Get a copy of the complete structure.
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


typedef enum TI2cDriverState																	// States used by the I2C device driver implementation
{
	hdlrNotI2cSensor                   = 0,			// Not an I2C Sensor
	hdlrStartup,																// Startup driver and check if initialization is required
	hdlrDriverDisabled,													// I2C sensor without any background driver activities required.
	hdlrSendInitializationMsg,									// Send initialization message as soon as I2C bus is ready
	hdlrWaitForInitializationFinished,					// Wait for initializaton msg to complete successfully; otherwise re-initialize sensor
	hdlrStartupDelay,														// Delay before first polling request
	hdlrNoBackgroundPolling,										// Sensor is not configured for background polling.
	hdlrSendDataRequest,												// Send polling message
	hdlrWaitingForReply,												// Waiting for successful reply from polling; otherwise re-initialize sensor
	hdlrDelayBetweenSamples											// Delay before next polling message.
} TI2cDriverState;


typedef struct TI2CStatistics
{
	// Applies to any I2C message (either from internal firmware drivers or from user program activity

	long              nNumbOfSendData;          // Number of calls to "send message"
	long              nNumbOffailedPullups;     // Number of times "send message" failed the "pull up" test
	long 							nNumbOfI2CSuccesses;			// Number of successful I2C Messages 					(counts user and integrated driver messages)
	long              nTotalRetransmits;				// Number of internal message retry attempts	(counts user and integrated driver messages)
	long 							nNumbOfI2CFailures;				// Number of failed I2C Messages							(counts user and integrated driver messages)

	// Applies to internal firmware background driver
	long 							nTotalDriverInitMsgs;			// Number of initialization messages sent
	long 							nTotalDriverPollMsgs;			// Number of polling messages sent

	TI2cDriverState		nHdlrState;								// I2C Internal firmware Device driver state
} TI2CStatistics;



#endif   //#if !defined(__I2C_Drivers__)

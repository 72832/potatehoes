#pragma once

#if defined(NXT)

//#include <RobotFirmware/Includes/I2CDrivers.h>
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                Access Internal Firmware I2C Background Polling Device Driver
//
// ROBOTC firmware uses an internal background polling mechanism to continuously retrieve the values from the
// digital sensors that it has a built-in driver. This file provides the functionality to expose this interface
// to end user application programs.
//
// Its relatively easy to use the background I2C polling capabilities. Simply follow these steps:
//
// 1. First configure the sensor port as a "custom I2C" type.
//        1) ROBOTC firmware supports three different types which provide varying improvement in I2C messaging
//           speed. The slowest speed is compatible with the NXT-G firmware implementation. Higher speeds work
//           with almost all third party sensors.
//
//           sensorI2CCustom                Slow. Compatible with NXT-G firmware.
//
//           sensorI2CCustomFast             2 to 3 times faster. So far, all 3rd party sensors are compatible.
//
//           sensorI2CCustomFastSkipStates   Up to 5 times faster. Not compatible with some 3rd party sensors.
//
//    The sensor will be configured as able to generate I2C messaging traffic. There will be no background polling.
//
//    There are another three sensor types (sensorI2CCustom9V, ...) which additionally instruct the firmware to
//    provide 9V power to the sensor.
//
// 2. Setup the background polling configuration. This is performed with a call to the "configureI2CPolling"
//    intrinsic. This provides a data configuration to the firmware that is now used to perform the background
//    polling.
//        1) The driver starts by sending an initialization message to configrue the sensor. It repeats this
//           message until it is successfully transmitted -- e.g. the sensor cable may be unplugged.
//        2) A short delay before sending the first polling message.
//        3) Send a message to poll the data from the sensor.
//        4) Receive the polling reply and buffer it. If there was an I2C error, then go back to initialization
//           state (1).
//        5) Delay between polling cycles.
//        6) Go back to state (3).
//
//
// Background polling can be halted with a call to "configureI2CPolling" with a "NULL" entry.
//
//
// Two intrinsic functions are available to read the last polled data.
//
//         getI2CSensorReply()     returns the last polled data.
//         getI2CSensorInfo()      reutrns not only the polled data but internal statisics on the driver.
//
// See the file <RobotFirmware/Includes/I2CDrivers.h> for a definition of the various data structures used by the above functions.
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

intrinsic void getI2CPolledData(const tSensors nSensorIndex, byte *pBuffer, const int nNumbOfBytes)

                asm(opcdSystemFunctions, byte(sysFuncI2CSensorReply),
                                         variable(nSensorIndex),
                                         variableRefPointer(pBuffer),
                                         variable(nNumbOfBytes));

intrinsic void getI2CDriverStatistics(const tSensors nSensorIndex, TI2CStatistics& nBuffer)
                asm(opcdSystemFunctions, byte(sysFuncI2CGetSensorInfo),
                                         variable(nSensorIndex),
                                         variableRefPointer(nBuffer));

intrinsic TI2cDriverState getI2CDriverState(const tSensors nSensorIndex)
                asm(opcdSystemFunctions, byte(sysFuncI2CGetDriverState),
                                         variable(nSensorIndex),
                                         functionReturn);

intrinsic void clearI2CStatistics(const tSensors nSensorIndex)

                asm(opcdSystemFunctions, byte(sysFuncI2CClearSensorInfo),
                                         variable(nSensorIndex));

intrinsic void setI2CSensorConfig(const tSensors nSensorIndex, TI2CSensorSetupData& nBuffer)

                asm(opcdSystemFunctions, byte(sysFuncI2CSensorConfig),
                                         variable(nSensorIndex),
                                         variableRefPointer(nBuffer));

//
// Get statistics on NXT FTC MUX drivers.
//
// 'nMuxIndex' is 0..15 for four sensor ports with up to four MUXes on each port.
// 'nBuffer' struct variable contains information on the MUX driver -- number of resets, cycles and msg sends.
// It can be used to determine the "health" of a MUX.

typedef enum TSensorPortHub
{
	muxNone											= 0,

	muxHiTMotor									= 1,
	muxHiTServo									= 2,

	muxMatrixRoboticsController	= 3,

	muxHiTTouch									= 4,

	muxLEGO485Nxt								= 5,  // Special -- not implemented yet.
	kNumbOfMuxTypes,
} TSensorPortHub;


typedef struct
{
  uword            nUpdateResets;
  uword            nUpdateCycles;
  uword            nMsgSends;
  TSensorPortHub   nHubType;
} TSensorPortMuxStatistics;

intrinsic void getI2CMuxDriverStatistics(const int nMuxIndex, TSensorPortMuxStatistics& nBuffer)
                asm(opcdSystemFunctions, byte(sysFuncNxtMuxStatistic),
                                         variable(nMuxIndex),
                                         variableRefPointer(nBuffer));

#endif

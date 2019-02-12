#if !defined(_BaudRates__H_)
#define _BaudRates__H_

#ifdef __cplusplus
extern "C" {
#endif


	typedef enum TSerialPortMode
	{
		uartNotUsed						= 0,
		uartSystemCommPort		= 1,					// StandardROBOTC IDE System Messaging Port
		uartUserControl				= 2,						// Fully user controlled.

	#if defined(hasAlphaNumericLCDDisplayViaUART) || defined(_WINDOWS)
		uartVEXLCD						= 3,						// VEX 2x15 LCD. Manages LCD updates in background.
	#endif

	#if defined(bHasMultRobotNetworking) || defined(_WINDOWS)
		uartXBeeZigBee				= 4,						// UART used for XBee ZigBee networking
		uartXBeeWiFi					= 5,						// UART used for XBee WiFi networking
	#endif

	#if defined(hasCamera) || defined(_WINDOWS)
		uartCMUCAM						= 6,
	#endif

		uartSerialGPS					= 7,						// Connected to a serial GPS

	} TSerialPortMode;


typedef enum TBaudRate
{
	baudRate1200				= 0,
	baudRate2400				= 1,
	baudRate4800				= 2,
	baudRate9600				= 3,
	baudRate14400				= 4,
	baudRate19200				= 5,
	baudRate28800				= 6,
	baudRate38400				= 7,
	baudRate57600				= 8,
	baudRate76800				= 9,
	baudRate100000			= 10,
	baudRate115200			= 11,
	baudRate200000			= 12,
	baudRate230400			= 13,
	baudRate250000			= 14,
	baudRateUndefined
} TBaudRate;

#define kArduinoBaudRate baudRate200000


typedef enum TParityType
{
	parityNone,
	parityEven,
	parityOdd
} TParityType;

typedef enum TArduinoBootLoaderType
{
	bootloadArduinoBad,
	bootloadArduinoSmall,
	bootloadArduinoLarge,
	bootloadArduinoAVR109,
	bootloadAtmelSAMBA,
} TArduinoBootLoaderType;

#ifdef __cplusplus
};
#endif


#endif

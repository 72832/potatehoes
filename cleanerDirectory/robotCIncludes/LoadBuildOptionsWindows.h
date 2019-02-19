////////////////////////////////////////////////////////////////////////////////////////////////////
//
//																Load Build Options for Windows
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once
#include "LoadBuildOptionsClearFields.h"
#include "CompilerTypedefs.h"
#define		romtype																											// Simulator or _WINDOWS
#define		ramtype																											// Simulator or _WINDOWS
#define		ramfunc																											// Simulator or _WINDOWS
#define		neartype																										// Simulator or _WINDOWS
#define		no_init																											// Simulator or _WINDOWS
#ifndef __cplusplus
#define		inline																											// Simulator or _WINDOWS
#endif

#define	 bFirmwareStoresMotorType																			// Simulator
#define	 bFirmwareStoresMotorDriveSide																// Simulator
extern unsigned char bIncludeMotorTypeInUploadMessage();		 					// Simulator or _WINDOWS
#define	bMotorMirroring																								// Simulator
#define	bMotorsWithEncoders																						// Simulator
#define	bMotorsWithCurrentLimiting																		// Simulator
#define	bSmartMotorsWithEncoders()   1																// Simulator
#define	kSizeOfDefaultPidSettingsTable 10															// Simulator
#define	bAssociateMotorWithEncoderSensor															// Simulator
#define	bHasEncoderTimeStamp																					// Simulator
#define	bHasEncoderVelocity																						// Simulator
#define	bAssociateMotorWithIOPins 																		// Simulator or _WINDOWS
#define bSupportsSoftwareBasedMotorPWM										 						// Simulator or _WINDOWS
#define bSupportsSoftwareBasedServoPWM										 						// Simulator or _WINDOWS
#undef	bHasHardwareAutoDetectionMotors					 											// Simulator or _WINDOWS
#undef	bHasHardwareAutoDetectionSensors					 										// Simulator or _WINDOWS


#define	kNumbOfBreakpoints 16      // Must be the highest number available of any controller. Used by IDE
																		// Windows cannot support > 16. They take up 16 bits in 32-bit mask word!
#define	kSystemClockSpeed  16000000L																	// Simulator or _WINDOWS

// Conditional Compile Flags that have significant impact on features and size of firmware file

#define	hasSupportForMessagingWithRobotcIDE														// Simulator or _WINDOWS
#define	optionalExceptions 																						// Simulator or _WINDOWS
#define	optionalBreakpoints 																					// Simulator or _WINDOWS
#define	optionalDebugSupport																					// Simulator or _WINDOWS
#define	hasBulkUploadlupport																					// Simulator or _WINDOWS
#define	hasOptionalUploadSupport																			// Simulator or _WINDOWS
#define	hasDeviceUploadSupport																				// Simulator or _WINDOWS
#define	optionalTrinaryOpcodes																				// Simulator or _WINDOWS
#define	bHasOpcodeStateMachine																				// Simulator or _WINDOWS

#define	useFloats																											// Simulator or _WINDOWS
#define	useLongs																											// Simulator or _WINDOWS
#define	useStackVariables																							// Simulator or _WINDOWS

#define	hasJoystickDownloadMessageOpcodes															// Simulator or _WINDOWS
#define	hasJoystickUploadMessageOpcodes																// Simulator or _WINDOWS
#define	hasMemMoveSupport																							// Simulator or _WINDOWS
#define	hasMemCmpSupport																							// Simulator or _WINDOWS
#define	hasStringSupport																							// Simulator or _WINDOWS
#define	has_sscanf_Support																						// Simulator or _WINDOWS
#define	hasTranscendentalSupport																			// Simulator or _WINDOWS

#define	bHasSoundSupport 1																						// Simulator or _WINDOWS
#define	bHasSoundVolume 1																							// Simulator or _WINDOWS
#define	bHasPlayMusicalNotes																					// Simulator or _WINDOWS
#define	bHasSoundsInRomChip																						// Simulator or _WINDOWS
#define	bHasSoundDriverOnOff 0																				// Simulator or _WINDOWS
#define	bHasSoundFiles 1																							// Simulator or _WINDOWS
#define	kSoundQueueSize 10																						// Simulator or _WINDOWS

#undef	hasCamera																											// Simulator or _WINDOWS
#undef	hasHarvardArchitecture																				// Simulator or _WINDOWS
#define	kMaxTimeBetweenMessageChars 25																// Simulator or _WINDOWS
#define	hasDebugStreamSupport																					// Simulator or _WINDOWS
#define	kDebugStreamDefaultBufferSize 20000														// Simulator or _WINDOWS
#define	kRamBaseAddr ((ubyte *) &VM_RAM.globalVariablesXX[0])					// Simulator or _WINDOWS
#define	bHasDatalog																										// Simulator or _WINDOWS
#define	kDatalogDefaultBufferSize 4000																// Simulator or _WINDOWS
#if defined(Simulator)
	#undef	bHasDatalogSave																							// Simulator
#else
	#define	 bHasDatalogSave																						// _WINDOWS
#endif
#define	bHasMessageStatistics																					// Simulator or _WINDOWS
#define	TETRIX_CAPABLE																								// Simulator or _WINDOWS

#define	hasAlphaNumericLCDDisplayViaUART															// Simulator or _WINDOWS
#define	bHasLCDGraphicsDisplay																				// Simulator or _WINDOWS
#define	hasLCDMenuFunctions																						// Simulator or _WINDOWS
extern bool bPixelByteContainsColumnData(void);												// Simulator of _WINDOWS
extern bool bPixelRowsLSBFirst(void);																	// Simulator of _WINDOWS
#define	hasLCDBacklight																								// Simulator or _WINDOWS
#define	kScreenWidth								  192															// Simulator or _WINDOWS
#define	kScreenHeight									192															// Simulator or _WINDOWS
extern int kFirstVisibleScreenRow(void);															// Simulator or _WINDOWS
#define	hasLCDButtons																									// Simulator or _WINDOWS
#define	hasStatusLEDs																									// Simulator or _WINDOWS
#define	bHasLCDContrast																								// Simulator or _WINDOWS
#define	bHasRemoteButtons																							// Simulator or _WINDOWS

#define	bUseLookupBreakpointTableInRAM																// Simulator or _WINDOWS
#define	bHasFloatDuringInactivePWM																		// Simulator or _WINDOWS
#define	nnRobotType	getEmulatorRobotType()														// Simulator or _WINDOWS
extern int getEmulatorRobotType();
#define	bHasServoSpecificSourceTypes																	// Simulator or _WINDOWS
#define	bHasSourceTypeSensorRawValue																	// Simulator or _WINDOWS
#define	bHasSourceTypeSensorMode																			// Simulator or _WINDOWS
#define	nUploadSensorValueType sensorUploadSizeShortX									// Simulator or _WINDOWS
#define	hasWatchDogTimerFunctions																			// Simulator or _WINDOWS
#define	bHasClearFlashOpcodes																					// Simulator or _WINDOWS
#define	hasSpareOption9																								// Simulator or _WINDOWS

#define	kSizeOfTLargeValue											 4										// Simulator or _WINDOWS

#define	bHasReadBatteryVoltage																				// Simulator or _WINDOWS
#define	bHasStandardBatteryMonitoring																	// Simulator or _WINDOWS
#define	bHasSoftwareControlledPowerOnOff															// Simulator or _WINDOWS
#define	bHasSoftwareControlledPowerOnOffTiming												// Simulator or _WINDOWS
#undef	UNUSED__23                     																// Simulator or _WINDOWS
#define bHasShiftOutOpcode																						// Simulator or _WINDOWS
#define bHasRandomFunctions																						// Simulator or _WINDOWS
#define bHasBulkClearSensorFunction																		// Simulator or _WINDOWS
#define bHasBulkClearMotorFunction																		// Simulator or _WINDOWS

// Flags to control level of sensor support

extern void sensorHandlerWorkBetweenOpcodes(void);										// Simulator or _WINDOWS
#undef  bSoftwareMustStartADCConversionCycle													// Simulator or _WINDOWS
#undef 	useSimpleADCArrayLookup																				// Simulator or _WINDOWS
#define	includeSonarSensorSupportTwoPins															// Simulator  -- For MaxBotic SRP-04 ultrasonic and similar
#define	includeSonarSensorSupportOnePin																// Simulator  -- For Parallax Ping ultrasonic and similar
#define	includeEncoderSensorSupport																		// Simulator  -- For minimal f/w size, this can be discarded
#define	includeGyroAndAccelSupport																		// Simulator  -- For minimal f/w size, this can be discarded
#define	nGyroJitterRange 1																						// Simulator  -- For minimal f/w size, this can be discarded
#define	bHasSensorBiasSettings																				// Simulator or _WINDOWS
#define	bHasI2CPort																										// Simulator or _WINDOWS
#define	bHasDaisyChainedI2C																						// Simulator or _WINDOWS
#define	nFirstAutoDaisyChainI2CAddress	0x60													// Simulator or _WINDOWS
#define	bHasGetSensorRawADValue																				// Simulator or _WINDOWS
#define	bHasFastSonarSensor																						// Simulator or _WINDOWS

#define	kNumbAnalogSensors 	 16																				// Simulator or _WINDOWS
#define	kNumbDigitalSensors  70																				// Simulator or _WINDOWS
#define	kNumbSensorsOnI2CPorts  12																		// Simulator or _WINDOWS
#define	kMaxNumbofComplicatedSensors  (kNumbAnalogSensors+kNumbDigitalSensors + kNumbSensorsOnI2CPorts)	// Simulator or _WINDOWS
#define	kNumbOfRealSensors            (kNumbAnalogSensors+kNumbDigitalSensors+kNumbSensorsOnI2CPorts)// Max value for any robot or will crash IDE // Simulator or _WINDOWS

#define	bHasSystemTaskForUserProgramComplete													// Simulator or _WINDOWS
#define	bHasSemaphoreSupport																					// Simulator or _WINDOWS
#define	kMaxBytesPerDataBytesUpload   (kSizeOfXmitUartBuffer - 2)			// Simulator or _WINDOWS
#define	kReceivedOpcodeSize           (kSizeOfRcvUartBuffer - 2)			// Simulator or _WINDOWS
#undef	bGetClearVariablesOnPgmStart																	// Simulator or _WINDOWS
#define	bHasPropertyOpcodes																						// Simulator or _WINDOWS
#define	hasDriveTrainConfiguration																		// Simulator or _WINDOWS
#define	hasJoystickAxisConfiguration																	// Simulator or _WINDOWS
#define	hasMotorOpcodes																								// Simulator or _WINDOWS
#undef	platformDriversBufferOutgoingSerialData												// Simulator or _WINDOWS
#undef	platformDriversBufferIncomingSerialData												// Simulator or _WINDOWS
#define	DelayTimeForStartupProgram()   0															// Simulator or _WINDOWS
#undef	spareOption16																									// Simulator or _WINDOWS
#undef	spareOption17																									// Simulator or _WINDOWS
#undef	spareOption18																									// Simulator or _WINDOWS
#undef	spareOption19																									// Simulator or _WINDOWS
#undef	spareOption20																									// Simulator or _WINDOWS
#undef	spareOption21																									// Simulator or _WINDOWS
#undef	spareOption22																									// Simulator or _WINDOWS
#undef	spareOption23																									// Simulator or _WINDOWS
#undef	spareOption24																									// Simulator or _WINDOWS
#define	bUserMessagesAllowed																					// Simulator or _WINDOWS

#define	bHasMultRobotNetworking																				// Simulator or _WINDOWS
#define	bSystemUARTSupportsZigBee																			// Simulator or _WINDOWS
#undef	bHasZigBeeFeatureSpare0																				// Simulator or _WINDOWS
#undef	bHasZigBeeFeatureSpare1																				// Simulator or _WINDOWS
#undef	bHasZigBeeFeatureSpare2																				// Simulator or _WINDOWS
#define kNumbOfZigBeeMessageBuffers		0																// Simulator or _WINDOWS

#define	kNumbOfTasks									(20)														// Simulator or _WINDOWS
#define	kNumbOfGlobalVariablesInBytes	(15000)													// Simulator or _WINDOWS
#define	kSpareA01											(16)														// Simulator or _WINDOWS
#define	kNumbOfRealMotors             (4 + 4 * 4 * 2)									// Simulator or _WINDOWS: Max value for any robot or will crash IDE // Simulator or _WINDOWS
#define	kNumbOfRealServos             ((unsigned char) 4 * (4 * 8))		// Simulator or _WINDOWS: Max value for any robot or will crash IDE // Simulator or _WINDOWS
#define	kNumbOfVirtualSensors         ((unsigned char) 8)							// Simulator or _WINDOWS
#define	kNumbOfVirtualMotors          ((unsigned char) 0)							// Simulator or _WINDOWS

#undef	bSupportsInvokeBootloaderOpcode																// Simulator or _WINDOWS
#define	bSupportsFlashWriteOpcodes							(1)										// Simulator or _WINDOWS
#define	bSpareFlagABC	(0)													 										// Simulator or _WINDOWS
#define	bSupportsRAMWriteOpcodes																			// Simulator or _WINDOWS

#define	bHasFlashFileSystem																						// Simulator or _WINDOWS
#undef	bHasFlashFileSystemDelete																			// Simulator or _WINDOWS
#undef	bUsesFlashStorage																							// Simulator or _WINDOWS
#define	kFlashSectorSize							4096														// Simulator or _WINDOWS
#define	kMaxNumbofFlashFiles					60															// Simulator or _WINDOWS
#define	kVTOCItemSize									4																// Simulator or _WINDOWS
#define	kStoreRobotNameInPersistentData 0    													// Simulator or _WINDOWS
#define	bStoreStartupUARTTypesInPersistentData() (1)									// Simulator or _WINDOWS
#define	kSizeOfFlashMemoryPtr					4																// Simulator or _WINDOWS
#define	kSizeOfRAMMemoryPtrInHW				4																// Simulator or _WINDOWS

#if defined(Simulator)
	#undef	bHasEvents																									// Simulator does not support events
#else
	#define	 bHasEvents																									// _WINDOWS
#endif
#define	kTotalCallSubStackSize				60*kNumbOfTasks									// Simulator or _WINDOWS
#define	kTotalVariablesStackSize			256*kNumbOfTasks								// Simulator or _WINDOWS

#define	kConstantPoolSize							3200 														// Simulator or _WINDOWS -- Maximum size in bytes of constant pool. None if "#undef".
#define	kStartOfFileSystem 						getStartOfFileSystem()					// Simulator or _WINDOWS -- Byte (and not word) address
#undef	kStartOfBootloader																						// Simulator or _WINDOWS
#define	bHasDivideByZeroException							 												// Simulator or _WINDOWS
#define	loadBuildOptionSpare1      							 											// Simulator or _WINDOWS
#define	loadBuildOptionSpare2      							 											// Simulator or _WINDOWS
#define	loadBuildOptionSpare3      							 											// Simulator or _WINDOWS
#define	loadBuildOptionSpare4      							 											// Simulator or _WINDOWS
#define	loadBuildOptionSpare5      							 											// Simulator or _WINDOWS
#define	loadBuildOptionSpare6      							 											// Simulator or _WINDOWS
#define	loadBuildOptionSpare7      							 											// Simulator or _WINDOWS
#define	loadBuildOptionSpare8      							 											// Simulator or _WINDOWS
#define	loadBuildOptionSpare9      							 											// Simulator or _WINDOWS
#define	loadBuildOptionSpare10      							 										// Simulator or _WINDOWS
#define	loadBuildOptionSpare11      							 										// Simulator or _WINDOWS
#define	loadBuildOptionSpare12      							 										// Simulator or _WINDOWS
#define	loadBuildOptionSpare13      							 										// Simulator or _WINDOWS
#define	loadBuildOptionSpare14      							 										// Simulator or _WINDOWS
#define	loadBuildOptionSpare15      							 										// Simulator or _WINDOWS

#define	SystemUartPortCanBeReconfigured																// Simulator or _WINDOWS
#define	hasUserSerialPorts																						// Simulator or _WINDOWS
extern int	getNumbOfSerialPorts(void);																// Simulator or _WINDOWS
#define	kSizeOfXmitUartBuffer 				10010														// Simulator or _WINDOWS
#define	kSizeOfRcvUartBuffer	 				10010														// Simulator or _WINDOWS


#undef	ARMCPU																												// Simulator or _WINDOWS
#undef	AVRCPU																												// Simulator or _WINDOWS
#undef	STM8CPU																												// Simulator or _WINDOWS
#undef	Spare1CPU																											// Simulator or _WINDOWS
#undef	Spare2CPU																											// Simulator or _WINDOWS
#undef	Spare3CPU																											// Simulator or _WINDOWS
#define	useParmForCurrOpPtr																						// Simulator or _WINDOWS
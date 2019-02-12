////////////////////////////////////////////////////////////////////////////////////////////////////
//
//																Load Build Options for VEX Cortex
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once
#include "LoadBuildOptionsClearFields.h"
#define	romtype																												// VEX_CORTEX
#define	ramtype																												// VEX_CORTEX
#define	neartype																											// VEX_CORTEX
#define	ramfunc __ramfunc																							// VEX_CORTEX
#define	no_init __no_init																							// VEX_CORTEX
//#undef inline																												// VEX_CORTEX

#define	bFirmwareStoresMotorType																			// VEX_CORTEX
#define	bFirmwareStoresMotorDriveSide																	// VEX_CORTEX
#define	bIncludeMotorTypeInUploadMessage() true		 										// VEX_CORTEX
#define	bMotorMirroring																								// VEX_CORTEX
#define	bMotorsWithEncoders																						// VEX_CORTEX
#undef	bMotorsWithCurrentLimiting																		// VEX_CORTEX
#define	bSmartMotorsWithEncoders()   1																// VEX_CORTEX
#define	kSizeOfDefaultPidSettingsTable 10															// VEX_CORTEX
#define	bAssociateMotorWithEncoderSensor															// VEX_CORTEX
#define	bHasEncoderTimeStamp																					// VEX_CORTEX
#define	bHasEncoderVelocity																						// VEX_CORTEX
#undef	bAssociateMotorWithIOPins 																		// VEX_CORTEX
#define	bIncludeMotorEncoderInUploadMessage() true			 							// VEX_CORTEX
#undef	bSupportsSoftwareBasedMotorPWM						 										// VEX_CORTEX
#undef	bSupportsSoftwareBasedServoPWM						 										// VEX_CORTEX
#undef	bHasHardwareAutoDetectionMotors					 											// VEX_CORTEX
#undef	bHasHardwareAutoDetectionSensors					 										// VEX_CORTEX

#define	kNumbOfBreakpoints 16																					// VEX_CORTEX
#undef	kSystemClockSpeed																							// VEX_CORTEX

// Conditional Compile Flags that have significant impact on features and size of firmware file

#define	hasSupportForMessagingWithRobotcIDE														// VEX_CORTEX
#define	optionalExceptions 																						// VEX_CORTEX
#define	optionalBreakpoints 																					// VEX_CORTEX
#define	optionalDebugSupport																					// VEX_CORTEX
#define	hasBulkUploadlupport																					// VEX_CORTEX
#define	hasOptionalUploadSupport																			// VEX_CORTEX
#define	hasDeviceUploadSupport																				// VEX_CORTEX
#define	optionalTrinaryOpcodes																				// VEX_CORTEX
#undef	bHasOpcodeStateMachine																				// VEX_CORTEX

#define	useFloats																											// VEX_CORTEX
#define	useLongs																											// VEX_CORTEX
#define	useStackVariables																							// VEX_CORTEX

#define	hasJoystickDownloadMessageOpcodes															// VEX_CORTEX
#define	hasJoystickUploadMessageOpcodes																// VEX_CORTEX
#define	hasMemMoveSupport																							// VEX_CORTEX
#define	hasMemCmpSupport																							// VEX_CORTEX
#define	hasStringSupport																							// VEX_CORTEX
#define	has_sscanf_Support																						// VEX_CORTEX
#define	hasTranscendentalSupport																			// VEX_CORTEX

#define	bHasSoundSupport 1																						// VEX_CORTEX
#define	bHasSoundVolume 1																							// VEX_CORTEX
#undef	bHasPlayMusicalNotes																					// VEX_CORTEX
#undef	bHasSoundsInRomChip																						// VEX_CORTEX
#define	bHasSoundDriverOnOff 0																				// VEX_CORTEX
#define	bHasSoundFiles	1																							// VEX_CORTEX
#define	kSoundQueueSize 10																						// VEX_CORTEX

#undef	hasCamera																											// VEX_CORTEX
#undef	hasHarvardArchitecture 																				// VEX_CORTEX
#define	kMaxTimeBetweenMessageChars 275																// VEX_CORTEX
#define	hasDebugStreamSupport																					// VEX_CORTEX
#define	kDebugStreamDefaultBufferSize 4000														// VEX_CORTEX
#define	kRamBaseAddr 0x20000000																				// VEX_CORTEX
#define	bHasDatalog																										// VEX_CORTEX
#define	kDatalogDefaultBufferSize 4000																// VEX_CORTEX
#undef	bHasDatalogSave																								// VEX_CORTEX
#define	bHasMessageStatistics																					// VEX_CORTEX
#undef	TETRIX_CAPABLE																								// VEX_CORTEX

#define	hasAlphaNumericLCDDisplayViaUART															// VEX_CORTEX
#undef	bHasLCDGraphicsDisplay																				// VEX_CORTEX
#undef	hasLCDMenuFunctions																						// VEX_CORTEX
#define bPixelByteContainsColumnData() true														// VEX_CORTEX
#define bPixelRowsLSBFirst()					 true														// VEX_CORTEX
#define	hasLCDBacklight																								// VEX_CORTEX
#undef	kScreenWidth																									// VEX_CORTEX
#undef	kScreenHeight																									// VEX_CORTEX
#undef	kFirstVisibleScreenRow																				// VEX_CORTEX
#define	hasLCDButtons																									// VEX_CORTEX
#undef	hasStatusLEDs																									// VEX_CORTEX
#undef	bHasLCDContrast																								// VEX_CORTEX
#define	bHasRemoteButtons																							// VEX_CORTEX

#undef	bUseLookupBreakpointTableInRAM																// VEX_CORTEX
#undef	bHasFloatDuringInactivePWM																		// VEX_CORTEX
#define	nnRobotType	rbtTypeVEX2_0																			// VEX_CORTEX
#undef	bHasServoSpecificSourceTypes																	// VEX_CORTEX
#undef	bHasSourceTypeSensorRawValue																	// VEX_CORTEX
#undef	bHasSourceTypeSensorMode																			// VEX_CORTEX
#define	nUploadSensorValueType sensorUploadSizeShortX									// VEX_CORTEX
#define	hasWatchDogTimerFunctions																			// VEX_CORTEX
#define	bHasClearFlashOpcodes																					// VEX_CORTEX
#undef	hasSpareOption9																								// VEX_CORTEX

#define	kSizeOfTLargeValue				 4																	// VEX_CORTEX

#define	bHasReadBatteryVoltage																				// VEX_CORTEX
#define	bHasStandardBatteryMonitoring																	// VEX_CORTEX
#undef	bHasSoftwareControlledPowerOnOff															// VEX_CORTEX
#undef	bHasSoftwareControlledPowerOnOffTiming												// VEX_CORTEX
#undef	UNUSED__23                     																// VEX_CORTEX
#define	bHasShiftOutOpcode																						// VEX_CORTEX
#define	bHasRandomFunctions																						// VEX_CORTEX
#define	bHasBulkClearSensorFunction																		// VEX_CORTEX
#define	bHasBulkClearMotorFunction																		// VEX_CORTEX

// Flags to control level of sensor support

#define	sensorHandlerWorkBetweenOpcodes()															// VEX_CORTEX
#undef  bSoftwareMustStartADCConversionCycle													// VEX_CORTEX
#undef	useSimpleADCArrayLookup																				// VEX_CORTEX
#define	includeSonarSensorSupportTwoPins															// VEX_CORTEX  -- For MaxBotic SRP-04 ultrasonic and similar
#define	includeSonarSensorSupportOnePin																// VEX_CORTEX  -- For Parallax Ping ultrasonic and similar
#define	includeEncoderSensorSupport																		// VEX_CORTEX  -- For minimal f/w size, this can be discarded
#define	includeGyroAndAccelSupport																		// VEX_CORTEX  -- For minimal f/w size, this can be discarded
#define	nGyroJitterRange 4																						// VEX_CORTEX
#define	bHasSensorBiasSettings																				// VEX_CORTEX
#define	bHasI2CPort																										// VEX_CORTEX
#define	bHasDaisyChainedI2C																						// VEX_CORTEX
#define	nFirstAutoDaisyChainI2CAddress  0x60													// VEX_CORTEX  -- First address for automatically assigned I2C addresses
#define	bHasGetSensorRawADValue																				// VEX_CORTEX
#define	bHasFastSonarSensor																						// VEX_CORTEX

#define	kNumbAnalogSensors 	 8																				// VEX_CORTEX
#define	kNumbDigitalSensors 12																				// VEX_CORTEX
#define	kNumbSensorsOnI2CPorts 8																			// VEX_CORTEX
#define	kMaxNumbofComplicatedSensors  12															// VEX_CORTEX
#define	kNumbOfRealSensors   (kNumbAnalogSensors+kNumbDigitalSensors+kNumbSensorsOnI2CPorts) // VEX_CORTEX

#define	bHasSystemTaskForUserProgramComplete													// VEX_CORTEX
#define	bHasSemaphoreSupport																					// VEX_CORTEX
#define	kMaxBytesPerDataBytesUpload   251															// VEX_CORTEX
#define	kReceivedOpcodeSize           262															// VEX_CORTEX
#undef	bGetClearVariablesOnPgmStart																	// VEX_CORTEX
#define	bHasPropertyOpcodes																						// VEX_CORTEX
#define	hasDriveTrainConfiguration																		// VEX_CORTEX
#define	hasJoystickAxisConfiguration																	// VEX_CORTEX
#define	hasMotorOpcodes																								// VEX_CORTEX
#undef	platformDriversBufferOutgoingSerialData												// VEX_CORTEX
#undef	platformDriversBufferIncomingSerialData												// VEX_CORTEX
#define	DelayTimeForStartupProgram()   0															// VEX_CORTEX
#undef	spareOption16																									// VEX_CORTEX
#undef	spareOption17																									// VEX_CORTEX
#undef	spareOption18																									// VEX_CORTEX
#undef	spareOption19																									// VEX_CORTEX
#undef	spareOption20																									// VEX_CORTEX
#undef	spareOption21																									// VEX_CORTEX
#undef	spareOption22																									// VEX_CORTEX
#undef	spareOption23																									// VEX_CORTEX
#undef	spareOption24																									// VEX_CORTEX
#undef	bUserMessagesAllowed																					// VEX_CORTEX

#define	bHasMultRobotNetworking																				// VEX_CORTEX
#undef 	bSystemUARTSupportsZigBee																			// VEX_CORTEX
#undef	bHasZigBeeFeatureSpare0																				// VEX_CORTEX
#undef	bHasZigBeeFeatureSpare1																				// VEX_CORTEX
#undef	bHasZigBeeFeatureSpare2																				// VEX_CORTEX
#define kNumbOfZigBeeMessageBuffers		0																// VEX_CORTEX

#define	kNumbOfTasks									(20)														// VEX_CORTEX
#define	kNumbOfGlobalVariablesInBytes	(15000)													// VEX_CORTEX
#define	kSpareA01											(0)															// VEX_CORTEX
#define	kNumbOfRealMotors             (10)														// VEX_CORTEX
#define	kNumbOfRealServos             ((ubyte) 10)										// VEX_CORTEX
#define	kNumbOfVirtualSensors         ((ubyte) 0)											// VEX_CORTEX
#define	kNumbOfVirtualMotors          ((ubyte) 0)											// VEX_CORTEX

#undef	bSupportsInvokeBootloaderOpcode																// VEX_CORTEX
#define	bSupportsFlashWriteOpcodes              (1) 									// VEX_CORTEX
#define	bSpareFlagABC	(0)													 										// VEX_CORTEX
#define	bSupportsRAMWriteOpcodes																			// VEX_CORTEX

#define	bHasFlashFileSystem																						// VEX_CORTEX
#undef	bHasFlashFileSystemDelete																			// VEX_CORTEX
#define	bUsesFlashStorage																							// VEX_CORTEX
#define	kFlashSectorSize							2048														// VEX_CORTEX
#define	kMaxNumbofFlashFiles					60															// VEX_CORTEX
#define	kVTOCItemSize									4																// VEX_CORTEX
#define	kStoreRobotNameInPersistentData 1   														// VEX_CORTEX
#define	bStoreStartupUARTTypesInPersistentData() (1)									// VEX_CORTEX
#define	kSizeOfFlashMemoryPtr					4																// VEX_CORTEX
#define	kSizeOfRAMMemoryPtrInHW				4																// VEX_CORTEX

#undef	bHasEvents																										// VEX_CORTEX
#define	kTotalCallSubStackSize				60*kNumbOfTasks									// VEX_CORTEX
#define	kTotalVariablesStackSize			256*kNumbOfTasks								// VEX_CORTEX
#define	kConstantPoolSize							3200														// VEX_CORTEX -- Maximum size in bytes of constant pool. None if "#undef".
#define	kStartOfFileSystem 						0x08018000											// VEX_CORTEX -- Byte (and not word) address
#undef	kStartOfBootloader																						// VEX_CORTEX
#define	bHasDivideByZeroException							 												// VEX_CORTEX
#define	loadBuildOptionSpare1      							 											// VEX_CORTEX
#define	loadBuildOptionSpare2      							 											// VEX_CORTEX
#define	loadBuildOptionSpare3      							 											// VEX_CORTEX
#define	loadBuildOptionSpare4      							 											// VEX_CORTEX
#define	loadBuildOptionSpare5      							 											// VEX_CORTEX
#define	loadBuildOptionSpare6      							 											// VEX_CORTEX
#define	loadBuildOptionSpare7      							 											// VEX_CORTEX
#define	loadBuildOptionSpare8      							 											// VEX_CORTEX
#define	loadBuildOptionSpare9      							 											// VEX_CORTEX
#define	loadBuildOptionSpare10      							 										// VEX_CORTEX
#define	loadBuildOptionSpare11      							 										// VEX_CORTEX
#define	loadBuildOptionSpare12      							 										// VEX_CORTEX
#define	loadBuildOptionSpare13      							 										// VEX_CORTEX
#define	loadBuildOptionSpare14      							 										// VEX_CORTEX
#define	loadBuildOptionSpare15      							 										// VEX_CORTEX

#undef	SystemUartPortCanBeReconfigured																// VEX_CORTEX
#define	hasUserSerialPorts																						// VEX_CORTEX
#define	getNumbOfSerialPorts()			3																	// VEX_CORTEX
#define	kSizeOfXmitUartBuffer 		255	// to fit index in single byte	// VEX_CORTEX
#define	kSizeOfRcvUartBuffer	 		 50																	// VEX_CORTEX

#define	ARMCPU																												// VEX_CORTEX
#undef	AVRCPU																												// VEX_CORTEX
#undef	STM8CPU																												// VEX_CORTEX
#undef	Spare1CPU																											// VEX_CORTEX
#undef	Spare2CPU																											// VEX_CORTEX
#undef	Spare3CPU																											// VEX_CORTEX
#define	useParmForCurrOpPtr																						// VEX_CORTEX

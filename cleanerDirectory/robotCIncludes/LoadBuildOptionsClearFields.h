////////////////////////////////////////////////////////////////////////////////////////////////////
//
//												Load Build Options for Clearing all Fields
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef LoadBuildOptionsClearFields__H_
#define	LoadBuildOptionsClearFields__H_
//
// List of all Toggles
//

#undef	romtype																							// Undefine load build options
#undef	ramtype																							// Undefine load build options
#undef	neartype																						// Undefine load build options
#undef	no_init																							// Undefine load build options

// Conditional Compile Flags that have significant impact on features and size of firmware file

#undef	hasSupportForMessagingWithRobotcIDE									// Undefine load build options
#undef	optionalExceptions																	// Undefine load build options
#undef	optionalBreakpoints																	// Undefine load build options
#undef	optionalDebugSupport																// Undefine load build options
#define	optionalSwitchOpcodes																// Undefine load build options
#undef	hasOptionalUploadSupport														// Undefine load build options
#undef	hasBulkUploadlupport																// Undefine load build options
#undef	hasDeviceUploadSupport															// Undefine load build options
#undef	optionalTrinaryOpcodes															// Undefine load build options
#undef	bHasOpcodeStateMachine															// Undefine load build options

#undef	useFloats																						// Undefine load build options
#undef	useLongs																						// Undefine load build options
#undef  useStackVariables																		// Undefine load build options

#undef	kSystemClockSpeed																		// Undefine load build options

#undef	bFirmwareStoresMotorType														// Undefine load build options
#undef	bFirmwareStoresMotorDriveSide												// Undefine load build options
#undef	bIncludeMotorTypeInUploadMessage										// Undefine load build options
#undef	bMotorMirroring																			// Undefine load build options
#undef	bMotorsWithEncoders																	// Undefine load build options
#undef	bMotorsWithCurrentLimiting													// Undefine load build options
#undef	bSmartMotorsWithEncoders														// Undefine load build options
#undef	kSizeOfDefaultPidSettingsTable											// Undefine load build options
#undef	bAssociateMotorWithEncoderSensor										// Undefine load build options
#undef	bHasEncoderTimeStamp																// Undefine load build options
#undef	bHasEncoderVelocity																	// Undefine load build options
#undef	bAssociateMotorWithIOPins														// Undefine load build options
#undef	bIncludeMotorEncoderInUploadMessage									// Undefine load build options
#undef	bSupportsSoftwareBasedMotorPWM											// Undefine load build options
#undef	bSupportsSoftwareBasedServoPWM											// Undefine load build options
#undef	bHasHardwareAutoDetectionMotors					 						// Undefine load build options
#undef	bHasHardwareAutoDetectionSensors					 					// Undefine load build options

#undef	bHasSoundSupport																		// Undefine load build options
#undef	bHasSoundVolume																			// Undefine load build options
#undef	bHasPlayMusicalNotes																// Undefine load build options
#undef	bHasSoundsInRomChip																	// Undefine load build options
#undef	bHasSoundDriverOnOff																// Undefine load build options
#undef	bHasSoundFiles																			// Undefine load build options

#undef	bHasZigBeeFeatureSpare0															// Undefine load build options
#undef	bHasZigBeeFeatureSpare1															// Undefine load build options
#undef	bHasZigBeeFeatureSpare2															// Undefine load build options
#undef  kNumbOfZigBeeMessageBuffers													// Undefine load build options

#undef		kNumbOfTasks																			// Undefine load build options
#undef		kNumbOfGlobalVariablesInBytes											// Undefine load build options
#undef		kNumbOfPrograms																		// Undefine load build options
#undef		kSpareA01																					// Undefine load build options		
#undef		kNumbOfRealMotors																	// Undefine load build options
#undef		kNumbOfRealServos																	// Undefine load build options
#undef		kNumbOfVirtualSensors															// Undefine load build options
#undef		kNumbOfVirtualMotors															// Undefine load build options

#undef	bSupportsInvokeBootloaderOpcode											// Undefine load build options
#undef	bSupportsFlashWriteOpcodes													// Undefine load build options
#undef	bSpareFlagABC																				// Undefine load build options
#undef	bSupportsRAMWriteOpcodes														// Undefine load build options

#undef	bHasFlashFileSystem																	// Undefine load build options
#undef	bHasFlashFileSystemDelete														// Undefine load build options
#undef	kMaxNumbofFlashFiles																// Undefine load build options
#undef	kMaxNumbofFlashFiles																// Undefine load build options
#undef	kVTOCItemSize																				// Undefine load build options
#undef	StoreRobotNameInPersistentData											// Undefine load build options
#undef	bStoreStartupUARTTypesInPersistentData							// Undefine load build options
#undef	kSizeOfFlashMemoryPtr																// Undefine load build options
#undef	kSizeOfRAMMemoryPtrInHW															// Undefine load build options

#define	bDebugAddressAlignment (0)													// Undefine load build options
#undef	hasCamera																						// Undefine load build options
#undef	hasJoystickDownloadMessageOpcodes										// Undefine load build options
#undef	hasJoystickUploadMessageOpcodes											// Undefine load build options
#undef	hasHarvardArchitecture															// Undefine load build options
#undef	hasMemMoveSupport																		// Undefine load build options
#undef	hasMemCmpSupport																		// Undefine load build options
#undef	hasStringSupport																		// Undefine load build options
#undef	has_sscanf_Support																	// Undefine load build options
#undef	hasDebugStreamSupport																// For minimal f/w size, this can be discarded
#undef	kDebugStreamDefaultBufferSize												// Undefine load build options
#undef	hasTranscendentalSupport														// Undefine load build options
#undef	bHasDatalog																					// Undefine load build options
#undef	kDatalogDefaultBufferSize														// Undefine load build options
#undef	bHasDatalogSave																			// Undefine load build options
#undef	bHasMessageStatistics																// Undefine load build options
#undef	TETRIX_CAPABLE																			// Undefine load build options

#undef	hasAlphaNumericLCDDisplayViaUART										// Undefine load build options
#undef	bHasLCDGraphicsDisplay															// Undefine load build options
#undef	hasLCDMenuFunctions																	// Undefine load build options
#undef	bPixelByteContainsColumnData												// Undefine load build options
#undef	hasLCDBacklight																			// Undefine load build options
#undef	kScreenWidth																				// Undefine load build options
#undef	kScreenHeight																				// Undefine load build options
#undef	kFirstVisibleScreenRow															// Undefine load build options
#undef	hasLCDButtons																				// Undefine load build options
#undef	hasStatusLEDs																				// Undefine load build options
#undef	bHasLCDContrast																			// Undefine load build options
#undef	bHasRemoteButtons																		// Undefine load build options

#undef	bUseLookupBreakpointTableInRAM											// Undefine load build options
#undef	bHasFloatDuringInactivePWM													// Undefine load build options
#undef	nnRobotType																					// Undefine load build options
#undef	bHasServoSpecificSourceTypes												// Undefine load build options
#undef	bHasSourceTypeSensorRawValue												// Undefine load build options
#undef	bHasSourceTypeSensorMode														// Undefine load build options
#undef	nUploadSensorValueType															// Undefine load build options
#undef	hasWatchDogTimerFunctions														// Undefine load build options
#undef	bHasClearFlashOpcodes																// Undefine load build options
#undef	hasSpareOption9																			// Undefine load build options

#undef	kSizeOfTLargeValue																	// Undefine load build options

#undef	bHasReadBatteryVoltage															// Undefine load build options
#undef	bHasStandardBatteryMonitoring												// Undefine load build options
#undef	bHasSoftwareControlledPowerOnOff										// Undefine load build options
#undef	bHasSoftwareControlledPowerOnOffTiming							// Undefine load build options
#undef	UNUSED__23                     											// Undefine load build options
#undef	bHasShiftOutOpcode																	// Undefine load build options
#undef  bHasRandomFunctions																	// Undefine load build options
#undef  bHasBulkClearSensorFunction
#undef  bHasBulkClearMotorFunction

// Flags to control level of sensor support

#undef	sensorHandlerWorkBetweenOpcodes											// Undefine load build options
#undef	bSoftwareMustStartADCConversionCycle								// Undefine load build options
#undef	useSimpleADCArrayLookup															// Undefine load build options
#undef	includeSonarSensorSupportTwoPins										// For MaxBotic SRP-04 ultrasonic and similar
#undef	includeSonarSensorSupportOnePin											// For Parallax Ping ultrasonic and similar
#undef	includeEncoderSensorSupport													// For minimal f/w size, this can be discarded
#undef	includeGyroAndAccelSupport													// For minimal f/w size, this can be discarded
#undef	nGyroJitterRange																		// Undefine load build options
#undef	bHasSensorBiasSettings															// For minimal f/w size, this can be discarded
#undef  bHasI2CPort																					// Undefine load build options
#undef  bHasDaisyChainedI2C																	// Undefine load build options
#undef  nFirstAutoDaisyChainI2CAddress											// Undefine load build options
#undef	bHasGetSensorRawADValue															// For minimal f/w size, this can be discarded
#undef	bHasFastSonarSensor																	// Undefine load build options

#undef	bHasSystemTaskForUserProgramComplete								// Flags a "system task" event when user program has finised
#undef	bHasSemaphoreSupport																// Undefine load build options
#undef	kReceivedOpcodeSize																	// Undefine load build options
#undef	bGetClearVariablesOnPgmStart												// Undefine load build options
#undef	bHasPropertyOpcodes																	// Undefine load build options
#undef	hasDriveTrainConfiguration													// Undefine load build options
#undef	hasJoystickAxisConfiguration												// Undefine load build options
#undef	hasMotorOpcodes																			// Undefine load build options
#undef	platformDriversBufferOutgoingSerialData							// Undefine load build options
#undef	platformDriversBufferIncomingSerialData							// Undefine load build options
#undef	DelayTimeForStartupProgram													// Undefine load build options
#undef	spareOption16																				// Undefine load build options
#undef	spareOption17																				// Undefine load build options
#undef	spareOption18																				// Undefine load build options
#undef	spareOption19																				// Undefine load build options
#undef	spareOption20																				// Undefine load build options
#undef	spareOption21																				// Undefine load build options
#undef	spareOption22																				// Undefine load build options
#undef	spareOption23																				// Undefine load build options
#undef	spareOption24																				// Undefine load build options
#undef	bUserMessagesAllowed																// Indicates wherther user messaging is allowed (Zigbee, Wi-Fi, Bluetooth, IR,...

#undef	bHasMultRobotNetworking															// Undefine load build options
#undef  bSystemUARTSupportsZigBee														// Undefine load build options
#undef	bHasZigBeeSupport																		// Undefine load build options
#undef	bHasZigBeeFeatureSpare1															// Undefine load build options
#undef	bHasZigBeeFeatureSpare2															// Undefine load build options
#undef  kNumbOfZigBeeMessageBuffers													// Undefine load build options

#undef	kStartOfFileSystem																	// Undefine load build options
#undef	kStartOfBootloader																	// Undefine load build options
#undef	bHasDivideByZeroException         									// Undefine load build options
#undef	loadBuildOptionSpare1      							 						// Undefine load build options
#undef	loadBuildOptionSpare2      							 						// Undefine load build options
#undef	loadBuildOptionSpare3      							 						// Undefine load build options
#undef	loadBuildOptionSpare4      							 						// Undefine load build options
#undef	loadBuildOptionSpare5      							 						// Undefine load build options
#undef	loadBuildOptionSpare6      							 						// Undefine load build options
#undef	loadBuildOptionSpare7      							 						// Undefine load build options
#undef	loadBuildOptionSpare8      							 						// Undefine load build options
#undef	loadBuildOptionSpare9      							 						// Undefine load build options
#undef	loadBuildOptionSpare10      							 					// Undefine load build options
#undef	loadBuildOptionSpare11      							 					// Undefine load build options
#undef	loadBuildOptionSpare12      							 					// Undefine load build options
#undef	loadBuildOptionSpare13      							 					// Undefine load build options
#undef	loadBuildOptionSpare14      							 					// Undefine load build options
#undef	loadBuildOptionSpare15      							 					// Undefine load build options

#undef	useFastMemoryHandlers																// Undefine load build options

#undef  SystemUartPortCanBeReconfigured											// Undefine load build options
#undef	hasUserSerialPorts																	// Undefine load build options
#undef	getNumbOfSerialPorts																// Undefine load build options
#undef	flipUART0_And_UART1																	// Undefine load build options
#undef	kSizeOfXmitUartBuffer																// Undefine load build options
#undef	kSizeOfRcvUartBuffer																// Undefine load build options

#undef	ARMCPU																							// Undefine load build options
#undef	AVRCPU																							// Undefine load build options
#undef	STM8CPU																							// Undefine load build options
#undef	Spare1CPU																						// Undefine load build options
#undef	Spare2CPU																						// Undefine load build options
#undef	Spare3CPU																						// Undefine load build options
#undef	useParmForCurrOpPtr																	// Undefine load build options

#endif // "if !defined(LoadBuildOptionsClearFields__H_)

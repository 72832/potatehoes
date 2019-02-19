#ifndef _CameraIntrinsics__
#define _CameraIntrinsics__

#pragma systemFile


#if defined(hasCamera)


///////////////////////////////////////////////////////////////////////////////////////////////////////
//
//																	Camera Control and Operation
//
///////////////////////////////////////////////////////////////////////////////////////////////////////

	intrinsic void StartCamera()                       asm(opcdSystemFunctions, byte(sysFuncCameraFunctions),  byte(camStart));
	intrinsic void StopCamera() 											 asm(opcdSystemFunctions, byte(sysFuncCameraFunctions),  byte(camStop));
	intrinsic void ResetCameraConfigurationToDefault() asm(opcdSystemFunctions, byte(sysFuncCameraFunctions),  byte(camUseDefaultConfig));
	intrinsic void SetCameraConfig(byte &config)       asm(opcdSystemFunctions, byte(sysFuncSetCameraConfig),  variableRefByte(config));
	intrinsic void GetCameraConfig(byte &config)       asm(opcdSystemFunctions, byte(sysFuncGetCameraConfig),  variableRefByte(config));
	intrinsic void GetTrackingParms(TTrackingDataWords &myData)
																											asm(opcdSystemFunctions, byte(sysFuncGetTrackingParms),  variableRefRAM(myData));

	intrinsic TTrackingDataWords propertyIndex(trackingData,  parmSourceExternMemoryRef, kExternCMUCAMTracking);

	intrinsic int           propertyIndex(nCameraVertical,         propertyCamera, camVertical)
	intrinsic int           propertyIndex(nCameraHorizontal,       propertyCamera, camHorizontal)
	intrinsic int           propertyIndex(nCameraConfidence,       propertyCamera, camConfidence)
	intrinsic int           propertyIndex(nCameraTrackingPackets,  propertyCamera, camNumbTrackingRecords)
	intrinsic TCameraStatus propertyIndex(nCameraStatus,           propertyCamera, camTrackingStatus)


#endif
#endif

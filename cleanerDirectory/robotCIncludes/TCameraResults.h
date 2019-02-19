#if  defined(hasCamera) && !defined(_TCameraResults_H__)
#define _TCameraResults_H__

#if defined(ROBOTC)
	#pragma systemFile
#endif




///////////////////////////////////////////////////////////////////////////////////////////////////////
//
//            Convert current state machine into a user understandable value!!
//
///////////////////////////////////////////////////////////////////////////////////////////////////////

enum TCameraStatus
{
	camStatusInactive						= 0,
	camStatusNoResponse					= 1,
	camStatusSettingRegisters   = 2,
	camStatusSettingResolution  = 3,
	camStatusDownloadingConfig  = 4,
	camStatusTracking           = 5,
	camStatusShutdown           = 6,
	camStatusBadState           = 7,
	camStatusBadStateBig        = 5000   // To force into a 16-bit value
;


//
// Tracking data is unsigned byte which can cause problems with user programs which only have 'signed byte' support. So make it
// visible to user data as a struct of 16-bit ints.
//
struct TTrackingDataWords
{
	TCameraStatus nCameraStatus;
	short nHorizontal;
	short nVertical;
	short nTrackingPacketsRcvd;			//Used to determine whenever new tracking data is available

	short nHorizontalRaw;
	short nVerticalRaw;
	short left;
	short top;
	short right;
	short bottom;
	short pixels;
	short confidence;
};

enum TCameraPassThru
{
	// States where chars are passed to ROBOTC VM

	normalMessaging,
	normalCheckForAA,
	normalCheckFor55,

	// States where chars are passed to camera serial port

	cameraPassThru,
	cameraPassThruCheckForFF,
	cameraPassThruCheckForFE
};

enum TCameraSources
{
	camConfidence                 = 0,
	camVertical                   = 1,
	camHorizontal                 = 2,
	camNumbTrackingRecords        = 3,
	camTrackingStatus             = 4
};


#endif

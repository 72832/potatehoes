///////////////////////////////////////////////////////////
//
// ROBOTC Camera Initialization File
//
//
// Created: 4/20/2010 11:29 AM
//
///////////////////////////////////////////////////////////

#pragma camera(Config,    255, 4, 15, 1, 1, 1, 1, 1)
//#define Resolution                          255   //HR
//#define DownSampling                        4     //DS
//#define StabilizationTime                   15    //Unused by ROBOTC F/W
//#define RGBColorSpace                       1     //Unused by ROBOTC F/W
//#define AGCEnable                           1     //Unused by ROBOTC F/W
//#define AutoWhiteBalance                    1     //Unused by ROBOTC F/W
//#define AECDisable                          1     //Unused by ROBOTC F/W
//#define FluoresecentFilter                  1     //Unused by ROBOTC F/W

#pragma camera(BasicRegisters,    44, 33, 0, 4)
//#define ComA                                44    //
//#define ComB                                33    //
//#define ComI                                0     //
//#define ComJ                                4     //

#pragma camera(TrackingColor,    96, 141, 39, 59, 16, 22, 2, 0)
//#define redMin                              96    //
//#define redMax                              141   //
//#define greenMin                            39    //
//#define greenMax                            59    //
//#define blueMin                             16    //
//#define blueMax                             22    //
//#define NoiseFiler                          2     //
//#define TrackInverted                       0     //

#pragma camera(FrameSize,    1, 1, 40, 60)
//#define Left                                1     //
//#define Top                                 1     //
//#define Right                               40    //
//#define Bottom                              60    //

#pragma camera(ViewPort,    0, 0, 40, 60)
//#define Left                                0     //
//#define Top                                 0     //
//#define Right                               40    //
//#define Bottom                              60    //

#pragma camera(ServoControl,    0, 0, 0, 0, 0, 0)
//#define PanRangeFar                         0     //
//#define PanRangeNear                        0     //
//#define PanStep                             0     //
//#define TiltRangeFar                        0     //
//#define TiltRangeNear                       0     //
//#define TiltStep                            0     //

#pragma camera(Registers,    63, 15, 128, 128, 255, 128, 0, 0, 0)
//#define AGCGain                             63    //
//#define BlueGain                            15    //
//#define RedGain                             128   //
//#define Saturation                          128   //
//#define Brightness                          255   //
//#define Exposure                            128   //
//#define Sharpness                           0     //
//#define WhiteBalanceBlue                    0     //
//#define WhiteBalanceRed                     0     //

#include <Includes/TCamera.h>


//////////////////////////////////////////////////////////////////////////////////////////
//
//                        Advanced Sensor Functions
//                        =========================
//
// As of April 2008, ROBOTC now contains basic drivers for many I2C sensor types directly
// implemented in the ROBOTC firmware. ROBOTC firmware will send appropriate initialization
// messages to the sensors and automatically periodically poll the sensor "value".
//
// The appropriate initialization message and the frequency of polling is internally
// stored as a data table within the ROBOTC firmware.
//
// The firmware contains sufficient intelligence to retrieve a single value for a sensor.
// For example:
//    - a single compositivbe color "index" from the HiTechnic sensor
//    - the first, or "X" axis, value from a triple axis accelerometer.
//
// This file contains a few routines that provide easy access to more data from the
// sensors.
//
//////////////////////////////////////////////////////////////////////////////////////////
#include "I2CIntrinsics.h"

//#pragma platform(NXT)
#pragma systemFile      // this eliminates warning for "unreferenced" functions

//
// Get the Red, Green, Blue components from a HiTechnic Color Sensor
//
void getRGB(const tSensors nSensorIndex, int &nRed, int &nGreen, int &nBlue);

//
// Determine whether a color is "close" enough to the color detected by the HiTechnic
// color sensor. i.e. are all color components of the currently detected color within
// "nRange" of the specified color components
//
bool bColorMatched(const tSensors nSensorIndex, int nRed, int nGreen, int nBlue, int nRange);


//
// Get the X, y and Z components from a HiTechnic Accelerometer Sensor
//
void getAccel(const tSensors nSensorIndex, int &nXAxis, int &nYAxis, int &nZAxis);

//
// Find an I2C sensor with the specified manufacturer and sensor type
//
bool findSensor(tSensors& nSensor, const string& sVendor, const string& sSensorType);

//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
//
//                        HiTechnic Color Sensor
//                        ======================
//
// Get the Red, Green, Blue components from a HiTechnic Color Sensor
//
// Determine whether a color is "close" enough to the color detected by the HiTechnic
// color sensor. i.e. are all color components of the currently detected color within
// "nRange" of the specified color components
//
//////////////////////////////////////////////////////////////////////////////////////////

void getRGB(const tSensors nSensorIndex, int &nRed, int &nGreen, int &nBlue)
{
  ubyte nColorSensorData[4];

  getI2CPolledData(nSensorIndex, &nColorSensorData[0], sizeof(nColorSensorData));
  nRed    = nColorSensorData[1] & 0x00FF;
  nGreen  = nColorSensorData[2] & 0x00FF;
  nBlue   = nColorSensorData[3] & 0x00FF;
  return;
}


//
// Determine whether a color is "close" enough to the color detected by the HiTechnic
// color sensor. i.e. are all color components of the currently detected color within
// "nRange" of the specified color components
//

bool bColorMatched(const tSensors nSensorIndex, const int nRed, const int nGreen, const int nBlue, const int nRange)
{
  ubyte nColorSensorData[4];

  getI2CPolledData(nSensorIndex, &nColorSensorData[0], sizeof(nColorSensorData));

  if (abs(nRed -(nColorSensorData[1] & 0x00FF)) > nRange)
    return false;
  if (abs(nGreen -(nColorSensorData[2] & 0x00FF)) > nRange)
    return false;
  if (abs(nBlue -(nColorSensorData[3] & 0x00FF)) > nRange)
    return false;

  return true;
}


//////////////////////////////////////////////////////////////////////////////////////////
//
//                        HiTechnic Accelerometer
//                        =======================
//
//////////////////////////////////////////////////////////////////////////////////////////

void getAccel(const tSensors nSensorIndex, int &nXAxis, int &nYAxis, int &nZAxis)
{
  ubyte nAccelSensorData[6];

  getI2CPolledData(nSensorIndex, &nAccelSensorData[0], sizeof(nAccelSensorData));
  nXAxis   = (nAccelSensorData[0] << 2) | nAccelSensorData[3];
  nYAxis   = (nAccelSensorData[1] << 2) | nAccelSensorData[4];
  nZAxis   = (nAccelSensorData[2] << 2) | nAccelSensorData[5];
  return;
}


//////////////////////////////////////////////////////////////////////////////////////////
//
//                                  getI2CInfo
//                                  ==========
//
// Handy utility routine to read the "manufacturer" and "sensor type" information from
// an I2C sensor.
//
// By default, all NXT I2C sensors utilize the first 24 locations of the sensor to contain
// information about the sensor. There are three 8-byte fields containing ASCII strings.
//   0..7     The sensor version
//   8..15    The Manufacturer
//  16..23    The sensor type.
//
//////////////////////////////////////////////////////////////////////////////////////////

bool getI2CInfo(const tSensors nIntPort, string& sManufacturer, string& sType)
{
  tSensors nPort = nIntPort;
  TSensorTypes nSaveSensorType;
  ubyte nTemp[9];
  bool bI2CType;
  static const ubyte kReadManufacturer[] = {2, 0x02, 0x08};
  static const ubyte kReadSensorType[]   = {2, 0x02, 0x10};

  nSaveSensorType = SensorType[nPort];
  SensorType[nPort] = sensorI2CCustom;
  wait1Msec(50);

  while (true)
  {
    switch (nI2CStatus[nPort])
    {
    case NO_ERR:
    case ERR_COMM_BUS_ERR:
      break;

    default:
    case STAT_COMM_PENDING:
      wait1Msec(1);
      // Keep waiting for reply. I2C messaging is not complete
      continue;
    }
    break;
  }

  bI2CType = false;
  sendI2CMsg(nPort, &kReadManufacturer[0], 8);
  while (true)
  {
    switch (nI2CStatus[nPort])
    {
    case NO_ERR:
      bI2CType = true;
      break;

   case ERR_COMM_BUS_ERR:
      break;

    default:
    case STAT_COMM_PENDING:
      wait1Msec(1);
      // Keep waiting for reply. I2C messaging is not complete
      continue;
    }
    break;
  }
  if (!bI2CType)
  {
    sManufacturer = "";
    sType         = "";
    SensorType[nPort] = nSaveSensorType;
    return false;
  }
  readI2CReply(nPort, &nTemp[0], 8);
  nTemp[8] = 0;
  strcpy(sManufacturer, &nTemp[0]);


  sendI2CMsg(nPort, &kReadSensorType[0], 8);
  while (true)
  {
    switch (nI2CStatus[nPort])
    {
    case NO_ERR:
    case ERR_COMM_BUS_ERR:
      break;

    default:
    case STAT_COMM_PENDING:
      wait1Msec(1);
      // Keep waiting for reply. I2C messaging is not complete
      continue;
    }
    break;
  }
  if (!bI2CType)
  {
    sManufacturer = "";
    sType         = "";
    SensorType[nPort] = nSaveSensorType;
    return false;
  }
  readI2CReply(nPort, &nTemp[0], 8);
  nTemp[8] = 0;
  strcpy(sType, &nTemp[0]);
  SensorType[nPort] = nSaveSensorType;
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
//
//                                  findSensor
//                                  ==========
//
// Handy utility routine to search all the sensor ports to find a
//
//////////////////////////////////////////////////////////////////////////////////////////

bool findSensor(tSensors& nSensor, const string& sVendor, const string& sSensorType)
{
  string sManufacturer;
  string sType;

  for (nSensor = S1; nSensor <= S4; ++nSensor)
  {
    if (!getI2CInfo(nSensor, sManufacturer, sType))
      continue;
    if (sManufacturer != sVendor)
      continue;
    if (sType != sSensorType)
      continue;
    return true;
  }
  return false;
}

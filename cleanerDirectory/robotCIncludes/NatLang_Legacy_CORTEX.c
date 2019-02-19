/*------------------------------------------*\
|*	- Natural Language for CORTEX -			*|
|*                                      	*|
|*	This file includes all of the 			*|
|*	Natural Language functions for CORTEX.	*|
\*-------------------------------------4246-*/

#pragma systemFile            	// eliminates warning for "unreferenced" functions

// enum for RIGHT and LEFT direction:
typedef enum tDirections
{
  right = 0,
  left  = 1
}tDirections;

// enum for VEX LCD buttons:
typedef enum tVEXLCDButtons
{
  leftBtnVEX            = 1,
  centerBtnVEX          = 2,
  //leftANDcenterBtnVEX   = 3,
  rightBtnVEX           = 4//,
  //leftANDrightBtnVEX    = 5,
  //centerANDrightBtnVEX  = 6,
  //allBtnsVEX            = 7
}tVEXLCDButtons;

// enum for units
typedef enum tMovementUnits
{
  degrees             = 0,
  rotations           = 1,
  milliseconds        = 2,
  seconds             = 3,
  minutes             = 4
} tMovementUnits;

// enum for Robot Types:
typedef enum tRobotType
{
  none		= 0,
  recbot	= 1,
  swervebot = 2,
  squarebot = 3,
}tRobotType;

// enum for LED (on or off):
typedef enum tLedCommand
{
	ledOff	= 0,
	ledOn	= 1,
}tLedCommand;

/*********************\
|*  SETUP FUNCTIONS  *|
\*******************************************************************************************************************************************/

//-------------------------------------------| RobotType |--------------------------------------------
#ifndef _ROBOTTYPE_H_GUARD
#define _ROBOTTYPE_H_GUARD
void robotType(tRobotType type = none)
{
  // MOTORS:
  if(type == none)                   // None
  {
	  bMotorReflected[port2] = false;
    bMotorReflected[port3] = false;
    return; // skip sensors setup
  }
  else if(type == recbot || type == squarebot) // Recbot + Squarebot
  {
    bMotorReflected[port2] = true;
    bMotorReflected[port3] = false;
  }
  else if(type == swervebot)              // Swervebot
  {
    bMotorReflected[port2] = false;
    bMotorReflected[port3] = true;
  }

  // SENSORS (for all robots - 'none'):
  if(1)
  {
	  if(SensorType[in1] == sensorNone)
	  {
	    SensorType[in1] = sensorLineFollower;
	  }

	  if(SensorType[in2] == sensorNone)
	  {
	    SensorType[in2] = sensorLineFollower;
	  }

	  if(SensorType[in3] == sensorNone)
	  {
	    SensorType[in3] = sensorLineFollower;
	  }

	  if(SensorType[in6] == sensorNone)
	  {
	    SensorType[in6] = sensorPotentiometer;
	  }

	  if(SensorType[dgtl1] == sensorNone && SensorType[dgtl2] == sensorNone)
	  {
	    SensorType[dgtl1] = sensorQuadEncoder;
		//SensorType[dgtl2] = sensorQuadEncoderSecondPort;
	  }

	  if(SensorType[dgtl3] == sensorNone && SensorType[dgtl4] == sensorNone)
	  {
	    SensorType[dgtl3] = sensorQuadEncoder;
		//SensorType[dgtl4] = sensorQuadEncoderSecondPort;
	  }

	  if(SensorType[dgtl6] == sensorNone)
	  {
	    SensorType[dgtl6] = sensorTouch;
	  }

	  if(SensorType[dgtl7] == sensorNone)
	  {
	    SensorType[dgtl7] = sensorTouch;
	  }
	  if(SensorType[dgtl8] == sensorNone && SensorType[dgtl9] == sensorNone)
	  {
	    SensorType[dgtl8] = sensorSONAR_TwoPins_cm;
	  }
	}
}
#endif
//----------------------------------------------------------------------------------------------------

/*********************************************************************************************************************************************\
																													|*  END SETUP FUNCTIONS  *|
																													\*************************/
/************************\
|*  MOVEMENT FUNCTIONS  *|
\*******************************************************************************************************************************************/

//-------------------------------------------| startMotor |-------------------------------------------
#ifndef _STARTMotorTypes_H_GUARD
#define _STARTMotorTypes_H_GUARD

void startMotor(tMotor motorPort = port6, signed byte speed = 95)
{
  motor[motorPort] = speed;
}

#endif
//----------------------------------------------------------------------------------------------------

//-------------------------------------------| StopMotor |--------------------------------------------
#ifndef _STOPMOTOR_H_GUARD
#define _STOPMOTOR_H_GUARD

void stopMotor(tMotor motorPort = port6)
{
  motor[motorPort] = 0;
}

#endif
//----------------------------------------------------------------------------------------------------

//--------------------------------------------| SetServo |--------------------------------------------
#ifndef _SETSERVO_H_GUARD
#define _SETSERVO_H_GUARD

void setServo(tMotor servoPort = port6, signed byte position = 0)
{
  motor[servoPort] = position;
}

#endif
//----------------------------------------------------------------------------------------------------

/********************************************************************************************************************************************\
																												|*  END MOVEMENT FUNCTIONS  *|
																												\****************************/
/**********************\
|*  "WAIT" FUNCTIONS  *|
\*******************************************************************************************************************************************/

//----------------------------------------------| Wait |----------------------------------------------
#ifndef _WAIT_H_GUARD
#define _WAIT_H_GURAD

void wait(float waitTime = 1.0)
{
  long secs = abs(waitTime);
  long millisecs = ((long) (abs(waitTime) * 1000)) % 1000;

  while (secs > 0)      // seconds loop
  {
    sleep(1000);
    --secs;
  }

  sleep(millisecs);
}

#endif
//----------------------------------------------------------------------------------------------------

//---------------------------------------| WaitInMilliseconds |---------------------------------------
#ifndef _WAITINMILLISECONDS_H_GUARD
#define _WAITINMILLISECONDS_H_GUARD

void waitInMilliseconds(long waitTime = 1000)
{
  wait1Msec((long)waitTime);
}

#endif
//----------------------------------------------------------------------------------------------------
/******************************************************************************************************************************************\
																												|*  END "WAIT" FUNCTIONS  *|
																												\**************************/
/***********************\
|*  "UNTIL" FUNCTIONS  *|
\*******************************************************************************************************************************************/

//-------------------------------------------| UntilTouch |-------------------------------------------
#ifndef _UNTILTOUCH_H_GUARD
#define _UNTILTOUCH_H_GUARD

void untilTouch(tSensors sensorPort = dgtl6)
{
  while(SensorValue[sensorPort] != 1){wait1Msec(1);}
}

#endif
//----------------------------------------------------------------------------------------------------

//------------------------------------------| UntilRelease |------------------------------------------
#ifndef _UNTILRELEASE_H_GUARD
#define _UNTILRELEASE_H_GUARD

void untilRelease(tSensors sensorPort = dgtl6)
{
  while(SensorValue[sensorPort] != 0){wait1Msec(1);}
}

#endif
//----------------------------------------------------------------------------------------------------

//-------------------------------------------| UntilBump |--------------------------------------------
#ifndef _UNTILBUMP_H_GUARD
#define _UNTILBUMP_H_GUARD

void untilBump(tSensors sensorPort = dgtl6, short delayTimeMS = 10)
{
  while(SensorValue[sensorPort] != 1)
  		wait1Msec(1);

  int tempvar = delayTimeMS;
  wait1Msec(tempvar);

  while(SensorValue[sensorPort] == 1)
  		wait1Msec(1);
}

#endif
//----------------------------------------------------------------------------------------------------

//---------------------------------------| UntilSonarLessThan |---------------------------------------
#ifndef _UNTILSONARLESSTHAN_H_GUARD
#define _UNTILSONARLESSTHAN_H_GUARD

void untilSonarLessThan(short distance = 30, tSensors sensorPort = dgtl8)
{
  while(SensorValue[sensorPort] > distance || SensorValue[sensorPort] == -1){wait1Msec(1);}
}

#endif
//----------------------------------------------------------------------------------------------------

//-------------------------------------| UntilSonarGreaterThan |--------------------------------------
#ifndef _UNTILSONARGREATERTHAN_H_GUARD
#define _UNTILSONARGREATERTHAN_H_GUARD

void untilSonarGreaterThan(short distance = 30, tSensors sensorPort = dgtl8)
{
  while(SensorValue[sensorPort] < distance || SensorValue[sensorPort] == -1){wait1Msec(1);}
}

#endif
//----------------------------------------------------------------------------------------------------

//-----------------------------------| UntilPotentiometerLessThan |-----------------------------------
#ifndef _UNTILPOTENTIOMETERLESSTHAN_H_GUARD
#define _UNTILPOTENTIOMETERLESSTHAN_H_GUARD

void untilPotentiometerLessThan(int position = 2048, tSensors sensorPort = in6)
{
  while(SensorValue[sensorPort] > abs(position)){wait1Msec(1);}
}

#endif
//----------------------------------------------------------------------------------------------------

//---------------------------------| UntilPotentiometerGreaterThan |----------------------------------
#ifndef _UNTILPOTENTIOMETERGREATERTHAN_H_GUARD
#define _UNTILPOTENTIOMETERGREATERTHAN_H_GUARD

void untilPotentiometerGreaterThan(int position = 2048, tSensors sensorPort = in6)
{
  while(SensorValue[sensorPort] < abs(position)){wait1Msec(1);}
}

#endif
//----------------------------------------------------------------------------------------------------

//----------------------------------------| UntilButtonPress |----------------------------------------
#ifndef _UNTILBUTTONPRESS_H_GUARD
#define _UNTILBUTTONPRESS_H_GUARD

void untilButtonPress(short button = centerBtnVEX)
{
  while(true)
  {
    if(nLCDButtons == button)
    {
      break;
    }
    wait1Msec(1);
  }
}

#endif
//----------------------------------------------------------------------------------------------------

//-------------------------------------------| UntilDark |--------------------------------------------
#ifndef _UNTILDARK_H_GUARD
#define _UNTILDARK_H_GUARD

void untilDark(int threshold = 1500, tSensors sensorPort = in2)
{
  while(SensorValue[sensorPort] < threshold){wait1Msec(1);}
}

#endif
//----------------------------------------------------------------------------------------------------

//-------------------------------------------| UntilLight |-------------------------------------------
#ifndef _UNTILLIGHT_H_GUARD
#define _UNTILLIGHT_H_GUARD

void untilLight(int threshold = 1500, tSensors sensorPort = in2)
{
  while(SensorValue[sensorPort] > threshold){wait1Msec(1);}
}

#endif
//----------------------------------------------------------------------------------------------------

//-----------------------------------------| UntilRotations |-----------------------------------------
#ifndef _UNTILROTATIONS_H_GUARD
#define _UNTILROTATIONS_H_GUARD

void untilRotations(float rotations = 1.0, tSensors sensorPort = dgtl1)
{
  SensorValue[sensorPort] = 0;

  while(abs(SensorValue[sensorPort]) < (abs(rotations) * 360.0)){wait1Msec(1);}
}

#endif
//----------------------------------------------------------------------------------------------------

//---------------------------------------| UntilEncoderCounts |---------------------------------------
#ifndef _UNTILENCODERCOUNTS_H_GUARD
#define _UNTILENCODERCOUNTS_H_GUARD

void untilEncoderCounts(long distance = 360, tSensors sensorPort = dgtl1)
{
  SensorValue[sensorPort] = 0;

  while(abs(SensorValue[sensorPort]) < abs(distance)){wait1Msec(1);}
}

#endif
//----------------------------------------------------------------------------------------------------

/*******************************************************************************************************************************************\
																												|*  END "UNTIL" FUNCTIONS  *|
																												\***************************/
/****************************\
|*  ROBOT MOTION FUNCTIONS  *|
\*******************************************************************************************************************************************/

//--------------------------------------------| Forward |---------------------------------------------
#ifndef _FORWARD_H_GUARD
#define _FORWARD_H_GUARD

void forward(signed byte speed = 95)
{
  motor[port2] = abs(speed);
  motor[port3] = abs(speed);
}

#endif
//----------------------------------------------------------------------------------------------------

//--------------------------------------------| Backward |--------------------------------------------
#ifndef _BACKWARD_H_GUARD
#define _BACKWARD_H_GUARD

void backward(signed byte speed = -95)
{
  motor[port2] = abs(speed) * -1;
  motor[port3] = abs(speed) * -1;
}

#endif
//----------------------------------------------------------------------------------------------------

//-------------------------------------------| SwingTurn |--------------------------------------------
#ifndef _SWINGTURN_H_GUARD
#define _SWINGTURN_H_GUARD

void swingTurn(tDirections direction = right, signed byte speed = 95)
{
  if(direction == left)      // LEFT
  {
    if(speed >= 0)
    {
      motor[port2] = speed;
      motor[port3] = 0;
    }
    else if(speed < 0)
    {
      motor[port2] = 0;
      motor[port3] = speed;
    }
  }
  else if(direction == right) // RIGHT
  {
    if(speed >= 0)
    {
      motor[port2] = 0;
      motor[port3] = abs(speed);
    }
    else if(speed < 0)
    {
      motor[port2] = speed;
      motor[port3] = 0;
    }
  }
}

#endif
//----------------------------------------------------------------------------------------------------

//-------------------------------------------| PointTurn |--------------------------------------------
#ifndef _POINTTURN_H_GUARD
#define _POINTTURN_H_GUARD

void pointTurn(tDirections direction = right, signed byte speed = 95)
{
  if(direction == left)      // LEFT
  {
    motor[port2] = abs(speed);
    motor[port3] = abs(speed) * -1;
  }
  else if(direction == right) // RIGHT
  {
    motor[port2] = abs(speed) * -1;
    motor[port3] = abs(speed);
  }
}

#endif
//----------------------------------------------------------------------------------------------------

//----------------------------------------------| Stop |----------------------------------------------
#ifndef _STOP_H_GUARD
#define _STOP_H_GUARD

void stop()
{
  motor[port2] = 0;
  motor[port3] = 0;
}

#endif
//----------------------------------------------------------------------------------------------------

//-----------------------------------------| LineTrackTIME |------------------------------------------
#ifndef _LINETRACKTIME_H_GUARD
#define _LINETRACKTIME_H_GUARD

void lineTrackForTime(float trackTime = 5.0, int threshold = 1500, tSensors leftSensorPort = in1, tSensors centerSensorPort = in2, tSensors rightSensorPort = in3)
{
  float timeStart = ((float)nPgmTime / 1000);

  while(((float)nPgmTime / 1000) - timeStart < trackTime)
  {
    // RIGHT sensor sees dark:
    if(SensorValue(rightSensorPort) > threshold)
    {
      // counter-steer right:
      motor[port2] = 0;
      motor[port3] = 45;
    }
    // CENTER sensor sees dark:
    else if(SensorValue(centerSensorPort) > threshold)
    {
      // go straight
      motor[port2] = 45;
      motor[port3] = 45;
    }
    // LEFT sensor sees dark:
    else if(SensorValue(leftSensorPort) > threshold)
    {
      // counter-steer left:
      motor[port2] = 45;
      motor[port3] = 0;
    }
    wait1Msec(1);
  }
}

#endif
//----------------------------------------------------------------------------------------------------

//---------------------------------------| LineTrackROTATIONS |---------------------------------------
#ifndef _LINETRACKROTS_H_GUARD
#define _LINETRACKROTS_H_GUARD

void lineTrackForRotations(float rotations = 3.0, int threshold = 1500, tSensors leftSensorPort = in1, tSensors centerSensorPort = in2, tSensors rightSensorPort = in3)
{
  SensorValue[dgtl1] = 0;
  SensorValue[dgtl3] = 0;

  while((SensorValue[dgtl1] < (rotations * 360))  &&  (SensorValue[dgtl3] < (rotations * 360)))
  {
    // RIGHT sensor sees dark:
    if(SensorValue(rightSensorPort) > threshold)
    {
      // counter-steer right:
      motor[port2] = 0;
      motor[port3] = 45;
    }
    // CENTER sensor sees dark:
    else if(SensorValue(centerSensorPort) > threshold)
    {
      // go straight
      motor[port2] = 45;
      motor[port3] = 45;
    }
    // LEFT sensor sees dark:
    else if(SensorValue(leftSensorPort) > threshold)
    {
      // counter-steer left:
      motor[port2] = 45;
      motor[port3] = 0;
    }
    wait1Msec(1);
  }
}

#endif
//----------------------------------------------------------------------------------------------------

//------------------------------------------| TankControl |-------------------------------------------
#ifndef _TANKCONTROL_H_GUARD
#define _TANKCONTROL_H_GUARD

void tankControl(TVexJoysticks rightJoystick = Ch2, TVexJoysticks leftJoystick = Ch3, short threshold = 10)
{

	if(abs(vexRT[rightJoystick]) <= abs(threshold))
	{
		motor[port2] = 0;
	}
	else
	{
		motor[port2] = vexRT[rightJoystick];
	}

	if(abs(vexRT[leftJoystick]) <= abs(threshold))
	{
		motor[port3] = 0;
	}
	else
	{
		motor[port3] = vexRT[leftJoystick];
	}
}

#endif
//----------------------------------------------------------------------------------------------------

//-----------------------------------------| ArcadeControl |------------------------------------------
#ifndef _ARCADECONTROL_H_GUARD
#define _ARCADECONTROL_H_GUARD

void arcadeControl(TVexJoysticks verticalJoystick = Ch2, TVexJoysticks horizontalJoystick = Ch1, short threshold = 10)
{
	if(abs(vexRT[verticalJoystick]) <= abs(threshold) && abs(vexRT[horizontalJoystick]) <= abs(threshold))
	{
		motor[port2] = 0;
		motor[port3] = 0;
	}
	else
	{
		motor[port3] = (vexRT[verticalJoystick] + vexRT[horizontalJoystick])/2;  // (y + x)/2
		motor[port2] = (vexRT[verticalJoystick] - vexRT[horizontalJoystick])/2;  // (y - x)/2
	}
}

#endif
//----------------------------------------------------------------------------------------------------

//-------------------------------------| MoveStraightROTATIONS |--------------------------------------
#ifndef _MOVESTRAIGHTROTS_H_GUARD
#define _MOVESTRAIGHTROTS_H_GUARD

void moveStraightForRotations(float rotations = 1.0, tSensors rightEncoderPort = dgtl1, tSensors leftEncoderPort = dgtl3)
{
  SensorValue[rightEncoderPort] = 0;
  SensorValue[leftEncoderPort] = 0;

  while(SensorValue[rightEncoderPort] < (abs(rotations) * 360))
  {
		if(SensorValue[rightEncoderPort] == SensorValue[leftEncoderPort]) // If rightEncoder has counted the same amount as leftEncoder:
		{
			// Move Forward
			motor[port2] = 85;		// Right Motor is run at power level 85
			motor[port3]  = 85;		// Left Motor is run at power level 85
		}
		else if(SensorValue[rightEncoderPort] > SensorValue[leftEncoderPort])	// If rightEncoder has counted more encoder counts
		{
			// Turn slightly right
			motor[port2] = 65;		// Right Motor is run at power level 65
			motor[port3]  = 85;		// Left Motor is run at power level 85
		}
		else	// Only runs if leftEncoder has counted more encoder counts
		{
			// Turn slightly left
			motor[port2] = 85;		// Right Motor is run at power level 85
			motor[port3]  = 65;		// Left Motor is run at power level 65
		}
		wait1Msec(1);
  }
}

#endif
//----------------------------------------------------------------------------------------------------

//----------------------------------------| MoveStraightTIME |----------------------------------------
#ifndef _MOVESTRAIGHTTIME_H_GUARD
#define _MOVESTRAIGHTTIME_H_GUARD

void moveStraightForTime(float seconds = 5.0, tSensors rightEncoderPort = dgtl1, tSensors leftEncoderPort = dgtl3)
{
  SensorValue[rightEncoderPort] = 0;
  SensorValue[leftEncoderPort] = 0;

  float timeStart = ((float)nPgmTime / 1000);

  while(((float)nPgmTime / 1000) - timeStart < seconds)
  {
		if(SensorValue[rightEncoderPort] == SensorValue[leftEncoderPort]) // If rightEncoder has counted the same amount as leftEncoder:
		{
			// Move Forward
			motor[port2] = 85;		// Right Motor is run at power level 85
			motor[port3]  = 85;		// Left Motor is run at power level 85
		}
		else if(SensorValue[rightEncoderPort] > SensorValue[leftEncoderPort])	// If rightEncoder has counted more encoder counts
		{
			// Turn slightly right
			motor[port2] = 65;		// Right Motor is run at power level 65
			motor[port3]  = 85;		// Left Motor is run at power level 85
		}
		else	// Only runs if leftEncoder has counted more encoder counts
		{
			// Turn slightly left
			motor[port2] = 85;		// Right Motor is run at power level 85
			motor[port3]  = 65;		// Left Motor is run at power level 65
		}
		wait1Msec(1);
  }
}

#endif
//----------------------------------------------------------------------------------------------------

/********************************************************************************************************************************************\
																											|*  END ROBOT MOTION FUNCTIONS  *|
																											\********************************/
/***********************\
|*  SPECIAL FUNCTIONS  *|
\*******************************************************************************************************************************************/

//-------------------------------------------| TurnLEDON |--------------------------------------------
#ifndef _TURNLEDON_H_GUARD
#define _TURNLEDON_H_GUARD

void turnLEDOn(tSensors digitalPort = dgtl2)
{
  SensorValue[digitalPort] = (word)ledOn;
}

#endif
//----------------------------------------------------------------------------------------------------

//-------------------------------------------| TurnLEDOFF |-------------------------------------------
#ifndef _TURNLEDOFF_H_GUARD
#define _TURNLEDOFF_H_GUARD

void turnLEDOff(tSensors digitalPort = dgtl2)
{
  SensorValue[digitalPort] = (word)ledOff;
}

#endif
//----------------------------------------------------------------------------------------------------

//----------------------------------------| TurnFlashlightON |----------------------------------------
#ifndef _TURNFLASHLIGHTON_H_GUARD
#define _TURNFLASHLIGHTON_H_GUARD

void turnFlashlightOn(tMotor flashlightPort = port4, short brightness = 63)
{
  if(bMotorReflected[flashlightPort] == false)
  {
    motor[flashlightPort] = abs(brightness) * -1;
  }
  else
  {
    motor[flashlightPort] = abs(brightness);
  }
}

#endif
//----------------------------------------------------------------------------------------------------

//---------------------------------------| TurnFlashlightOFF |----------------------------------------
#ifndef _TURNFLASHLIGHTOFF_H_GUARD
#define _TURNFLASHLIGHTOFF_H_GUARD

void turnFlashlightOff(tMotor flashlightPort = port4)
{
  motor[flashlightPort] = 0;
}

#endif
//----------------------------------------------------------------------------------------------------

/*******************************************************************************************************************************************\
																												|*  END SPECIAL FUNCTIONS  *|
																												\***************************/
void resetTimer(TTimers currentTimer = T1)
{
  clearTimer(currentTimer);
}

float getTimer(TTimers currentTimer = T1, const tMovementUnits unitType = seconds)
{
  if(unitType == milliseconds)
    return (time1[currentTimer]);
  else if(unitType == seconds)
    return ((float)time1[currentTimer]/1000.0);
  else if(unitType == minutes)
    return (((float)time1[currentTimer] / 1000.0) / 60.0);

  return 0.0;
}

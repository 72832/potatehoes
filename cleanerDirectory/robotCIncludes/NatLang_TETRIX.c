/*------------------------------------------*\
|*	- Natural Language for TETRIX -			*|
|*                                      	*|
|*	This file includes all of the 			*|
|*	Natural Language functions for TETRIX.	*|
\*-------------------------------------4246-*/
#

#pragma systemFile            	// eliminates warning for "unreferenced" functions
#include "JoystickDriver.c"		// for use with joystick commands


// enum for RIGHT and LEFT direction:
typedef enum
{
	right = 0,
	left  = 1
}tDirections;

// enum for NXT buttons:
typedef enum
{
	cancelBtnNXT = 0,
	rightBtnNXT  = 1,
	leftBtnNXT   = 2,
	centerBtnNXT = 3
}tNxtButtons;

// enum for Robot Types:
typedef enum
{
	none		= 0,
	mantis    = 1,
	ranger 	= 2
}tRobotType;


/*********************\
|*  SETUP FUNCTIONS  *|
\*********************************************************************************************************************************************/

//-------------------------------------------| RobotType |--------------------------------------------
#ifndef _ROBOTTYPE_H_GUARD
#define _ROBOTTYPE_H_GUARD

void robotType(tRobotType type = none)
{
	// MOTORS:
	if(type == 0)
	{
		bMotorReflected[mtr_S1_C1_1] = false;
		bMotorReflected[mtr_S1_C1_2] = false;
		return; // skip sensors setup
	}
	else if(type == mantis || type == ranger)    // MANTIS or RANGER
	{
		bMotorReflected[mtr_S1_C1_1] = false;
		bMotorReflected[mtr_S1_C1_2] = true;
	}

}

#endif
//----------------------------------------------------------------------------------------------------

/*********************************************************************************************************************************************\
|*  END SETUP FUNCTIONS  *|
\*************************/
/************************\
|*  MOVEMENT FUNCTIONS  *|
\*********************************************************************************************************************************************/

//-------------------------------------------| startMotor |-------------------------------------------
#ifndef _STARTMotorTypes_H_GUARD
#define _STARTMotorTypes_H_GUARD

void startMotor(tMotor motorPort = motorA, byte speed = 50)
{
	motor[motorPort] = speed;
}

#endif
//----------------------------------------------------------------------------------------------------

//-------------------------------------------| StopMotor |--------------------------------------------
#ifndef _STOPMOTOR_H_GUARD
#define _STOPMOTOR_H_GUARD

void stopMotor(tMotor motorPort = motorA)
{
	motor[motorPort] = 0;
}

#endif
//----------------------------------------------------------------------------------------------------

//--------------------------------------------| SetServo |--------------------------------------------
#ifndef _SETSERVO_H_GUARD
#define _SETSERVO_H_GUARD

void setServo(TServoIndex servoPort = 0, int position = 0)
{
	while(ServoValue[servoPort] != abs(position))
	{
		servo[servoPort] = position;
		wait1Msec(1);
	}
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
	long mins = (abs(waitTime) / 60.0);
	long secs = ((abs(waitTime) * 1000) / 1000) - (mins * 60);
	long millisecs = (abs(waitTime) * 1000) - ((abs((long)waitTime) * 1000));

	for(long i=0; i<mins; i++)      // minutes loop
	{
		wait1Msec(30000);
		wait1Msec(30000);
	}

	for(long j=0; j<secs; j++)      // seconds loop
	{
		wait1Msec(1000);
	}

	wait1Msec(millisecs);
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

/**********************************************************************************************************************************************\
|*  END "WAIT" FUNCTIONS  *|
\**************************/
/***********************\
|*  "UNTIL" FUNCTIONS  *|
\**********************************************************************************************************************************************/

//-------------------------------------------| UntilTouch |-------------------------------------------
#ifndef _UNTILTOUCH_H_GUARD
#define _UNTILTOUCH_H_GUARD

void untilTouch(tSensors sensorPort = S2)
{
	while(SensorValue[sensorPort] != 1){wait1Msec(1);}
}

#endif
//----------------------------------------------------------------------------------------------------

//------------------------------------------| UntilRelease |------------------------------------------
#ifndef _UNTILRELEASE_H_GUARD
#define _UNTILRELEASE_H_GUARD

void untilRelease(tSensors sensorPort = S2)
{
	while(SensorValue[sensorPort] != 0){wait1Msec(1);}
}

#endif
//----------------------------------------------------------------------------------------------------

//-------------------------------------------| UntilBump |--------------------------------------------
#ifndef _UNTILBUMP_H_GUARD
#define _UNTILBUMP_H_GUARD

void untilBump(tSensors sensorPort = S2, int delayTimeMS = 10)
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

void untilSonarLessThan(short distance = 30, tSensors sensorPort = S4)
{
	while(SensorValue[sensorPort] > distance || SensorValue[sensorPort] == 255){wait1Msec(1);}
}

#endif
//----------------------------------------------------------------------------------------------------

//-------------------------------------| UntilSonarGreaterThan |--------------------------------------
#ifndef _UNTILSONARGREATERTHAN_H_GUARD
#define _UNTILSONARGREATERTHAN_H_GUARD

void untilSonarGreaterThan(short distance = 30, tSensors sensorPort = S4)
{
	while(SensorValue[sensorPort] < distance || SensorValue[sensorPort] == 255){wait1Msec(1);}
}

#endif
//----------------------------------------------------------------------------------------------------

//----------------------------------------| UntilButtonPress |----------------------------------------
#ifndef _UNTILBUTTONPRESS_H_GUARD
#define _UNTILBUTTONPRESS_H_GUARD

void untilButtonPress(short button = centerBtnNXT)
{
	while(true)
	{
		if(nNxtButtonPressed == button)
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

void untilDark(int threshold = 30, tSensors sensorPort = S3)
{
	while(SensorValue[sensorPort] > threshold){wait1Msec(1);}
}

#endif
//----------------------------------------------------------------------------------------------------

//-------------------------------------------| UntilLight |-------------------------------------------
#ifndef _UNTILLIGHT_H_GUARD
#define _UNTILLIGHT_H_GUARD

void untilLight(int threshold = 30, tSensors sensorPort = S3)
{
	while(SensorValue[sensorPort] < threshold){wait1Msec(1);}
}

#endif
//----------------------------------------------------------------------------------------------------

//-------------------------------------| UntilSoundGreaterThan |--------------------------------------
#ifndef _UNTILSOUNDGREATER_H_GUARD
#define _UNTILSOUNDGREATER_H_GUARD

void untilSoundGreaterThan(int threshold = 50, tSensors sensorPort = S2)
{
	while(SensorValue[sensorPort] <= threshold){wait1Msec(1);}
}

#endif
//----------------------------------------------------------------------------------------------------

//---------------------------------------| UntilSoundLessThan |---------------------------------------
#ifndef _UNTILSOUNDLESS_H_GUARD
#define _UNTILSOUNDLESS_H_GUARD

void untilSoundLessThan(int threshold = 50, tSensors sensorPort = S2)
{
	while(SensorValue[sensorPort] >= threshold){wait1Msec(1);}
}

#endif
//----------------------------------------------------------------------------------------------------

//-----------------------------------------| UntilRotations |-----------------------------------------
#ifndef _UNTILROTATIONS_H_GUARD
#define _UNTILROTATIONS_H_GUARD

void untilRotations(float rotations = 1.0, tMotor motorPort = mtr_S1_C1_1)
{
	nMotorEncoder[motorPort] = 0;

	while(nMotorEncoder[motorPort] < (abs(rotations) * 360.0)  &&  nMotorEncoder[motorPort] > (abs(rotations) * (360.0 * -1.0))){wait1Msec(1);}
}

#endif
//----------------------------------------------------------------------------------------------------

//---------------------------------------| UntilEncoderCounts |---------------------------------------
#ifndef _UNTILENCODERCOUNTS_H_GUARD
#define _UNTILENCODERCOUNTS_H_GUARD

void untilEncoderCounts(long distance = 360, tMotor motorPort = mtr_S1_C1_1)
{
	nMotorEncoder[motorPort] = 0;

	while(nMotorEncoder[motorPort] < (abs(distance))  &&  nMotorEncoder[motorPort] > (abs(distance) * -1)){wait1Msec(1);}
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

void forward(byte speed = 75)
{
	motor[mtr_S1_C1_1] = abs(speed);
	motor[mtr_S1_C1_2] = abs(speed);
}

#endif
//----------------------------------------------------------------------------------------------------

//--------------------------------------------| Backward |--------------------------------------------
#ifndef _BACKWARD_H_GUARD
#define _BACKWARD_H_GUARD

void backward(byte speed = -75)
{
	motor[mtr_S1_C1_1] = abs(speed) * -1;
	motor[mtr_S1_C1_2] = abs(speed) * -1;
}

#endif
//----------------------------------------------------------------------------------------------------

//-------------------------------------------| SwingTurn |--------------------------------------------
#ifndef _SWINGTURN_H_GUARD
#define _SWINGTURN_H_GUARD

void swingTurn(tDirections direction = right, byte speed = 75)
{
	if(direction == left)      // LEFT
	{
		if(speed >= 0)
		{
			motor[mtr_S1_C1_1] = speed;
			motor[mtr_S1_C1_2] = 0;
		}
		else if(speed < 0)
		{
			motor[mtr_S1_C1_1] = 0;
			motor[mtr_S1_C1_2] = speed;
		}
	}
	else if(direction == right) // RIGHT
	{
		if(speed >= 0)
		{
			motor[mtr_S1_C1_1] = 0;
			motor[mtr_S1_C1_2] = abs(speed);
		}
		else if(speed < 0)
		{
			motor[mtr_S1_C1_1] = speed;
			motor[mtr_S1_C1_2] = 0;
		}
	}
}

#endif
//----------------------------------------------------------------------------------------------------

//-------------------------------------------| PointTurn |--------------------------------------------
#ifndef _POINTTURN_H_GUARD
#define _POINTTURN_H_GUARD

void pointTurn(tDirections direction = right, byte speed = 75)
{
	if(direction == left)      // LEFT
	{
		motor[mtr_S1_C1_1] = abs(speed);
		motor[mtr_S1_C1_2] = abs(speed) * -1;
	}
	else if(direction == right) // RIGHT
	{
		motor[mtr_S1_C1_1] = abs(speed) * -1;
		motor[mtr_S1_C1_2] = abs(speed);
	}
}

#endif
//----------------------------------------------------------------------------------------------------

//----------------------------------------------| Stop |----------------------------------------------
#ifndef _STOP_H_GUARD
#define _STOP_H_GUARD

void stop()
{
	motor[mtr_S1_C1_1] = 0;
	motor[mtr_S1_C1_2] = 0;
}

#endif
//----------------------------------------------------------------------------------------------------

//-----------------------------------------| LineTrackTIME |------------------------------------------
#ifndef _LINETRACKTIME_H_GUARD
#define _LINETRACKTIME_H_GUARD

void lineTrackForTime(float trackTime = 5.0, int threshold = 45, tSensors sensorPort = S3)
{
	float timeStart = ((float)nPgmTime / 1000);

	while(((float)nPgmTime / 1000) - timeStart < trackTime)
	{
		//nxtDisplayBigTextLine(3, "%0.4f", ((float)nPgmTime / 1000));  // debug
		//Line Follow:
		if(SensorValue[sensorPort] < threshold)        // DARK:
		{
			motor[mtr_S1_C1_1] = 25;
			motor[mtr_S1_C1_2] = 55;
		}
		else if(SensorValue[sensorPort] >= threshold)  // LIGHT:
		{
			motor[mtr_S1_C1_1] = 55;
			motor[mtr_S1_C1_2] = 25;
		}
		wait1Msec(1);
	}
}

#endif
//----------------------------------------------------------------------------------------------------

//---------------------------------------| LineTrackROTATIONS |---------------------------------------
#ifndef _LINETRACKROTS_H_GUARD
#define _LINETRACKROTS_H_GUARD

void lineTrackForRotations(float rotations = 3.0, int threshold = 45, tSensors sensorPort = S3)
{
	nMotorEncoder[mtr_S1_C1_1] = 0;
	nMotorEncoder[mtr_S1_C1_2] = 0;

	while((nMotorEncoder[mtr_S1_C1_1] < (rotations * 360))  &&  (nMotorEncoder[mtr_S1_C1_2] < (rotations * 360)))
	{
		//nxtDisplayBigTextLine(3, "%0.4f", ((float)nPgmTime / 1000));  // debug
		//Line Follow:
		if(SensorValue[sensorPort] < threshold)        // DARK:
		{
			motor[mtr_S1_C1_1] = 25;
			motor[mtr_S1_C1_2] = 55;
		}
		else if(SensorValue[sensorPort] >= threshold)  // LIGHT:
		{
			motor[mtr_S1_C1_1] = 55;
			motor[mtr_S1_C1_2] = 25;
		}
		wait1Msec(1);
	}
}

#endif
//----------------------------------------------------------------------------------------------------
//------------------------------------------| StartJoystickControl |----------------------------------
#ifndef _STARTJOYCONTROL_H_GUARD
#define _STARTJOYCONTROL_H_GUARD

void startJoystickControl()
{
	startTask(readMsgFromPC);
}

#endif
//----------------------------------------------------------------------------------------------------

//------------------------------------------| StopJoystickControl |-----------------------------------
#ifndef _STOPJOYCONTROL_H_GUARD
#define _STOPJOYCONTROL_H_GUARD

void stopJoystickControl()
{
	//Reset joystick values
	bool bUserModeTemp = joystick.UserMode;
	bool bStopPgmTemp = joystick.StopPgm;

	memset(joystickCopy, 0, sizeof(joystickCopy));

	joystick.UserMode = bUserModeTemp;
	joystick.StopPgm = bStopPgmTemp;
	joystick.joy1_TopHat = -1;
	joystick.joy2_TopHat = -1;
	stopTask(readMsgFromPC);
}

#endif
//----------------------------------------------------------------------------------------------------
//------------------------------------------| TankControl |-------------------------------------------
#ifndef _TANKCONTROL_H_GUARD
#define _TANKCONTROL_H_GUARD

void tankControl(short &rightJoystick = joystick.joy1_y2, short &leftJoystick = joystick.joy1_y1, short threshold = 10)
{
	getJoystickSettings(joystick);
	if(rightJoystick <= abs(threshold)  &&  rightJoystick >= (abs(threshold) * -1))
		motor[mtr_S1_C1_1] = 0;
	else
		motor[mtr_S1_C1_1] = rightJoystick;

	if(leftJoystick <= abs(threshold)  &&  leftJoystick >= (abs(threshold) * -1))
		motor[mtr_S1_C1_2] = 0;
	else
		motor[mtr_S1_C1_2] = leftJoystick;
}

#endif
//----------------------------------------------------------------------------------------------------

//-----------------------------------------| ArcadeControl |------------------------------------------
#ifndef _ARCADECONTROL_H_GUARD
#define _ARCADECONTROL_H_GUARD

void arcadeControl(short &verticalJoystick = joystick.joy1_y2, short &horizontalJoystick = joystick.joy1_x2, short threshold = 10)
{
	getJoystickSettings(joystick);
	if( (verticalJoystick <= abs(threshold)  &&  verticalJoystick >= (abs(threshold) * -1))  &&
		(horizontalJoystick <= abs(threshold)  &&  horizontalJoystick >= (abs(threshold) * -1)))
	{
		motor[mtr_S1_C1_1] = 0;
		motor[mtr_S1_C1_2] = 0;
	}
	else
	{
		motor[mtr_S1_C1_1] = (verticalJoystick + horizontalJoystick) / 2;
		motor[mtr_S1_C1_2] = (verticalJoystick - horizontalJoystick) / 2;
	}
}

#endif
//----------------------------------------------------------------------------------------------------

//-------------------------------------| MoveStraightROTATIONS |--------------------------------------
#ifndef _MOVESTRAIGHTROTS_H_GUARD
#define _MOVESTRAIGHTROTS_H_GUARD

void moveStraightForRotations(float rotations = 1.0, tMotor rightEncoderPort = mtr_S1_C1_1, tMotor leftEncoderPort = mtr_S1_C1_2)
{
	nMotorEncoder[rightEncoderPort] = 0;
	nMotorEncoder[leftEncoderPort]  = 0;

	while(nMotorEncoder[rightEncoderPort] < (abs(rotations) * 360))
	{
		if(nMotorEncoder[rightEncoderPort] == nMotorEncoder[leftEncoderPort])     // If rightEncoder has counted the same amount as leftEncoder:
		{
			// Move Forward
			motor[rightEncoderPort] = 75;		// Right Motor is run at power level 75
			motor[leftEncoderPort]  = 75;		// Left Motor is run at power level 75
		}
		else if(nMotorEncoder[rightEncoderPort] > nMotorEncoder[leftEncoderPort]) // If rightEncoder has counted more encoder counts
		{
			// Turn slightly right
			motor[rightEncoderPort] = 50;		// Right Motor is run at power level 50
			motor[leftEncoderPort]  = 75;		// Left Motor is run at power level 75
		}
		else	// Only runs if leftEncoder has counted more encoder counts
		{
			// Turn slightly left
			motor[rightEncoderPort] = 75;		// Right Motor is run at power level 75
			motor[leftEncoderPort]  = 50;		// Left Motor is run at power level 50
		}
		wait1Msec(1);
	}
}

#endif
//----------------------------------------------------------------------------------------------------

//----------------------------------------| MoveStraightTIME |----------------------------------------
#ifndef _MOVESTRAIGHTTIME_H_GUARD
#define _MOVESTRAIGHTTIME_H_GUARD

void moveStraightForTime(float seconds = 5.0, tMotor rightEncoderPort = mtr_S1_C1_1, tMotor leftEncoderPort = mtr_S1_C1_2)
{
	nMotorEncoder[rightEncoderPort] = 0;
	nMotorEncoder[leftEncoderPort] = 0;

	float timeStart = ((float)nPgmTime / 1000);

	while(((float)nPgmTime / 1000) - timeStart < seconds)
	{
		if(nMotorEncoder[rightEncoderPort] == nMotorEncoder[leftEncoderPort])     // If rightEncoder has counted the same amount as leftEncoder:
		{
			// Move Forward
			motor[rightEncoderPort] = 75;		// Right Motor is run at power level 75
			motor[leftEncoderPort]  = 75;		// Left Motor is run at power level 75
		}
		else if(nMotorEncoder[rightEncoderPort] > nMotorEncoder[leftEncoderPort]) // If rightEncoder has counted more encoder counts
		{
			// Turn slightly right
			motor[rightEncoderPort] = 50;		// Right Motor is run at power level 50
			motor[leftEncoderPort]  = 75;		// Left Motor is run at power level 75
		}
		else	// Only runs if leftEncoder has counted more encoder counts
		{
			// Turn slightly left
			motor[rightEncoderPort] = 75;		// Right Motor is run at power level 75
			motor[leftEncoderPort]  = 50;		// Left Motor is run at power level 50
		}
		wait1Msec(1);
	}
}

#endif
//----------------------------------------------------------------------------------------------------

/********************************************************************************************************************************************\
|*  END ROBOT MOTION FUNCTIONS  *|
\********************************/

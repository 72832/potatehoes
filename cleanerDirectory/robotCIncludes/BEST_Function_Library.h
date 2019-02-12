////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//																								scaleJoystick
//
//  Provide a dead band where power level is zero. Eliminates noise / humming from motors when they the joysticks
//   don't precisely center themselves at zero.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int nDeadBandSize = 25;												// Adjust this number as appropriate

int scalePower(int nInputPower)
{
	if (nInputPower > nDeadBandSize)
		return nInputPower;

	if (nInputPower < -nDeadBandSize)
		return nInputPower;

	return 0;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//																								Arcade Steering
//
//  Only a single joystick is used where the speed is mapped to the y-axis on the joystick
//  and the turn rate is mixed in from the x-axis.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ArcadeTwoWheelDrive(TVexJoysticks MoveChannel,		// The joystick button for forward and reverse.
												 TVexJoysticks RotateChannel,	// The channel on the rotate port that controls left and right rotation. This value is mixed with the move channel.
												 tMotor leftMotor,						// The motor on the left side of the robot.
												 tMotor rightMotor)						// The motor on the right side of the robot.
{
	int nPower;
	int nRotateAdjust;

	nPower = scalePower(vexRT[MoveChannel]);
	nRotateAdjust = vexRT[RotateChannel];
	motor[leftMotor]  = nPower + nRotateAdjust;
	motor[rightMotor] = nPower - nRotateAdjust;
}


void ArcadeFourWheelDrive(TVexJoysticks MoveChannel, 	// The joystick button that controls forward and backward movement in arcade steering.
												TVexJoysticks RotateChannel,  // The channel on the rotate port that controls left and right rotation. This value is mixed with the move port/channel.
												tMotor LeftFrontMotor, 				// The motor on the left  front side of the robot.
												tMotor RightFrontMotor,				// The motor on the right front side of the robot.
												tMotor LeftRearMotor, 				// The motor on the left   rear of the robot.
												tMotor RightRearMotor)				// The motor on the right  rear of the robot.
{
	int nPower;
	int nRotateAdjust;

	nPower = scalePower(vexRT[MoveChannel]);
	nRotateAdjust = vexRT[RotateChannel];
	motor[LeftFrontMotor]  = nPower + nRotateAdjust;
	motor[LeftRearMotor]   = nPower + nRotateAdjust;
	motor[RightFrontMotor] = nPower - nRotateAdjust;
	motor[RightRearMotor]  = nPower - nRotateAdjust;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//																								Tank Steering
//
//  The left and right joystick y-axis control the speed (forward and reverse) of the left and right wheels.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TankTwoWheelDrive(TVexJoysticks leftChannel,			// The joystick button that controls left  motor
											 TVexJoysticks rightChannel, 		// The joystick button that controls right motor
											 tMotor leftMotor,							// The motor on the left side of the robot.
											 tMotor rightMotor)							// The motor on the right side of the robot.
{
	int nLeftPower;
	int nRightPower;

	nLeftPower = scalePower(vexRT[leftChannel]);
	nRightPower = scalePower(vexRT[rightChannel]);

	motor[leftMotor]  =  nLeftPower;
	motor[rightMotor] =  nRightPower;
}

void TankFourWheelDrive(TVexJoysticks leftChannel,  // The joystick button that controls left  motor
												TVexJoysticks rightChannel, // The joystick button that controls right motor
												tMotor LeftFrontMotor, 			// The motor on the left  front side of the robot.
												tMotor RightFrontMotor,			// The motor on the right front side of the robot.
												tMotor LeftRearMotor, 			// The motor on the left   rear of the robot.
												tMotor RightRearMotor)			// The motor on the right  rear of the robot.
{
	int nLeftPower;
	int nRightPower;

	nLeftPower = scalePower(vexRT[leftChannel]);
	nRightPower = scalePower(vexRT[rightChannel]);

	motor[RightFrontMotor] =  nRightPower;
	motor[RightRearMotor]  =  nRightPower;
	motor[LeftFrontMotor]  =  nLeftPower;
	motor[LeftRearMotor]   =  nLeftPower;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//																					ServoControlViaDigitalButtons
//
//  Use two buttons for "down" and "up" servo positioning speed.
//
//  1. Push down button to gradually move servo to position 'lowerPosition'.
//
//  2. Push   up button to gradually move servo to position 'upperPosition'.
//
//  If neither button is pressed, or if both buttons are pressed, then servo position is not updated.
//
//  'lowerPosition' and 'upperPosition' contain the minimum and maximum values for the servo.
//
//  'rateOfChange' determines how fast the servo position is updated.
//     -- To immediately go to the extreme positions use a value of zero.
//     -- A non-zero value indicates how many milliseconds before adjusting the servo position. Each adjustment is
//        10 counts
//     -- Typical values are in the range 10 to 50. A value of 10 will take 0.25 seconds to go from one extreme to
//        the other. A value of 50 will take 1.25 seconds.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static long nTimeOfNextPositionUpdate[kNumbOfRealMotors]; // Keeps track of when to perform the next position update of the servo.

void ServoControlViaDigitalButtons(TVexJoysticks downChannel,		// The joystick button that controls downward movement
														 TVexJoysticks upChannel, 					// The joystick button that controls upward movement
														 tMotor theServo,					// The servo  that is being controlled
														 int lowerPosition,				// The lowest DOWN value for the servo position
														 int upperPosition,				// The highest UP value for the servo position
														 int rateOfChange)				// How fast to adjust the speed. 0 -- fastest. 50 -- 1.25 seconds to cover the compete range.
{
	const int kPositionAdjustmentStepSize = 10;

	int nOldPosition;
	int nNewPosition;

	if ((vexRT[downChannel] != 0) && (vexRT[upChannel] != 0))
	{
		// Both buttons have been pushed. Do nothing until it is just a single button pushed.
		return;
	}

	if ((vexRT[downChannel] == 0) && (vexRT[upChannel] == 0))
	{
		// Niether button has been pushed. Do nothing until it is just a single button pushed.
		return;
	}

	if (nTimeOfNextPositionUpdate[theServo] > nPgmTime)
	{
		// Too soon to do an update
		return;
	}

	nTimeOfNextPositionUpdate[theServo] = nPgmTime + rateOfChange;

	nOldPosition = motor[theServo];
	if (vexRT[downChannel] != 0)
	{
		if (rateOfChange == 0)
			nNewPosition = lowerPosition;
		else
		{
			if (lowerPosition < upperPosition)
			{
				nNewPosition = nOldPosition - kPositionAdjustmentStepSize;
				if (nNewPosition < lowerPosition)
					nNewPosition = lowerPosition;
			}
			else
			{
				nNewPosition = nOldPosition + kPositionAdjustmentStepSize;
				if (nNewPosition > lowerPosition)
					nNewPosition = lowerPosition;
			}
		}
		motor[theServo] = nNewPosition;
	}

	else 	if (vexRT[upChannel] != 0)
	{
		if (rateOfChange == 0)
			nNewPosition = upperPosition;
		else
		{
			if (lowerPosition < upperPosition)
			{
				nNewPosition = nOldPosition + kPositionAdjustmentStepSize;
				if (nNewPosition > upperPosition)
					nNewPosition = upperPosition;
			}
			else
			{
				nNewPosition = nOldPosition - kPositionAdjustmentStepSize;
				if (nNewPosition < upperPosition)
					nNewPosition = upperPosition;
			}
		}
		motor[theServo] = nNewPosition;
	}
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//																					MotorControlViaDigitalButtons
//
//  Use two buttons for "down" and "up" servo positioning or motor speed.
//
//  1. Push down button to gradually move adjust motor speed to a minimum speed of  'lowestSpeed'.
//
//  2. Push   up button to gradually adjust motor speed to position 'highestSpeed'.
//
//  If neither button is pressed, or if both buttons are pressed, then motor is stopped
//
//  'lowestSpeed' and 'highestSpeed' contain the minimum and maximum values for the servo.
//
//  'rateOfChange' determines how fast the servo position is updated.
//     -- To immediately go to the extreme positions use a value of zero.
//     -- A non-zero value indicates how many milliseconds before adjusting the servo position. Each adjustment is
//        10 counts
//     -- Typical values are in the range 10 to 50. A value of 10 will take 0.25 seconds to go from one extreme to
//        the other. A value of 50 will take 1.25 seconds.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotorControlViaDigitalButtons(TVexJoysticks downChannel,		// The joystick button that controls speed decrease
														 TVexJoysticks upChannel, 					// The joystick button that controls speed increase
														 tMotor theMotor,					// The motor that is being controlled
														 int lowestSpeed,					// The lowest  value for the motor speed
														 int highestSpeed,				// The highest value for motor speed
														 int rateOfChange)				// How fast to adjust the speed. 0 -- fastest. 50 -- 1.25 seconds to cover the compete range.
{
	const int kSpeedAdjustmentStepSize = 10;

	int nOldPosition;
	int nNewPosition;

	if ((vexRT[downChannel] != 0) && (vexRT[upChannel] != 0))
	{
		// Both buttons have been pushed. Do nothing until it is just either a single button or no pushed.
		return;
	}

	if ((vexRT[downChannel] == 0) && (vexRT[upChannel] == 0))
	{
		// Neither button has been pushed. Stop motor
		motor[theMotor] = 0;
		return;
	}

	if (nTimeOfNextPositionUpdate[theMotor] > nPgmTime)
	{
		// Too soon to do an update
		return;
	}

	nTimeOfNextPositionUpdate[theMotor] = nPgmTime + rateOfChange;

	nOldPosition = motor[theMotor];
	if (vexRT[downChannel] != 0)
	{
		if (rateOfChange == 0)
			nNewPosition = lowestSpeed;
		else
		{
			if (lowestSpeed < highestSpeed)
			{
				nNewPosition = nOldPosition - kSpeedAdjustmentStepSize;
				if (nNewPosition < lowestSpeed)
					nNewPosition = lowestSpeed;
			}
			else
			{
				nNewPosition = nOldPosition + kSpeedAdjustmentStepSize;
				if (nNewPosition > lowestSpeed)
					nNewPosition = lowestSpeed;
			}
		}
		motor[theMotor] = nNewPosition;
	}

	else 	if (vexRT[upChannel] != 0)
	{
		if (rateOfChange == 0)
			nNewPosition = highestSpeed;
		else
		{
			if (lowestSpeed < highestSpeed)
			{
				nNewPosition = nOldPosition + kSpeedAdjustmentStepSize;
				if (nNewPosition > highestSpeed)
					nNewPosition = highestSpeed;
			}
			else
			{
				nNewPosition = nOldPosition - kSpeedAdjustmentStepSize;
				if (nNewPosition < highestSpeed)
					nNewPosition = highestSpeed;
			}
		}
		motor[theMotor] = nNewPosition;
	}
}

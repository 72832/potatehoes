/*------------------------------------------*\
|*	- Natural Language for VEX Cortex		*|
\*------------------------------------------*/
#pragma once
#pragma systemFile            	// eliminates warning for "unreferenced" functions

/*
--------------- Enumerations and Defines ----------------------
*/

#define VexCortexClawbot 0
const tMotor noMotorOnPort     = (tMotor) -1;
const tMotor noMotor     = (tMotor) -1;
const tSensors noEncoderOnPort = (tSensors) -1;
#define nMaxJoyScaleValue 127.0

#define setMotorSpeed(motorPort, speedValue) motor[motorPort] = speedValue

// enum for units
typedef enum tMovementUnits
{
	degrees						= 0,
	rotations					= 1,
	milliseconds				= 2,
	seconds						= 3,
	minutes						= 4
} tMovementUnits;

/*
typedef enum tVEXCortexLCDLine
{
	line1				= 0,
	line2				= 1,
} tVEXCortexLCDLine;
*/

typedef enum tVEXCompetitionMode
{
	compDisable			= 0,
	compAuto				= 1,
	compTeleOp			= 2
} tVEXCompetitionMode;

// enum for LED (on or off):
typedef enum tLedCommand
{
	ledOff	= 0,
	ledOn	= 1,
}tLedCommand;

bool bPopulatedMotorDriveSides = false;
TMotorList internalLeftMotors;
TMotorList internalRightMotors;
short nGlobalJoyScaledValue = nMaxJoyScaleValue;
float fMotorScaleRatio[10] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
bool  bMotorI2CBased[10] = {false,false,false,false,false,false,false,false,false,false};
tVEXCompetitionMode compCurrentMode;
tVEXCompetitionMode compPreviousMode;
bool bAutonomousOneTimeFlag = false;

#define VEX_269_Standard				((float) 240.448)
#define VEX_393_Torque          ((float) 627.2)
#define VEX_393_Speed          	((float) 392.0)
#define VEX_393_Turbo           ((float) 261.333)
#define VEX_ShaftEncoder        ((float) 360.0)

void stopAllMotors();

/*
------------------- Utility Commands --------------------------
*/

void toggleMainTask()
{
	stopAllMotors();
	stopTask(0);
	sleep(2);
	startTask(0);
}

task monitorCompetitionFlags()
{
	while(true)
	{
		if(bIfiRobotDisabled) //We're disabled, we should restart "task main" and disable Auto/Tele Flags
		{
			if(compCurrentMode != compDisable)
			{
				compPreviousMode = compCurrentMode;
				compCurrentMode = compDisable;
				bAutonomousOneTimeFlag = false;
				toggleMainTask();
				displayLCDCenteredString(0, "Disabled");
			}
		}
		else //We're enabled, let's check the Autonomous Flag.
		{
			if(bIfiAutonomousMode) //Autonomous Mode, let's set the auto flag.
			{
				if(compCurrentMode != compAuto)
				{
					compPreviousMode = compCurrentMode;
					compCurrentMode = compAuto;
					toggleMainTask();
					displayLCDCenteredString(0, "Autonomous");
				}
			}
			else //TeleOp Mode, let's set the tele flag.
			{
				if(compCurrentMode != compTeleOp)
				{
					compPreviousMode = compCurrentMode;
					compCurrentMode = compTeleOp;
					toggleMainTask();
					displayLCDCenteredString(0, "TeleOp");
				}
			}
		}
		sleep(25);
	}
}

void enableCompetitionMode()
{
	startTask(monitorCompetitionFlags);
	return;
}

bool getCompetitionAutonomous()
{
	if(compCurrentMode == compAuto)
	{
		if(!bAutonomousOneTimeFlag)
		{
			bAutonomousOneTimeFlag = true;
			return true;
		}
		else
		{
			stopAllMotors();
			return false;
		}
	}
	else
	{
		return false;
	}
}

bool getCompetitionDriverControl()
{
	if(compCurrentMode == compTeleOp)
	{
		bAutonomousOneTimeFlag = false;
		return true;
	}
	else
	{
		return false;
	}
}

void setJoystickScale(short nScaledPercentage = nMaxJoyScaleValue)
{
	if(nScaledPercentage > nMaxJoyScaleValue)
	{
		nGlobalJoyScaledValue = nMaxJoyScaleValue;
	}
	else if(nScaledPercentage < 0)
	{
		nGlobalJoyScaledValue = 0;
	}
	else
	{
		nGlobalJoyScaledValue = nScaledPercentage;
	}
}

void updateMotorEncoderScale()
{
	for(tMotor i = port1; i < kNumbOfRealMotors; i++)
	{
		tSensors theEncoder =  getEncoderForMotor(i);

		//We have no associated encoder
		if(theEncoder == noEncoderOnPort)
		{
			fMotorScaleRatio[i] = 0.0;
			bMotorI2CBased[i] = false;
			SensorValue[getEncoderForMotor(i)] = 0;
		}
		//We have an I2C encoder
		else if(theEncoder >= I2C_1 && theEncoder <= I2C_8)
		{
			bMotorI2CBased[i] = true;
			resetMotorEncoder(i);
			switch(motorType[i])
			{
			case tmotorVex269_HBridge:
			case tmotorVex269_MC29:
				fMotorScaleRatio[i] = (VEX_269_Standard / 360.0);
				break;

			case tmotorVex393_HBridge:
			case tmotorVex393_MC29:
				fMotorScaleRatio[i] = (VEX_393_Torque / 360.0);
				break;

			case tmotorVex393HighSpeed_HBridge:
			case tmotorVex393HighSpeed_MC29:
				fMotorScaleRatio[i] = (VEX_393_Speed / 360.0);
				break;

			case tmotorVex393TurboSpeed_HBridge:
			case tmotorVex393TurboSpeed_MC29:
				fMotorScaleRatio[i] = (VEX_393_Turbo / 360.0);
				break;

			default:
				fMotorScaleRatio[i] = 0.0;
			}
		}
		//We have a shaft encoder
		else
		{
			bMotorI2CBased[i] = false;
			fMotorScaleRatio[i] = (VEX_ShaftEncoder / 360.0);
			SensorValue[getEncoderForMotor(i)] = 0;
		}
	}
}

void updateMotorDriveTrain()
{
	if(!bPopulatedMotorDriveSides)
	{
		updateMotorEncoderScale();
		getMotorsWithDriveSideType(driveRight, internalRightMotors);
		getMotorsWithDriveSideType(driveLeft,	 internalLeftMotors);
		bPopulatedMotorDriveSides = true;
	}
}

void setPIDEnableForDrivetrain(bool bEnable)
{
	if(bEnable)
	{
		//Turn on PID
		if(internalLeftMotors[0] != noMotor)	nMotorPIDSpeedCtrl[internalLeftMotors[0]] = RegSpeed;
		if(internalLeftMotors[1] != noMotor)	nMotorPIDSpeedCtrl[internalLeftMotors[1]] = RegSpeed;
		if(internalLeftMotors[2] != noMotor)	nMotorPIDSpeedCtrl[internalLeftMotors[2]] = RegSpeed;
		if(internalLeftMotors[3] != noMotor)	nMotorPIDSpeedCtrl[internalLeftMotors[3]] = RegSpeed;

		if(internalRightMotors[0] != noMotor)	nMotorPIDSpeedCtrl[internalRightMotors[0]] = RegSpeed;
		if(internalRightMotors[1] != noMotor)	nMotorPIDSpeedCtrl[internalRightMotors[1]] = RegSpeed;
		if(internalRightMotors[2] != noMotor)	nMotorPIDSpeedCtrl[internalRightMotors[2]] = RegSpeed;
		if(internalRightMotors[3] != noMotor)	nMotorPIDSpeedCtrl[internalRightMotors[3]] = RegSpeed;
	}
	else
	{
		//Turn off PID
		if(internalLeftMotors[0] != noMotor)	nMotorPIDSpeedCtrl[internalLeftMotors[0]] = RegIdle;
		if(internalLeftMotors[1] != noMotor)	nMotorPIDSpeedCtrl[internalLeftMotors[1]] = RegIdle;
		if(internalLeftMotors[2] != noMotor)	nMotorPIDSpeedCtrl[internalLeftMotors[2]] = RegIdle;
		if(internalLeftMotors[3] != noMotor)	nMotorPIDSpeedCtrl[internalLeftMotors[3]] = RegIdle;

		if(internalRightMotors[0] != noMotor)	nMotorPIDSpeedCtrl[internalRightMotors[0]] = RegIdle;
		if(internalRightMotors[1] != noMotor)	nMotorPIDSpeedCtrl[internalRightMotors[1]] = RegIdle;
		if(internalRightMotors[2] != noMotor)	nMotorPIDSpeedCtrl[internalRightMotors[2]] = RegIdle;
		if(internalRightMotors[3] != noMotor)	nMotorPIDSpeedCtrl[internalRightMotors[3]] = RegIdle;
	}
}

void setPIDforMotor(tMotor theMotor, bool bEnable)
{
	if(bEnable)
		nMotorPIDSpeedCtrl[theMotor] = RegSpeed;
	else
		nMotorPIDSpeedCtrl[theMotor] = RegIdle;
}

/*
------------------- Low Level Commands ------------------------
*/

void lowLevelMoveDistance(const float quantity, short nLeftSpeed, short nRightSpeed, const tMovementUnits nUnitType)
{
	long nConvertedUnitsLeft = 0;
	long nConvertedUnitsRight = 0;
	bool bMotorsStillRunning = false;

	setPIDEnableForDrivetrain(true);

	if(nUnitType == degrees)
	{
		nConvertedUnitsLeft = quantity;
		nConvertedUnitsRight = quantity;
	}
	else if(nUnitType == rotations)
	{
		nConvertedUnitsLeft = quantity * 360.0;
		nConvertedUnitsRight = quantity * 360.0;
	}
	else
	{
		return;
	}

	updateMotorDriveTrain();

	if(nLeftSpeed < 0)
	{
		nConvertedUnitsLeft *= -1;
	}

	if(nRightSpeed < 0)
	{
		nConvertedUnitsRight *= -1;
	}

	if(internalLeftMotors[0] != noMotor)
	{
		if(bMotorI2CBased[internalLeftMotors[0]])
		{
			resetMotorEncoder(internalLeftMotors[0]);
			moveMotorTarget(internalLeftMotors[0], nConvertedUnitsLeft * fMotorScaleRatio[internalLeftMotors[0]], nLeftSpeed);
		}
		else
		{
			SensorValue[getEncoderForMotor(internalLeftMotors[0])] = 0;
			setMotorSpeed(internalLeftMotors[0], nLeftSpeed);
		}
	}
	if(internalLeftMotors[1] != noMotor)
	{
		if(bMotorI2CBased[internalLeftMotors[1]])
		{
			resetMotorEncoder(internalLeftMotors[1]);
			moveMotorTarget(internalLeftMotors[1], nConvertedUnitsLeft * fMotorScaleRatio[internalLeftMotors[1]], nLeftSpeed);
		}
		else
		{
			SensorValue[getEncoderForMotor(internalLeftMotors[1])] = 0;
			setMotorSpeed(internalLeftMotors[1], nLeftSpeed);
		}
	}
	if(internalLeftMotors[2] != noMotor)
	{
		if(bMotorI2CBased[internalLeftMotors[2]])
		{
			resetMotorEncoder(internalLeftMotors[2]);
			moveMotorTarget(internalLeftMotors[2], nConvertedUnitsLeft * fMotorScaleRatio[internalLeftMotors[2]], nLeftSpeed);
		}
		else
		{
			SensorValue[getEncoderForMotor(internalLeftMotors[2])] = 0;
			setMotorSpeed(internalLeftMotors[2], nLeftSpeed);
		}
	}
	if(internalLeftMotors[3] != noMotor)
	{
		if(bMotorI2CBased[internalLeftMotors[3]])
		{
			resetMotorEncoder(internalLeftMotors[3]);
			moveMotorTarget(internalLeftMotors[3], nConvertedUnitsLeft * fMotorScaleRatio[internalLeftMotors[3]], nLeftSpeed);
		}
		else
		{
			SensorValue[getEncoderForMotor(internalLeftMotors[3])] = 0;
			setMotorSpeed(internalLeftMotors[3], nLeftSpeed);
		}
	}

	if(internalRightMotors[0] != noMotor)
	{
		if(bMotorI2CBased[internalRightMotors[0]])
		{
			resetMotorEncoder(internalRightMotors[0]);
			moveMotorTarget(internalRightMotors[0], nConvertedUnitsRight * fMotorScaleRatio[internalRightMotors[0]], nRightSpeed);
		}
		else
		{
			SensorValue[getEncoderForMotor(internalRightMotors[0])] = 0;
			setMotorSpeed(internalRightMotors[0], nRightSpeed);
		}
	}
	if(internalRightMotors[1] != noMotor)
	{
		if(bMotorI2CBased[internalRightMotors[1]])
		{
			resetMotorEncoder(internalRightMotors[1]);
			moveMotorTarget(internalRightMotors[1], nConvertedUnitsRight * fMotorScaleRatio[internalRightMotors[1]], nRightSpeed);
		}
		else
		{
			SensorValue[getEncoderForMotor(internalRightMotors[1])] = 0;
			setMotorSpeed(internalRightMotors[1], nRightSpeed);
		}
	}
	if(internalRightMotors[2] != noMotor)
	{
		if(bMotorI2CBased[internalRightMotors[2]])
		{
			resetMotorEncoder(internalRightMotors[2]);
			moveMotorTarget(internalRightMotors[2], nConvertedUnitsRight * fMotorScaleRatio[internalRightMotors[2]], nRightSpeed);
		}
		else
		{
			SensorValue[getEncoderForMotor(internalRightMotors[2])] = 0;
			setMotorSpeed(internalRightMotors[2], nRightSpeed);
		}
	}
	if(internalRightMotors[3] != noMotor)
	{
		if(bMotorI2CBased[internalRightMotors[3]])
		{
			resetMotorEncoder(internalRightMotors[2]);
			moveMotorTarget(internalRightMotors[3], nConvertedUnitsRight * fMotorScaleRatio[internalRightMotors[3]], nRightSpeed);
		}
		else
		{
			SensorValue[getEncoderForMotor(internalRightMotors[3])] = 0;
			setMotorSpeed(internalRightMotors[3], nRightSpeed);
		}
	}

	sleep(50);

	while(true)
	{
		bMotorsStillRunning = false;
		sleep(1);

		//Left Motor 1 (0)
		if(internalLeftMotors[0] != noMotor)
		{
			if(bMotorI2CBased[internalLeftMotors[0]])
			{
				if(!getMotorTargetCompleted(internalLeftMotors[0]))
					bMotorsStillRunning = true;
			}
			else
			{
				if(abs(SensorValue[getEncoderForMotor(internalLeftMotors[0])]) < abs(nConvertedUnitsLeft * fMotorScaleRatio[internalLeftMotors[0]]))
					bMotorsStillRunning = true;
				else
					motor[internalLeftMotors[0]] = 0;
			}
		}

		//Left Motor 2 (1)
		if(internalLeftMotors[1] != noMotor)
		{
			if(bMotorI2CBased[internalLeftMotors[1]])
			{
				if(!getMotorTargetCompleted(internalLeftMotors[1]))
					bMotorsStillRunning = true;
			}
			else
			{
				if(abs(SensorValue[getEncoderForMotor(internalLeftMotors[1])]) < abs(nConvertedUnitsLeft * fMotorScaleRatio[internalLeftMotors[1]]))
					bMotorsStillRunning = true;
				else
					motor[internalLeftMotors[1]] = 0;
			}
		}

		//Left Motor 3 (2)
		if(internalLeftMotors[2] != noMotor)
		{
			if(bMotorI2CBased[internalLeftMotors[2]])
			{
				if(!getMotorTargetCompleted(internalLeftMotors[2]))
					bMotorsStillRunning = true;
			}
			else
			{
				if(abs(SensorValue[getEncoderForMotor(internalLeftMotors[2])]) < abs(nConvertedUnitsLeft * fMotorScaleRatio[internalLeftMotors[2]]))
					bMotorsStillRunning = true;
				else
					motor[internalLeftMotors[2]] = 0;
			}
		}

		//Left Motor 4 (3)
		if(internalLeftMotors[3] != noMotor)
		{
			if(bMotorI2CBased[internalLeftMotors[3]])
			{
				if(!getMotorTargetCompleted(internalLeftMotors[3]))
					bMotorsStillRunning = true;
			}
			else
			{
				if(abs(SensorValue[getEncoderForMotor(internalLeftMotors[3])]) < abs(nConvertedUnitsLeft * fMotorScaleRatio[internalLeftMotors[3]]))
					bMotorsStillRunning = true;
				else
					motor[internalLeftMotors[3]] = 0;
			}
		}

		//Right Motor 1 (0)
		if(internalRightMotors[0] != noMotor)
		{
			if(bMotorI2CBased[internalRightMotors[0]])
			{
				if(!getMotorTargetCompleted(internalRightMotors[0]))
					bMotorsStillRunning = true;
			}
			else
			{
				if(abs(SensorValue[getEncoderForMotor(internalRightMotors[0])]) < abs(nConvertedUnitsRight * fMotorScaleRatio[internalRightMotors[0]]))
					bMotorsStillRunning = true;
				else
					motor[internalRightMotors[0]] = 0;
			}
		}

		//Right Motor 2 (1)
		if(internalRightMotors[1] != noMotor)
		{
			if(bMotorI2CBased[internalRightMotors[1]])
			{
				if(!getMotorTargetCompleted(internalRightMotors[1]))
					bMotorsStillRunning = true;
			}
			else
			{
				if(abs(SensorValue[getEncoderForMotor(internalRightMotors[1])]) < abs(nConvertedUnitsRight * fMotorScaleRatio[internalRightMotors[1]]))
					bMotorsStillRunning = true;
				else
					motor[internalRightMotors[1]] = 0;
			}
		}

		//Right Motor 3 (2)
		if(internalRightMotors[2] != noMotor)
		{
			if(bMotorI2CBased[internalRightMotors[2]])
			{
				if(!getMotorTargetCompleted(internalRightMotors[2]))
					bMotorsStillRunning = true;
			}
			else
			{
				if(abs(SensorValue[getEncoderForMotor(internalRightMotors[2])]) < abs(nConvertedUnitsRight * fMotorScaleRatio[internalRightMotors[2]]))
					bMotorsStillRunning = true;
				else
					motor[internalRightMotors[2]] = 0;
			}
		}

		//Right Motor 4 (3)
		if(internalRightMotors[3] != noMotor)
		{
			if(bMotorI2CBased[internalRightMotors[3]])
			{
				if(!getMotorTargetCompleted(internalRightMotors[3]))
					bMotorsStillRunning = true;
			}
			else
			{
				if(abs(SensorValue[getEncoderForMotor(internalRightMotors[3])]) < abs(nConvertedUnitsRight * fMotorScaleRatio[internalRightMotors[3]]))
					bMotorsStillRunning = true;
				else
					motor[internalRightMotors[3]] = 0;
			}
		}

		if(!bMotorsStillRunning)
			break;
	}

	sleep(100);
}

void setMotorSpeeds(short nRightSideSpeed, short nLeftSideSpeed)
{
	if(internalRightMotors[0] != noMotor)	setMotorSpeed(internalRightMotors[0], nRightSideSpeed);
	if(internalRightMotors[1] != noMotor)	setMotorSpeed(internalRightMotors[1], nRightSideSpeed);
	if(internalRightMotors[2] != noMotor)	setMotorSpeed(internalRightMotors[2], nRightSideSpeed);
	if(internalRightMotors[3] != noMotor)	setMotorSpeed(internalRightMotors[3], nRightSideSpeed);

	if(internalLeftMotors[0] != noMotor)	setMotorSpeed(internalLeftMotors[0], nLeftSideSpeed);
	if(internalLeftMotors[1] != noMotor)	setMotorSpeed(internalLeftMotors[1], nLeftSideSpeed);
	if(internalLeftMotors[2] != noMotor)	setMotorSpeed(internalLeftMotors[2], nLeftSideSpeed);
	if(internalLeftMotors[3] != noMotor)	setMotorSpeed(internalLeftMotors[3], nLeftSideSpeed);
}

void lowLevelMoveTiming(float quantity, const short nLeftSpeed, const short nRightSpeed, const tMovementUnits nUnitType)
{
	updateMotorDriveTrain();

	setPIDEnableForDrivetrain(false);

	setMotorSpeeds(nRightSpeed, nLeftSpeed);

	if(nUnitType == milliseconds)
		sleep(quantity);
	else if(nUnitType == seconds)
		sleep(quantity * 1000);
	else if(nUnitType == minutes)
		sleep(quantity * 1000 * 60);
	else
		sleep(5);

	setMotorSpeeds(0, 0);

	sleep(25);
}

/*
------------------- Wait Commands -------------------------
*/
void wait(const float quantity = 1.0, const tMovementUnits unitType = seconds)
{
	if(unitType == seconds)
		sleep(quantity * 1000.0);
	else if(unitType == minutes)
		sleep(quantity * 1000.0 * 60.0);
	else if(unitType == milliseconds)
		sleep(quantity);
}



/*
------------------- Discrete Movement -------------------------
*/

void forward(const float quantity = 1.0, const tMovementUnits unitType = rotations, const short speed = 50)
{
	short adjustedSpeed = abs(speed) * 1.27;
	float adjustedQuantity = abs(quantity);

	switch(unitType)
	{
	case rotations:
	case degrees:
		lowLevelMoveDistance(adjustedQuantity, adjustedSpeed, adjustedSpeed, unitType);
		break;

	case milliseconds:
	case seconds:
	case minutes:
		lowLevelMoveTiming(adjustedQuantity, adjustedSpeed, adjustedSpeed, unitType);
		break;
	}
}

void backward(const float quantity = 1.0, const tMovementUnits unitType = rotations, const short speed = 50)
{
	short adjustedSpeed = abs(speed) * -1.27;
	float adjustedQuantity = abs(quantity);

	switch(unitType)
	{
	case rotations:
	case degrees:
		lowLevelMoveDistance(adjustedQuantity, adjustedSpeed, adjustedSpeed, unitType);
		break;

	case milliseconds:
	case seconds:
	case minutes:
		lowLevelMoveTiming(adjustedQuantity, adjustedSpeed, adjustedSpeed, unitType);
		break;
	}
}

void turnLeft(const float quantity = 1.0, const tMovementUnits unitType = rotations, const short speed = 50)
{
	short adjustedSpeed = abs(speed) * 1.27;
	float adjustedQuantity = abs(quantity);

	switch(unitType)
	{
	case rotations:
	case degrees:
		lowLevelMoveDistance(adjustedQuantity, -adjustedSpeed, adjustedSpeed, unitType);
		break;

	case milliseconds:
	case seconds:
	case minutes:
		lowLevelMoveTiming(adjustedQuantity, -adjustedSpeed, adjustedSpeed, unitType);
		break;
	}
}

void turnRight(const float quantity = 1.0, const tMovementUnits unitType = rotations, const short speed = 50)
{
	short adjustedSpeed = abs(speed) * 1.27;
	float adjustedQuantity = abs(quantity);

	switch(unitType)
	{
	case rotations:
	case degrees:
		lowLevelMoveDistance(adjustedQuantity, adjustedSpeed, -adjustedSpeed, unitType);
		break;

	case milliseconds:
	case seconds:
	case minutes:
		lowLevelMoveTiming(adjustedQuantity, adjustedSpeed, -adjustedSpeed, unitType);
		break;
	}
}

void moveMotor(const tMotor motorPort = port10, const float quantity = 1.0, const tMovementUnits unitType = rotations, const short speed = 50)
{
	short adjustedSpeed = abs(speed);
	float adjustedQuantity = abs(quantity);

	updateMotorDriveTrain();

	switch(unitType)
	{
		case rotations:
		{
			if(bMotorI2CBased[motorPort])
			{
				if(speed < 0 || quantity < 0)
					moveMotorTarget(motorPort, ((-quantity * 360.0) * fMotorScaleRatio[motorPort]), -adjustedSpeed);
				else
					moveMotorTarget(motorPort, ((quantity * 360.0) * fMotorScaleRatio[motorPort]), adjustedSpeed);

				waitUntilMotorStop(motorPort);
			}
			else
			{
				SensorValue[getEncoderForMotor(motorPort)] = 0;

				if(speed < 0 || quantity < 0)
				{
					SensorValue[getEncoderForMotor(motorPort)] = 0;
					setMotorSpeed(motorPort, -adjustedSpeed);
					while(SensorValue[getEncoderForMotor(motorPort)] > ((-quantity * 360.0) * fMotorScaleRatio[motorPort])) sleep(1);
				}
				else
				{
					SensorValue[getEncoderForMotor(motorPort)] = 0;
					setMotorSpeed(motorPort, adjustedSpeed);
					while(SensorValue[getEncoderForMotor(motorPort)] < ((quantity * 360.0) * fMotorScaleRatio[motorPort])) sleep(1);
				}
			}

			setMotorSpeed(motorPort, 0);
			break;
		}

		//Degrees - moves in reverse if speed and/or quantity are set negative
		case degrees:
		{
			if(bMotorI2CBased[motorPort])
			{
				if(speed < 0 || quantity < 0)
					moveMotorTarget(motorPort, ((-quantity) * fMotorScaleRatio[motorPort]), -adjustedSpeed);
				else
					moveMotorTarget(motorPort, ((quantity) * fMotorScaleRatio[motorPort]), adjustedSpeed);

				waitUntilMotorStop(motorPort);
			}
			else
			{
				SensorValue[getEncoderForMotor(motorPort)] = 0;

				if(speed < 0 || quantity < 0)
				{
					SensorValue[getEncoderForMotor(motorPort)] = 0;
					setMotorSpeed(motorPort, -adjustedSpeed);
					while(SensorValue[getEncoderForMotor(motorPort)] > ((-quantity) * fMotorScaleRatio[motorPort])) sleep(1);
				}
				else
				{
					SensorValue[getEncoderForMotor(motorPort)] = 0;
					setMotorSpeed(motorPort, adjustedSpeed);
					while(SensorValue[getEncoderForMotor(motorPort)] < ((quantity) * fMotorScaleRatio[motorPort])) sleep(1);
				}
			}

			setMotorSpeed(motorPort, 0);
			break;
		}

		//Milliseconds
		case milliseconds:
			setMotorSpeed(motorPort, speed);
			sleep(adjustedQuantity);
			setMotorSpeed(motorPort, 0);
			break;

		//Seconds
		case seconds:
			setMotorSpeed(motorPort, speed);
			sleep(adjustedQuantity * 1000.0);
			setMotorSpeed(motorPort, 0);
			break;

		//Minutes
		case minutes:
			setMotorSpeed(motorPort, speed);
			sleep(adjustedQuantity * 1000.0 * 60.0);
			setMotorSpeed(motorPort, 0);
			break;
	}

	sleep(250);
}

/*
------------------- Constant Movements -------------------------
*/

void setMotor(const tMotor motorPort = port1, const short speed = 50)
{
	setMotorSpeed(motorPort, speed);
}

void stopMotor(const tMotor motorPort = port1)
{
	setMotorSpeed(motorPort, 0);
}

void stopAllMotors()
{
	for (tMotor nPortIndex = port1; nPortIndex <= kNumbOfRealMotors; ++nPortIndex)
	{
		setMotorSpeed(nPortIndex, 0);
	}
}

void setMultipleMotors(const short speed = 50, const tMotor firstMotor = port1, const tMotor secondMotor = noMotor, const tMotor thirdMotor = noMotor, const tMotor fourthMotor = noMotor)
{
	if(firstMotor != noMotor) 	setMotorSpeed(firstMotor, speed);
	if(secondMotor != noMotor) 	setMotorSpeed(secondMotor, speed);
	if(thirdMotor != noMotor) 	setMotorSpeed(thirdMotor, speed);
	if(fourthMotor != noMotor) 	setMotorSpeed(fourthMotor, speed);
}

void stopMultipleMotors(const tMotor firstMotor = port1, const tMotor secondMotor = noMotor, const tMotor thirdMotor = noMotor, const tMotor fourthMotor = noMotor)
{
	if(firstMotor != noMotor) 	setMotorSpeed(firstMotor, 0);
	if(secondMotor != noMotor) 	setMotorSpeed(secondMotor, 0);
	if(thirdMotor != noMotor) 	setMotorSpeed(thirdMotor, 0);
	if(fourthMotor != noMotor) 	setMotorSpeed(fourthMotor, 0);
}

/*
------------------- Timer Commands -------------------------
*/

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
		return ((float) time1[currentTimer]) / 1000.0 / 60.0;

	return 0.0;
}

void lineTrackLeft(const tSensors sensorName = in1, const short threshold = 50, const short speedPrimary = 50, const short speedSecondary = 0)
{
	updateMotorDriveTrain();

	if(SensorValue[sensorName] > threshold)
		setMotorSpeeds(speedSecondary, speedPrimary);
	else
		setMotorSpeeds(speedPrimary, speedSecondary);
}

void lineTrackRight(const tSensors sensorName = in1, const short threshold = 50, const short speedPrimary = 50, const short speedSecondary = 0)
{
	updateMotorDriveTrain();

	if(SensorValue[sensorName] < threshold)
		setMotorSpeeds(speedSecondary, speedPrimary);
	else
		setMotorSpeeds(speedPrimary, speedSecondary);
}


void resetSensor(const tSensors sensorName = in1)
{
	SensorValue[sensorName] = 0;
}

/*
------------------- Remote Control Movements -------------------------
*/

void tankControl(TVexJoysticks leftJoystick = Ch3, TVexJoysticks rightJoystick = Ch2, short threshold = 10)
{
	short nRightSideSpeed;
	short nLeftSideSpeed;

	updateMotorDriveTrain();

	setPIDEnableForDrivetrain(false);

	nRightSideSpeed = getJoystickValue(rightJoystick);
	nLeftSideSpeed = getJoystickValue(leftJoystick);

	if(nGlobalJoyScaledValue != nMaxJoyScaleValue)
	{
		nRightSideSpeed = nRightSideSpeed * (nGlobalJoyScaledValue / nMaxJoyScaleValue);
		nLeftSideSpeed 	= nLeftSideSpeed * (nGlobalJoyScaledValue / nMaxJoyScaleValue);
	}

	if(abs(getJoystickValue(rightJoystick)) <= abs(threshold))
		nRightSideSpeed = 0;

	if(abs(getJoystickValue(leftJoystick)) <= abs(threshold))
		nLeftSideSpeed = 0;

	setMotorSpeeds(nRightSideSpeed, nLeftSideSpeed);
}

void arcadeControl(TVexJoysticks verticalJoystick = Ch2, TVexJoysticks horizontalJoystick = Ch1, short threshold = 10)
{
	short nRightSideSpeed = 0;
	short nLeftSideSpeed = 0;

	updateMotorDriveTrain();

	setPIDEnableForDrivetrain(false);

	if(abs(vexRT[verticalJoystick]) <= abs(threshold) && abs(vexRT[horizontalJoystick]) <= abs(threshold))
	{
		nRightSideSpeed = 0;
		nLeftSideSpeed = 0;
	}
	else
	{
		nRightSideSpeed = ((getJoystickValue(verticalJoystick) - getJoystickValue(horizontalJoystick)) / 2);
		nLeftSideSpeed = ((getJoystickValue(verticalJoystick) + getJoystickValue(horizontalJoystick)) / 2);

		if (nGlobalJoyScaledValue != nMaxJoyScaleValue)
		{
			nRightSideSpeed = nRightSideSpeed * (nGlobalJoyScaledValue / nMaxJoyScaleValue);
			nLeftSideSpeed = nLeftSideSpeed * (nGlobalJoyScaledValue / nMaxJoyScaleValue);
		}
	}

	setMotorSpeeds(nRightSideSpeed, nLeftSideSpeed);
}

void armControl(tMotor armMotorPort = port10, TVexJoysticks upButton = Btn5U, TVexJoysticks downButton = Btn5D, short armSpeed = 75)
{
	if(vexRT[upButton])
	{
		setMotorSpeed(armMotorPort, armSpeed);
	}
	else if(vexRT[downButton])
	{
		setMotorSpeed(armMotorPort, -armSpeed);
	}
	else
	{
		setMotorSpeed(armMotorPort, 0);
	}
}

void buttonControl(tMotor armMotorPort = port10, TVexJoysticks upButton = Btn5U, TVexJoysticks downButton = Btn5D, short armSpeed = 75)
{
	if(vexRT[upButton])
	{
		setMotorSpeed(armMotorPort, armSpeed);
	}
	else if(vexRT[downButton])
	{
		setMotorSpeed(armMotorPort, -armSpeed);
	}
	else
	{
		setMotorSpeed(armMotorPort, 0);
	}
}

void joystickControl(tMotor armMotorPort = port10, TVexJoysticks motorJoystick = Ch2, short threshold = 10)
{
	if(abs(vexRT[motorJoystick]) <= abs(threshold))
	{
		setMotorSpeed(armMotorPort, 0);
	}
	else
	{
		setMotorSpeed(armMotorPort, vexRT[motorJoystick]);
	}
}

//-------------------------------------------| TurnLEDON |--------------------------------------------
#ifndef _TURNLEDON_H_GUARD
#define _TURNLEDON_H_GUARD

void turnLEDOn(tSensors digitalPort = dgtl12)
{
    if( digitalPort >= dgtl1 && digitalPort <= dgtl12 )
	    SensorValue[digitalPort] = (word)ledOn;
}

#endif
//----------------------------------------------------------------------------------------------------

//-------------------------------------------| TurnLEDOFF |-------------------------------------------
#ifndef _TURNLEDOFF_H_GUARD
#define _TURNLEDOFF_H_GUARD

void turnLEDOff(tSensors digitalPort = dgtl12)
{
    if( digitalPort >= dgtl1 && digitalPort <= dgtl12 )
        SensorValue[digitalPort] = (word)ledOff;
}

#endif
//----------------------------------------------------------------------------------------------------

/*
------------------- Datalog wrappers -------------------------
*/

// These globals are used by a task included in the NatLang_Datalog.c
// file
static long								  _datalogUpdateInterval = 50;
static long 								_datalogUpdatePeriod   = 0;

void
datalogRate( const float quantity = 50.0, const tMovementUnits unitType = milliseconds)
{
	long	tmpPeriod;
	
	if(unitType == seconds)
		tmpPeriod = quantity * 1000.0;
	else if(unitType == minutes)
		tmpPeriod = quantity * 1000.0 * 60.0;
	else if(unitType == milliseconds)
		tmpPeriod = quantity;
		
	if( tmpPeriod >= 10 )
		_datalogUpdateInterval = tmpPeriod;
	else
		_datalogUpdateInterval = 10;
}

void 
datalogStart(const float quantity = 5.0, const tMovementUnits unitType = seconds)
{
	if(unitType == seconds)
		_datalogUpdatePeriod = quantity * 1000.0;
	else if(unitType == minutes)
		_datalogUpdatePeriod = quantity * 1000.0 * 60.0;
	else if(unitType == milliseconds)
		_datalogUpdatePeriod = quantity;
}

void
datalogStop()
{
	_datalogUpdatePeriod = 0;
}


/*------------------------------------------*\
|*	- Natural Language for VEX Cortex		*|
\*------------------------------------------*/
#pragma once
#pragma systemFile            	// eliminates warning for "unreferenced" functions

#include "JoystickDriver.c"

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
	degrees							= 0,
	rotations						= 1,
	milliseconds				= 2,
	seconds							= 3,
	minutes							= 4
} tMovementUnits;

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

#define VEX_269_Standard				((float) 240.448)
#define VEX_393_Torque          ((float) 627.2)
#define VEX_393_Speed          	((float) 392.0)
#define VEX_393_Turbo           ((float) 261.333)
#define VEX_ShaftEncoder        ((float) 360.0)

void stopAllMotors();

/*
------------------- Utility Commands --------------------------
*/

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

/*
------------------- Low Level Commands ------------------------
*/

void lowLevelMoveDistance(const float quantity, short nLeftSpeed, short nRightSpeed, const tMovementUnits nUnitType)
{
	long nConvertedUnitsLeft = 0;
	long nConvertedUnitsRight = 0;
	bool bMotorsStillRunning = false;

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
			nMotorEncoder[internalLeftMotors[0]] = 0;
			setMotorSpeed(internalLeftMotors[0], nLeftSpeed);
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
			nMotorEncoder[internalLeftMotors[1]] = 0;
			setMotorSpeed(internalLeftMotors[1], nLeftSpeed);
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
			nMotorEncoder[internalLeftMotors[2]] = 0;
			setMotorSpeed(internalLeftMotors[2], nLeftSpeed);
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
			nMotorEncoder[internalLeftMotors[3]] = 0;
			setMotorSpeed(internalLeftMotors[3], nLeftSpeed);
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
			nMotorEncoder[internalRightMotors[0]] = 0;
			setMotorSpeed(internalRightMotors[0], nRightSpeed);
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
			nMotorEncoder[internalRightMotors[1]] = 0;
			setMotorSpeed(internalRightMotors[1], nRightSpeed);
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
			nMotorEncoder[internalRightMotors[2]] = 0;
			setMotorSpeed(internalRightMotors[2], nRightSpeed);
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
			nMotorEncoder[internalRightMotors[3]] = 0;
			setMotorSpeed(internalRightMotors[3], nRightSpeed);
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
				if(abs(nMotorEncoder[internalLeftMotors[0]]) < abs(nConvertedUnitsLeft * fMotorScaleRatio[internalLeftMotors[0]]))
					bMotorsStillRunning = true;
				else
					motor[internalLeftMotors[0]] = 0;
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
				if(abs(nMotorEncoder[internalLeftMotors[1]]) < abs(nConvertedUnitsLeft * fMotorScaleRatio[internalLeftMotors[1]]))
					bMotorsStillRunning = true;
				else
					motor[internalLeftMotors[1]] = 0;
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
				if(abs(nMotorEncoder[internalLeftMotors[2]]) < abs(nConvertedUnitsLeft * fMotorScaleRatio[internalLeftMotors[2]]))
					bMotorsStillRunning = true;
				else
					motor[internalLeftMotors[2]] = 0;
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
				if(abs(nMotorEncoder[internalLeftMotors[3]]) < abs(nConvertedUnitsLeft * fMotorScaleRatio[internalLeftMotors[3]]))
					bMotorsStillRunning = true;
				else
					motor[internalLeftMotors[3]] = 0;
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
				if(abs(nMotorEncoder[internalRightMotors[0]]) < abs(nConvertedUnitsLeft * fMotorScaleRatio[internalRightMotors[0]]))
					bMotorsStillRunning = true;
				else
					motor[internalRightMotors[0]] = 0;
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
				if(abs(nMotorEncoder[internalRightMotors[1]]) < abs(nConvertedUnitsLeft * fMotorScaleRatio[internalRightMotors[1]]))
					bMotorsStillRunning = true;
				else
					motor[internalRightMotors[1]] = 0;
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
				if(abs(nMotorEncoder[internalRightMotors[2]]) < abs(nConvertedUnitsLeft * fMotorScaleRatio[internalRightMotors[2]]))
					bMotorsStillRunning = true;
				else
					motor[internalRightMotors[2]] = 0;
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
				if(abs(nMotorEncoder[internalRightMotors[3]]) < abs(nConvertedUnitsLeft * fMotorScaleRatio[internalRightMotors[3]]))
					bMotorsStillRunning = true;
				else
					motor[internalRightMotors[3]] = 0;
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

void moveMotor(const tMotor motorPort = port10, const float quantity = 1.0, const tMovementUnits unitType = rotations, const short speed = 75)
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
				{
					nMotorEncoder[motorPort] = 0;
					setMotorSpeed(motorPort, -(adjustedSpeed*1.27));
					while(nMotorEncoder[motorPort] > ((-quantity * 360.0) * fMotorScaleRatio[motorPort])) sleep(1);
				}
				else
				{
					nMotorEncoder[motorPort] = 0;
					setMotorSpeed(motorPort, (adjustedSpeed*1.27));
					while(nMotorEncoder[motorPort] < ((quantity * 360.0) * fMotorScaleRatio[motorPort])) sleep(1);
				}
			}
			else
			{
				SensorValue[getEncoderForMotor(motorPort)] = 0;

				if(speed < 0 || quantity < 0)
				{
					SensorValue[getEncoderForMotor(motorPort)] = 0;
					setMotorSpeed(motorPort, -(adjustedSpeed*1.27));
					while(SensorValue[getEncoderForMotor(motorPort)] > ((-quantity * 360.0) * fMotorScaleRatio[motorPort])) sleep(1);
				}
				else
				{
					SensorValue[getEncoderForMotor(motorPort)] = 0;
					setMotorSpeed(motorPort, (adjustedSpeed*1.27));
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
				{
					nMotorEncoder[motorPort] = 0;
					setMotorSpeed(motorPort, -(adjustedSpeed*1.27));
					while(nMotorEncoder[motorPort] > ((-quantity) * fMotorScaleRatio[motorPort])) sleep(1);
				}
				else
				{
					nMotorEncoder[motorPort] = 0;
					setMotorSpeed(motorPort, (adjustedSpeed*1.27));
					while(nMotorEncoder[motorPort] < ((quantity) * fMotorScaleRatio[motorPort])) sleep(1);
				}
			}
			else
			{
				SensorValue[getEncoderForMotor(motorPort)] = 0;

				if(speed < 0 || quantity < 0)
				{
					SensorValue[getEncoderForMotor(motorPort)] = 0;
					setMotorSpeed(motorPort, -(adjustedSpeed*1.27));
					while(SensorValue[getEncoderForMotor(motorPort)] > ((-quantity) * fMotorScaleRatio[motorPort])) sleep(1);
				}
				else
				{
					SensorValue[getEncoderForMotor(motorPort)] = 0;
					setMotorSpeed(motorPort, (adjustedSpeed*1.27));
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
		return (((float)time1[currentTimer] / 1000.0) / 60.0);

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

void tankControl(TJoystickAxis leftJoystick = JoyY1, TJoystickAxis rightJoystick = JoyY2, short threshold = 10)
{
	short nVirtualRightJoystick = 0;
	short nVirtualLeftJoystick = 0;
	getJoystickSettings(joystick);
	updateMotorDriveTrain();

	switch(rightJoystick)
	{
	case JoyX1:    nVirtualRightJoystick = joystick.joy1_x1;     break;
	case JoyY1:    nVirtualRightJoystick = joystick.joy1_y1;     break;
  case JoyX2:    nVirtualRightJoystick = joystick.joy1_x2;     break;
  case JoyY2:    nVirtualRightJoystick = joystick.joy1_y2;     break;
	default:       nVirtualRightJoystick = 0;
	}

	switch(leftJoystick)
	{
	case JoyX1:    nVirtualLeftJoystick = joystick.joy1_x1;      break;
	case JoyY1:    nVirtualLeftJoystick = joystick.joy1_y1;      break;
  case JoyX2:    nVirtualLeftJoystick = joystick.joy1_x2;      break;
  case JoyY2:    nVirtualLeftJoystick = joystick.joy1_y2;      break;
	default:	   nVirtualLeftJoystick = 0;
	}

	if(nGlobalJoyScaledValue != nMaxJoyScaleValue)
	{
		nVirtualRightJoystick = nVirtualRightJoystick * (nGlobalJoyScaledValue / 127.0);
		nVirtualLeftJoystick  = nVirtualLeftJoystick * (nGlobalJoyScaledValue / 127.0);
	}

	short nRightSideSpeed;
	short nLeftSideSpeed;
	
	if (abs(nVirtualRightJoystick) <= abs(threshold))
		nRightSideSpeed = 0;
	else
		nRightSideSpeed = nVirtualRightJoystick;

	if (abs(nVirtualLeftJoystick) <= abs(threshold))
		nLeftSideSpeed = 0;
	else
		nLeftSideSpeed = nVirtualLeftJoystick;

	setMotorSpeeds(nRightSideSpeed, nLeftSideSpeed);
}

void arcadeControl(TJoystickAxis verticalJoystick = JoyY2, TJoystickAxis horizontalJoystick = JoyX2, short threshold = 10)
{
	short nVirtualVerticalJoystick = 0;
	short nVirtualHorizontalJoystick = 0;
	short nRightSideSpeed = 0;
	short nLeftSideSpeed = 0;

	updateMotorDriveTrain();
	getJoystickSettings(joystick);

	switch (verticalJoystick)
	{
	case JoyX1:    nVirtualVerticalJoystick = joystick.joy1_x1;     break;
	case JoyY1:    nVirtualVerticalJoystick = joystick.joy1_y1;	    break;
  case JoyX2:    nVirtualVerticalJoystick = joystick.joy1_x2;     break;
  case JoyY2:    nVirtualVerticalJoystick = joystick.joy1_y2;     break;
	default:       nVirtualVerticalJoystick = 0;
	}

	switch (horizontalJoystick)
	{
	case JoyX1:    nVirtualHorizontalJoystick = joystick.joy1_x1;   break;
	case JoyY1:    nVirtualHorizontalJoystick = joystick.joy1_y1;   break;
  case JoyX2:    nVirtualHorizontalJoystick = joystick.joy1_x2;   break;
  case JoyY2:    nVirtualHorizontalJoystick = joystick.joy1_y2;   break;
	default:       nVirtualHorizontalJoystick = 0;
	}

	if(abs(nVirtualVerticalJoystick) <= abs(threshold) && abs(nVirtualHorizontalJoystick) <= abs(threshold))
	{
		nRightSideSpeed = 0;
		nLeftSideSpeed = 0;
	}
	else
	{
		nRightSideSpeed = ((nVirtualVerticalJoystick - nVirtualHorizontalJoystick) / 2);
		nLeftSideSpeed = ((nVirtualVerticalJoystick + nVirtualHorizontalJoystick) / 2);

		if (nGlobalJoyScaledValue != nMaxJoyScaleValue)
		{
			nRightSideSpeed = nRightSideSpeed * (nGlobalJoyScaledValue / 127.0);
			nLeftSideSpeed = nLeftSideSpeed * (nGlobalJoyScaledValue / 127.0);
		}
	}

	setMotorSpeeds(nRightSideSpeed, nLeftSideSpeed);
}

void armControl(tMotor armMotorPort = port1, TJoystickButtons upButton = Btn5, TJoystickButtons downButton = Btn7, short armSpeed = 75)
{
	bool bVirtualUpButton = false;
	bool bVirtualDownButton = false;
	getJoystickSettings(joystick);

	switch (upButton)
	{
		case Btn1:     bVirtualUpButton = joy1Btn(1);     break;
		case Btn2:     bVirtualUpButton = joy1Btn(2);     break;
		case Btn3:     bVirtualUpButton = joy1Btn(3);     break;
		case Btn4:     bVirtualUpButton = joy1Btn(4);     break;
		case Btn5:     bVirtualUpButton = joy1Btn(5);     break;
		case Btn6:     bVirtualUpButton = joy1Btn(6);     break;
		case Btn7:     bVirtualUpButton = joy1Btn(7);     break;
		case Btn8:     bVirtualUpButton = joy1Btn(8);     break;
		case Btn9:     bVirtualUpButton = joy1Btn(9);     break;
		case Btn10:    bVirtualUpButton = joy1Btn(10);    break;

		case Pov0:	   if(joystick.joy1_TopHat == 0)	bVirtualDownButton = true;  	break;
		case Pov1:     if(joystick.joy1_TopHat == 1)	bVirtualDownButton = true;  	break;
		case Pov2:     if(joystick.joy1_TopHat == 2)	bVirtualDownButton = true;  	break;
		case Pov3:     if(joystick.joy1_TopHat == 3)	bVirtualDownButton = true;  	break;
		case Pov4:     if(joystick.joy1_TopHat == 4)	bVirtualDownButton = true;  	break;
		case Pov5:     if(joystick.joy1_TopHat == 5)	bVirtualDownButton = true;	  break;
		case Pov6:     if(joystick.joy1_TopHat == 6)	bVirtualDownButton = true;    break;
		case Pov7:     if(joystick.joy1_TopHat == 7)	bVirtualDownButton = true;    break;
		default:       bVirtualUpButton = false;												            break;
	}

	switch (downButton)
	{
		case Btn1:     bVirtualDownButton = joy1Btn(1);     break;
		case Btn2:     bVirtualDownButton = joy1Btn(2);     break;
		case Btn3:     bVirtualDownButton = joy1Btn(3);     break;
		case Btn4:     bVirtualDownButton = joy1Btn(4);     break;
		case Btn5:     bVirtualDownButton = joy1Btn(5);     break;
		case Btn6:     bVirtualDownButton = joy1Btn(6);     break;
		case Btn7:     bVirtualDownButton = joy1Btn(7);     break;
		case Btn8:     bVirtualDownButton = joy1Btn(8);     break;
		case Btn9:     bVirtualDownButton = joy1Btn(9);     break;
		case Btn10:    bVirtualDownButton = joy1Btn(10);    break;

		case Pov0:		 if(joystick.joy1_TopHat == 0)	bVirtualDownButton = true;  break;
		case Pov1:     if(joystick.joy1_TopHat == 1)	bVirtualDownButton = true;  break;
		case Pov2:     if(joystick.joy1_TopHat == 2)	bVirtualDownButton = true;  break;
		case Pov3:     if(joystick.joy1_TopHat == 3)	bVirtualDownButton = true;  break;
		case Pov4:     if(joystick.joy1_TopHat == 4)	bVirtualDownButton = true;  break;
		case Pov5:     if(joystick.joy1_TopHat == 5)	bVirtualDownButton = true;	break;
		case Pov6:     if(joystick.joy1_TopHat == 6)	bVirtualDownButton = true;  break;
		case Pov7:     if(joystick.joy1_TopHat == 7)	bVirtualDownButton = true;  break;
		default:       bVirtualDownButton = false;												        break;
	}

	if (bVirtualUpButton)
		setMotorSpeed(armMotorPort, armSpeed);
	else if (bVirtualDownButton)
		setMotorSpeed(armMotorPort, -armSpeed);
	else
		setMotorSpeed(armMotorPort, 0);
}


void buttonControl(tMotor armMotorPort = port10, TJoystickButtons upButton = Btn5U, TJoystickButtons downButton = Btn5D, short armSpeed = 75)
{
	armControl(armMotorPort, upButton, downButton, armSpeed);
}

void joystickControl(tMotor armMotorPort = port10, TJoystickAxis motorJoystick = JoyY1, short threshold = 10)
{
	int nMotorSpeed = 0;
	switch(motorJoystick)
	{
	case JoyX1:    nMotorSpeed = joystick.joy1_x1;     break;
	case JoyY1:    nMotorSpeed = joystick.joy1_y1;     break;
  case JoyX2:    nMotorSpeed = joystick.joy1_x2;     break;
  case JoyY2:    nMotorSpeed = joystick.joy1_y2;     break;
	default:       nMotorSpeed = 0;
	}

	if(abs(nMotorSpeed) <= abs(threshold))
		nMotorSpeed = 0;

	setMotorSpeed(armMotorPort, nMotorSpeed);
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


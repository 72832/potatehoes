//////////////////////////////////////////////////////////////////////////////////////////
//
// Robot Algebra Library
// =====================
//
// This file contains specialized RobotC routines that provide a framework for students
// to easily create sequences of moves using the NXT without having to know the full
// RobotC programming environment.
//
// Version: v0.r08
//
// Works with ROBOTC version 1.40
//
// Author: Eli M. Silk (esilk@mac.com)
//
//////////////////////////////////////////////////////////////////////////////////////////

#pragma platform(NXT)
#pragma systemFile      // this eliminates warning for "unreferenced" functions


//
// Version History
//
// v0 r01 EMS (2008-09-25)
//   - The initial Robot Algebra include file
// v0.r02 EMS (2008-10-01)
//   - Added different dancer names
// v0.r03 EMS (2008-10-23)
//   - Added option for different Wait Fors when starting dance routine
//   - Added ability to display time elapsed for each move
// v0.r04 EMS (2008-11-17)
//   - Modified the first parameter for robot movement functions to be
//     revolutions rather than degrees
// v0.r05 EMS (2008-11-19)
//   - Modified the second parameter for robot movement functions to be
//     revolutions per second rather than motor power
// v0.r06 EMS (2009-01-05)
//   - Added new "modes" to make the coordination easier, the timing mode
//     controls whether the moves are executed continuously or one at a time
//     and the type mode controls whether only certain types of moves are
//     executed (e.g., only straights or only turns)
// v0.r07 EMS (2009-03-06)
//   - Updated the function names for the new Robot Algebra platform in RobotC
// v0.r07+ DSK (2009-06-18)
//   - Added turnArc and turnArcBackward.  Does a large circle around center point
//	   to the left or right of the robot's axel.  Tested with all MoveTypes,
//	   Continuous and Single Stepping.
// v0.08 EMS (2009-06-24)
//   - Made changes so that use REM bot instead of GoBot Rover
//      - Switched sound sensor and touch sensor ports
//          - NOW Sound = S2; Touch = S1
//      - Changed direction of forward for the robots
//          - NOW front of robot is side with powered wheels; back is side with caster wheel
// v0.09 EMS (2009-07-23)
//   - Skipped the first wait for in the movesStart function when the 
//	   1-move-at-a-time mode is turned on, so that no longer have release
//	   touch sensor twice in that mode to get to first move.
//


//////////////////////////////////////////////////////////////////////////////////////////
//
//  Constant and Variable Declarations
//
//////////////////////////////////////////////////////////////////////////////////////////


// #pragma config(Motor,  motorB,          ,              tmotorNormal, PIDControl, )
// #pragma config(Motor,  motorC,          ,              tmotorNormal, PIDControl, )


// #pragma config(Sensor, S1, soundSensor, sensorSoundDB)

const int soundSensor = S2;
const int touchSensor = S1;


#define degreesInOneCircle 360


// const tMotor leftWheelMotor = motorC;
// const tMotor rightWheelMotor = motorB;
const int leftWheelMotor = motorC;
const int rightWheelMotor = motorB;


// Motor Direction

#define forwardMotorDirection 1
#define backwardMotorDirection -1

int motorDirection = forwardMotorDirection;


// Robot Models

typedef enum
{
	RMRobotEducator = 0,
	RMTaskbot,
	RMDomabot,
	RMDragRacer,
	RMJLo,
	RMJustinTimberlake, // Domabot - Wide Base, Small Wheels (Standard Robot - Robot A)
	RMMadonna, // REM - Standard Base, Standard Wheels (Robot B)
	RMSammyDavisJr,
	RMCiara,
	RMMichaelJackson
} RobotModel;


// Wait For

typedef enum
{
	WFNothing = 0,
	WFTouch,
	WFLoud,
	WFTime
} WaitFor;


// Step Through Mode

#define stepThroughModeOff 0
#define stepThroughModeOn 1

int stepThroughMode = stepThroughModeOff;


// Move Modes

typedef enum
{
	MMContinuousMoves = 0,
	MMOneMoveAtATime,
	MMAllMoveTypes,
	MMStraightsOnly,
	MMTurnsOnly,
	MMSwingTurnsOnly,
	MMPointTurnsOnly,
	MMArcTurnsOnly
} MoveMode;


MoveMode moveModeTiming = MMContinuousMoves;
MoveMode moveModeType = MMAllMoveTypes;


// Parameter Range

#define paramDegreesToRotateWheelsMin 0
#define paramMRotMin 0

#define adjustedMotorPowerMax 80
#define adjustedMaxRegulatedSpeed 720

#define paramMRotPerSecMin 0
#define paramMRotPerSecMax 2.0


//////////////////////////////////////////////////////////////////////////////////////////
//
//  Function Definitions
//
//////////////////////////////////////////////////////////////////////////////////////////


void initializeRobot (RobotModel robot = RMRobotEducator);
void movesStart (WaitFor waitForRoutine = WFTouch, float waitForParam = 50);
void movesEnd ();


// Movement Functions

void straightForward (float mRot = 1, float mRotPerSec = 1);
void straightBackward (float mRot = 1, float mRotPerSec = 1);

void turnPointRight (float mRot = 1, float mRotPerSec = 1);
void turnPointLeft (float mRot = 1, float mRotPerSec = 1);

void turnSwingRightForward (float mRot = 1, float mRotPerSec = 1);
void turnSwingRightBackward (float mRot = 1, float mRotPerSec = 1);
void turnSwingLeftForward (float mRot = 1, float mRotPerSec = 1);
void turnSwingLeftBackward (float mRot = 1, float mRotPerSec = 1);

void turnArc (float mRotLeft, float mRotLeftPerSec, float mRotRight, float mRotRightPerSec);
void turnArcBackwards (float mRotLeft, float mRotLeftPerSec, float mRotRight, float mRotRightPerSec);

// Math Functions

int round (float f);


// Miscellaneous Functions

void setMotorDirection (int robotMotorDirection);
void enableSpeedCtrl ();
float adjustMotorPower (float motorPower);
float revolutionsPerSecondToMotorPower (float revolutionsPerSecond);

void waitForTouch ();
void waitForLoud (int thresholdForLoud = 50);
void waitForTime (float durationSecs = 1.0);
void useStepThroughMode ();
void setMoveMode (MoveMode timingMoveMode = MMContinuousMoves, MoveMode typeMoveMode = MMAllMoveTypes);

int checkParameterRange (float mRot, float mRotPerSec);


//////////////////////////////////////////////////////////////////////////////////////////
//
//  Function Implementations
//
//////////////////////////////////////////////////////////////////////////////////////////

void initializeRobot (RobotModel robot)
{
	SensorType[soundSensor] = sensorSoundDB;
	SensorType[touchSensor] = sensorTouch;

	// Tells the motor to break (rather than coast) when stopping
	bFloatDuringInactiveMotorPWM = false;

	// Set the logical motor direction
	switch (robot)
	{
	case RMTaskbot:
	case RMDragRacer:
	case RMSammyDavisJr:
	case RMMichaelJackson:
		setMotorDirection(backwardMotorDirection);
		break;
	case RMRobotEducator:
	case RMDomabot:
	case RMCiara:
	case RMJLo:
	case RMJustinTimberlake:
	case RMMadonna:
	default:
		setMotorDirection(forwardMotorDirection);
		break;
	}

	eraseDisplay();
	switch (robot)
	{
	case RMRobotEducator: nxtDisplayCenteredTextLine(0, "Robot Educator"); break;
	case RMTaskbot: nxtDisplayCenteredTextLine(0, "Taskbot"); break;
	case RMDomabot: nxtDisplayCenteredTextLine(0, "Domabot"); break;
	case RMDragRacer: nxtDisplayCenteredTextLine(0, "Drag Racer"); break;
	case RMJLo: nxtDisplayCenteredTextLine(0, "J. Lo"); break;
	case RMJustinTimberlake: nxtDisplayCenteredTextLine(0, "J. Timberlake"); break;
	case RMMadonna: nxtDisplayCenteredTextLine(0, "Madonna"); break;
	case RMSammyDavisJr: nxtDisplayCenteredTextLine(0, "Sammy Davis Jr."); break;
	case RMCiara: nxtDisplayCenteredTextLine(0, "Ciara"); break;
	case RMMichaelJackson: nxtDisplayCenteredTextLine(0, "Michael Jackson"); break;
	default: nxtDisplayCenteredTextLine(0, "Unknown (%d)", robot); break;
	}
	nxtDisplayCenteredTextLine(2, "Initializing");

	enableSpeedCtrl();

	wait1Msec(5000);
}

void movesStart (WaitFor waitForRoutine, float waitForParam)
{
	nxtDisplayCenteredTextLine(5, "Starting");

	if (moveModeTiming == MMOneMoveAtATime || stepThroughMode == stepThroughModeOn) {
	} else {
		switch (waitForRoutine) {
		case WFTouch:
				waitForTouch();
				break;
		case WFLoud:
			waitForLoud(waitForParam);
			break;
		case WFTime:
			waitForTime(waitForParam);
			break;
		case WFNothing:
			default:
			break;
		}
	}
}

void movesEnd ()
{
	nxtDisplayCenteredTextLine(5, "Ending");
	wait1Msec(5000);
}


//////////////////////////////////////////////////////////////////////////////////////////
//
//  Movement Function Implementations
//
//////////////////////////////////////////////////////////////////////////////////////////

void straightForward (float mRot, float mRotPerSec)
{
	nxtDisplayCenteredTextLine(5, "SF(%.2f,%.2f)",
		mRot, mRotPerSec);

	if (moveModeType != MMAllMoveTypes && moveModeType != MMStraightsOnly) {
		return;
	}


	if (moveModeTiming == MMOneMoveAtATime) {
		waitForTouch();
	}


	if (stepThroughMode == stepThroughModeOn) {
		waitForTouch();
	}

	checkParameterRange(mRot, mRotPerSec);

	int leftWheelInitial = nMotorEncoder[leftWheelMotor];
	int rightWheelInitial = nMotorEncoder[rightWheelMotor];
	int leftWheelTarget = nMotorEncoder[leftWheelMotor] + 360 * mRot;
	int rightWheelTarget = nMotorEncoder[rightWheelMotor] + 360 * mRot;

	ClearTimer(T1);

	float motorPower = revolutionsPerSecondToMotorPower(mRotPerSec);
	motor[leftWheelMotor] = motorPower;
	motor[rightWheelMotor] = motorPower;

	while ((nMotorEncoder[leftWheelMotor] < leftWheelTarget)
		&& (nMotorEncoder[rightWheelMotor] < rightWheelTarget))
	{
		nxtDisplayCenteredTextLine(7, "%d", nMotorEncoder[leftWheelMotor]);
	}

	motor[leftWheelMotor] = 0;
	motor[rightWheelMotor] = 0;

	int leftWheelChange = nMotorEncoder[leftWheelMotor] - leftWheelInitial;
	int rightWheelChange = nMotorEncoder[rightWheelMotor] - rightWheelInitial;
	float revolutionsWheelsRotated =
	  ((float) ( abs(leftWheelChange) > abs(rightWheelChange) ?
	              leftWheelChange : rightWheelChange ))
	      / 360.0;

	nxtDisplayCenteredTextLine(2, "SF(%.2f,%.2f)",
		mRot, mRotPerSec);
	nxtDisplayCenteredTextLine(3, "%.2frev %.2fsec",
	  (float) revolutionsWheelsRotated, (float) time10(T1) / 100);
	nxtDisplayCenteredTextLine(5, "");
	nxtDisplayCenteredTextLine(7, "");
}

void straightBackward (float mRot, float mRotPerSec)
{
	nxtDisplayCenteredTextLine(5, "SB(%.2f,%.2f)",
		mRot, mRotPerSec);

	if (moveModeType != MMAllMoveTypes && moveModeType != MMStraightsOnly) {
		return;
	}

	if (moveModeTiming == MMOneMoveAtATime) {
		waitForTouch();
	}


	if (stepThroughMode == stepThroughModeOn) {
		waitForTouch();
	}

	checkParameterRange(mRot, mRotPerSec);

	int leftWheelInitial = nMotorEncoder[leftWheelMotor];
	int rightWheelInitial = nMotorEncoder[rightWheelMotor];
	int leftWheelTarget = nMotorEncoder[leftWheelMotor] - 360 * mRot;
	int rightWheelTarget = nMotorEncoder[rightWheelMotor] - 360 * mRot;

	ClearTimer(T1);

	float motorPower = revolutionsPerSecondToMotorPower(mRotPerSec);
	motor[leftWheelMotor] = -1 * motorPower;
	motor[rightWheelMotor] = -1 * motorPower;

	while ((nMotorEncoder[leftWheelMotor] > leftWheelTarget)
		&& (nMotorEncoder[rightWheelMotor] > rightWheelTarget))
	{
		nxtDisplayCenteredTextLine(7, "%d", nMotorEncoder[leftWheelMotor]);
	}

	motor[leftWheelMotor] = 0;
	motor[rightWheelMotor] = 0;

	int leftWheelChange = nMotorEncoder[leftWheelMotor] - leftWheelInitial;
	int rightWheelChange = nMotorEncoder[rightWheelMotor] - rightWheelInitial;
	float revolutionsWheelsRotated =
	  ((float) ( abs(leftWheelChange) > abs(rightWheelChange) ?
	              leftWheelChange : rightWheelChange ))
	       / 360.0;

	nxtDisplayCenteredTextLine(2, "SB(%.2f,%.2f)",
		mRot, mRotPerSec);
	nxtDisplayCenteredTextLine(3, "%.2frev %.2fsec",
	  (float) revolutionsWheelsRotated, (float) time10(T1) / 100);
	nxtDisplayCenteredTextLine(5, "");
	nxtDisplayCenteredTextLine(7, "");
}


void turnPointRight (float mRot, float mRotPerSec)
{
	nxtDisplayCenteredTextLine(5, "TPR(%.2f,%.2f)",
		mRot, mRotPerSec);

	if (moveModeType != MMAllMoveTypes
	      && moveModeType != MMTurnsOnly
	      && moveModeType != MMPointTurnsOnly) {
		return;
	}

	if (moveModeTiming == MMOneMoveAtATime) {
		waitForTouch();
	}


	if (stepThroughMode == stepThroughModeOn) {
		waitForTouch();
	}

	checkParameterRange(mRot, mRotPerSec);

	int leftWheelInitial = nMotorEncoder[leftWheelMotor];
	int rightWheelInitial = nMotorEncoder[rightWheelMotor];
	int leftWheelTarget = nMotorEncoder[leftWheelMotor] + 360 * mRot;
	int rightWheelTarget = nMotorEncoder[rightWheelMotor] - 360 * mRot;

	ClearTimer(T1);

	float motorPower = revolutionsPerSecondToMotorPower(mRotPerSec);
	motor[leftWheelMotor] = motorPower;
	motor[rightWheelMotor] = -1 * motorPower;

	while ((nMotorEncoder[leftWheelMotor] < leftWheelTarget)
		&& (nMotorEncoder[rightWheelMotor] > rightWheelTarget))
	{
		nxtDisplayCenteredTextLine(7, "%d", nMotorEncoder[leftWheelMotor]);
	}

	motor[leftWheelMotor] = 0;
	motor[rightWheelMotor] = 0;

	int leftWheelChange = nMotorEncoder[leftWheelMotor] - leftWheelInitial;
	int rightWheelChange = nMotorEncoder[rightWheelMotor] - rightWheelInitial;
	float revolutionsWheelsRotated =
	  ((float) ( abs(leftWheelChange) > abs(rightWheelChange) ?
	              leftWheelChange : rightWheelChange ))
	      / 360.0;

	nxtDisplayCenteredTextLine(2, "TPR(%.2f,%.2f)",
		mRot, mRotPerSec);
	nxtDisplayCenteredTextLine(3, "%.2frev %.2fsec",
	  (float) revolutionsWheelsRotated, (float) time10(T1) / 100);
	nxtDisplayCenteredTextLine(5, "");
	nxtDisplayCenteredTextLine(7, "");
}

void turnPointLeft (float mRot, float mRotPerSec)
{
	nxtDisplayCenteredTextLine(5, "TPL(%.2f,%.2f)",
		mRot, mRotPerSec);

	if (moveModeType != MMAllMoveTypes
	      && moveModeType != MMTurnsOnly
	      && moveModeType != MMPointTurnsOnly) {
		return;
	}

	if (moveModeTiming == MMOneMoveAtATime) {
		waitForTouch();
	}


	if (stepThroughMode == stepThroughModeOn) {
		waitForTouch();
	}

	checkParameterRange(mRot, mRotPerSec);

	int leftWheelInitial = nMotorEncoder[leftWheelMotor];
	int rightWheelInitial = nMotorEncoder[rightWheelMotor];
	int leftWheelTarget = nMotorEncoder[leftWheelMotor] - 360 * mRot;
	int rightWheelTarget = nMotorEncoder[rightWheelMotor] + 360 * mRot;

	ClearTimer(T1);

	float motorPower = revolutionsPerSecondToMotorPower(mRotPerSec);
	motor[leftWheelMotor] = -1 * motorPower;
	motor[rightWheelMotor] = motorPower;

	while ((nMotorEncoder[leftWheelMotor] > leftWheelTarget)
		&& (nMotorEncoder[rightWheelMotor] < rightWheelTarget))
	{
		nxtDisplayCenteredTextLine(7, "%d", nMotorEncoder[leftWheelMotor]);
	}

	motor[leftWheelMotor] = 0;
	motor[rightWheelMotor] = 0;

	int leftWheelChange = nMotorEncoder[leftWheelMotor] - leftWheelInitial;
	int rightWheelChange = nMotorEncoder[rightWheelMotor] - rightWheelInitial;
	float revolutionsWheelsRotated =
	  ((float) ( abs(leftWheelChange) > abs(rightWheelChange) ?
	                leftWheelChange : rightWheelChange ))
	        / 360.0;

	nxtDisplayCenteredTextLine(2, "TPL(%.2f,%.2f)",
		mRot, mRotPerSec);
	nxtDisplayCenteredTextLine(3, "%.2frev %.2fsec",
	  (float) revolutionsWheelsRotated, (float) time10(T1) / 100);
	nxtDisplayCenteredTextLine(5, "");
	nxtDisplayCenteredTextLine(7, "");
}

void turnSwingRightForward (float mRot, float mRotPerSec)
{
	nxtDisplayCenteredTextLine(5, "TSRF(%.2f,%.2f)",
		mRot, mRotPerSec);

	if (moveModeType != MMAllMoveTypes
	      && moveModeType != MMTurnsOnly
	      && moveModeType != MMSwingTurnsOnly) {
		return;
	}

	if (moveModeTiming == MMOneMoveAtATime) {
		waitForTouch();
	}


	if (stepThroughMode == stepThroughModeOn) {
		waitForTouch();
	}

	checkParameterRange(mRot, mRotPerSec);

	int leftWheelInitial = nMotorEncoder[leftWheelMotor];
	int leftWheelTarget = nMotorEncoder[leftWheelMotor] + 360 * mRot;

	ClearTimer(T1);

	float motorPower = revolutionsPerSecondToMotorPower(mRotPerSec);
	motor[leftWheelMotor] = motorPower;
	motor[rightWheelMotor] = 0;

	while (nMotorEncoder[leftWheelMotor] < leftWheelTarget)
	{
		nxtDisplayCenteredTextLine(7, "%d", nMotorEncoder[leftWheelMotor]);
	}

	motor[leftWheelMotor] = 0;
	motor[rightWheelMotor] = 0;

	int leftWheelChange = nMotorEncoder[leftWheelMotor] - leftWheelInitial;
	float revolutionsWheelsRotated = ((float) leftWheelChange) / 360.0;

	nxtDisplayCenteredTextLine(2, "TSRF(%.2f,%.2f)",
		mRot, mRotPerSec);
	nxtDisplayCenteredTextLine(3, "%.2frev %.2fsec",
	  (float) revolutionsWheelsRotated, (float) time10(T1) / 100);
	nxtDisplayCenteredTextLine(5, "");
	nxtDisplayCenteredTextLine(7, "");
}

void turnSwingRightBackward (float mRot, float mRotPerSec)
{
	nxtDisplayCenteredTextLine(5, "TSRB(%.2f,%.2f)",
		mRot, mRotPerSec);

	if (moveModeType != MMAllMoveTypes
	      && moveModeType != MMTurnsOnly
	      && moveModeType != MMSwingTurnsOnly) {
		return;
	}

	if (moveModeTiming == MMOneMoveAtATime) {
		waitForTouch();
	}


	if (stepThroughMode == stepThroughModeOn) {
		waitForTouch();
	}

	checkParameterRange(mRot, mRotPerSec);

	int rightWheelInitial = nMotorEncoder[rightWheelMotor];
	int rightWheelTarget = nMotorEncoder[rightWheelMotor] - 360 * mRot;

	ClearTimer(T1);

	float motorPower = revolutionsPerSecondToMotorPower(mRotPerSec);
	motor[leftWheelMotor] = 0;
	motor[rightWheelMotor] = -1 * motorPower;

	while (nMotorEncoder[rightWheelMotor] > rightWheelTarget)
	{
		nxtDisplayCenteredTextLine(7, "%d", nMotorEncoder[rightWheelMotor]);
	}

	motor[leftWheelMotor] = 0;
	motor[rightWheelMotor] = 0;

	int rightWheelChange = nMotorEncoder[rightWheelMotor] - rightWheelInitial;
	float revolutionsWheelsRotated = ((float) rightWheelChange) / 360.0;

	nxtDisplayCenteredTextLine(2, "TSRB(%.2f,%.2f)",
		mRot, mRotPerSec);
	nxtDisplayCenteredTextLine(3, "%.2frev %.2fsec",
	  (float) revolutionsWheelsRotated, (float) time10(T1) / 100);
	nxtDisplayCenteredTextLine(5, "");
	nxtDisplayCenteredTextLine(7, "");
}

void turnSwingLeftForward (float mRot, float mRotPerSec)
{
	nxtDisplayCenteredTextLine(5, "TSLF(%.2f,%.2f)",
		mRot, mRotPerSec);

	if (moveModeType != MMAllMoveTypes
	      && moveModeType != MMTurnsOnly
	      && moveModeType != MMSwingTurnsOnly) {
		return;
	}

	if (moveModeTiming == MMOneMoveAtATime) {
		waitForTouch();
	}


	if (stepThroughMode == stepThroughModeOn) {
		waitForTouch();
	}

	checkParameterRange(mRot, mRotPerSec);

	int rightWheelInitial = nMotorEncoder[rightWheelMotor];
	int rightWheelTarget = nMotorEncoder[rightWheelMotor] + 360 * mRot;

	ClearTimer(T1);

	float motorPower = revolutionsPerSecondToMotorPower(mRotPerSec);
	motor[leftWheelMotor] = 0;
	motor[rightWheelMotor] = motorPower;

	while (nMotorEncoder[rightWheelMotor] < rightWheelTarget)
	{
		nxtDisplayCenteredTextLine(7, "%d", nMotorEncoder[rightWheelMotor]);
	}

	motor[leftWheelMotor] = 0;
	motor[rightWheelMotor] = 0;

	int rightWheelChange = nMotorEncoder[rightWheelMotor] - rightWheelInitial;
	float revolutionsWheelsRotated = ((float) rightWheelChange) / 360.0;

	nxtDisplayCenteredTextLine(2, "TSLF(%.2f,%.2f)",
		mRot, mRotPerSec);
	nxtDisplayCenteredTextLine(3, "%.2frev %.2fsec",
	  (float) revolutionsWheelsRotated, (float) time10(T1) / 100);
	nxtDisplayCenteredTextLine(5, "");
	nxtDisplayCenteredTextLine(7, "");
}

void turnSwingLeftBackward (float mRot, float mRotPerSec)
{
	nxtDisplayCenteredTextLine(5, "TSLB(%.2f,%.2f)",
		mRot, mRotPerSec);

	if (moveModeType != MMAllMoveTypes
	      && moveModeType != MMTurnsOnly
	      && moveModeType != MMSwingTurnsOnly) {
		return;
	}

	if (moveModeTiming == MMOneMoveAtATime) {
		waitForTouch();
	}


	if (stepThroughMode == stepThroughModeOn) {
		waitForTouch();
	}

	checkParameterRange(mRot, mRotPerSec);

	int leftWheelInitial = nMotorEncoder[leftWheelMotor];
	int leftWheelTarget = nMotorEncoder[leftWheelMotor] - 360 * mRot;

	ClearTimer(T1);

	float motorPower = revolutionsPerSecondToMotorPower(mRotPerSec);
	motor[leftWheelMotor] = -1 * motorPower;
	motor[rightWheelMotor] = 0;

	while (nMotorEncoder[leftWheelMotor] > leftWheelTarget)
	{
		nxtDisplayCenteredTextLine(7, "%d", nMotorEncoder[leftWheelMotor]);
	}

	motor[leftWheelMotor] = 0;
	motor[rightWheelMotor] = 0;

	int leftWheelChange = nMotorEncoder[leftWheelMotor] - leftWheelInitial;
	float revolutionsWheelsRotated = ((float) leftWheelChange) / 360.0;

	nxtDisplayCenteredTextLine(2, "TSLB(%.2f,%.2f)",
		mRot, mRotPerSec);
	nxtDisplayCenteredTextLine(3, "%.2frev %.2fsec",
	  (float) revolutionsWheelsRotated, (float) time10(T1) / 100);
	nxtDisplayCenteredTextLine(5, "");
	nxtDisplayCenteredTextLine(7, "");
}
/**
* Arc turn has the center point to the left or right of the wheel in line
* with the axel.  The idea is that the Rotations Per Second difference is
* the ratio between the outside and inside circle circumference.
*
* The only reason we really ask for both wheel numbers is to facilitate
* the teaching of algebra concepts.  A more useful function would just
* use the robot dimensions and the number of degrees to turn.
*
* There isn't a left and right function because the only difference is which
* wheel has to travel the furthest distance.
*/
void turnArc (float mRotLeft, float mRotLeftPerSec, float mRotRight, float mRotRightPerSec) {

  nxtDisplayCenteredTextLine(5, "TAL(%.2f,%.2f)",mRotLeft, mRotLeftPerSec);
  nxtDisplayCenteredTextLine(6, "TAL(%.2f,%.2f)",mRotRight, mRotRightPerSec);
	 

	if (moveModeType != MMAllMoveTypes
	      && moveModeType != MMTurnsOnly
	      && moveModeType != MMArcTurnsOnly) {
		return;
	}

	if (moveModeTiming == MMOneMoveAtATime) {
		waitForTouch();
	}


	if (stepThroughMode == stepThroughModeOn) {
		waitForTouch();
	}

	checkParameterRange(mRotLeft, mRotLeftPerSec);
	checkParameterRange(mRotRight, mRotRightPerSec);

	int leftWheelInitial = nMotorEncoder[leftWheelMotor];
	int leftWheelTarget = nMotorEncoder[leftWheelMotor] + 360 * mRotLeft;
	int rightWheelInitial = nMotorEncoder[rightWheelMotor];
	int rightWheelTarget = nMotorEncoder[rightWheelMotor] + 360 * mRotRight;
	
	ClearTimer(T1);

	float leftMotorPower = revolutionsPerSecondToMotorPower(mRotLeftPerSec);
	float rightMotorPower = revolutionsPerSecondToMotorPower(mRotRightPerSec);
	
	motor[leftWheelMotor] = 1 * leftMotorPower;
	motor[rightWheelMotor] = 1 * rightMotorPower;

	while ( (nMotorEncoder[leftWheelMotor] < leftWheelTarget) &&
		(nMotorEncoder[rightWheelMotor] < rightWheelTarget) )
	{
		nxtDisplayCenteredTextLine(7, "%d", nMotorEncoder[leftWheelMotor]);
	}

	motor[leftWheelMotor] = 0;
	motor[rightWheelMotor] = 0;

	int leftWheelChange = nMotorEncoder[leftWheelMotor] - leftWheelInitial;
	float revolutionsWheelsRotatedLeft = ((float) leftWheelChange) / 360.0;
	int rightWheelChange = nMotorEncoder[rightWheelMotor] - rightWheelInitial;
	float revolutionsWheelsRotatedRight = ((float) rightWheelChange) / 360.0;
	
	nxtDisplayCenteredTextLine(2, "TAL(%.2f,%.2f)",
		mRotLeft, mRotLeftPerSec);
	nxtDisplayCenteredTextLine(3, "%.2frev %.2fsec",
	  (float) revolutionsWheelsRotatedLeft, (float) time10(T1) / 100);
	nxtDisplayCenteredTextLine(4, "TAL(%.2f,%.2f)",
		mRotRight, mRotRightPerSec);
	nxtDisplayCenteredTextLine(5, "%.2frev %.2fsec",
	  (float) revolutionsWheelsRotatedRight, (float) time10(T1) / 100);
	  nxtDisplayCenteredTextLine(6, "");
	nxtDisplayCenteredTextLine(7, "");

}

/**
* Arc turn has the center point to the left or right of the wheel in line
* with the axel.  The idea is that the Rotations Per Second difference is
* the ratio between the outside and inside circle circumference.
*
* The only reason we really ask for both wheel numbers is to facilitate
* the teaching of algebra concepts.  A more useful function would just
* use the robot dimensions and the number of degrees to turn.
*
* There isn't a left and right function because the only difference is which
* wheel has to travel the furthest distance.
*/
void turnArcBackwards (float mRotLeft, float mRotLeftPerSec, float mRotRight, float mRotRightPerSec) {

  nxtDisplayCenteredTextLine(5, "TAL(%.2f,%.2f)",mRotLeft, mRotLeftPerSec);
  nxtDisplayCenteredTextLine(6, "TAL(%.2f,%.2f)",mRotRight, mRotRightPerSec);
	 

	if (moveModeType != MMAllMoveTypes
	      && moveModeType != MMTurnsOnly
	      && moveModeType != MMArcTurnsOnly) {
		return;
	}

	if (moveModeTiming == MMOneMoveAtATime) {
		waitForTouch();
	}


	if (stepThroughMode == stepThroughModeOn) {
		waitForTouch();
	}

	checkParameterRange(mRotLeft, mRotLeftPerSec);
	checkParameterRange(mRotRight, mRotRightPerSec);

	int leftWheelInitial = nMotorEncoder[leftWheelMotor];
	int leftWheelTarget = nMotorEncoder[leftWheelMotor] - 360 * mRotLeft;
	int rightWheelInitial = nMotorEncoder[rightWheelMotor];
	int rightWheelTarget = nMotorEncoder[rightWheelMotor] - 360 * mRotRight;
	
	ClearTimer(T1);

	float leftMotorPower = revolutionsPerSecondToMotorPower(mRotLeftPerSec);
	float rightMotorPower = revolutionsPerSecondToMotorPower(mRotRightPerSec);
	
	motor[leftWheelMotor] = -1 * leftMotorPower;
	motor[rightWheelMotor] = -1 * rightMotorPower;

	while ( (nMotorEncoder[leftWheelMotor] > leftWheelTarget) &&
		(nMotorEncoder[rightWheelMotor] > rightWheelTarget) )
	{
		nxtDisplayCenteredTextLine(7, "%d", nMotorEncoder[leftWheelMotor]);
	}

	motor[leftWheelMotor] = 0;
	motor[rightWheelMotor] = 0;

	int leftWheelChange = nMotorEncoder[leftWheelMotor] - leftWheelInitial;
	float revolutionsWheelsRotatedLeft = ((float) leftWheelChange) / 360.0;
	int rightWheelChange = nMotorEncoder[rightWheelMotor] - rightWheelInitial;
	float revolutionsWheelsRotatedRight = ((float) rightWheelChange) / 360.0;
	
	nxtDisplayCenteredTextLine(2, "TAL(%.2f,%.2f)",
		mRotLeft, mRotLeftPerSec);
	nxtDisplayCenteredTextLine(3, "%.2frev %.2fsec",
	  (float) revolutionsWheelsRotatedLeft, (float) time10(T1) / 100);
	nxtDisplayCenteredTextLine(4, "TAL(%.2f,%.2f)",
		mRotRight, mRotRightPerSec);
	nxtDisplayCenteredTextLine(5, "%.2frev %.2fsec",
	  (float) revolutionsWheelsRotatedRight, (float) time10(T1) / 100);
	  nxtDisplayCenteredTextLine(6, "");
	nxtDisplayCenteredTextLine(7, "");

}

//////////////////////////////////////////////////////////////////////////////////////////
//
//  Math Function Implementations
//
//////////////////////////////////////////////////////////////////////////////////////////


int round (float f)
{
	if (f < 0) {
		return (int) (f - 0.5);
	} else {
		return (int) (f + 0.5);
	}
}


//////////////////////////////////////////////////////////////////////////////////////////
//
//  Miscellaneous Function Implementations
//
//////////////////////////////////////////////////////////////////////////////////////////


void setMotorDirection (int robotMotorDirection)
{
	motorDirection = robotMotorDirection;
	if (motorDirection == backwardMotorDirection) {
		// Flips/reflects the logical motor direction
		bMotorReflected[leftWheelMotor] = true;
		bMotorReflected[rightWheelMotor] = true;
	} else {
		bMotorReflected[leftWheelMotor] = false;
		bMotorReflected[rightWheelMotor] = false;
	}
}


void enableSpeedCtrl ()
{
	// Enables closed loop control algorithm that uses feedback
	// from the motor encoder to adjust the raw power to provide
	// a consistent speed.

	nMotorPIDSpeedCtrl[leftWheelMotor] = mtrSpeedReg;
	nMotorPIDSpeedCtrl[rightWheelMotor] = mtrSpeedReg;

	// Max regulated speed level
	// In units of encoder counts per second. There are 360
	// encoder counts in one full rotation (revolution).
	// Default is 1000 encoder counts per second, but that
	// cannot always be maintained with lower battery levels

  nMaxRegulatedSpeedNxt = (short) round(adjustedMaxRegulatedSpeed);
}


float adjustMotorPower (float motorPower)
{
	return ((motorPower / 100) * adjustedMotorPowerMax);
}


float revolutionsPerSecondToMotorPower (float revolutionsPerSecond)
{
	return ((revolutionsPerSecond / (adjustedMaxRegulatedSpeed / 360.0)) * 100);
}


void waitForTouch ()
{
	nxtDisplayCenteredTextLine(7, "Wait(Touch)");

	while(SensorValue(touchSensor) == 0) {
		// Wait while the touch sensor is not being pressed
	}
	while(SensorValue(touchSensor) == 1) {
		// Wait until the touch sensor stops being pressed
	}

	// nxtDisplayCenteredTextLine(2, "Wait(Touch)");
	// nxtDisplayCenteredTextLine(4, "");
	nxtDisplayCenteredTextLine(7, "");
}


void waitForLoud (int thresholdForLoud)
{
	nxtDisplayCenteredTextLine(5, "Wait(Sound>%d)", thresholdForLoud);

	int triggeredValue;

	// Wait for sound to go above a given threshold value
	while (SensorValue(soundSensor) < thresholdForLoud) {
		nxtDisplayCenteredTextLine(7, "%d", SensorValue(soundSensor));
	}

	// Save the sound value that was above the threshold
	triggeredValue = SensorValue(soundSensor);

	nxtDisplayCenteredTextLine(2, "Wait(Sound=%d)", triggeredValue);
	nxtDisplayCenteredTextLine(3, "");
	nxtDisplayCenteredTextLine(5, "");
	nxtDisplayCenteredTextLine(7, "");
}


void waitForTime (float durationSecs)
{
	nxtDisplayCenteredTextLine(5, "Wait(Time=%.2f)", durationSecs);

	wait1Msec(durationSecs * 1000);

	nxtDisplayCenteredTextLine(2, "Wait(Time=%.2f)", durationSecs);
	nxtDisplayCenteredTextLine(3, "");
	nxtDisplayCenteredTextLine(5, "");
	nxtDisplayCenteredTextLine(7, "");
}


void useStepThroughMode ()
{
	nxtDisplayCenteredTextLine(5, "Step Through");

	stepThroughMode = stepThroughModeOn;

	nxtDisplayCenteredTextLine(2, "Step Through");
	nxtDisplayCenteredTextLine(3, "");
	nxtDisplayCenteredTextLine(5, "");
	nxtDisplayCenteredTextLine(7, "");
}


void setMoveMode (MoveMode timingMoveMode, MoveMode typeMoveMode)
{
	nxtDisplayCenteredTextLine(5, "Move Mode");

	moveModeTiming = timingMoveMode;
	moveModeType = typeMoveMode;

	nxtDisplayCenteredTextLine(2, "Move Mode (%d,%d)", moveModeTiming, moveModeType);
	nxtDisplayCenteredTextLine(3, "");
	nxtDisplayCenteredTextLine(5, "");
	nxtDisplayCenteredTextLine(7, "");
}


int checkParameterRange (float mRot, float mRotPerSec)
{
	if (mRot < paramMRotMin) {
		nxtDisplayCenteredTextLine(5, "ERR: MtrRots");
		nxtDisplayCenteredTextLine(7, "%.0f (min %d)",
			mRot, paramMRotMin);
		wait1Msec(5000);
		StopAllTasks();
		return -1;
	}
	if (mRotPerSec < paramMRotPerSecMin)  {
		nxtDisplayCenteredTextLine(5, "ERR: MtrSpeed");
		nxtDisplayCenteredTextLine(7, "%.2f (min %.2f)",
			mRotPerSec,
			paramMRotPerSecMin);
		wait1Msec(5000);
		StopAllTasks();
		return -1;
	}
	if (mRotPerSec > paramMRotPerSecMax) {
		nxtDisplayCenteredTextLine(5, "ERR: MtrSpeed");
		nxtDisplayCenteredTextLine(7, "%.2f (max %.2f)",
			mRotPerSec,
			paramMRotPerSecMax);
		wait1Msec(5000);
		StopAllTasks();
		return -1;
	}
	return 1;
}



//////////////////////////////////////////////////////////////////////////////////////////
//
//  End File
//
//////////////////////////////////////////////////////////////////////////////////////////

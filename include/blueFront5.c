/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

/** ------------------- 72832S autonomous functions -----------------*/

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/


	startTask(intakeOnTask);
	delayFunc(50);
	driveForward(1.5);
	delayFunc(50);
	driveBackward(1.0);
	delayFunc(50);
	driveBackward(0.3);
	delayFunc(50);
	driveForward(0.3);
	delayFunc(50);
	driveTurn(false, 7.75, 50);
	delayFunc(50);
	punch();
	delayFunc(50);
	driveBackward(1.1);
	delayFunc(50);
	driveTurn(true, 13.5, 50);
	delayFunc(50);
	driveBackward(0.5);
	delayFunc(50);
	driveForward(1.75);
	delayFunc(5000);
	intakeOff();
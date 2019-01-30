//
// Created by jonesdav004 on 1/23/2019
//

#ifndef POTATOES_API_H
#define POTATOES_API_H

#endif //POTATOES_API_H

/** @file APIh
 * @brief Provides the high-level user functionality intended for use by typical VEX Cortex
 * programmers
 *
 * This file should be included for you in the predefined stubs in each new VEX Cortex PROS
 * project through the inclusion of "mainh" In any new C source file, it is advisable to
 * include mainh instead of referencing APIh by name, to better handle any nomenclature
 * changes to this file or its contents
 *
 * Copyright (c) 2011-2018, Purdue University ACM SIGBots
 * All rights reserved
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v 20 If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozillaorg/MPL/20/
 *
 * PROS contains FreeRTOS (http://wwwfreertosorg) whose source code may be
 * obtained from http://sourceforgenet/projects/freertos/files/ or on request
 */

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

/** ------------------- 72832S variable definitions -----------------*/

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

int intake;
int right;
int left;
int puncher;

int one;
int two;
int three;
int four;
int five;
int six;
int seven;
int eight;
int nine;
int ten;

int autonProg;

int autonColor;

int autonPos;

int back=0;

int front=1;

int red=0;

int blue=1;

//auton cutoffs
int cutoffs[] = {0, 1024, 2048, 3072, 4096};

//auton reading potentionmeters
int val1 = SensorValue[posPotent];
int val2 = SensorValue[progPotent];

bool driveReverse = false;

string mainBattery, backupBattery;

int leftEnc=SensorValue[leftEncoder]*-1;
int rightEnc=SensorValue[rightEncoder];

int maxVal=127;
int minVal=-127;

<<<<<<< HEAD
int equals=1;
int minus=2;
int plus=3;

=======
>>>>>>> 6c14ead3db44d726cd505c5b4d416a9f831fd04e
/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

/** --------------------- 72832S global functions -------------------*/

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

void delayFunc(int time){
	wait1Msec(time);
}

void motorRefresh() {

    one = intake;
    two = left;
    three = left;
    four = left;
    five = puncher;
    six = 0;//unused
    seven = right;
    eight = right;
    nine = right;
    ten = intake;
}

void motorVarSet(){
    motorRefresh();

    motor[port1]=one;
<<<<<<< HEAD
    motor[port2]=left;
    motor[port3]=three;
    motor[port4]=four;
=======
    SetMotor(2,two);
    SetMotor(3,three);
    SetMotor(4,four);
>>>>>>> 6c14ead3db44d726cd505c5b4d416a9f831fd04e
    motor[port5]=five;
    motor[port6]=six;
    SetMotor(7,seven);
    SetMotor(8,eight);
    SetMotor(9,nine);
    motor[port10]=ten;
}

void motorChange(int var, int change = 0, int val = 0) {
    if (change == 0)
        var = 0;
    else if (change == 1) {
        var = val;
    }
    if (change == 2) {
        var -= val;
    }
    if (change == 3) {
        var += val;
    }
}

void resetEncoders() {
    SensorValue[leftEncoder] = 0;
    SensorValue[rightEncoder] = 0;
}

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

/** ----------------- 72832S initialization functions ---------------*/

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

void clearLCD() {
    clearLCDLine(0);
    clearLCDLine(1);
}

//void lcd display voltage
void lcdBattery() {
    clearLCD();

    //Display the Primary Robot battery voltage
    displayLCDString(0, 0, "Primary: ");
    sprintf(mainBattery, "%12f%c", nImmediateBatteryLevel / 10000, 'V'); //Build the value to be displayed
    displayNextLCDString(mainBattery);

    //Display the Backup battery voltage
    displayLCDString(1, 0, "Backup: ");
    sprintf(backupBattery, "%12f%c", BackupBatteryLevel / 10000, 'V');    //Build the value to be displayed
    displayNextLCDString(backupBattery);

    //Short delay for the LCD refresh rate
    wait1Msec(100);
}

void init() {
	//auton menu
if (val1 >= cutoffs[0] && val1 < cutoffs[1]) {
    autonColor=red;
    autonPos=back;
    if (val2 >= cutoffs[0] && val2 < cutoffs[1])
        autonProg = 0;
    else if (val2 >= cutoffs[1] && val2 < cutoffs[2])
        autonProg = 1;
    else if (val2 >= cutoffs[2] && val2 < cutoffs[3])
        autonProg = 2;
    else if (val2 >= cutoffs[3] && val2 < cutoffs[4])
        autonProg = 3;
} else if (val1 >= cutoffs[1] && val1 < cutoffs[2]) {
    autonColor=red;
    autonPos=front;
    if (val2 >= cutoffs[0] && val2 < cutoffs[1])
        autonProg = 0;
    else if (val2 >= cutoffs[1] && val2 < cutoffs[2])
        autonProg = 1;
    else if (val2 >= cutoffs[2] && val2 < cutoffs[3])
        autonProg = 2;
    else if (val2 >= cutoffs[3] && val2 < cutoffs[4])
        autonProg = 3;
} else if (val1 >= cutoffs[2] && val1 < cutoffs[3]) {
    autonColor=blue;
    autonPos=front;
    if (val2 >= cutoffs[0] && val2 < cutoffs[1])
        autonProg = 0;
    else if (val2 >= cutoffs[1] && val2 < cutoffs[2])
        autonProg = 1;
    else if (val2 >= cutoffs[2] && val2 < cutoffs[3])
        autonProg = 2;
    else if (val2 >= cutoffs[3] && val2 < cutoffs[4])
        autonProg = 3;
} else if (val1 >= cutoffs[3] && val1 < cutoffs[4]) {
    autonColor=blue;
    autonPos=back;
    if (val2 >= cutoffs[0] && val2 < cutoffs[1])
        autonProg = 0;
    else if (val2 >= cutoffs[1] && val2 < cutoffs[2])
        autonProg = 1;
    else if (val2 >= cutoffs[2] && val2 < cutoffs[3])
        autonProg = 2;
    else if (val2 >= cutoffs[3] && val2 < cutoffs[4])
        autonProg = 3;
}
    resetEncoders();
<<<<<<< HEAD
    // Set bStopTasksBetweenModes to false if you want to keep
   // user created tasks running between Autonomous and Driver
   // controlled modes. You will need to manage all user created
   // tasks if set to false.
   bStopTasksBetweenModes = false;

   // Initialize the Smart Motor Library
   SmartMotorsInit();

   // Define which motors are linked
   SmartMotorLinkMotors( left, right );

   // Declare that you want the Library to keep a lid on the
   // motors by measuring the temperature of the PTC components
   SmartMotorPtcMonitorEnable();

   // Run smart motors
   SmartMotorRun();
=======
    SmartMotorsInit();
    SmartMotorRun();
>>>>>>> 6c14ead3db44d726cd505c5b4d416a9f831fd04e
}

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

/** ------------------- 72832S autonomous functions -----------------*/

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

task posPID(){

 float  pidSensorCurrentValue;
 float  pidError;
 float  pidLastError;
 float  pidIntegral;
 float  pidDerivative;
 float  pidDrive;

 // If we are using an encoder then clear it
 if(SensorType[rightEncoder] == sensorQuadEncoder )
   SensorValue[ rightEncoder ] = 0;

  	// Init the variables - thanks Glenn :)
  pidLastError  = 0;
  pidIntegral   = 0;

  while( true ){
    // Is PID control active ?
    if( pidRunning ){
      // Read the sensor value and scale
      pidSensorCurrentValue = SensorValue[ rightEncoder ] * 1;

      // calculate error
      pidError = pidSensorCurrentValue - pidRequestedValue;

      // integral - if Ki is not 0
      if( pid_Ki != 0 ){
        // If we are inside controlable window then integrate the error
        if( abs(pidError) < 50 )
          pidIntegral = pidIntegral + pidError;
        else
          pidIntegral = 0;
        }
      else
        pidIntegral = 0;

        // calculate the derivative
        pidDerivative = pidError - pidLastError;
        pidLastError  = pidError;

        // calculate drive
        pidDrive = (pid_Kp * pidError) + (pid_Ki * pidIntegral) + (pid_Kd * pidDerivative);

        // limit drive
        if( pidDrive > 90 )
          pidDrive = 90;
        if( pidDrive < -90 )
          pidDrive = -90;

            // send to motor
           	driveFunc(pidDrive, pidDrive);
    }else{
       // clear all
       pidError      = 0;
       pidLastError  = 0;
       pidIntegral   = 0;
       pidDerivative = 0;
       driveFunc(0,0);
     }

    // Run at 50Hz
    wait1Msec( 25 );
  }
}

/*-----------------------------------------------------------------------------*/
/*
*/
/*  main task
*/
/*
*/
/*-----------------------------------------------------------------------------*/

void rightDrivePID(int clicks){
	// send the motor off somewhere
  pidRequestedValue = clicks;
	// start the PID task
  startTask( rightPIDController );

  // use joystick to modify the requested position
  while( true ){
  	// maximum change for pidRequestedValue will be 127/4*20, around 640 counts per second
  	// free spinning motor is 100rmp so 1.67 rotations per second
		// 1.67 * 360 counts is 600

  	wait1Msec(50);
  }
}

task intakeOnTask() {
    while (true)
        motor[intake2]=127;
}

task intakeOffTask() {
    while (true)
        motor[intake2]=0;
}

void intakeOn(){
    startTask(intakeOnTask);
}

void intakeOff(){
    stopTask(intakeOnTask);
    startTask(intakeOffTask);
    stopTask(intakeOffTask);
}

task puncherOnTask() {
    while (true)
        motor[puncher]=127;
}

task puncherOffTask() {
    while (true)
        motor[puncher]=0;
}

void punch() {

    //puncher on
    startTask(puncherOnTask);
    delayFunc(2000);
    stopTask(puncherOnTask);
    startTask(puncherOffTask);
    stopTask(puncherOffTask);
}

<<<<<<< HEAD
void straightForward(int modifier){
    if(leftEnc-give>rightEnc){
        motorChange(left,minus,modifier);
    }else if(leftEnc<rightEnc-give){
        motorChange(right,minus,modifier);
    }else if((-give<rightEnc-leftEnc) && (rightEnc-leftEnc<give)){
        motorChange(left,equals,minVal);
        motorChange(right,equals,minVal);
    }
}

void straightBackward(int modifier){
    if(leftEnc-give>rightEnc){
        motorChange(left,plus,modifier);
    }else if(leftEnc<rightEnc-give){
        motorChange(right,plus,modifier);
    }else if(-give<rightEnc-leftEnc && rightEnc-leftEnc<give){
        motorChange(left,equals,minVal);
        motorChange(right,equals,minVal);
    }
}

void reqValueStraight(int val){
    int diff=val-leftEnc;
    int diffNeg=val+leftEnc;
    if(val>0){
        if(diff >=1800){
            straightForward(15);
            maxVal=90;
        }else if(diff>=360){
            straightForward(3);
            maxVal=60;
        }
    }
    else if(val<0){
        if(diffNeg <=-1800){
            straightBackward(15);
            maxVal=90;
        }else if(diffNeg<=-360){
            straightBackward(3);
            maxVal=60;
        }
    }
}

void auton(){
    delayFunc(100);
}
=======
// PID using optical shaft encoder
//
// Shaft encoder has 360 pulses per revolution
//

#define PID_SENSOR_SCALE    1

#define PID_MOTOR_SCALE     -1

#define PID_INTEGRAL_LIMIT  50

// These could be constants but leaving
// as variables allows them to be modified in the debugger "live"
float  pid_Kp = 2.0;
float  pid_Ki = 0.0;
float  pid_Kd = 0.0;

static int   pidRunning = 1;
static float pidRequestedValue;

/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*  pid control task                                                           */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

task pidPos()
{
    float  pidSensorCurrentValue;

    float  pidError;
    float  pidLastError;
    float  pidIntegral;
    float  pidDerivative;
    float  pidDrive;

    // If we are using an encoder then clear it
    if( SensorType[ leftEncoder ] == sensorQuadEncoder )
        SensorValue[ leftEncoder ] = 0;

    if( SensorType[ rightEncoder ] == sensorQuadEncoder )
        SensorValue[ rightEncoder ] = 0;

    // Init the variables - thanks Glenn :)
    pidLastError  = 0;
    pidIntegral   = 0;

    while( true )
        {
        // Is PID control active ?
        if( pidRunning )
            {
            // Read the sensor value and scale
            pidSensorCurrentValue = SensorValue[ leftEncoder ];

            // calculate error
            pidError = pidSensorCurrentValue - pidRequestedValue;

            // integral - if Ki is not 0
            if( pid_Ki != 0 )
                {
                // If we are inside controlable window then integrate the error
                if( abs(pidError) < PID_INTEGRAL_LIMIT )
                    pidIntegral = pidIntegral + pidError;
                else
                    pidIntegral = 0;
                }
            else
                pidIntegral = 0;

            // calculate the derivative
            pidDerivative = pidError - pidLastError;
            pidLastError  = pidError;

            // calculate drive
            pidDrive = (pid_Kp * pidError) + (pid_Ki * pidIntegral) + (pid_Kd * pidDerivative);

            // limit drive
            if( pidDrive > 90 )
                pidDrive = 90;
            if( pidDrive < -90 )
                pidDrive = -90;

            // send to motor
            motor[left1] = pidDrive;
            motor[left2] = pidDrive;
            motor[left3] = pidDrive;

            motor[right1] = pidDrive;
            motor[right2] = pidDrive;
            motor[right3] = pidDrive;
            }
        else
            {
            // clear all
            pidError      = 0;
            pidLastError  = 0;
            pidIntegral   = 0;
            pidDerivative = 0;

            // send to motor
            motor[left1] = 0;
            motor[left2] = 0;
            motor[left3] = 0;

            motor[right1] = 0;
            motor[right2] = 0;
            motor[right3] = 0;
            }

        // Run at 50Hz
        wait1Msec( 25 );
        }
}

/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*  main task                                                                  */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

void pidPosReq(int val)
{
    // send the motor off somewhere
    pidRequestedValue = val;

    // start the PID task
    startTask( pidPos );

    // use joystick to modify the requested position
    while( true )
        {
        // maximum change for pidRequestedValue will be 127/4*20, around 640 counts per second
        // free spinning motor is 100rmp so 1.67 rotations per second
        // 1.67 * 360 counts is 600

        pidRequestedValue = pidRequestedValue + (vexRT[ Ch2 ]/4);

        wait1Msec(50);
        }

}

void auton(){
    punch();
    startTask(intakeOn);
    pidPosReq(1100);

}

>>>>>>> 6c14ead3db44d726cd505c5b4d416a9f831fd04e
/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

/** ------------------- 72832S opcontrol functions -----------------*/

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

void opcontrol(){
		if(vexRT[Btn5D]==1){
			motor[intake2]=127;
		}else if(vexRT[Btn5U]==1){
			motor[intake2]=-127;
    }else{
			motor[intake2]=0;
    }

	if(vexRT[Btn6U]==1){
		motor[puncherMain]=127;
    }else if( vexRT[Btn6D]==1){
			motor[puncherMain]=-127;
    }else{
			motor[puncherMain]=0;
    }

    if(vexRT[Btn7D]==1){
        init();
        startTask(autonomous);
        waitUntil(vexRT[Btn7D]==0);
    }

    if(vexRT[Btn7U]==1){
        stopTask(autonomous);
        waitUntil(vexRT[Btn7U]==0);
    }
    if(vexRT[Btn7R]==1){
        auton();
    }

<<<<<<< HEAD
    SetMotor(left1,vexRT[Ch3]*-1);
	SetMotor(left2,vexRT[Ch3]*-1);
	SetMotor(left3,vexRT[Ch3]*-1);

    SetMotor(right1,vexRT[Ch2]);
    SetMotor(right2,vexRT[Ch2]);
	SetMotor(right3,vexRT[Ch2]);
}
=======
    motorRefresh();

    motorVarSet();
}
>>>>>>> 6c14ead3db44d726cd505c5b4d416a9f831fd04e

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

int autonPlatform;

//auton cutoffs
int cutoffs[] = {0, 512, 2048, 3584, 4096};

//auton reading potentionmeters
int val1 = SensorValue[posPotent];
int val2 = SensorValue[progPotent];

string mainBattery, backupBattery;

bool pidRunning=false;

float pidRequestedValue;

int leftEnc=SensorValue[leftEncoder]*-1;
int rightEnc=SensorValue[rightEncoder]*-1;

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

void resetEncoders() {
    SensorValue[leftEncoder] = 0;
    SensorValue[rightEncoder] = 0;
}

void delay(){
    delayFunc(50);
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
    string mainBattery, backupBattery;

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

    // Set bStopTasksBetweenModes to false if you want to keep
    // user created tasks running between Autonomous and Driver
    // controlled modes. You will need to manage all user created
    // tasks if set to false.
    bStopTasksBetweenModes = false;

    // Initialize the Smart Motor Library
    SmartMotorsInit();

    // Define which motors are linked

    // Declare that you want the Library to keep a lid on the
    // motors by measuring the temperature of the PTC components
    SmartMotorPtcMonitorEnable();

    // Run smart motors
    SmartMotorRun();
    SmartMotorsInit();
    SmartMotorRun();


    pidRunning=true;

    pidRequestedValue=0;

    bLCDBacklight=true;

}

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

/** ------------------- 72832S autonomous functions -----------------*/

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

task intakeOnTask() {
    while (true){
        motor[intake1]=-127;
        motor[intake2]=-127;
    }
}

task intakeOn(){
    while (true){
        motor[intake1]=127;
        motor[intake2]=127;
    }
}

task intakeOffTask() {
    while (true){
        motor[intake1]=0;
        motor[intake2]=0;
    }
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
    delayFunc(1000);
    stopTask(puncherOnTask);
    startTask(puncherOffTask);
    stopTask(puncherOffTask);
}

// PID using optical shaft encoder
//
// Shaft encoder has 360 pulses per revolution
//

#define PID_INTEGRAL_LIMIT  50

// These could be constants but leaving
// as variables allows them to be modified in the debugger "live"

float wheelCircumference=4 /*diameter*/ * 3.141529 /*PI simplified*/;

void driveForward(static float tiles, static int speed=75){

    static float wheelRotations;

    static float clicks;

    wheelRotations = (tiles*24) / (wheelCircumference);

    clicks = wheelRotations*360;

    resetEncoders();

    while(leftEnc<=clicks){

        leftEnc=SensorValue[leftEncoder]*-1;

        motor[left1]=speed*-1;
        motor[left2]=speed*-1;
        motor[left3]=speed*-1;

        motor[right1]=speed;
        motor[right2]=speed;
        motor[right3]=speed;
    }
    motor[left1]=0;
    motor[left2]=0;
    motor[left3]=0;

    motor[right1]=0;
    motor[right2]=0;
    motor[right3]=0;
}

void driveBackward(static float tiles, static int speed=75){

    static float wheelRotations;

    static float clicks;

    wheelRotations = (tiles*24) / (wheelCircumference);

    clicks = wheelRotations*360;

    resetEncoders();

    leftEnc=SensorValue[leftEncoder]*1;

    while(leftEnc<=clicks){

        leftEnc=SensorValue[leftEncoder]*1;

        motor[left1]=speed*1;
        motor[left2]=speed*1;
        motor[left3]=speed*1;

        motor[right1]=speed*-1;
        motor[right2]=speed*-1;
        motor[right3]=speed*-1;
    }

    motor[left1]=0;
    motor[left2]=0;
    motor[left3]=0;

    motor[right1]=0;
    motor[right2]=0;
    motor[right3]=0;
}

void driveTurn90(static bool left, static int speed=75){

		//fixxxx

    static float wheelRotations;

    static float clicks;

    wheelRotations = (inches) / (wheelCircumference);

    clicks = wheelRotations*360;

    resetEncoders();

    if(left==true){
        leftEnc=SensorValue[leftEncoder]*1;

        while(leftEnc<=clicks){
            leftEnc=SensorValue[leftEncoder]*1;

            motor[left1]=speed;
            motor[left2]=speed;
            motor[left3]=speed;

            motor[right1]=speed;
            motor[right2]=speed;
            motor[right3]=speed;
        }
    }else if(left==false){
        leftEnc=SensorValue[leftEncoder]*-1;

        while(leftEnc<=clicks){
            leftEnc=SensorValue[leftEncoder]*-1;

            motor[left1]=speed*-1;
            motor[left2]=speed*-1;
            motor[left3]=speed*-1;

            motor[right1]=speed*-1;
            motor[right2]=speed*-1;
            motor[right3]=speed*-1;
        }
    }
    motor[left1]=0;
    motor[left2]=0;
    motor[left3]=0;

    motor[right1]=0;
    motor[right2]=0;
    motor[right3]=0;
}

/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                                                                             */
/*  autonomous code                                                            */
/*                                                                             */
/*                                                                             */
/*-----------------------------------------------------------------------------*/



/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

/** ------------------- 72832S opcontrol functions -----------------*/

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

bool driveReverse;

void opcontrol(){
    driveReverse=false;

    if(driveReverse==false){
        SetMotor(left1,vexRT[Ch3]*-1);
	    SetMotor(left2,vexRT[Ch3]*-1);
        SetMotor(left3,vexRT[Ch3]*-1);

        SetMotor(right1,vexRT[Ch2]);
        SetMotor(right2,vexRT[Ch2]);
	    SetMotor(right3,vexRT[Ch2]);
    }else if(driveReverse==true){
        SetMotor(left1,vexRT[Ch2]*1);
	    SetMotor(left2,vexRT[Ch2]*1);
        SetMotor(left3,vexRT[Ch2]*1);

        SetMotor(right1,vexRT[Ch3]*-1);
        SetMotor(right2,vexRT[Ch3]*-1);
	    SetMotor(right3,vexRT[Ch3]*-1);
    }

	if(vexRT[Btn5D]==1){
		motor[intake1]=127;
		motor[intake2]=127;
	}else if(vexRT[Btn5U]==1){
		motor[intake1]=-127;
	    motor[intake2]=-127;
    }else{
        motor[intake1]=0;
		motor[intake2]=0;
    }

	if(vexRT[Btn6U]==1){
		motor[puncher]=127;
    }else{
		motor[puncher]=0;
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

    if(vexRT[Btn8D]==1){
	motor[flipper]=127;
    }else if(vexRT[Btn6D]==1){
    	motor[flipper]=-127;
    }else {
  	motor[flipper]=0;
    }

    if(vexRT[Btn7D]==1){
        waitUntil(vexRT[Btn7D]==0);
        driveReverse=!driveReverse;
    }
}

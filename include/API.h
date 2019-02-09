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

int autonColor;

int autonPos;

int autonRun;

int skillsRun;

int autonPlatform;

int back=0;

int front=1;

int red=0;

int blue=1;

int no = 0;

int yes = 1;

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

void autonLCD(){
    string mainBattery, backupBattery;

    bLCDBacklight=true;

    clearLCD();

    if(autonRun){

        setLCDPosition(0,0);

        if(autonColor==red){

            displayNextLCDString("Red, ");

        }else if(autonColor==blue){

            displayNextLCDString("Blue, ");

        }

        if(autonPos==back){

            displayNextLCDString("Back, ");

        }else if(autonPos==front){

            displayNextLCDString("Front, ");

        }

        if(autonPlatform==yes){
            displayNextLCDString("P");
        }else if(autonPlatform==no)
            displayNextLCDString("No P");

        displayLCDString(1, 0, "Primary: ");
        sprintf(mainBattery, "%12f%c", nImmediateBatteryLevel / 10000, 'V'); //Build the value to be displayed
        displayNextLCDString(mainBattery);
    }
}

void autonInit(){

    resetEncoders();

	//auton menu
    if (val1 >= cutoffs[0] && val1 < cutoffs[1]) {
        autonColor=red;
        autonPos=back;
    } else if (val1 >= cutoffs[1] && val1 < cutoffs[2]) {
        autonColor=red;
        autonPos=front;
    } else if (val1 >= cutoffs[2] && val1 < cutoffs[3]) {
        autonColor=blue;
        autonPos=front;
    } else if (val1 >= cutoffs[3] && val1 < cutoffs[4]) {
        autonColor=blue;
        autonPos=back;
    }
	//auton menu
    if (val2 >= cutoffs[0] && val2 < cutoffs[1]) {
        autonRun=yes;
        autonPlatform=yes;
        skillsRun=no;
    } else if (val2 >= cutoffs[1] && val2 < cutoffs[2]) {
        autonRun=yes;
        autonPlatform=no;
        skillsRun=no;
    } else if (val2 >= cutoffs[2] && val2 < cutoffs[3]) {
        autonRun=no;
        skillsRun=no;
    } else if (val2 >= cutoffs[3] && val2 < cutoffs[4]) {
        autonRun=no;
        skillsRun=yes;
    }
}

void init() {

    autonInit();

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

        leftEnc=SensorValue[leftEncoder];

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

    leftEnc=SensorValue[leftEncoder]*-1;

    while(leftEnc<=clicks){

        leftEnc=SensorValue[leftEncoder]*-1;

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

void driveTurn(static bool left, static float inches, static int speed=75){

    static float wheelRotations;

    static float clicks;

    wheelRotations = (inches) / (wheelCircumference);

    clicks = wheelRotations*360;

    resetEncoders();

    if(left==true){
        leftEnc=SensorValue[leftEncoder]*-1;

        while(leftEnc<=clicks){
            leftEnc=SensorValue[leftEncoder]*-1;

            motor[left1]=speed;
            motor[left2]=speed;
            motor[left3]=speed;

            motor[right1]=speed;
            motor[right2]=speed;
            motor[right3]=speed;
        }
    }else if(left==false){
        leftEnc=SensorValue[leftEncoder]*1;

        while(leftEnc<=clicks){
            leftEnc=SensorValue[leftEncoder];

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

void skills(){
    autonInit();
	autonLCD();
    resetEncoders();

    autonFront();
    delayFunc(50);
    driveForward(1);
}

void autonFront(){
    autonInit();
    autonLCD();
    resetEncoders();

    startTask(intakeOnTask);

    punch();

    delayFunc(50);

 	if(autonPlatform==false){
        driveForward(1.75);
    	delayFunc(50);
	}else if(autonPlatform==true){
        delayFunc(50);
		driveBackward(.5);
        delayFunc(50);

		if(autonColor==blue){
			driveTurn(false, 7.75, 50);
		}else if(autonColor==red){
			driveTurn(true,7.75, 50);
		}

			driveBackward(.3);
			driveForward(1);

 	}
    intakeOff();
}//void end

void autonBack(){
    autonInit();
    autonLCD();
    resetEncoders();

    startTask(intakeOnTask);

    punch();

    delayFunc(50);

	if(autonPlatform==false){
        delayFunc(50);
    }else if(autonPlatform==true){
		punch();

		driveForward(.5);

		if(autonColor==blue){
			driveTurn(false, 7.75, 50);
		}else if(autonColor==red){
			driveTurn(true,7.75, 50);
		}

		driveBackward(.3);

		driveForward(1);

	}
    intakeOff();
}

void auton(){
    if(autonPos==front){
        autonFront();
    }else if(autonPos==back){
        autonBack();
    }
}

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

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

bool autonRun=false;
bool skillsRun=false;


  // The PID controller that will be used
  // The basic usage of this PID controller is as following:
  //
  // PID pid1;
  // PIDInit(&pid1, PConstant, IConstant, DConstant);
  // float feedback = PIDCompute(&pid1, your_error);
  //
  
  PID pid1;
  PIDInit(&pid1, 0.1, 0, 0.1); // Set P, I, and D constants
  
  // We start at 0 units and want to reach 100 units
  float motorSpeed = 0;
  float targetMotorSpeed = 100;


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
    delayFunc(200);
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

int btnLcdMenu(){
	clearLCD();
	setLCDPosition(0,0);
	displayNextLCDString("Program Select");
	setLCDPosition(1,0);
	displayNextLCDString("Match  No  Skill");
	waitUntil(nLCDButtons != 0);
	if(nLCDButtons>5){
		clearLCD();
		setLCDPosition(0,0);
		displayNextLCDString("Selection  Error");
		return false;
	}else if(nLCDButtons==3){
		clearLCD();
		setLCDPosition(0,0);
		displayNextLCDString("Selection  Error");
		return false;
	}else if(nLCDButtons==1){
		clearLCD();
		setLCDPosition(0,0);
		displayNextLCDString(">Running  Match<");
		autonRun=true;
		return true;
	}else if(nLCDButtons==2){
		clearLCD();
		setLCDPosition(0,0);
		displayNextLCDString("Running  Nothing");
		autonRun=false;
		skillsRun=false;
		return true;
	}else if(nLCDButtons==4){
		clearLCD();
		setLCDPosition(0,0);
		displayNextLCDString(">Program Select<");
		setLCDPosition(1,0);
		displayNextLCDString(">Running Skills<");
		skillsRun=true;
		return true;
	}else
		return false;
}

void init() {

    // Set bStopTasksBetweenModes to false if you want to keep
    // user created tasks running between Autonomous and Driver
    // controlled modes. You will need to manage all user created
    // tasks if set to false.
    bStopTasksBetweenModes = false;

    pidRunning=true;

    pidRequestedValue=0;

    bLCDBacklight=true;

  //  waitUntil(btnLcdMenu());

    delayFunc(100);

    autonRun=true;

    BNS();
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
    stopTask(intakeOn);
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

// These could be constants but leaving
// as variables allows them to be modified in the debugger "live"

float wheelCircumference=4 /*diameter*/ * 3.141529 /*PI simplified*/;

void driveForward(static float tiles, static int speed=75){

    static float wheelRotations;

    static float clicks;

    wheelRotations = (tiles*24) / (wheelCircumference);

    clicks = wheelRotations*360;

    resetEncoders();
  // Output instructions to view the PID response
  writeDebugStreamLine("*** Copy/paste all the results in the debug window to Excel and graph what the PID response looks like! ***");

  // Loop through many times so we can graph
  //  the PID loop
  for(int i = 0; i < 200; i++){
    // This calculates how far off we are from the true value
    //  The PID will return a response that will hopefully minimize this error over time
    float pidResult = PIDCompute(&pid1, targetMotorSpeed - motorSpeed);

    // Add pid to motor value
    motorSpeed = pidResult;

    writeDebugStreamLine("%f", motorSpeed);

    // There is a bug in RobotC where if you print too fast,
    //   you might get weird characters at random
    delay(1);
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

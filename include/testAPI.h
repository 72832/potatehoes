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

bool autonRun=false;
bool skillsRun=false;

float pidRequestedValue;

int leftEnc=SensorValue[leftEncoder]*-1;
int rightEnc=SensorValue[rightEncoder]*-1;

float pidReqVal;


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

    PID pid1;
    PIDInit(&pid1, 1.0, 0.0, 0.0); // Set P, I, and D constants

    // We start at 0 units and want to reach 100 units
    float pidSensorCurrentValue;
    float pidReqVal = -500;

    	// Motion Profile object
	MotionProfile ramp1;
	MotionProfileInit(&ramp1);

	// Initiate the motion profile
	// It is very important that the paramters get set correctly,
	//  otherwise you'll likely get a "IMPOSSIBLE TO REACH LOCATION"
	//  error on your debug stream.
	// Be sure, if you initial velocity is LESS then your max velocity,
	//  to set the acceleration to positive, otherwise negative. Same goes
	//  for max to end velocity as well.  (Think, are you increasing in speed
	//  to reach the next velocity?  Then positive acceleration)
	// Be sure if you want your distance to be positive, the max velocity
	//  is set the same, positive.  And the opposite for negatives.
	// Use MotionProfilePrint(&prof1, #.##); to see the motion profile will
	//  execute on the debug stream
	//
	// ADVANCED PARAMS SETTING:
	//
	// Forward:
	// MotionProfileSetAccel(&prof1, 0.5, -1); // Accel = 0.5, decel = -1
	// MotionProfileSetDistance(&prof1, 500); // Distance = 1200
	// MotionProfileSetVelocity(&prof1, 0, 15, 0); // V_0 = 0, V_max = 15, V_exit = 0
	//
	// Backward:
	// MotionProfileSetAccel(&prof1, -0.5, 1); // Accel = -0.5, decel = 1
	// MotionProfileSetDistance(&prof1, -500); // Distance = -1200
	// MotionProfileSetVelocity(&prof1, 0, -15, 0); // V_0 = 0, V_max = -15, V_exit = 0
	//
	// SIMPLE PARAMS SETTING:
	//
	// Forward:
  	// MotionProfileEasyParams(&ramp1, 1.0, 15.0, 1000.0); // Acceleration of 1, Velocity of 15, and distance of 1200
	//
	// Backward:
	// MotionProfileEasyParams(&prof1, 1.0, 15.0, -1200.0);
	//

	// "Execute" motion profile by printing the velocities to the screen
	float time = 0;
	float dt = 1; // Update rate: in this instance once per second


}

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

/** ------------------- 72832S autonomous functions -----------------*/

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

// PID using optical shaft encoder
//
// Shaft encoder has 360 pulses per revolution
//

#define PID_MOTOR_SCALE     -1

#define PID_INTEGRAL_LIMIT  50

// These could be constants but leaving
// as variables allows them to be modified in the debugger "live"

void driveFunc(int val){
    motor[right1]=val*-1;
    motor[right2]=val*-1;
    motor[right3]=val*-1;

    motor[left1]=val;
    motor[left2]=val;
    motor[left3]=val;
}

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

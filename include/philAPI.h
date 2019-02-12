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

int back=0;

int front=1;

int red=0;

int blue=1;

//auton cutoffs
int cutoffs[] = {0, 512, 2048, 3584, 4096};

//auton reading potentionmeters
int val1 = SensorValue[posPotent];

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

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

/** ----------------- 72832S initialization functions ---------------*/

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

void autonInit(){
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
        motor[intake]=-127;
    }
}

task intakeOffTask() {
    while (true){
        motor[intake]=0;
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
    delayFunc(800);
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

/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                                                                             */
/*  autonomous code                                                            */
/*                                                                             */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

void autonFront(){
    autonInit();

    startTask(intakeOnTask);

    punch();

    delayFunc(50);

    intakeOff();
}//void end

void autonBack(){
    autonInit();

    startTask(intakeOnTask);

    punch();

    delayFunc(50);

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
	if(vexRT[Btn6U]==1){
		motor[intake]=127;
	}else if(vexRT[Btn6D]==1){
		motor[intake]=-127;
    }else{
        motor[intake]=0;
    }

	if(vexRT[Btn8U]==1){
		motor[puncher]=127;
    }else{
		motor[puncher]=0;
    }

    if(vexRT[Btn7U]==1){
	    motor[flipper]=127;
    }else if(vexRT[Btn7D]==1){
        motor[flipper]=-127;
    }else {
  	    motor[flipper]=0;
    }

    if(vexRT[Btn8R]==1){
        waitUntil(vexRT[Btn7D]==0);
        driveReverse=!driveReverse;
    }
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

}

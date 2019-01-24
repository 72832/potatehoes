//
// Created by jonesdav004 on 1/23/2019.
//

#ifndef POTATOES_API_H
#define POTATOES_API_H

#endif //POTATOES_API_H

/** @file API.h
 * @brief Provides the high-level user functionality intended for use by typical VEX Cortex
 * programmers.
 *
 * This file should be included for you in the predefined stubs in each new VEX Cortex PROS
 * project through the inclusion of "main.h". In any new C source file, it is advisable to
 * include main.h instead of referencing API.h by name, to better handle any nomenclature
 * changes to this file or its contents.
 *
 * Copyright (c) 2011-2018, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

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

int autonRun;

bool driveReverse = false;

string mainBattery, backupBattery;

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

/** ----------------- 72832S initialization functions ---------------*/

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

void init() {
    smartMotor();
    resetEncoders();
}

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

/** ------------------- 72832S autonomous functions -----------------*/

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

task puncherOn() {
    while (true)
        puncher = 127;
}

task puncherOff() {
    while (true)
        puncher = 0;
}

task intakeOn() {
    while (true)
        intake = 127;
}

task intakeOff() {
    while (true)
        intake = 0;
}

void punch() {

    //puncher on
    startTask(puncherOn);
    delayFunc(2000);
    stopTask(puncherOn);
    startTask(puncherOff);
    stopTask(puncherOff);
}

//auton
void auton() {

    //1200 from place to flag or to alliance park
    //2000 from place to center
    punch();

    //intake on
    startTask(intakeOn);

    //intake stop
    stopTask(intakeOn);
    startTask(intakeOff);
    stopTask(intakeOff);

}

void auton1() {
    auton();
}

void auton2() {
    auton();
}

void auton3() {
    auton();
}

void auton4() {
    auton();
}

void auton5() {
    auton();
}

void auton6() {
    auton();
}

void auton7() {
    auton();
}

void auton8() {
    auton();
}

void auton9() {
    auton();
}

void auton10() {
    auton();
}

void auton11() {
    auton();
}

void auton12() {
    auton();
}

void auton13() {
    auton();
}

void auton14() {
    auton();
}

void auton15() {
    auton();
}

void auton_1() {
    auton();
}


/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

/** ------------------- 72832S opcontrol functions -----------------*/

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

void motorRefresh(){
    intake=one;
    left=two;
    left=three;
    left=four;
    right=five;
    right=six;
    right=seven;
    eight=0;//unused
    puncher=nine;
    intake=ten;
}

void motorChange(int var, int change=0, int val=0){
    if (change > 3)
        break;
    else if (change < 0)
        break;
    else if (change==0)
        var = 0;
    else if(change==1){
        if (val > 127)
            break;
        else if (val < -127)
            break;
        else {
            var = val;
            break;
        }
    }
    if (change==2){
        if(val>0)
            break;
        else if( (var-val) < -127)
            break;
        else {
            var -= val;
            break;
        }
    }
    if(change==3){
        if (val>0)
            break;
        else if((var+val)>127)
            break;
        else{
            var+=val;
        }
    }
}

void opcontrol(){
    motorSet();

    left=vexRT[Ch2];
    right=vexRT[Ch3];

// Puncher program
    if (vexRT[Btn6U] == 1) {
        motorChange(puncher,1,127);
    } else {
        motorChange(puncher,0,0);
    }

//reverse drive so that you can easily flip caps (find in functions)
    if (vexRT[Btn8D] == 1) {
        if (!driveReverse) {waitUntil(vexRT[Btn8D]== 0);
            driveReverse = true;
            delayFunc(500);
        } else if (driveReverse) {
            waitUntil(vexRT[Btn8D]== 0);
            driveReverse = false;
            delayFunc(500);
        }
    }

// Intake program
    if (vexRT[Btn5U] == 1) {
        motorChange(intake,1,127);
    } else if (vexRT[Btn6D] == 1) {
        motorChange(intake,1,-127);
    } else {
        motorChange(intake,0,0);
    }
    motorRefresh();
}

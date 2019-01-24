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

/** --------------------- 72832S global functions -------------------*/

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

void delayFunc(int time){
	wait1Msec(time);
}

void motorRefresh() {
    right *= -1;

    one = intake;
    two = left;
    three = left;
    four = left;
    five = right;
    six = right;
    seven = right;
    eight = 0;//unused
    nine = puncher;
    ten = intake;
}

void motorVarSet(){
    motor[port1]=one;
    motor[port2]=two;
    motor[port3]=three;
    motor[port4]=four;
    motor[port5]=five;
    motor[port6]=six;
    motor[port7]=seven;
    motor[port8]=eight;
    motor[port9]=nine;
    motor[port10]=ten;
}

void motorChange(int var, int change = 0, int val = 0) {
    if (change > 3)

    else
    if (change < 0)

    else
    if (change == 0)
        var = 0;
    else if (change == 1) {
        if (val > 127)

        else
        if (val < -127)

        else
        {
            var = val;

        }
    }
    if (change == 2) {
        if (val > 0)

        else
        if ((var - val) < -127)

        else
        {
            var -= val;

        }
    }
    if (change == 3) {
        if (val > 0)

        else
        if ((var + val) > 127)

        else
        var += val;
    }
}

void resetEncoders() {
    SensorValue[leftEncoder] = 0;
    SensorValue[rightEncoder] = 0;
}

void smrtMtr() {
    // All activities that occur before the competition starts
    // Example: clearing encoders, setting servo positions, ...
    // Enable smart motor library
    SmartMotorsInit();

    // Define motors plugged into power expander
    // SmartMotorsAddPowerExtender( motorA, motorB, motorC, motorD );

    // Link motors
    SmartMotorLinkMotors(left1, left2);
    SmartMotorLinkMotors(right1, right2);
    // Current monitor
    SmartMotorCurrentMonitorEnable();
    // Smart motor start
    SmartMotorRun();
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
    sprintf(mainBattery, "%1.2f%c", nImmediateBatteryLevel / 1000.0, 'V'); //Build the value to be displayed
    displayNextLCDString(mainBattery);

    //Display the Backup battery voltage
    displayLCDString(1, 0, "Backup: ");
    sprintf(backupBattery, "%1.2f%c", BackupBatteryLevel / 1000.0, 'V');    //Build the value to be displayed
    displayNextLCDString(backupBattery);

    //Short delay for the LCD refresh rate
    wait1Msec(100);
}

void init() {
    smrtMtr();
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

void auton0() {
    auton();
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

void opcontrol(){

    left = vexRT[Ch3];
    right = vexRT[Ch2];

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

    motorVarSet();

}

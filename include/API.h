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

/** ------------------- 72832S opcontrol functions -----------------*/

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

void smartMotor() {
    bStopTasksBetweenModes = false;

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

void intakeFunc(int power1, int power2) {
    intake1Func(power1);
    intake2Func(power2);
}

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

/** ------------------- 72832S autonomous functions -----------------*/

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

/**
 * Auton Placeholder; placeholder for auton
 *
 * @param channel the channel the likelihood of having an paramater
 */

task puncherOn() {
    while (true) {
        puncherFunc(127);
    }
}

/**
 * Auton Placeholder; placeholder for auton
 *
 * @param channel the channel the likelihood of having an paramater
 */

task puncherOff() {
    while (true) {
        puncherFunc(0);
    }
}

/**
 * Auton Placeholder; placeholder for auton
 *
 * @param channel the channel the likelihood of having an paramater
 */

task intakeOn() {
    while (true) {
        intakeFunc(127, 127);
    }
}

/**
 * Auton Placeholder; placeholder for auton
 *
 * @param channel the channel the likelihood of having an paramater
 */

task intakeOff() {
    while (true) {
        intakeFunc(0, 0);
    }
}

/**
 * Auton Placeholder; placeholder for auton
 *
 * @param channel the channel the likelihood of having an paramater
 */

void punch() {

    //puncher on
    startTask(puncherOn);
    delayFunc(2000);
    stopTask(puncherOn);
    startTask(puncherOff);
    stopTask(puncherOff);
}

/**
 * Auton Placeholder; placeholder for auton
 *
 * @param channel the channel the likelihood of having an paramater
 */

void auton1() {
    punch();
}

/**
 * Auton Placeholder; placeholder for auton
 *
 * @param channel the channel the likelihood of having an paramater
 */

void auton2() {
    punch();
}

/**
 * Auton Placeholder; placeholder for auton
 *
 * @param channel the channel the likelihood of having an paramater
 */

void auton3() {
    punch();
}

/**
 * Auton Placeholder; placeholder for auton
 *
 * @param channel the channel the likelihood of having an paramater
 */

void auton4() {
    punch();
}

/**
 * Auton Placeholder; placeholder for auton
 *
 * @param channel the channel the likelihood of having an paramater
 */

void auton5() {
    punch();
}

/**
 * Auton Placeholder; placeholder for auton
 *
 * @param channel the channel the likelihood of having an paramater
 */

void auton6() {
    punch();
}

/**
 * Auton Placeholder; placeholder for auton
 *
 * @param channel the channel the likelihood of having an paramater
 */

void auton7() {
    punch();
}

/**
 * Auton Placeholder; placeholder for auton
 *
 * @param channel the channel the likelihood of having an paramater
 */

void auton8() {
    punch();
}

/**
 * Auton Placeholder; placeholder for auton
 *
 * @param channel the channel the likelihood of having an paramater
 */

void auton9() {
    punch();
}

/**
 * Auton Placeholder; placeholder for auton
 *
 * @param channel the channel the likelihood of having an paramater
 */

void auton10() {
    punch();
}

/**
 * Auton Placeholder; placeholder for auton
 *
 * @param channel the channel the likelihood of having an paramater
 */

void auton11() {
    punch();
}

/**
 * Auton Placeholder; placeholder for auton
 *
 * @param channel the channel the likelihood of having an paramater
 */

void auton12() {
    punch();
}

/**
 * Auton Placeholder; placeholder for auton
 *
 * @param channel the channel the likelihood of having an paramater
 */

void auton13() {
    punch();
}

/**
 * Auton Placeholder; placeholder for auton
 *
 * @param channel the channel the likelihood of having an paramater
 */

void auton14() {
    punch();
}

/**
 * Auton Placeholder; placeholder for auton
 *
 * @param channel the channel the likelihood of having an paramater
 */

void auton15() {
    punch();
}

/**
 * Auton Placeholder; placeholder for auton
 *
 * @param channel the channel the likelihood of having an paramater
 */

void auton_1() {
    punch();
}
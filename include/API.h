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

int autonRun;

//auton cutoffs
int cutoffs[] = {-4096, -2048, 0, 2048, 4096};

//auton reading potentionmeters
int val1 = SensorValue[posPotent];
int val2 = SensorValue[progPotent];

bool driveReverse = false;

string mainBattery, backupBattery;

int leftEnc=SensorValue[leftEncoder]*-1;
int rightEnc=SensorValue[rightEncoder];

int maxVal=127;
int minVal=-127;

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
    five = puncher;
    six = 0;//unused
    seven = right;
    eight = right;
    nine = right;
    ten = intake;
}

void motorVarSet(){
    motor[port1]=one;
    SetMotor(2,two);
    SetMotor(3,three);
    SetMotor(4,four);
    motor[port5]=five;
    motor[port6]=six;
    SetMotor(7,seven);
    SetMotor(8,eight);
    SetMotor(9,nine);
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
        if (val > maxVal)

        else
        if (val < minVal)

        else
        {
            var = val;

        }
    }
    if (change == 2) {
        if (val > 0)

        else
        if ((var - val) < minVal)

        else
        {
            var -= val;

        }
    }
    if (change == 3) {
        if (val > 0)

        else
        if ((var + val) > maxVal)

        else
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
    if (val2 >= cutoffs[0] && val2 < cutoffs[1])
        autonRun = 0;
    else if (val2 >= cutoffs[1] && val2 < cutoffs[2])
        autonRun = 1;
    else if (val2 >= cutoffs[2] && val2 < cutoffs[3])
        autonRun = 2;
    else if (val2 >= cutoffs[3] && val2 < cutoffs[4])
        autonRun = 3;
} else if (val1 >= cutoffs[1] && val1 < cutoffs[2]) {
    if (val2 >= cutoffs[0] && val2 < cutoffs[1])
        autonRun = 4;
    else if (val2 >= cutoffs[1] && val2 < cutoffs[2])
        autonRun = 5;
    else if (val2 >= cutoffs[2] && val2 < cutoffs[3])
        autonRun = 6;
    else if (val2 >= cutoffs[3] && val2 < cutoffs[4])
        autonRun = 7;
} else if (val1 >= cutoffs[2] && val1 < cutoffs[3]) {
    if (val2 >= cutoffs[0] && val2 < cutoffs[1])
        autonRun = 8;
    else if (val2 >= cutoffs[1] && val2 < cutoffs[2])
        autonRun = 9;
    else if (val2 >= cutoffs[2] && val2 < cutoffs[3])
        autonRun = 10;
    else if (val2 >= cutoffs[3] && val2 < cutoffs[4])
        autonRun = 11;
} else if (val1 >= cutoffs[3] && val1 < cutoffs[4]) {
    if (val2 >= cutoffs[0] && val2 < cutoffs[1])
        autonRun = 12;
    else if (val2 >= cutoffs[1] && val2 < cutoffs[2])
        autonRun = 13;
    else if (val2 >= cutoffs[2] && val2 < cutoffs[3])
        autonRun = 14;
    else if (val2 >= cutoffs[3] && val2 < cutoffs[4])
        autonRun = 15;
}else{
    autonRun = -1;
}
    resetEncoders();
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

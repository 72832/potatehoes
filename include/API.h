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

int autonProg;

int autonColor;

int autonPos;

int back=0;

int front=1;

int red=0;

int blue=1;

int normal =0;

int skills=1;

int alt1=2;

int alt2=3;

//auton cutoffs
int cutoffs[] = {0, 1024, 2048, 3072, 4096};

//auton reading potentionmeters
int val1 = SensorValue[posPotent];
int val2 = SensorValue[progPotent];

string mainBattery, backupBattery;

bool pidRunning=false;

float pidRequestedValue;

float pid_Kp=1.0;
float pid_Ki=0.0;
float pid_Kd=0.0;

float leftChange;
float rightChange;

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
    bLCDBacklight=true;

    clearLCD();

    setLCDPosition(0,0);

    if(autonColor==0){

        displayNextLCDString("Color = Red");

    }else if(autonColor==1){

        displayNextLCDString("Color = Blue");

    }

    setLCDPosition(1,0);

    if(autonPos==0){

        displayNextLCDString("Pos = Back");

    }else if(autonPos==1){

        displayNextLCDString("Pos = Front");

    }
}

void init() {
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

    if (val2 >= cutoffs[0] && val2 < cutoffs[1]) {
        autonProg=normal;
    } else if (val2 >= cutoffs[1] && val2 < cutoffs[2]) {
        autonProg=skills;
    } else if (val2 >= cutoffs[2] && val2 < cutoffs[3]) {
        autonProg=alt1;
    } else if (val2 >= cutoffs[3] && val2 < cutoffs[4]) {
        autonProg=alt2;
    }

    resetEncoders();
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
        motor[intake2]=127;
    }
}

task intakeOffTask() {
    while (true){
        motor[intake2]=0;
    }
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
    delayFunc(1000);
    stopTask(puncherOnTask);
    startTask(puncherOffTask);
    stopTask(puncherOffTask);
}

void bckFwdIntkBck(){
    while(leftEnc>=-200){
        resetEncoders();

        leftEnc=SensorValue[leftEncoder]*1;
        rightEnc=SensorValue[rightEncoder]*1;

            // send to motor
        motor[left1] = 90;
        motor[left2] = 90;
        motor[left3] = 90;

        motor[right1] = -90;
        motor[right2] = -90;
        motor[right3] = -90;
    }

        resetEncoders();
        while(leftEnc<=1100){

            leftEnc=SensorValue[leftEncoder]*1;
            rightEnc=SensorValue[rightEncoder]*1;

            // send to motor
            motor[left1] = -90;
            motor[left2] = -90;
            motor[left3] = -90;

            motor[right1] = 90;
            motor[right2] = 90;
            motor[right3] = 90;
        }

        for(int k; k<=10;k++){
            motor[intake1]=90;
            motor[intake2]=90;

            delayFunc(100);
        }

        motor[intake1]=0;
        motor[intake2]=0;

        resetEncoders();
        while(leftEnc<=400){

            leftEnc=SensorValue[leftEncoder]*-1;
            rightEnc=SensorValue[rightEncoder]*1;

                // send to motor
            motor[left1] = 90;
            motor[left2] = 90;
            motor[left3] = 90;

            motor[right1] = -90;
            motor[right2] = -90;
            motor[right3] = -90;
        }
}

void bckFwdFront(){
            resetEncoders();
            while(leftEnc<=200){

                leftEnc=SensorValue[leftEncoder]*-1;
                rightEnc=SensorValue[rightEncoder]*1;

                // send to motor
                motor[left1] = 90;
                motor[left2] = 90;
                motor[left3] = 90;

                motor[right1] = -90;
                motor[right2] = -90;
                motor[right3] = -90;
            }

            resetEncoders();
            while(leftEnc<=400){

                leftEnc=SensorValue[leftEncoder]*1;
                rightEnc=SensorValue[rightEncoder]*1;

                // send to motor
                motor[left1] = -90;
                motor[left2] = -90;
                motor[left3] = -90;

                motor[right1] = 90;
                motor[right2] = 90;
                motor[right3] = 90;
            }
}

void bckBckFront(){
            resetEncoders();
            while(leftEnc<=400){

                leftEnc=SensorValue[leftEncoder]*-1;
                rightEnc=SensorValue[rightEncoder]*1;

                // send to motor
                motor[left1] = 90;
                motor[left2] = 90;
                motor[left3] = 90;

                motor[right1] = -90;
                motor[right2] = -90;
                motor[right3] = -90;
            }

            delayFunc(1000);

            resetEncoders();
            while(leftEnc<=600){

                leftEnc=SensorValue[leftEncoder]*-1;
                rightEnc=SensorValue[rightEncoder]*1;

                // send to motor
                motor[left1] = 90;
                motor[left2] = 90;
                motor[left3] = 90;

                motor[right1] = -90;
                motor[right2] = -90;
                motor[right3] = -90;
            }
}

void fwdBckFwdBack(){
            resetEncoders();
            while(leftEnc<=200){

                leftEnc=SensorValue[leftEncoder]*1;
                rightEnc=SensorValue[rightEncoder]*1;

                // send to motor
                motor[left1] = -90;
                motor[left2] = -90;
                motor[left3] = -90;

                motor[right1] = 90;
                motor[right2] = 90;
                motor[right3] = 90;
            }

            resetEncoders();
            while(leftEnc<=1100){

                leftEnc=SensorValue[leftEncoder]*-1;
                rightEnc=SensorValue[rightEncoder]*1;

                // send to motor
                motor[left1] = 90;
                motor[left2] = 90;
                motor[left3] = 90;

                motor[right1] = -90;
                motor[right2] = -90;
                motor[right3] = -90;
            }

            resetEncoders();
            while(leftEnc<=400){

                leftEnc=SensorValue[leftEncoder]*1;
                rightEnc=SensorValue[rightEncoder]*1;

                // send to motor
                motor[left1] = -90;
                motor[left2] = -90;
                motor[left3] = -90;

                motor[right1] = 90;
                motor[right2] = 90;
                motor[right3] = 90;
            }
}


void bckFwdFwdBack(){
            resetEncoders();
            while(leftEnc<=400){

                leftEnc=SensorValue[leftEncoder]*-1;
                rightEnc=SensorValue[rightEncoder]*1;

                // send to motor
                motor[left1] = 90;
                motor[left2] = 90;
                motor[left3] = 90;

                motor[right1] = -90;
                motor[right2] = -90;
                motor[right3] = -90;
            }

            resetEncoders();
            while(leftEnc<=1100){

                leftEnc=SensorValue[leftEncoder]*1;
                rightEnc=SensorValue[rightEncoder]*1;

                // send to motor
                motor[left1] = -90;
                motor[left2] = -90;
                motor[left3] = -90;

                motor[right1] = 90;
                motor[right2] = 90;
                motor[right3] = 90;
            }

            delayFunc(1000);

            resetEncoders();
            while(leftEnc<=500){

                leftEnc=SensorValue[leftEncoder]*1;
                rightEnc=SensorValue[rightEncoder]*1;

                // send to motor
                motor[left1] = -90;
                motor[left2] = -90;
                motor[left3] = -90;

                motor[right1] = 90;
                motor[right2] = 90;
                motor[right3] = 90;
            }
}

/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                                                                             */
/*  autonomous code                                                            */
/*                                                                             */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

void auton(){
    resetEncoders();

	autonLCD();

    if(autonProg==normal){
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*  autonomous red                                                             */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

        if(autonColor==red){
            if(autonPos==back){
                resetEncoders();
                while(leftEnc<=200){

                    leftEnc=SensorValue[leftEncoder]*-1;
                    rightEnc=SensorValue[rightEncoder]*1;

                    // send to motor
                    motor[left1] = 90;
                    motor[left2] = 90;
                    motor[left3] = 90;

                    motor[right1] = -90;
                    motor[right2] = -90;
                    motor[right3] = -90;
                }

                punch();

                resetEncoders();
                while(leftEnc<=200){

                    leftEnc=SensorValue[leftEncoder]*1;
                    rightEnc=SensorValue[rightEncoder]*1;

                    // send to motor
                    motor[left1] = -90;
                    motor[left2] = -90;
                    motor[left3] = -90;

                    motor[right1] = 90;
                    motor[right2] = 90;
                    motor[right3] = 90;
                }
            }else{
                punch();
            }

            resetEncoders();
            while(leftEnc<=600){
                leftEnc=SensorValue[leftEncoder]*1;
                rightEnc=SensorValue[rightEncoder]*1;

                // send to motor
                motor[left1] = -75;
                motor[left2] = -75;
                motor[left3] = -75;

                motor[right1] = -75;
                motor[right2] = -75;
                motor[right3] = -75;

            }

            bckFwdIntkBck();

            resetEncoders();
            while(leftEnc<=600){
                leftEnc=SensorValue[leftEncoder]*-1;
                rightEnc=SensorValue[rightEncoder]*1;

                // send to motor
                motor[left1] = 50;
                motor[left2] = 50;
                motor[left3] = 50;

                motor[right1] = 75;
                motor[right2] = 75;
                motor[right3] = 75;

            }


/*-----------------------------------------------------------------------------*/
/*  autonomous front                                                           */
/*-----------------------------------------------------------------------------*/

            if(autonPos==front){

                bckFwdFront();

                resetEncoders();
                while(leftEnc<=300){
                    leftEnc=SensorValue[leftEncoder]*1;
                    rightEnc=SensorValue[rightEncoder]*1;

                    // send to motor
                    motor[left1] = -75;
                    motor[left2] = -75;
                    motor[left3] = -75;

                    motor[right1] = -75;
                    motor[right2] = -75;
                    motor[right3] = -75;
                }

                punch();

                resetEncoders();
                while(leftEnc<=300){
                    leftEnc=SensorValue[leftEncoder]*-1;
                    rightEnc=SensorValue[rightEncoder]*1;

                    // send to motor
                    motor[left1] = 75;
                    motor[left2] = 75;
                    motor[left3] = 75;

                    motor[right1] = 75;
                    motor[right2] = 75;
                    motor[right3] = 75;
                }

                bckBckFront();
            }

/*-----------------------------------------------------------------------------*/
/*  autonomous back                                                            */
/*-----------------------------------------------------------------------------*/

            else if(autonPos==back){

                fwdBckFwdBack();

                resetEncoders();
                while(leftEnc<=300){

                    leftEnc=SensorValue[leftEncoder]*1;
                    rightEnc=SensorValue[rightEncoder]*1;

                    // send to motor
                    motor[left1] = -75;
                    motor[left2] = -75;
                    motor[left3] = -75;

                    motor[right1] = -75;
                    motor[right2] = -75;
                    motor[right3] = -75;
                }

                punch();

                resetEncoders();
                while(leftEnc<=300){
                    leftEnc=SensorValue[leftEncoder]*1;
                    rightEnc=SensorValue[rightEncoder]*1;

                    // send to motor
                    motor[left1] = 75;
                    motor[left2] = 75;
                    motor[left3] = 75;

                    motor[right1] = 75;
                    motor[right2] = 75;
                    motor[right3] = 75;
                }

                bckFwdFwdBack();

            }
        }

/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*  autonomous blue                                                            */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

        else if(autonColor==blue){
            if(autonPos==back){
                resetEncoders();
                while(leftEnc<=200){

                    leftEnc=SensorValue[leftEncoder]*-1;
                    rightEnc=SensorValue[rightEncoder]*1;

                    // send to motor
                    motor[left1] = 90;
                    motor[left2] = 90;
                    motor[left3] = 90;

                    motor[right1] = -90;
                    motor[right2] = -90;
                    motor[right3] = -90;
                }

                punch();

                resetEncoders();
                while(leftEnc<=200){

                    leftEnc=SensorValue[leftEncoder]*1;
                    rightEnc=SensorValue[rightEncoder]*1;

                    // send to motor
                    motor[left1] = -90;
                    motor[left2] = -90;
                    motor[left3] = -90;

                    motor[right1] = 90;
                    motor[right2] = 90;
                    motor[right3] = 90;
                }
            }else{
                punch();
            }

            resetEncoders();
            while(leftEnc<=600){
                leftEnc=SensorValue[leftEncoder]*-1;
                rightEnc=SensorValue[rightEncoder]*1;

                // send to motor
                motor[left1] = 75;
                motor[left2] = 75;
                motor[left3] = 75;

                motor[right1] = 75;
                motor[right2] = 75;
                motor[right3] = 75;

            }

            bckFwdIntkBck();

            resetEncoders();
            while(leftEnc<=600){
                leftEnc=SensorValue[leftEncoder]*1;
                rightEnc=SensorValue[rightEncoder]*1;

                // send to motor
                motor[left1] = -75;
                motor[left2] = -75;
                motor[left3] = -75;

                motor[right1] = -75;
                motor[right2] = -75;
                motor[right3] = -75;

            }

/*-----------------------------------------------------------------------------*/
/*  autonomous front                                                           */
/*-----------------------------------------------------------------------------*/

            if(autonPos==front){
                resetEncoders();
                while(leftEnc<=600){
                    leftEnc=SensorValue[leftEncoder]*1;
                    rightEnc=SensorValue[rightEncoder]*1;

                    // send to motor
                    motor[left1] = -75;
                    motor[left2] = -75;
                    motor[left3] = -75;

                    motor[right1] = -75;
                    motor[right2] = -75;
                    motor[right3] = -75;
                }

                bckFwdFront();

                resetEncoders();
                while(leftEnc<=300){
                    leftEnc=SensorValue[leftEncoder]*-1;
                    rightEnc=SensorValue[rightEncoder]*1;

                    // send to motor
                    motor[left1] = 75;
                    motor[left2] = 75;
                    motor[left3] = 75;

                    motor[right1] = 75;
                    motor[right2] = 75;
                    motor[right3] = 75;
                }

                punch();

                resetEncoders();
                while(leftEnc<=300){
                    leftEnc=SensorValue[leftEncoder]*1;
                    rightEnc=SensorValue[rightEncoder]*1;

                    // send to motor
                    motor[left1] = -75;
                    motor[left2] = -75;
                    motor[left3] = -75;

                    motor[right1] = -75;
                    motor[right2] = -75;
                    motor[right3] = -75;
                }

                bckBckFront();
            }

/*-----------------------------------------------------------------------------*/
/*  autonomous back                                                            */
/*-----------------------------------------------------------------------------*/

            else if(autonPos==back){

                fwdBckFwdBack();

                resetEncoders();
                while(leftEnc<=300){

                    leftEnc=SensorValue[leftEncoder]*1;
                    rightEnc=SensorValue[rightEncoder]*1;

                    // send to motor
                    motor[left1] = -75;
                    motor[left2] = -75;
                    motor[left3] = -75;

                    motor[right1] = -75;
                    motor[right2] = -75;
                    motor[right3] = -75;
                }

                punch();

                resetEncoders();
                while(leftEnc<=300){
                    leftEnc=SensorValue[leftEncoder]*1;
                    rightEnc=SensorValue[rightEncoder]*1;

                    // send to motor
                    motor[left1] = 75;
                    motor[left2] = 75;
                    motor[left3] = 75;

                    motor[right1] = 75;
                    motor[right2] = 75;
                    motor[right3] = 75;
                }

                bckFwdFwdBack();
            }//auton back
        }//auton color
    }//auton prog
}//void end

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

/** ------------------- 72832S opcontrol functions -----------------*/

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

void opcontrol(){

    SetMotor(left1,vexRT[Ch3]*-1);
	SetMotor(left2,vexRT[Ch3]*-1);
    SetMotor(left3,vexRT[Ch3]*-1);

    SetMotor(right1,vexRT[Ch2]);
    SetMotor(right2,vexRT[Ch2]);
	SetMotor(right3,vexRT[Ch2]);

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
    }else if( vexRT[Btn6D]==1){
		motor[puncher]=-127;
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

    if(vexRT[Btn7R]==1){
        auton();
    }
}

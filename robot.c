#pragma config(Sensor, in1,    posPotent,      sensorPotentiometer)
#pragma config(Sensor, in2,    progPotent,     sensorPotentiometer)
#pragma config(Sensor, dgtl1,  rightEncoder,   sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  leftEncoder,    sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  leftSonar,      sensorSONAR_mm)
#pragma config(Motor,  port1,           intake1,       tmotorVex393TurboSpeed_HBridge, openLoop)
#pragma config(Motor,  port2,           left1,         tmotorNone, openLoop)
#pragma config(Motor,  port3,           left2,         tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port4,           left3,         tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port5,           puncher,       tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port6,           noUse,         tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port7,           right3,        tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port8,           right2,        tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port9,           right1,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          intake2,       tmotorVex393TurboSpeed_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

/** --------------------- 72832S competition ctrl -------------------*/

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

// Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(15)
#pragma userControlDuration(105)

#pragma systemFile

/*---------------------------------------------------------------------------*/
/*                                      9                                     */
/*        Description: Competition template for VEX EDR                      */
/*                                                                           */
/*---------------------------------------------------------------------------*/

// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

// Main competition background code...do not modify!

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

/** ---------------------- 72832S include files ---------------------*/

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

#include "cleanerDirectory/jpearman/SmartMotorLib.c"

#include "Vex_Competition_Includes.c"  // Main competition background code...do not modify!

#include "include/API.h"

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

/** ------------------------ 72832S pre auton -----------------------*/

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

void pre_auton() {
    // Set bStopTasksBetweenModes to false if you want to keep user created tasks
    // running between Autonomous and Driver controlled modes. You will need to
    // manage all user created tasks if set to false.
    init();
}

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

/** ------------------- 72832S autonomous functions -----------------*/

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

task autonomous() {
	auton();
}

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

/** ------------------- 72832S opcontrol functions ------------------*/

/*********************************************************************/
/*********************************************************************/
/*********************************************************************/

task usercontrol() {

    //display battery voltage
    lcdBattery();

    bLCDBacklight = true;                                    // Turn on LCD Backlight

    pidRunning=false;

    while (true) {
      opcontrol();
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

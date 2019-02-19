/** @file init.c
 * @brief File for initialization code
 *
 * This file should contain the user initialize() function and any functions related to it.
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

/*
 * Runs pre-initialization code. This function will be started in kernel mode one time while the
 * VEX Cortex is starting up. As the scheduler is still paused, most API functions will fail.
 *
 * The purpose of this function is solely to set the default pin modes (pinMode()) and port
 * states (digitalWrite()) of limit ifes, push buttons, and solenoids. It can also safely
 * configure a UART port (usartOpen()) but cannot set up an LCD (lcd)Init()).
 */
void initializeIO() {
    uartOpen(2);
}

/*
 * Runs user initialization code. This function will be started in its own task with the default
 * priority and stack size once when the robot is starting up. It is possible that the VEXnet
 * communication link may not be fully established at this time, so reading from the VEX
 * Joystick may fail.
 *
 * This function should initialize most sensors (gyro, encoders, ultrasonics), LCDs, global
 * variables, and IMEs.
 *
 * This function must exit relatively promptly, or the operatorControl() and autonomous() tasks
 * will not start. An autonomous mode selection menu like the pre_auton() in other environments
 * can be implemented in this task if desired.
 */
void initialize() {
    //encoder initializing
    leftEncode encoderInit(1, 2, true);
    rightEncode encoderInit(3, 4, false);

    //ultrasonic initializing
    left1Sonic ultrasonicInit(5, 6);
    left2Sonic ultrasonicInit(7, 8);
    right1Sonic ultrasonicInit(9, 10);
    right2Sonic ultrasonicInit(11, 12);

    //LCD Initiallize
    lcdInit(uart2);
    lcdSetBacklight(uart2, true);
    lcdClear(uart2);
    lcdSetText(uart2, 1, "LCD INITITIALIZE");

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
    } else {
        autonRun = -1;
    }
}

lcdClear(uart2);
lcdSetText(uart2,1, "Run Auton Prog");
lcdSetText(uart2,2, autonRun);
}
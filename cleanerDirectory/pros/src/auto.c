/** @file auto.c
 * @brief File for autonomous code
 *
 * This file should contain the user autonomous() function and any functions related to it.
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

/*
 * Runs the user autonomous code. This function will be started in its own task with the default
 * priority and stack size whenever the robot is enabled via the Field Management System or the
 * VEX Competition Switch in the autonomous mode. If the robot is disabled or communications is
 * lost, the autonomous task will be stopped by the kernel. Re-enabling the robot will restart
 * the task, not re-start it from where it left off.
 *
 * Code running in the autonomous task cannot access information from the VEX Joystick. However,
 * the autonomous function can be invoked from another task if a VEX Competition Switch is not
 * available, and it can access joystick information if called in this way.
 *
 * The autonomous task may exit, unlike operatorControl() which should never exit. If it does
 * so, the robot will await a switch to another mode or disable/enable cycle.
 */
void autonomous() {
    switch (autonRun) {
        case autonRun=0:
            auton0();
            break;
        case autonRun=1:
            auton1();
            break;
        case autonRun=2:
            auton2();
            break;
        case autonRun=3:
            auton3();
            break;
        case autonRun=4:
            auton4();
            break;
        case autonRun=5:
            auton5();
            break;
        case autonRun=6:
            auton6();
            break;
        case autonRun=7:
            auton7();
            break;
        case autonRun=8:
            auton8();
            break;
        case autonRun=9:
            auton9();
            break;
        case autonRun=10:
            auton10();
            break;
        case autonRun=11:
            auton11();
            break;
        case autonRun=12:
            auton12();
            break;
        case autonRun=13:
            auton13();
            break;
        case autonRun=14:
            auton14();
            break;
        case autonRun=15:
            auton15();
            break;
        default:
            auton_1();
            break;
    }
}
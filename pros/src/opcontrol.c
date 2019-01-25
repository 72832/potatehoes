/** @file opcontrol.c
 * @brief File for operator control code
 *
 * This file should contain the user operatorControl() function and any functions related to it.
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

/*
 * Runs the user operator control code. This function will be started in its own task with the
 * default priority and stack size whenever the robot is enabled via the Field Management System
 * or the VEX Competition Switch in the operator control mode. If the robot is disabled or
 * communications is lost, the operator control task will be stopped by the kernel. Re-enabling
 * the robot will restart the task, not resume it from where it left off.
 *
 * If no VEX Competition Switch or Field Management system is plugged in, the VEX Cortex will
 * run the operator control task. Be warned that this will also occur if the VEX Cortex is
 * tethered directly to a computer via the USB A to A cable without any VEX Joystick attached.
 *
 * Code running in this task can take almost any action, as the VEX Joystick is available and
 * the scheduler is operational. However, proper use of delay() or taskDelayUntil() is highly
 * recommended to give other tasks (including system tasks such as updating LCDs) time to run.
 *
 * This task should never exit; it should end with some kind of infinite loop, even if empty.
 */
void operatorControl() {
	while(true){
		// drive program
		drive();

		// Puncher program
        if (joystickGetDigital(joyMain, 6, JOY_UP) == 1) {
			puncherFunc(127);
		}else{
			puncherFunc(0);
		}

        if (joystickGetDigital(joyMain, 7, JOY_UP) == 1) {
			auton();
		}

        if (joystickGetDigital(joyMain, 7, JOY_DOWN) == 1) {
			lcdBattery();
		}

		//reverse drive so that you can easily flip caps (find in functions)
        if (joystickGetDigital(joyMain, 8, JOY_DOWN) == 1) {
			if(driveReverse==false){
                waitUntil(joystickGetDigital(joyMain, 8, JOY_DOWN), 1000);
				driveReverse=true;
				delayFunc(500);
			}else if(driveReverse==true){
                waitUntil(joystickGetDigital(joyMain, 8, JOY_DOWN), 1000);
				driveReverse=false;
				delayFunc(500);
			}
		}

		// Intake program
        if (joystickGetDigital(joyMain, 5, JOY_UP) == 1) {
			 intake1Func(127);
			 intake2Func(127);
        } else if (joystickGetDigital(joyMain, 6, JOY_DOWN) == 1) {
				intake1Func(-127);
        } else if (joystickGetDigital(joyMain, 5, JOY_DOWN) == 1) {
				intake2Func(-127);
			}else{
				intake1Func(0);
				intake2Func(0);
			}
		}
}
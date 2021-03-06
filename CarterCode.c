#include "SmartMotorLib.c"

//--------------------------------------Driver control--------------------------------------------------------------
task main()
{
	//while driver control is active...
	while (true) {

	//DRIVE PROGRAM: right drive motors should be on ports 3, 7 and 9. Left drive motors should be set to 2, 4, and 8.

			//gets right stick input and drives right motors
			motor [right1] = (vexRT[Ch3]*-1);
			motor [right2] = (vexRT[Ch3]*-1);
			motor [right3] = (vexRT[Ch3]*-1);

			//gets left stick input and drives left motors
			motor [left1] = (vexRT[Ch2]*-1);
			motor [left2] = (vexRT[Ch2]*-1);
			motor [left3] = (vexRT[Ch2]*-1);


	//PUNCHER PROGRAM: Puncher motors should be plugged into ports 5 and 6

			//gets input and drives puncher motors
			//if left back trigger is pressed
			if (vexRT[Btn5D] == 1) {
				motor[puncher1] = 127;
				motor[puncher2] = 127;
			//if left back trigger is not pressed, do not drive the puncher
			}else{
				motor[puncher1] = 0;
				motor[puncher2] = 0;
		}


	//INTAKE PROGRAM: Intake motor should be plugged into port 1

		//gets input and drives intake motor
		//if right back trigger is pressed, drive intake forward
		if (vexRT[Btn6D] == 1) {
			motor[intake] = 127;

		//if right front trigger is pressed, drive intake backward
		} else if (vexRT[Btn6U] == 1) {
			motor[intake] = -127;

		//if none of those buttons are pressed, do not drive the intake
		} else {
			motor[intake] = 0;
		}

	}
}

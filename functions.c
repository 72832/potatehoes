#pragma platform(VEX)

#pragma competitionControl(Competition)
#pragma autonomousDuration(15)
#pragma userControlDuration(105)

void resetEncoders(){
	SensorValue[ leftEncoder ] = 0;
  SensorValue[ rightEncoder ] = 0;
}

//after ints
//sensor encoders int
int leftEncode(){
	return(SensorValue[leftEncoder]);
}

int rightEncode(){
	return(SensorValue[rightEncoder]);
}

/*
int liftLPotent(){
	return(SensorValue[liftLeft]);
}

int liftRPotent(){
	return(SensorValue[liftRight]);
}
*/

void delayFunc(int time){
	wait1Msec(time);
}

string mainBattery, backupBattery;

// User control drive functions
void leftFunc(int speed1){
	speed1*=1.1;
	SetMotor(left1,speed1);
	SetMotor(left2,speed1);
	SetMotor(left3,speed1);
}

void rightFunc(int speed2){
	speed2*=1.0;
	SetMotor(right1,speed2);
	SetMotor(right2,speed2);
	SetMotor(right3,speed2);
}

bool driveReverse=false;

void drive(){
	if(driveReverse==true){
		leftFunc(vexRT[Ch2]*-1);
		rightFunc(vexRT[Ch3]*-1);
	} else if(driveReverse==false){
		rightFunc(vexRT[Ch2]);
		leftFunc(vexRT[Ch3]);
	}
}

void puncherFunc(int power){
	motor[puncher]=power;
}

void intakeFunc(int power){
	motor[intake]=power;
}

void liftFunc(int power){
	motor[lift]=power;
}

void flipperFunc(int power){
	motor[flipper]=power;
}

//lcd clear
void clearLCD(){
	clearLCDLine(0);
	clearLCDLine(1);
}

//void lcd display voltage
void lcdBattery(){

	clearLCD();

	//Display the Primary Robot battery voltage
	displayLCDString(0, 0, "Primary: ");
	sprintf(mainBattery, "%1.2f%c", nImmediateBatteryLevel/1000.0,'V'); //Build the value to be displayed
	displayNextLCDString(mainBattery);

	//Display the Backup battery voltage
	displayLCDString(1, 0, "Backup: ");
	sprintf(backupBattery, "%1.2f%c", BackupBatteryLevel/1000.0, 'V');	//Build the value to be displayed
	displayNextLCDString(backupBattery);

	//Short delay for the LCD refresh rate
	wait1Msec(100);
}

task lcdEncode(){
	clearLCD();
	while(true){
		displayLCDPos(0,0);
		displayNextLCDString("> Enc: ");
		displayNextLCDNumber(rightEncode());
		displayLCDPos(1,0);
		displayNextLCDString("< Enc:  ");
		displayNextLCDNumber(leftEncode());
	}
}

task lcdPWM(){
	clearLCD();
	while(true){
		displayLCDPos(0,0);
		displayNextLCDString("Right PWM: ");
		displayNextLCDNumber(vexRT[Ch3]);
		displayLCDPos(1,0);
		displayNextLCDString("Left PWM: ");
		displayNextLCDNumber(vexRT[Ch2]);
	}
}

/*
//Wait for Press--------------------------------------------------
void waitForPress()
{
while(nLCDButtons == 0){}
wait1Msec(5);
}
//----------------------------------------------------------------
//Wait for Release------------------------------------------------
void waitForRelease()
{
while(nLCDButtons != 0){}
wait1Msec(5);
}
//----------------------------------------------------------------
const short leftButton = 1;
const short centerButton = 2;
const short rightButton = 4;
void menuFunc(){
	clearLCD();
	displayLCDCenteredString(0,"------STOP------");
	displayLCDCenteredString(1,"----PROGRAM?----");
	waitForPress();
	if(nLCDButtons==centerButton){
		waitForRelease();
		stopTask(usercontrol);
		stopTask(autonomous);
		clearLCD();
		displayLCDCenteredString(0,"----PROGRAM-----");
		displayLCDCenteredString(1,"----STOPPED-----");
	}
	clearLCD();
	displayLCDCenteredString(0,"-----START------");
	displayLCDCenteredString(1,"----PROGRAM?----");
	waitForPress();
	if(nLCDButtons==centerButton){
		waitForRelease();
		startTask(usercontrol);
		clearLCD();
		displayLCDCenteredString(0,"----PROGRAM-----");
		displayLCDCenteredString(1,"----RUNNING-----");
	}
}
*/
//lcd driver control
task lcd(){
	clearLCD();

	lcdBattery();
	delayFunc(1500);
	clearLCD();
	if( nVexRCReceiveState & 0x02 )
	{
		// second joystick is connected
		displayLCDCenteredString(0,"--PARTNER CTRl--");
		displayLCDCenteredString(1,"----WORKING ----");
		delayFunc(1000);
		clearLCD();
	}else{
		// second joystick is connected
		displayLCDCenteredString(0,"--PARTNER CTRl--");
		displayLCDCenteredString(1,"--NOT WORKING---");
		delayFunc(1000);
		clearLCD();
	}

	while(true){
//		if(nLCDButtons!=0){
//			waitForRelease();
//			menuFunc();
//		}else{


			startTask(lcdEncode);
			delayFunc(1000);
			stopTask(lcdEncode);
			clearLCD();

			startTask(lcdPWM);
			delayFunc(1000);
			stopTask(lcdPWM);
			clearLCD();
//		}
	}
}

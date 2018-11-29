#pragma platform(VEX)

#pragma competitionControl(Competition)
#pragma autonomousDuration(15)
#pragma userControlDuration(105)

//after ints
//sensor encoders int
int leftEncode(){
	return(SensorValue[leftEncoder]);
}

int rightEncode(){
	return(SensorValue[rightEncoder]);
}

void delayFunc(int time){
	wait1Msec(time);
}

string mainBattery, backupBattery;

// User control drive functions
void driveFunc(int speed1, int speed2){
	SetMotor(left1,speed1);
	SetMotor(right1,speed2);
	SetMotor(left2,speed1);
	SetMotor(right2,speed2);
	SetMotor(left3,speed1);
	SetMotor(right3,speed2);
}
void drive(){
	driveFunc(vexRT[Ch3], vexRT[Ch2]);
}

// Stop motor
void stopMotor(int time){
	wait1Msec(time);
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
  	displayNextLCDString("Right Encode: ");
		displayNextLCDNumber(rightEncode());
		displayLCDPos(1,0);
		displayNextLCDString("Left Encode:  ");
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

//lcd driver control
task lcd(){
	clearLCD();
	displayLCDCenteredString(0,"--INITIALIZING--");
	displayLCDCenteredString(1,"--USER CONTROL--");
	delayFunc(2000);
	clearLCD();

	lcdBattery();
	delayFunc(2000);
	clearLCD();

	while(true){
  	startTask(lcdEncode);
		delayFunc(1000);
		stopTask(lcdEncode);
		clearLCD();

		startTask(lcdPWM);
		delayFunc(1000);
		stopTask(lcdPWM);
		clearLCD();
	}
}

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
void leftFunc(int speed1){
	SetMotor(left1,speed1);
	SetMotor(left2,speed1);
	SetMotor(left3,speed1);
}
void rightFunc(int speed2){
	SetMotor(right1,speed2);
	SetMotor(right2,speed2);
	SetMotor(right3,speed2);
}

void drive(){
	rightFunc(vexRT[Ch2]);
	leftFunc(vexRT[Ch3]);
}

void puncherFunc(int power){
	motor[puncher1]=power;
	motor[puncher2]=power;
}

void intakeFunc(int power){
	motor[intake]=power;
}

void flipperFunc(int power){
	motor[flipper]=power;
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

//lcd driver control
task lcd(){
	clearLCD();
	displayLCDCenteredString(0,"--INITIALIZING--");
	displayLCDCenteredString(1,"--USER CONTROL--");
	delayFunc(1000);
	clearLCD();

	lcdBattery();
	delayFunc(1000);
	if( nVexRCReceiveState & 0x02 )
    {
    	// second joystick is connected
    	clearLCD();
			displayLCDCenteredString(0,"--PARTNER CTRl--");
			displayLCDCenteredString(1,"----WORKING ----");
			delayFunc(1000);
			clearLCD();
    }
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

#pragma platform(VEX)

#pragma competitionControl(Competition)
#pragma autonomousDuration(15)
#pragma userControlDuration(105)

//sensor encoders int
int leftEncode(){
	return(SensorValue[leftEncoder]);
}

int rightEncode(){
	return(SensorValue[rightEncoder]);
}

void driveFunc(int power1, int power2){
	SetMotor(right1, power2);
	SetMotor(right2, power2);
	SetMotor(right3, power2);
	SetMotor(left1,  power1);
	SetMotor(left2,  power1);
	SetMotor(left3,  power1);
}

//Drive Function High Level
void drive(){
	driveFunc(vexRT[Ch3],vexRT[Ch2]);
}

#pragma platform(VEX)

#pragma competitionControl(Competition)
#pragma autonomousDuration(15)
#pragma userControlDuration(105)



//ints
int leftDriveEncode(){
	return(SensorValue[leftEncoder]*-1);
}

int rightDriveEncode(){
	return(SensorValue[rightEncoder]);
}

//drive PID
void drivePID(int power1, int power2){
	rightDrivePID(power2);
}

//Auton Mddle Level
//drive functions
//Drive Function Middle Level
void driveStraight(int power){
	drivePID(power,power);
}

void driveStop(){
	drivePID(0,0);
}

//Auton High Level
void auton(){
	driveStraight(127);
}

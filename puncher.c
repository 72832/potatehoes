#pragma platform(VEX)

#pragma competitionControl(Competition)
#pragma autonomousDuration(15)
#pragma userControlDuration(105)

//Drive Function Base Level
void puncherFunc(int power){
	SetMotor(puncher1, power);
	SetMotor(puncher2, power);
}

//Drive Function Middle Level
//drive stop
void puncherStop(){
	puncherFunc(0);
}

void puncherForward(){
	puncherFunc(127);
}

void puncher(){
	if(vexRT[Btn5D]==1){
		puncherForward();
	} else{
			puncherStop();
	}
}

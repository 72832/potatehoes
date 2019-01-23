#pragma platform(VEX)

#pragma competitionControl(Competition)
#pragma autonomousDuration(15)
#pragma userControlDuration(105)

//Drive Function Base Level
void puncherFunc(int power){
	motor[puncher1]=power;
	motor[puncher2]=power;
}

//Dive Function Middle Level
//drive stop
void puncherStop(){
	puncherFunc(0);
}

void puncherForward(){
	puncherFunc(127);
}

void puncher(){
	if(vexRT[Btn5U]==1){
		puncherForward();
	} else{
			puncherStop();
	}
}

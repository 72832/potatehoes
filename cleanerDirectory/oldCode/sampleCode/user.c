#include "functions.c"

//user contol lift functions
void liftUp(){
	liftFunc(127);
}

void liftDown(){
	liftFunc(-127);
}

void liftStay(){
	liftFunc(0);
}

//user contol lift program
void lift(){
	if (vexRT[Btn6D]==1){
		liftUp();
	} else if (vexRT[Btn6U]==1) {
			liftDown();
	} else {
			liftStay();
	}
}

//user control flipper functions
void flip1(){
	flipFunc(127);
}

void flip2(){
	flipFunc(-127);
}

void flipStay(){
	flipFunc(0);
}

//user contol flipper program
void flip(){
	if (vexRT[Btn5D]==1) {
		flip1();
	} else if(vexRT[Btn5D]==1){
			flip2();
	} else{
			flipStay();
	}
}

//user control drive function
void drive(){
	liftFunc(vexRT[Ch3],vexRT[Ch2]);
}

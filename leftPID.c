#pragma platform(VEX)
#pragma competitionControl(Competition)
#pragma autonomousDuration(15)
#pragma userControlDuration(105)

//PID
//Drive Top Level
// PID using optical shaft encoder
//
// Shaft encoder has 360 pulses per revolution

#define LEFT_SENSOR_INDEX    leftEncoder
#define RIGHT_SENSOR_INDEX   rightEncoder
#define PID_SENSOR_SCALE     1

#define PID_MOTOR_SCALE     -1

#define PID_DRIVE_MAX       80
#define PID_DRIVE_MIN     (-80)

// These could be constants but leaving
// as variables allows them to be modified in the debugger "live"

static int   pidRunning = 1;
static float pidRequestedValue;

static int   pdRunning = 1;
static float pdRequestedValue;


// These could be constants but leaving


/*-----------------------------------------------------------------------------*/
/*
*/
/*  pid control task
*/
/*
*/
/*-----------------------------------------------------------------------------*/

task leftPIDController(){

 float  pidSensorCurrentValue;
 float  pidError;
 float  pidLastError;
 float  pidDerivative;
 float  pidDrive;

 // If we are using an encoder then clear it
 if(SensorType[ LEFT_SENSOR_INDEX ] == sensorQuadEncoder )
   SensorValue[ LEFT_SENSOR_INDEX ] = 0;

  	// Init the variables - thanks Glenn :)
  pidLastError  = 0;

  while( true ){
    // Is PID control active ?
    if( pidRunning ){
      // Read the sensor value and scale
      pidSensorCurrentValue = SensorValue[ LEFT_SENSOR_INDEX ] * PID_SENSOR_SCALE;

      // calculate error
      pidError = pidSensorCurrentValue - pidRequestedValue;

        // calculate the derivative
        pidDerivative = pidError - pidLastError;
        pidLastError  = pidError;

        // calculate drive
        pidDrive = (pid_Kp * pidError) + (pid_Kd * pidDerivative);

        // limit drive
        if( pidDrive > PID_DRIVE_MAX )
          pidDrive = PID_DRIVE_MAX;
        if( pidDrive < PID_DRIVE_MIN )
          pidDrive = PID_DRIVE_MIN;

            // send to motor
           	leftFunc(pidDrive * PID_MOTOR_SCALE);
    }else{
       // clear all
       pidError      = 0;
       pidLastError  = 0;
       pidDerivative = 0;
       leftFunc(0);
     }

    // Run at 50Hz
    wait1Msec( 25 );
  }
}

task rightPIDController(){

 float  pidSensorCurrentValue;
 float  pidError;
 float  pidLastError;
 float  pidDerivative;
 float  pidDrive;

 // If we are using an encoder then clear it
 if(SensorType[ RIGHT_SENSOR_INDEX ] == sensorQuadEncoder )
   SensorValue[ RIGHT_SENSOR_INDEX ] = 0;

  	// Init the variables - thanks Glenn :)
  pidLastError  = 0;

  while( true ){
    // Is PID control active ?
    if( pdRunning ){
      // Read the sensor value and scale
      pidSensorCurrentValue = SensorValue[ RIGHT_SENSOR_INDEX ] * PID_SENSOR_SCALE;

      // calculate error
      pidError = pidSensorCurrentValue - pdRequestedValue;

        // calculate the derivative
        pidDerivative = pidError - pidLastError;
        pidLastError  = pidError;

        // calculate drive
        pidDrive = (pd_Kp * pidError)+(pd_Kd * pidDerivative);

        // limit drive
        if( pidDrive > PID_DRIVE_MAX )
          pidDrive = PID_DRIVE_MAX;
        if( pidDrive < PID_DRIVE_MIN )
          pidDrive = PID_DRIVE_MIN;

            // send to motor
           	rightFunc(pidDrive * PID_MOTOR_SCALE);
    }else{
       // clear all
       pidError      = 0;
       pidLastError  = 0;
       pidDerivative = 0;
       rightFunc(0);
     }

    // Run at 50Hz
    wait1Msec( 25 );
  }
}


/*-----------------------------------------------------------------------------*/
/*
*/
/*  main task
/*
*/
/*-----------------------------------------------------------------------------*/

void drivePID(int clicks, int clicks2){
	// send the motor off somewhere
  pidRequestedValue= clicks;
  pdRequestedValue=  clicks2;

	// start the PID task
  startTask( leftPIDController );
	startTask( rightPIDController );

  // use joystick to modify the requested position
  for(int i=0; i<=2; i++ ){
  	// maximum change for pidRequestedValue will be 127/4*20, around 640 counts per second
  	// free spinning motor is 100rmp so 1.67 rotations per second
		// 1.67 * 360 counts is 600
  	wait1Msec(1000);
  }
	stopTask(leftPIDController);
	stopTask(rightPIDController);
}


//auton functions
//forward slightly

//auton
void auton(){
	//1200 from place to flag or to alliance park
	//2000 from place to center
	delayFunc(20);
	puncherFunc(127);
	delayFunc(3000);
	puncherFunc(0);
	drivePID(1400,1400);
	drivePID(-2000,-2000);
	drivePID(340,-340);
	drivePID(2000,2000);
}

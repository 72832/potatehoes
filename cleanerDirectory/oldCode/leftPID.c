
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

#define PID_INTEGRAL_LIMIT  50

// These could be constants but leaving
// as variables allows them to be modified in the debugger "live"
float  pid_Kp = 2.0;
float  pid_Ki = 0.04;
float  pid_Kd = 0.0;

static int   pidRunning = 1;
static float pidRequestedValue;

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
 float  pidIntegral;
 float  pidDerivative;
 float  pidDrive;

 // If we are using an encoder then clear it
 if(SensorType[ LEFT_SENSOR_INDEX ] == sensorQuadEncoder )
   SensorValue[ LEFT_SENSOR_INDEX ] = 0;

  	// Init the variables - thanks Glenn :)
  pidLastError  = 0;
  pidIntegral   = 0;

  while( true ){
    // Is PID control active ?
    if( pidRunning ){
      // Read the sensor value and scale
      pidSensorCurrentValue = SensorValue[ LEFT_SENSOR_INDEX ] * PID_SENSOR_SCALE;

      // calculate error
      pidError = pidSensorCurrentValue - pidRequestedValue;

      // integral - if Ki is not 0
      if( pid_Ki != 0 ){
        // If we are inside controlable window then integrate the error
        if( abs(pidError) < PID_INTEGRAL_LIMIT )
          pidIntegral = pidIntegral + pidError;
        else
          pidIntegral = 0;
        }
      else
        pidIntegral = 0;

        // calculate the derivative
        pidDerivative = pidError - pidLastError;
        pidLastError  = pidError;

        // calculate drive
        pidDrive = (pid_Kp * pidError) + (pid_Ki * pidIntegral) + (pid_Kd * pidDerivative);

        // limit drive
        if( pidDrive > PID_DRIVE_MAX )
          pidDrive = PID_DRIVE_MAX;
        if( pidDrive < PID_DRIVE_MIN )
          pidDrive = PID_DRIVE_MIN;

            // send to motor
           	driveFunc(pidDrive * PID_MOTOR_SCALE, pidDrive* PID_MOTOR_SCALE);
    }else{
       // clear all
       pidError      = 0;
       pidLastError  = 0;
       pidIntegral   = 0;
       pidDerivative = 0;
       driveFunc(0,0);
     }

    // Run at 50Hz
    wait1Msec( 25 );
  }
}

/*-----------------------------------------------------------------------------*/
/*
*/
/*  main task
*/
/*
*/
/*-----------------------------------------------------------------------------*/

void leftDrivePID(int clicks){
	// send the motor off somewhere
  pidRequestedValue = clicks;
	// start the PID task
  startTask( leftPIDController );

  // use joystick to modify the requested position
  while( true ){
  	// maximum change for pidRequestedValue will be 127/4*20, around 640 counts per second
  	// free spinning motor is 100rmp so 1.67 rotations per second
		// 1.67 * 360 counts is 600

  	wait1Msec(50);
  }

  stopTask(leftPIDController);

}

/*+++++++++++++++++++++++++++++++++++++++++++++| Notes |++++++++++++++++++++++++++++++++++++++++++++++
Gyro Based Turns - Better
-This program instructs your robot to turn for the specified number of degrees in "degrees10".
-The first while loop moves the robot most of the way toward the specified number of degrees, exiting
 10 degrees (100 sensor counts) short of the goal.
-This program includes a second while loop which incorporates a specified error value to determine
 an acceptable range and allow the robot to turn more slowly toward its goal, eliminating much of
 the drift in the system.
-For best sensor results, clear out the gyro and manually configure it at the begging of the code.
-The Gyro is configured by default to provide values from 0 to -3600 for clockwise rotation,
 and 0 to 3600 for counter-clockwise rotation

Robot Model(s): Swervebot

[I/O Port]          [Name]              [Type]                [Description]
Motor Port 2        rightMotor          VEX Motor             Right side motor
Motor Port 3        leftMotor           VEX Motor             Left side motor
Analog Port 8       in8                 VEX Gyro              Top-center mounted,
                                                              away from the Cortex
----------------------------------------------------------------------------------------------------*/

void gyroTurn(int degrees10)
{
  //Completely clear out any previous sensor readings by setting the port to "sensorNone"
  SensorType[in8] = sensorNone;
  wait1Msec(1000);
  //Reconfigure Analog Port 8 as a Gyro sensor and allow time for ROBOTC to calibrate it
  SensorType[in8] = sensorGyro;
  wait1Msec(2000);

  //Adjust SensorScale to correct the scaling for your gyro
  //SensorScale[in8] = 260;
  //Adjust SensorFullCount to set the "rollover" point. A value of 3600 sets the rollover point to +/-3600
  //SensorFullCount[in8] = 3600;

  //Specify the number of degrees for the robot to turn (1 degree = 10, or 900 = 90 degrees)
  degrees10 = 900;
  //Specify the amount of acceptable error in the turn
  int error = 5;

  //While the absolute value of the gyro is less than the desired rotation - 100...
  while(abs(SensorValue[in8]) < degrees10 - 100)
  {
    driveFunc(-50,50);
  }
  //Brief brake to eliminate some drift
  driveFunc(5,-5);
  wait1Msec(100);

  //Second while loop to move the robot more slowly to its goal, also setting up a range
  //for the amount of acceptable error in the system
  while(abs(SensorValue[in8]) > degrees10 + error || abs(SensorValue[in8]) < degrees10 - error)
  {
    if(abs(SensorValue[in8]) > degrees10)
    {
      driveFunc(-30,30);
    }
    else
    {
      driveFunc(30,-30)
     }
  }
  //Stop
  driveFunc(0,0);
  wait1Msec(250);
}

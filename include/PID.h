// PID using optical shaft encoder
//
// Shaft encoder has 360 pulses per revolution
//

#define PID_INTEGRAL_LIMIT  50

// These could be constants but leaving
// as variables allows them to be modified in the debugger "live"

float wheelCircumference=4 /*diameter*/ * 3.141529 /*PI simplified*/;

void pidPos(static float inches){

    static float Kp=1.0;
    static float Ki=0.0;
    static float Kd=0.0;

    static float  wheelRotations;
    static float  pidSensorCurrentValue;
    static float  pidReqVal;
    static float  clicks;

    static float  pidError;
    static float  pidLastError;
    static float  pidIntegral;
    static float  pidDerivative;
    static float  pidDrive;

    bool pidRunning;

    pidRunning == true;

    pidLastError  = 0;
    pidIntegral   = 0;

    while( pidRunning==true ){
            // Read the sensor value and scale
        pidSensorCurrentValue = SensorValue[leftEncoder]*-1;

        wheelRotations=(inches) / (wheelCircumference);

        pidReqVal=wheelRotations*360;

            // calculate error
        pidError = pidSensorCurrentValue - pidReqVal;

            // integral - if Ki is not 0
        if( Ki != 0 ){
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
        pidDrive = (Kp * pidError) + (Ki * pidIntegral) + (Kd * pidDerivative);

            // limit drive
        if( pidDrive > 50 )
            pidDrive = 50;
        if( pidDrive < -50 )
            pidDrive = -50;

            // send to motor
        motor[left1] = pidDrive*-1;
        motor[left2] = pidDrive*-1;
        motor[left3] = pidDrive*-1;

        motor[right1]=pidDrive;
        motor[right2]=pidDrive;
        motor[right3]=pidDrive;

        if(pidDrive<=15 && pidDrive>=-15){
            pidRunning=false;
        }else{
            // clear all
            pidError      = 0;
            pidLastError  = 0;
            pidIntegral   = 0;
            pidDerivative = 0;

            // send to motor
            motor[left1] = 0;
            motor[left2] = 0;
            motor[left3] = 0;

            motor[right1] = 0;
            motor[right2] = 0;
            motor[right3] = 0;
            }

        // Run at 50Hz
        wait1Msec( 50 );
        }
}

void pidTurn(float turn, static float pidReqVal){
    static float Kp=1.0;
    static float Ki=0.0;
    static float Kd=0.0;

    static float  pidLeftCurrentValue;
    static float  pidRightCurrentValue;

    static float  pidLeftError;
    static float  pidRightError;

    static float  pidLeftLastError;
    static float  pidRightLastError;

    static float  pidIntegral;
    static float  pidDerivative;
    static float  pidDrive;

    bool pidRunning=true;

    pidLastError  = 0;
    pidIntegral   = 0;

    while( pidRunning==true ){
            // Read the sensor value and scale
        if(turn==left){
            pidLeftCurrentValue = SensorValue[leftEncoder]*1;
            pidRightCurrentValue = SensorValue[rightEncoder]*1;

        }else if(turn==right){
            pidLeftCurrentValue = SensorValue[leftEncoder]*-1;
            pidRightCurrentValue = SensorValue[rightEncoder]*-1;

        }
            // calculate error
        pidLeftError  = pidLeftCurrentValue - pidReqVal;
        pidRightError = pidRightCurrentValue - pidReqVal;

            // integral - if Ki is not 0
        if( pid_Ki != 0 ){
                // If we are inside controlable window then integrate the error
            if( abs(pidError) < PID_INTEGRAL_LIMIT )
                pidIntegral = pidIntegral + pidError;
            else
                pidIntegral = 0;
        }else
            pidIntegral = 0;

            // calculate the derivative
        pidDerivative = pidError - pidLastError;
        pidLastError  = pidError;

            // calculate drive
        pidDrive = (Kp * pidError) + (Ki * pidIntegral) + (Kd * pidDerivative);

            // limit drive
        if( pidDrive > 50 )
            pidDrive = 50;
        if( pidDrive < -50 )
            pidDrive = -50;

        if(turn==left){
            // send to motor
            motor[left1] = pidDrive;
            motor[left2] = pidDrive;
            motor[left3] = pidDrive;

            motor[right1]=pidDrive;
            motor[right2]=pidDrive;
            motor[right3]=pidDrive;
        }else if(turn==right){
            // send to motor
            motor[left1] = pidDrive*-1;
            motor[left2] = pidDrive*-1;
            motor[left3] = pidDrive*-1;

            motor[right1]=pidDrive*-1;
            motor[right2]=pidDrive*-1;
            motor[right3]=pidDrive*-1;
        }

        if(pidDrive<=15 && pidDrive>=-15)
            pidRunning=false;
        else{
            // clear all
            pidError      = 0;
            pidLastError  = 0;
            pidIntegral   = 0;
            pidDerivative = 0;

            // send to motor
            motor[left1] = 0;
            motor[left2] = 0;
            motor[left3] = 0;

            motor[right1] = 0;
            motor[right2] = 0;
            motor[right3] = 0;
            }

        // Run at 50Hz
        wait1Msec( 50 );
        }
}

void driveForward(static float tiles, static int speed=75){

    static float wheelRotations;

    static float clicks;

    wheelRotations = (tiles*24) / (wheelCircumference);

    clicks = wheelRotations*360;

    resetEncoders();

    while(leftEnc<=clicks){

        leftEnc=SensorValue[leftEncoder];

        motor[left1]=speed*-1;
        motor[left2]=speed*-1;
        motor[left3]=speed*-1;

        motor[right1]=speed;
        motor[right2]=speed;
        motor[right3]=speed;
    }
}

void driveBackward(static float tiles, static int speed=75){

    static float wheelRotations;

    static float clicks;

    wheelRotations = (tiles*24) / (wheelCircumference);

    clicks = wheelRotations*360;

    resetEncoders();

    leftEnc=SensorValue[leftEncoder]*-1;

    while(leftEnc<=clicks){

        leftEnc=SensorValue[leftEncoder]*-1;

        motor[left1]=speed*1;
        motor[left2]=speed*1;
        motor[left3]=speed*1;

        motor[right1]=speed*-1;
        motor[right2]=speed*-1;
        motor[right3]=speed*-1;
    }
}

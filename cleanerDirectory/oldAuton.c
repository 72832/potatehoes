// PID using optical shaft encoder
//
// Shaft encoder has 360 pulses per revolution
//

#define PID_MOTOR_SCALE     -1

#define PID_INTEGRAL_LIMIT  50

// These could be constants but leaving
// as variables allows them to be modified in the debugger "live"

int range;
int pidReqVal;

task pidPos(){    
    static float Kp=1.0;
    static float Ki=0.0;
    static float Kd=0.0;
    
    static float  pidSensorCurrentValue;

    static float  pidError;
    static float  pidLastError;
    static float  pidIntegral;
    static float  pidDerivative;
    static float  pidDrive;

    pidLastError  = 0;
    pidIntegral   = 0;

    while( pidRunning==true ){
            // Read the sensor value and scale
        pidSensorCurrentValue = SensorValue[leftEncoder]*-1;

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

        if(pidReqVal<=350/*less than or equal to 12 inches*/){
            limit=limit+1;
            if (limit>60){
                limit=60;
            }
        }else if(350<pidReqVal<=1050/*between 12 and 36 inches*/){
            limit=limit+2;
            if (limit>90){
                limit=90;
            }
        }else if(1050<pidReqVal/*greater than 36 inches*/){
            limit=limit+1;
            if (limit>127){
                limit=127;
            }
        }

            // limit drive
        if( pidDrive > limit )
            pidDrive = limit;
        if( pidDrive < -limit )
            pidDrive = -limit;

            // send to motor
        motor[left1] = pidDrive*-1;
        motor[left2] = pidDrive*-1;
        motor[left3] = pidDrive*-1;

        motor[right1]=pidDrive;
        motor[right2]=pidDrive;
        motor[right3]=pidDrive;

        if(pidDrive==0){
            pidRunning==false;
        }

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
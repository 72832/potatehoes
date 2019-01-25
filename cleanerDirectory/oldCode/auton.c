task leftPIDController() {
    taskRunning = true;
    float pidSensorCurrentValue;
    float pidError;
    float pidLastError;
    float pidIntegral;
    float pidDerivative;
    float pidDrive;

    // If we are using an encoder then clear it
    if (SensorType[LEFT_SENSOR_INDEX] == sensorQuadEncoder)
        SensorValue[LEFT_SENSOR_INDEX] = 0;

    // Init the variables - thanks Glenn :)
    pidLastError = 0;
    pidIntegral = 0;

    while (true) {
        if (pidRunning) {
            if (pidSensorCurrentValue == pidRequestedValue) {
                taskRunning = false;
                stopTask(leftPIDController);
                // Read the sensor value and scale
                pidSensorCurrentValue = SensorValue[LEFT_SENSOR_INDEX] * PID_SENSOR_SCALE;


                // calculate error
                pidError = pidSensorCurrentValue - pidRequestedValue;

                // integral - if Ki is not 0
                if (pid_Ki != 0) {
                    // If we are inside controlable window then integrate the error
                    if (abs(pidError) < PID_INTEGRAL_LIMIT)
                        pidIntegral = pidIntegral + pidError;
                    else
                        pidIntegral = 0;
                } else
                    pidIntegral = 0;

                // calculate the derivative
                pidDerivative = pidError - pidLastError;
                pidLastError = pidError;

                // calculate drive
                pidDrive = (pid_Kp * pidError) + (pid_Ki * pidIntegral) + (pid_Kd * pidDerivative);

                // limit drive
                if (pidDrive > PID_DRIVE_MAX)
                    pidDrive = PID_DRIVE_MAX;
                if (pidDrive < PID_DRIVE_MIN)
                    pidDrive = PID_DRIVE_MIN;

                leftFunc(pidDrive);
                rightFunc(pidDrive);
            }
        } else {
            // clear all
            pidError = 0;
            pidLastError = 0;
            pidIntegral = 0;
            pidDerivative = 0;

            leftFunc(0);
            rightFunc(0);
        }

        // Run at 50Hz
        wait1Msec(25);
    }
}

void drivePID(int clicks) {

    // send the motor off somewhere
    pidRequestedValue = clicks * -1;

    // start the PID task
    startTask(leftPIDController);
    taskRunning = true;
    //	startTask( rightPIDController );

    // use joystick to modify the requested position
    while (taskRunning) {
        if (pidRequestedValue == clicks * -1)
            taskRunning = false;
    }
    // maximum change for pidRequestedValue will be 127/4*20, around 640 counts per second
    // free spinning motor is 100rmp so 1.67 rotations per second
    // 1.67 * 360 counts is 600
    wait1Msec(20);

    resetEncoders();
    stopTask(leftPIDController);
}

task main()
{
	bLCDBacklight = true;                                    // Turn on LCD Backlight
	string mainBattery, backupBattery;

	//Clear LCD of text
	clearLCDLine(0);                                            // Clear line 1 (0) of the LCD
	clearLCDLine(1);                                            // Clear line 2 (1) of the LCD

	//Boot Sequence
	displayLCDString(0,0, "Booting Robot");
	displayLCDString(1,0, "1");
	clearLCDLine(1);
	wait1Msec(250);
	displayLCDString(1,0, "1 2");
	clearLCDLine(1);
	wait1Msec(250);
	displayLCDString(1,0, "1 2 3");
	clearLCDLine(1);
	wait1Msec(250);
	displayLCDString(1,0, "1 2 3 A");
	clearLCDLine(1);
	wait1Msec(250);
	displayLCDString(1,0, "1 2 3 A B");
	clearLCDLine(1);
	wait1Msec(250);
	displayLCDString(1,0, "1 2 3 A B C");
	wait1Msec(1000);

	//Display the Primary Robot battery voltage
	clearLCDLine(0);
	clearLCDLine(1);
	displayLCDString(0, 0, "Primary: ");
	sprintf(mainBattery, "%1.2f%c", nImmediateBatteryLevel/1000.0,'V'); //Build the value to be displayed
	displayNextLCDString(mainBattery);

	//Display the Backup battery voltage
	displayLCDString(1, 0, "Backup: ");
	sprintf(backupBattery, "%1.2f%c", BackupBatteryLevel/1000.0, 'V');    //Build the value to be displayed
	displayNextLCDString(backupBattery);
	wait1Msec(2000);

	//Clear LCD
	clearLCDLine(0);
	clearLCDLine(1);

	while(true) {
		//Variables
		//Display sensor values
		displayLCDString(00,sensorValue[in1]);

	}
}

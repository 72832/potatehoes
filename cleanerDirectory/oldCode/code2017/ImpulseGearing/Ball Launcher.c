#pragma config(Motor,  port1,           left,     tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port10,          right,     tmotorVex393_HBridge, openLoop)



task main()
{
  while (true)
	{
		motor[port1] = abs(vexRT[Ch2]);
		motor[port10] = abs(vexRT[Ch2]);
	}
}


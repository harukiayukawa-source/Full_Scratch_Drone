#include <Arduino.h>
#include "struct.h"
#include "imu.h"
#include "madgwick.h"
#include "control.h"
#include "interfaceBLE.h"
#include "debug.h"

IMUData raw = {0};
IMUData flt = {0};
Attitude att = {0};
PIDCtrl pid;
PIDCtrl pids_def =           //単ループ用デフォルト値
{
	0.40,                    //throttle_hover
	0.0140, 0.0000, 0.0020,  //roll
	0.0140, 0.0000, 0.0020,  //pitch
	0.0000, 0.0000, 0.0000,  //yaw
	0.0000, 0.0000, 0.0000,  //roll_rate
	0.0000, 0.0000, 0.0000,  //pitch_rate
	0.0010, 0.0000, 0.0000,  //yaw_rate
};
PIDCtrl pidw_def =           //カスケード用デフォルト値
{
	0.40,                    //throttle_hover
	4.0   , 0.0000, 0.0000,  //roll
	4.0   , 0.0000, 0.0000,  //pitch
	2.0   , 0.0000, 0.0000,  //yaw
	0.0020, 0.0000, 0.0000,  //roll_rate
	0.0020, 0.0000, 0.0000,  //pitch_rate
	0.0010, 0.0000, 0.0000,  //yaw_rate
};
CmdrBLE cmdr;
CmdtBLE cmdt_local;
DutyMotor duty;

unsigned long last_loop;
void setup();
void loop();

void setup()
{
	Serial.begin(115200);
  	delay(1000);

	initIMU();
	calibIMU();
	setupMotor();
	initBLE();
	Serial.println("WAITING FOR W...");

	while (!writeCheckOK)
    {
        delay(100);
    }
	delay(500);

	sendDebug("RECEIVED W");
	MadgwickFilter.begin(LOOP_HZ);
	last_loop = micros();
	Serial.println("Flight controller start");
}

void loop()
{
  	while(micros() - last_loop < 1000000/LOOP_HZ);
  	last_loop += 1000000/LOOP_HZ;

	noInterrupts();
    cmdt_local = cmdt;
    interrupts();

	if(cmdt_local.defaultFlag)
	{
		/*
			角度PID制御(単ループ)     → 1行目を有効
    		カスケード制御(二重ループ) → 2行目を有効
		*/
		if(cmdt_local.w_to_s_Flag)
		{
			pid = pids_def;			
		}
		else
		{
			pid = pidw_def;
		}
	}

	else
	{
		updateBLEPID(cmdt_local, pid);
	}
	
  	readIMU(raw);
  	lpfilterIMU(raw, flt);
  	mwf_to_Euler(flt, att);
	updateBLECMD(cmdt_local, cmdr);

	/*
		角度PID制御(単ループ)     → 1行目を有効
    	カスケード制御(二重ループ) → 2行目を有効
	*/
	if(cmdt_local.w_to_s_Flag)
	{
		computePIDS(att, pid, cmdr);		
	}
	else
	{
		computePIDW(att, pid, cmdr);
	}

	if(!cmdt_local.startFlag || (millis() - cmdt_local.LastCmdTime) > 500)
	{
		stopMotor(duty);
		cmdr.yaw_last = att.yaw;
	}

	else
	{
		ctrlMotor(duty);
	}

  	static int div_UART_A = 0;
  	div_UART_A++;
  	if(div_UART_A > 400)
  	{
    	debug_IMU(flt);
		debug_RPY(att);
		debug_PID(pid);
		debug_Cmd(cmdr);
		debug_Duty(duty);
		debug_LCT(cmdt_local);
		Serial.println();Serial.println();
		
		if(cmdt_local.w_to_s_Flag)
		{
			sendDebug("SINGLE");		
		}
		else
		{
			sendDebug("CASCADE");
		}
		
    	div_UART_A = 0;
  	}

	static int div_UART_B = 200;
	div_UART_B++;
	if(div_UART_B > 400)
	{
		if(cmdt_local.defaultFlag)
		{
			sendDebug("DEFAULT");
		}
		else
		{
			sendDebug("TUNED");
		}

		div_UART_B = 0;
	}

	static int div_BLE_A = 0;
	div_BLE_A++;
	if(div_BLE_A > 75)
  	{
		char buf_A[32];
		snprintf(buf_A, sizeof(buf_A),"M%04.1f%04.1f%04.1f%04.1f",duty.FR*100, duty.FL*100, duty.RL*100, duty.RR*100);
		sendDebug(buf_A);
		div_BLE_A = 0;
	}
	static int div_BLE_B = 25;
	div_BLE_B++;
	if(div_BLE_B > 75)
  	{
		char buf_B[32];
		snprintf(buf_B, sizeof(buf_B),"A%+04.0f%+04.0f%+04.0f",att.roll, att.pitch, att.yaw);
		sendDebug(buf_B);
		div_BLE_B = 0;
	}
	static int div_BLE_C = 50;
	div_BLE_C++;
	if(div_BLE_C > 75)
  	{
		char buf_C[32];
		snprintf(buf_C, sizeof(buf_C),"R%+04.0f%+04.0f%+04.0f",att.roll_rate, att.pitch_rate, att.yaw_rate);
		sendDebug(buf_C);
		div_BLE_C = 0;
	}
}

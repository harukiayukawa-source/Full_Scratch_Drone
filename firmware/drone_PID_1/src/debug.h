#pragma once

#include <Arduino.h>
#include "struct.h"

#define LED_debug 18

void debug_IMU(IMUData &Imu);
void debug_RPY(Attitude &Mwf);
void debug_PID(PIDCtrl &Pid);
void debug_Cmd(CmdrBLE &Cmdr);
void debug_Duty(DutyMotor &Duty);
void debug_LCT(CmdtBLE &Cmdt);
void debugLED(float a, float b);
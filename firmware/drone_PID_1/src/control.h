#pragma once

#include <Arduino.h>
#include "struct.h"

#define MOTOR_FL 32
#define MOTOR_FR 19
#define MOTOR_RL 33
#define MOTOR_RR 23
#define PWM_FREQ 20000
#define PWM_RES 10

#define LOOP_HZ 750
#define DT (1.0f/LOOP_HZ)
#define mabiki_DT 5
#define lpf_d 0.5
#define thr_yaw_rate 5.0

float wrap180(float a);
void setupMotor();
void driveMotor(int ch,float per);
void computePIDS(Attitude &Mwf, PIDCtrl &Pid, CmdrBLE &Cmdr);
void computePIDW(Attitude &Mwf, PIDCtrl &Pid, CmdrBLE &Cmdr);
void ratePIDW(Attitude &Mwf, PIDCtrl &Pid, CmdrBLE &Cmdr);
void anglePIDW(Attitude &Mwf, PIDCtrl &Pid, CmdrBLE &Cmdr);
void ctrlMotor(DutyMotor &Duty);
void stopMotor(DutyMotor &Duty);
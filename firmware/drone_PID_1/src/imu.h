#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <ICM_20948.h>
#include "struct.h"

#define AD0_VAL 0
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_FREQ 400000

extern ICM_20948_I2C imui2c;
extern ICM_20948_dlpcfg_t myDLPF;

void initIMU();
void calibIMU();
void readIMU(IMUData &Read);
void lpfilterIMU(IMUData &Read, IMUData &Flt);

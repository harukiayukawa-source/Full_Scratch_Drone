#pragma once

#include <Arduino.h>
#include <MadgwickAHRS.h>
#include "struct.h"

extern Madgwick MadgwickFilter;

void mwf_to_Euler(IMUData &flt, Attitude &mwf);



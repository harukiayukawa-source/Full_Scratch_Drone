#pragma once

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <vector>
#include "struct.h"

void initBLE();
void updateBLEPID(const CmdtBLE &Cmdt, PIDCtrl &Pid);
void updateBLECMD(const CmdtBLE &Cmdt, CmdrBLE &Cmdr);
void sendDebug(String msg);

extern CmdtBLE cmdt;
extern bool writeCheckOK;


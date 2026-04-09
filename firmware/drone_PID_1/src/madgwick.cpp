#include "madgwick.h"

Madgwick MadgwickFilter;

void mwf_to_Euler(IMUData &Flt, Attitude &Mwf)
{
    //6軸  → updateIMU()を有効
    //9軸  → update()   を有効

    //MadgwickFilter.update(-Flt.gy, Flt.gx, -Flt.gz, -Flt.ay, Flt.ax, Flt.az, -Flt.my, Flt.mx, Flt.mz);
    MadgwickFilter.updateIMU(-Flt.gy, Flt.gx, -Flt.gz, -Flt.ay, Flt.ax, Flt.az);
    Mwf.roll  = -MadgwickFilter.getRoll();
    Mwf.pitch =  MadgwickFilter.getPitch();
    Mwf.yaw   =  MadgwickFilter.getYaw() - 180;
    Mwf.roll_rate  =  Flt.gy;
    Mwf.pitch_rate =  Flt.gx;
    Mwf.yaw_rate   = -Flt.gz;
}
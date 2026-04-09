#include "imu.h"

ICM_20948_I2C imui2c;
ICM_20948_dlpcfg_t myDLPF;

float gx_bias = 0;
float gy_bias = 0;
float gz_bias = 0;

void initIMU()
{
    Wire.begin(SDA_PIN,SCL_PIN);
    Wire.setClock(I2C_FREQ);
    imui2c.begin(Wire, AD0_VAL);

    if(imui2c.status == ICM_20948_Stat_Ok)
     {
        Serial.println("IMU OK");
        // ★代入処理は関数の中で行う★
        // 構造体のメンバーに合わせて代入
        myDLPF.g = 2; // ジャイロ 119.5Hz
        myDLPF.a = 3; // 加速度 23.9Hz

        // --- ジャイロの設定 ---
        imui2c.setDLPFcfg(ICM_20948_Internal_Gyr, myDLPF); 
        imui2c.enableDLPF(ICM_20948_Internal_Gyr, true);

        // --- 加速度計の設定 ---
        imui2c.setDLPFcfg(ICM_20948_Internal_Acc, myDLPF);
        imui2c.enableDLPF(ICM_20948_Internal_Acc, true);

        Serial.println("DLPF (120Hz) Enabled via setDLPFcfg");
    }
    else
    {
        Serial.println("IMU falut...");
        delay(500);
    }
}

void calibIMU()
{
    Serial.println("Gyro calibrating...");

    int samples = 1000;

    for(int i = 0; i < samples; i++)
    {
        while(!imui2c.dataReady());
        imui2c.getAGMT();
        
        gx_bias += imui2c.gyrX();
        gy_bias += imui2c.gyrY();
        gz_bias += imui2c.gyrZ();
    }

    gx_bias /= samples;
    gy_bias /= samples;
    gz_bias /= samples;

    Serial.println("Gyro calibration done");
    Serial.print(gx_bias);Serial.print(" ");
    Serial.print(gy_bias);Serial.print(" ");
    Serial.print(gz_bias);Serial.println(" ");
}

void readIMU(IMUData &Read)
{
    imui2c.getAGMT();

    Read.ax = imui2c.accX() / 1000;
    Read.ay = imui2c.accY() / 1000;
    Read.az = imui2c.accZ() / 1000;

    Read.gx = (imui2c.gyrX() - gx_bias);
    Read.gy = (imui2c.gyrY() - gy_bias);
    Read.gz = (imui2c.gyrZ() - gz_bias);

    Read.mx = imui2c.magX();
    Read.my = imui2c.magY();
    Read.mz = imui2c.magZ();
}

void lpfilterIMU(IMUData &Read, IMUData &Flt)
{
    float a_a = 1.0;
    float a_g = 1.0;
    float a_m = 1.0;

    Flt.ax += a_a * (Read.ax - Flt.ax);
    Flt.ay += a_a * (Read.ay - Flt.ay);
    Flt.az += a_a * (Read.az - Flt.az);

    Flt.gx += a_g * (Read.gx - Flt.gx);
    Flt.gy += a_g * (Read.gy - Flt.gy);
    Flt.gz += a_g * (Read.gz - Flt.gz);

    Flt.mx += a_m * (Read.mx - Flt.mx);
    Flt.my += a_m * (Read.my - Flt.my);
    Flt.mz += a_m * (Read.mz - Flt.mz);
}
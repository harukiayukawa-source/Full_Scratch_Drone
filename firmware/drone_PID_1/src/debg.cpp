#include "debug.h"

void debug_IMU(IMUData &Imu)
{
	Serial.print("ACC[g]     ");
  	Serial.printf("%6.2f", Imu.ax);Serial.print(" ");
  	Serial.printf("%6.2f", Imu.ay);Serial.print(" ");
  	Serial.printf("%6.2f", Imu.az);Serial.println(" ");

  	Serial.print("GYR[deg/s] ");
  	Serial.printf("%6.2f", Imu.gx);Serial.print(" ");
  	Serial.printf("%6.2f", Imu.gy);Serial.print(" ");
  	Serial.printf("%6.2f", Imu.gz);Serial.println(" ");

	Serial.print("MAG[µT]    ");
  	Serial.printf("%6.2f", Imu.mx);Serial.print(" ");
  	Serial.printf("%6.2f", Imu.my);Serial.print(" ");
  	Serial.printf("%6.2f", Imu.mz);Serial.println(" ");
    Serial.println();
}

void debug_RPY(Attitude &Mwf)
{
	Serial.printf("RPY[deg]  ");
  	Serial.printf("%6.2f", Mwf.roll);Serial.print(" ");
  	Serial.printf("%6.2f", Mwf.pitch);Serial.print(" ");
  	Serial.printf("%6.2f", Mwf.yaw);Serial.println(" ");
    Serial.println();
}

void debug_PID(PIDCtrl &Pid)
{
	Serial.printf("throttele_hover ");
  	Serial.printf("%5.4f", Pid.throttle_hover);Serial.println("");

	Serial.printf("Kx_roll  ");
  	Serial.printf("%6.4f", Pid.Kp_roll);Serial.print(" ");
  	Serial.printf("%6.4f", Pid.Ki_roll);Serial.print(" ");
	Serial.printf("%6.4f", Pid.Kd_roll);Serial.println(" ");

	Serial.printf("Kx_pitch ");
  	Serial.printf("%6.4f", Pid.Kp_pitch);Serial.print(" ");
  	Serial.printf("%6.4f", Pid.Ki_pitch);Serial.print(" ");
	Serial.printf("%6.4f", Pid.Kd_pitch);Serial.println(" ");

	Serial.printf("Kx_yaw   ");
  	Serial.printf("%6.4f", Pid.Kp_yaw);Serial.print(" ");
  	Serial.printf("%6.4f", Pid.Ki_yaw);Serial.print(" ");
	Serial.printf("%6.4f", Pid.Kd_yaw);Serial.println(" ");

	Serial.printf("Kx_roll_rate  ");
  	Serial.printf("%6.4f", Pid.Kp_roll_rate);Serial.print(" ");
  	Serial.printf("%6.4f", Pid.Ki_roll_rate);Serial.print(" ");
	Serial.printf("%6.4f", Pid.Kd_roll_rate);Serial.println(" ");

	Serial.printf("Kx_pitch_rate ");
  	Serial.printf("%6.4f", Pid.Kp_pitch_rate);Serial.print(" ");
  	Serial.printf("%6.4f", Pid.Ki_pitch_rate);Serial.print(" ");
	Serial.printf("%6.4f", Pid.Kd_pitch_rate);Serial.println(" ");

	Serial.printf("Kx_yaw_rate   ");
  	Serial.printf("%6.4f", Pid.Kp_yaw_rate);Serial.print(" ");
  	Serial.printf("%6.4f", Pid.Ki_yaw_rate);Serial.print(" ");
	Serial.printf("%6.4f", Pid.Kd_yaw_rate);Serial.println(" ");
	Serial.println();
}

void debug_Cmd(CmdrBLE &Cmdr)
{
    Serial.print("rollコマンド     ");
	Serial.printf("%3.1f", Cmdr.roll_tag);Serial.println(" [deg]");
	Serial.print("pitchコマンド    ");
	Serial.printf("%3.1f", Cmdr.pitch_tag);Serial.println(" [deg]");
	Serial.print("yawコマンド      ");
	Serial.printf("%3.1f", Cmdr.yaw_rate_tag);Serial.println(" [deg]");
	Serial.print("throttleコマンド ");
	Serial.printf("%3.1f", Cmdr.throttle_cmd * 100);Serial.println(" [%]");
	Serial.println();
}

void debug_Duty(DutyMotor &Duty)
{
    Serial.print("右前[%] ");
    Serial.print(Duty.FR * 100);Serial.println("");
    Serial.print("左前[%] ");
    Serial.print(Duty.FL * 100);Serial.println("");
	Serial.print("左後[%] ");
    Serial.print(Duty.RL * 100);Serial.println("");
	Serial.print("右後[%] ");
    Serial.print(Duty.RR * 100);Serial.println("");
    Serial.println();
}

void debug_LCT(CmdtBLE &Cmdt)
{
	Serial.println("現在秒 - LastCmdTime");
	Serial.printf("%d", millis() - Cmdt.LastCmdTime);Serial.println(" [millis]");
	Serial.println();
}

void debugLED(float a, float b)
{

}
#include "control.h"

float roll_err       = 0;
float roll_err_i     = 0;
float roll_err_d     = 0;
float roll_err_prev  = 0;
float roll_out       = 0;

float pitch_err      = 0;
float pitch_err_i    = 0;
float pitch_err_d    = 0;
float pitch_err_prev = 0;
float pitch_out      = 0;

float yaw_err        = 0;
float yaw_err_i      = 0;
float yaw_err_d      = 0;
float yaw_err_prev   = 0;
float yaw_out        = 0;

float roll_rate_err       = 0;
float roll_rate_err_i     = 0;
float roll_rate_err_d     = 0;
float roll_rate_err_prev  = 0;
float roll_rate_tag       = 0;
float roll_rate_prev      = 0;
float roll_rate_err_dr    = 0;

float pitch_rate_err      = 0;
float pitch_rate_err_i    = 0;
float pitch_rate_err_d    = 0;
float pitch_rate_err_prev = 0;
float pitch_rate_tag      = 0;
float pitch_rate_prev     = 0;
float pitch_rate_err_dr   = 0;

float yaw_rate_err        = 0;
float yaw_rate_err_i      = 0;
float yaw_rate_err_d      = 0;
float yaw_rate_err_prev   = 0;
float yaw_rate_tag        = 0;
float yaw_rate_prev       = 0;
float yaw_rate_err_dr     = 0;

float raw_FL;
float raw_FR;
float raw_RL;
float raw_RR;

float wrap180(float a)
{
    a = fmod(a + 180.0f, 360.0f);
    if (a < 0) a += 360.0f;
    return a - 180.0f;
}

void setupMotor()
{
    ledcSetup(0,PWM_FREQ,PWM_RES);
    ledcSetup(1,PWM_FREQ,PWM_RES);
    ledcSetup(2,PWM_FREQ,PWM_RES);
    ledcSetup(3,PWM_FREQ,PWM_RES);

    ledcAttachPin(MOTOR_FL,0);
    ledcAttachPin(MOTOR_FR,1);
    ledcAttachPin(MOTOR_RL,2);
    ledcAttachPin(MOTOR_RR,3);
}

void driveMotor(int ch, float per)
{
    int pwm = per * 1023;
    ledcWrite(ch,pwm);
}

void computePIDS(Attitude &Mwf, PIDCtrl &Pid, CmdrBLE &Cmdr)
{
    // --- 比例誤差計算 (P項) ---
    roll_err  = wrap180(Cmdr.roll_tag  - Mwf.roll);
    pitch_err = wrap180(Cmdr.pitch_tag - Mwf.pitch);
    yaw_rate_err = Cmdr.yaw_rate_tag   - Mwf.yaw_rate;

    // --- 微分誤差計算（D項） ---
    roll_err_d  = -Mwf.roll_rate;
    pitch_err_d = -Mwf.pitch_rate;
    yaw_rate_err_d = yaw_rate_prev - Mwf.yaw_rate;

    // --- 累積誤差計算（I項） ---
    roll_err_i  += roll_err  * DT;
    pitch_err_i += pitch_err * DT;
    yaw_rate_err_i += yaw_rate_err * DT;

    // --- 積分風袋防止 ---
    roll_err_i  = constrain(roll_err_i, -0.01, 0.01);
    pitch_err_i = constrain(pitch_err_i, -0.01, 0.01);
    yaw_rate_err_i = constrain(yaw_rate_err_i, -0.01, 0.01);

    // --- PID計算 ---
    roll_out  = Pid.Kp_roll  * roll_err  + Pid.Kd_roll  * roll_err_d  + Pid.Ki_roll  * roll_err_i;
    pitch_out = Pid.Kp_pitch * pitch_err + Pid.Kd_pitch * pitch_err_d + Pid.Ki_pitch * pitch_err_i;
    yaw_out   = Pid.Kp_yaw_rate * yaw_rate_err + Pid.Kd_yaw_rate * yaw_rate_err_d + Pid.Ki_yaw_rate * yaw_rate_err_i;

    // --- モーター出力計算（Quad X型） ---
    raw_FL = Pid.throttle_hover + roll_out + pitch_out + yaw_out + Cmdr.throttle_cmd;
    raw_FR = Pid.throttle_hover - roll_out + pitch_out - yaw_out + Cmdr.throttle_cmd;
    raw_RL = Pid.throttle_hover + roll_out - pitch_out - yaw_out + Cmdr.throttle_cmd;
    raw_RR = Pid.throttle_hover - roll_out - pitch_out + yaw_out + Cmdr.throttle_cmd;

    // --- 前回誤差を保存 ---
    roll_err_prev  = roll_err;
    pitch_err_prev = pitch_err;
    yaw_rate_prev  = Mwf.yaw_rate;
}

void computePIDW(Attitude &Mwf, PIDCtrl &Pid, CmdrBLE &Cmdr)
{
    static int cnt = 0;
    cnt++;

    ratePIDW(Mwf, Pid, Cmdr); // 毎回

    if (cnt >= mabiki_DT)
    {
        cnt = 0;
        anglePIDW(Mwf, Pid, Cmdr); // 150Hz
    }

    // --- モーター出力計算（Quad X型） ---
    raw_FL = Pid.throttle_hover + roll_out + pitch_out + yaw_out + Cmdr.throttle_cmd;
    raw_FR = Pid.throttle_hover - roll_out + pitch_out - yaw_out + Cmdr.throttle_cmd;
    raw_RL = Pid.throttle_hover + roll_out - pitch_out - yaw_out + Cmdr.throttle_cmd;
    raw_RR = Pid.throttle_hover - roll_out - pitch_out + yaw_out + Cmdr.throttle_cmd;
}

void ratePIDW(Attitude &Mwf, PIDCtrl &Pid, CmdrBLE &Cmdr)
{
    if(fabs(Cmdr.yaw_rate_tag) > thr_yaw_rate)
    {
        yaw_rate_tag  = Cmdr.yaw_rate_tag;
        Cmdr.yaw_last = Mwf.yaw;
    }

    // --- 角速度 比例誤差計算 (P項) ---
    roll_rate_err  = roll_rate_tag  - Mwf.roll_rate;
    pitch_rate_err = pitch_rate_tag - Mwf.pitch_rate;
    yaw_rate_err   = yaw_rate_tag   - Mwf.yaw_rate;

    // --- 角速度 今前差分計算（D項） ---
    roll_rate_err_dr  = roll_rate_prev  - Mwf.roll_rate;
    pitch_rate_err_dr = pitch_rate_prev - Mwf.pitch_rate;
    yaw_rate_err_dr   = yaw_rate_prev   - Mwf.yaw_rate;

    roll_rate_err_d  += lpf_d * (roll_rate_err_dr  - roll_rate_err_d);
    pitch_rate_err_d += lpf_d * (pitch_rate_err_dr - pitch_rate_err_d);
    yaw_rate_err_d   += lpf_d * (yaw_rate_err_dr   - yaw_rate_err_d);

    // --- 角速度 累積誤差計算（I項） ---
    roll_rate_err_i  += roll_rate_err  * DT;
    pitch_rate_err_i += pitch_rate_err * DT;
    yaw_rate_err_i   += yaw_rate_err   * DT;

    // --- 積分風袋防止 ---
    roll_rate_err_i  = constrain(roll_rate_err_i,  -10, 10);
    pitch_rate_err_i = constrain(pitch_rate_err_i, -10, 10);
    yaw_rate_err_i   = constrain(yaw_rate_err_i,   -10, 10);

    // --- モータ出力 PID計算 ---
    roll_out  = Pid.Kp_roll_rate  * roll_rate_err  + Pid.Kd_roll_rate  * roll_rate_err_d  + Pid.Ki_roll_rate  * roll_rate_err_i;
    pitch_out = Pid.Kp_pitch_rate * pitch_rate_err + Pid.Kd_pitch_rate * pitch_rate_err_d + Pid.Ki_pitch_rate * pitch_rate_err_i;
    yaw_out   = Pid.Kp_yaw_rate   * yaw_rate_err   + Pid.Kd_yaw_rate   * yaw_rate_err_d   + Pid.Ki_yaw_rate   * yaw_rate_err_i;

    // --- 前回誤差を保存 ---
    roll_rate_err_prev  = roll_rate_err;
    pitch_rate_err_prev = pitch_rate_err;
    yaw_rate_err_prev   = yaw_rate_err;

    roll_rate_prev  = Mwf.roll_rate;
    pitch_rate_prev = Mwf.pitch_rate;
    yaw_rate_prev   = Mwf.yaw_rate;
}

void anglePIDW(Attitude &Mwf, PIDCtrl &Pid, CmdrBLE &Cmdr)
{
    // --- 角度 比例誤差計算 (P項) ---
    roll_err  = wrap180(Cmdr.roll_tag  - Mwf.roll);
    pitch_err = wrap180(Cmdr.pitch_tag - Mwf.pitch);
    yaw_err   = wrap180(Cmdr.yaw_last  - Mwf.yaw);

    // --- 角度 微分誤差計算（D項） ---
    roll_err_d  = -Mwf.roll_rate;
    pitch_err_d = -Mwf.pitch_rate;
    yaw_err_d   = -Mwf.yaw_rate;

    // --- 角度 累積誤差計算（I項） ---
    roll_err_i  += roll_err  * (DT*mabiki_DT);
    pitch_err_i += pitch_err * (DT*mabiki_DT);
    yaw_err_i   += yaw_err   * (DT*mabiki_DT);

    // --- 積分風袋防止 ---
    roll_err_i  = constrain(roll_err_i,  -0.01, 0.01);
    pitch_err_i = constrain(pitch_err_i, -0.01, 0.01);
    yaw_err_i   = constrain(yaw_err_i,   -0.01, 0.01);

    // --- 目標角速度 PID計算 ---
    roll_rate_tag  = Pid.Kp_roll  * roll_err  + Pid.Kd_roll  * roll_err_d  + Pid.Ki_roll  * roll_err_i;
    pitch_rate_tag = Pid.Kp_pitch * pitch_err + Pid.Kd_pitch * pitch_err_d + Pid.Ki_pitch * pitch_err_i;
    yaw_rate_tag   = Pid.Kp_yaw   * yaw_err   + Pid.Kd_yaw   * yaw_err_d   + Pid.Ki_yaw   * yaw_err_i;

    // --- 前回誤差を保存 ---
    roll_err_prev  = roll_err;
    pitch_err_prev = pitch_err;
    yaw_err_prev   = yaw_err;
}

void ctrlMotor(DutyMotor &Duty)
{
    Duty.FL = constrain(raw_FL, 0.0, 1.0);
    Duty.FR = constrain(raw_FR, 0.0, 1.0);
    Duty.RL = constrain(raw_RL, 0.0, 1.0);
    Duty.RR = constrain(raw_RR, 0.0, 1.0);

    // --- モーター出力適用 ---
    driveMotor(0, Duty.FL);
    driveMotor(1, Duty.FR);
    driveMotor(2, Duty.RL);
    driveMotor(3, Duty.RR);
}

void stopMotor(DutyMotor &Duty)
{
    Duty.FL = 0;
    Duty.FR = 0;
    Duty.RL = 0;
    Duty.RR = 0;

    driveMotor(0, Duty.FL);
    driveMotor(1, Duty.FR);
    driveMotor(2, Duty.RL);
    driveMotor(3, Duty.RR);
}
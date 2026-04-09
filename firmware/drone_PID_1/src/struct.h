#pragma once

struct IMUData
{
    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;

    float mx;
    float my;
    float mz;
};

struct Attitude
{
    float roll;
    float pitch;
    float yaw;
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
};

struct PIDCtrl
{
    float throttle_hover;

    float Kp_roll;
    float Ki_roll;
    float Kd_roll;

    float Kp_pitch;
    float Ki_pitch;
    float Kd_pitch;

    float Kp_yaw;
    float Ki_yaw;
    float Kd_yaw;

    float Kp_roll_rate;
    float Ki_roll_rate;
    float Kd_roll_rate;

    float Kp_pitch_rate;
    float Ki_pitch_rate;
    float Kd_pitch_rate;

    float Kp_yaw_rate;
    float Ki_yaw_rate;
    float Kd_yaw_rate;
};

struct CmdrBLE
{
    float roll_tag     = 0;
    float pitch_tag    = 0;
    float yaw_rate_tag = 0;
    float throttle_cmd = 0;
    float yaw_last     = 0;
};

struct DutyMotor
{
    float FL;
    float FR;
    float RL;
    float RR;
};

struct CmdtBLE
{
    uint8_t throttle_hover;

    uint8_t Kp_roll;
    uint8_t Ki_roll;
    uint8_t Kd_roll;

    uint8_t Kp_pitch;
    uint8_t Ki_pitch;
    uint8_t Kd_pitch;

    uint8_t Kp_yaw;
    uint8_t Ki_yaw;
    uint8_t Kd_yaw;
    
    uint8_t Kp_roll_rate;
    uint8_t Ki_roll_rate;
    uint8_t Kd_roll_rate;

    uint8_t Kp_pitch_rate;
    uint8_t Ki_pitch_rate;
    uint8_t Kd_pitch_rate;

    uint8_t Kp_yaw_rate;
    uint8_t Ki_yaw_rate;
    uint8_t Kd_yaw_rate;

    uint8_t roll_tag = 0;
    uint8_t pitch_tag = 0;
    uint8_t yaw_rate_tag = 0;
    uint8_t throttle_cmd = 0;

    uint8_t startFlag = 0;
    uint8_t defaultFlag = 1;
    uint8_t w_to_s_Flag = 1;
    uint32_t LastCmdTime = 0;
};
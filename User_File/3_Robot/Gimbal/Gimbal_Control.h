//
// Created by Lenovo on 2026/1/13.
//

#ifndef TEST_ROBOWAKER_GIMBAL_CONTROL_H
#define TEST_ROBOWAKER_GIMBAL_CONTROL_H

#include <cmath>

#include "1_Middleware/2_Algorithm/PID/alg_pid.h"
#include "1_Middleware/1_Driver/Math/drv_math.h"

#include "2_Device/Motor/Motor_DM/dvc_motor_dm.h"
#include "2_Device/DM_IMU/dvc_dm_imu.h"

enum Enum_Gimbal_Control_Type
{
    Gimbal_Control_Type_DISABLE = 0,
    Gimbal_Control_Type_POSITION,
};

// 上电自举状态机：等IMU -> 抬头进安全区 -> 正常控制
enum Enum_Gimbal_Startup_State
{
    Gimbal_Startup_WAIT_IMU = 0,
    Gimbal_Startup_LIFT_TO_SAFE,
    Gimbal_Startup_NORMAL,
};

class Class_Gimbal
{
public:
    Class_DM_IMU DM_IMU;

    Class_Motor_DM_Normal Yaw_Motor_MIT;
    Class_Motor_DM_Normal Pitch_Motor_MIT;

    Class_PID Class_PID_Yaw_Angle_IMU;
    Class_PID Class_PID_Yaw_Omega_IMU;
    Class_PID Class_PID_Pitch_Angle_IMU;
    Class_PID Class_PID_Pitch_Omega_IMU;

    void Init();

    void TIM_100ms_Alive_PeriodElapedCallback();
    void TIM_1ms_Resolution_PeriodElapedCallback();
    void TIM_1ms_Control_PeriodElapedCallback();

    inline float Get_Now_Yaw_Angle();
    inline float Get_Now_Pitch_Angle();
    inline float Get_Now_Yaw_Omega();
    inline float Get_Now_Pitch_Omega();

    inline float Get_Target_Yaw_Angle();
    inline float Get_Target_Pitch_Angle();

    inline void Set_Target_Yaw_Angle(float __Target_Yaw_Angle);
    inline void Set_Target_Pitch_Angle(float __Target_Pitch_Angle);

    inline void Set_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type);

public:
    // 方向/零位
    float Yaw_Direction = 1.0f;
    float Pitch_Direction = 1.0f;

    float Yaw_Motor_Direction   = 1.0f;
    float Pitch_Motor_Direction = 1.0f;

    float Yaw_Offset_Rad   = 0.0f;
    float Pitch_Offset_Rad = 0.0f;

    // Pitch 限位（rad）
    float Min_Pitch_Angle = -0.60f;
    float Max_Pitch_Angle =  0.33f;

    // 力矩限幅（Nm）
    float Yaw_Torque_Limit   = 9.0f;
    float Pitch_Torque_Limit = 3.2f;

    // ---------- 上电自举参数 ----------
    bool Startup_Enable = true;

    // IMU 连续在线多少 ms 才认为 ready（你 100ms alive 会更新 status，这里用 30~80ms 都行）
    uint16_t IMU_Ready_Time_Threshold_ms = 30;

    // 安全边界：抬到 (Min+margin, Max-margin) 范围内
    float Startup_Safe_Margin_Rad = 0.06f;

    // 抬头完成判据
    float Startup_Finish_Err_Rad   = 0.05f;
    float Startup_Finish_Omega_Rad = 1.0f;
    // --------------------------------

protected:
    // yaw轴阻力前馈()
    float Yaw_Feed_Torque = 0.5f;

    float Now_Yaw_Angle = 0.0f;
    float Now_Pitch_Angle = 0.0f;
    float Now_Yaw_Omega = 0.0f;
    float Now_Pitch_Omega = 0.0f;

    Enum_Gimbal_Control_Type Gimbal_Control_Type = Gimbal_Control_Type_DISABLE;

    float Target_Yaw_Angle = 0.0f;
    float Target_Pitch_Angle = 0.0f;

    // yaw 解缠
    bool  yaw_inited   = false;
    float yaw_deg_last = 0.0f;
    float yaw_rad_cont = 0.0f;

    // 上电状态
    Enum_Gimbal_Startup_State Startup_State = Gimbal_Startup_WAIT_IMU;
    uint16_t imu_ready_cnt_ms = 0;

protected:
    float Normalize_Delta_Rad(float delta);
    float Yaw_Unwrap_To_Cont_Rad(float yaw_deg);

    void Self_Resolution();
    void Output();
};

// ---------------- inline 实现 ----------------

inline float Class_Gimbal::Get_Now_Yaw_Angle()   { return (Now_Yaw_Angle); }
inline float Class_Gimbal::Get_Now_Pitch_Angle() { return Now_Pitch_Angle; }
inline float Class_Gimbal::Get_Now_Yaw_Omega()   { return Now_Yaw_Omega; }
inline float Class_Gimbal::Get_Now_Pitch_Omega() { return Now_Pitch_Omega; }

inline float Class_Gimbal::Get_Target_Yaw_Angle()   { return Target_Yaw_Angle; }
inline float Class_Gimbal::Get_Target_Pitch_Angle() { return Target_Pitch_Angle; }

inline void Class_Gimbal::Set_Target_Yaw_Angle(float __Target_Yaw_Angle)     { Target_Yaw_Angle = __Target_Yaw_Angle; }
inline void Class_Gimbal::Set_Target_Pitch_Angle(float __Target_Pitch_Angle) { Target_Pitch_Angle = __Target_Pitch_Angle; }

inline void Class_Gimbal::Set_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type)
{
    // 从 DISABLE -> 使能：锁住当前角为目标，避免一上电抽一下
    if (Gimbal_Control_Type == Gimbal_Control_Type_DISABLE && __Gimbal_Control_Type != Gimbal_Control_Type_DISABLE)
    {
        if (DM_IMU.Get_IMU_Status() == Enum_DM_IMU_Status_ENABLE)
        {
            Target_Yaw_Angle = Now_Yaw_Angle;
            Target_Pitch_Angle = Now_Pitch_Angle;
        }
    }
    Gimbal_Control_Type = __Gimbal_Control_Type;
}

#endif //TEST_ROBOWAKER_GIMBAL_CONTROL_H
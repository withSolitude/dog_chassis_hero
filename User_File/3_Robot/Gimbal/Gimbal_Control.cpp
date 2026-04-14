//
// Created by Lenovo on 2026/1/13.
//
#include "Gimbal_Control.h"

static inline float limit(float x,float lo,float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void Class_Gimbal::Init()
{
    // YAW轴电机
    Yaw_Motor_MIT.Init(&hfdcan2,0x03,0x03,Motor_DM_Control_Method_NORMAL_MIT);
    Yaw_Motor_MIT.CAN_Send_Clear_Error();
    Yaw_Motor_MIT.CAN_Send_Enter();
    //
    // //PITCH轴电机
    // Pitch_Motor_MIT.Init(&hcan2,0x00,0x00,Motor_DM_Control_Method_NORMAL_MIT);
    // Pitch_Motor_MIT.CAN_Send_Clear_Error();
    // Pitch_Motor_MIT.CAN_Send_Enter();
    //
    Yaw_Motor_MIT.Set_K_P(0.0f);
    Yaw_Motor_MIT.Set_K_D(0.0f);
    // Pitch_Motor_MIT.Set_K_P(0.0f);
    // Pitch_Motor_MIT.Set_K_D(0.0f);

    DM_IMU.Init(&hfdcan2,0x01,0x02);
    DM_IMU.Enable_Active_Output(10,true);

    // // YAW轴角度环PID初始化
    // Class_PID_Yaw_Angle_IMU.Init(0.5f,0.0f,0.0f);
    // // YAW轴角速度环PID初始化
    // Class_PID_Yaw_Omega_IMU.Init(7.52661f,1.01194f,0.02244f,10.2586f,1.15474f,6.93817f);
    // Class_PID_Yaw_Omega_IMU.Init(7.52661f,1.01194f,0.02244f);

    // // PITCH角度环PID初始化
    // Class_PID_Pitch_Angle_IMU.Init(0.0f,0.0f,0.0f);
    // // PITCH角速度环PID初始化
    // Class_PID_Pitch_Omega_IMU.Init(0.0f,0.0f,0.0f);


}

void Class_Gimbal::TIM_100ms_Alive_PeriodElapedCallback()
{
    DM_IMU.TIM_100ms_Alive_PeriodElapsedCallback();
    Yaw_Motor_MIT.TIM_Alive_PeriodElapsedCallback();
    ///Pitch_Motor_MIT.TIM_Alive_PeriodElapsedCallback();
}

void Class_Gimbal::TIM_1ms_Resolution_PeriodElapedCallback()
{
    Self_Resolution();
}

void Class_Gimbal::TIM_1ms_Control_PeriodElapedCallback()
{
    Output();

    Yaw_Motor_MIT.TIM_Send_PeriodElapsedCallback();
    //Pitch_Motor_MIT.TIM_Send_PeriodElapsedCallback();
}

float Class_Gimbal::Normalize_Delta_Rad(float delta)
{
    // 归一到 [-pi, pi]
    delta = fmodf(delta, 2.0f * PI);
    if (delta > PI)  delta -= 2.0f * PI;
    if (delta < -PI) delta += 2.0f * PI;
    return delta;
}

float Class_Gimbal::Yaw_Unwrap_To_Cont_Rad(float yaw_deg)
{
    // IMU yaw: [-180,180] -> 连续角（rad）
    if (!yaw_inited)
    {
        yaw_inited   = true;
        yaw_deg_last = yaw_deg;
        yaw_rad_cont = yaw_deg * DEG_TO_RAD;
        return yaw_rad_cont;
    }

    float delta_deg = yaw_deg - yaw_deg_last;
    // 归一到 [-180, 180]
    delta_deg = Math_Modulus_Normalization(delta_deg, 360.0f);

    yaw_rad_cont += delta_deg * DEG_TO_RAD;
    yaw_deg_last = yaw_deg;
    return yaw_rad_cont;
}

void Class_Gimbal::Self_Resolution()
{
    if (DM_IMU.Get_IMU_Status() == Enum_DM_IMU_Status_DISABLE)  return;
    Now_Yaw_Angle = Yaw_Direction*Yaw_Unwrap_To_Cont_Rad(DM_IMU.Get_Yaw_Deg())+Yaw_Offset_Rad;

    Now_Pitch_Angle = Pitch_Direction * (DM_IMU.Get_Pitch_Deg() * DEG_TO_RAD) + Pitch_Offset_Rad;

    Now_Yaw_Omega = Yaw_Direction*Yaw_Motor_MIT.Get_Now_Omega();

    Now_Pitch_Omega = Pitch_Direction*Pitch_Motor_MIT.Get_Now_Omega();
}

void Class_Gimbal::Output()
{
    if (Gimbal_Control_Type == Gimbal_Control_Type_DISABLE)
    {
        //清积分
        Class_PID_Yaw_Angle_IMU.Set_Integral_Error(0.0f);
        Class_PID_Yaw_Omega_IMU.Set_Integral_Error(0.0f);
        // Class_PID_Pitch_Angle_IMU.Set_Integral_Error(0.0f);
        // Class_PID_Pitch_Omega_IMU.Set_Integral_Error(0.0f);

        Yaw_Motor_MIT.Set_K_P(0.0f);
        Yaw_Motor_MIT.Set_K_D(0.0f);
        // Pitch_Motor_MIT.Set_K_P(0.0f);
        // Pitch_Motor_MIT.Set_K_D(0.0f);

        Yaw_Motor_MIT.Set_Control_Torque(0.0f);
        // Pitch_Motor_MIT.Set_Control_Torque(0.0f);

        return;
    }

    // 2) IMU 未 ready：先不控（防止追默认 Target=0）
    static bool imu_latched = false;
    if (DM_IMU.Get_IMU_Status() != Enum_DM_IMU_Status_ENABLE)
    {
        imu_latched = false;
        Yaw_Motor_MIT.Set_Control_Torque(0.0f);
        return;
    }

    // 3) IMU 第一次 ready：锁一次目标 = 当前角
    if (!imu_latched)
    {
        Target_Yaw_Angle = Now_Yaw_Angle;
        imu_latched = true;
    }


    Target_Pitch_Angle = limit(Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);
    // yaw “就近转位”：目标永远落在当前附近的等效角
    float yaw_delta = Normalize_Delta_Rad(Target_Yaw_Angle - Now_Yaw_Angle);
    float yaw_target_near = Now_Yaw_Angle + yaw_delta;

    Class_PID_Yaw_Angle_IMU.Set_Target(yaw_target_near);
    Class_PID_Yaw_Angle_IMU.Set_Now(Now_Yaw_Angle);
    Class_PID_Yaw_Angle_IMU.TIM_Calculate_PeriodElapsedCallback();
    float yaw_omega_tar = Class_PID_Yaw_Angle_IMU.Get_Out();


    //Class_PID_Yaw_Omega_IMU.Set_Target(yaw_omega_tar);
    Class_PID_Yaw_Omega_IMU.Set_Target(1.0f);
    Class_PID_Yaw_Omega_IMU.Set_Now(Now_Yaw_Omega);
    Class_PID_Yaw_Omega_IMU.TIM_Calculate_PeriodElapsedCallback();
    float yaw_torque = Class_PID_Yaw_Omega_IMU.Get_Out();
    yaw_torque = limit(yaw_torque, -Yaw_Torque_Limit, Yaw_Torque_Limit);

    // Class_PID_Pitch_Angle_IMU.Set_Target(Target_Pitch_Angle);
    // Class_PID_Pitch_Angle_IMU.Set_Now(Now_Pitch_Angle);
    // Class_PID_Pitch_Angle_IMU.TIM_Calculate_PeriodElapsedCallback();
    // float pitch_omega_tar = Class_PID_Pitch_Angle_IMU.Get_Out();
    //
    // Class_PID_Pitch_Omega_IMU.Set_Target(pitch_omega_tar);
    // Class_PID_Pitch_Omega_IMU.Set_Now(Now_Pitch_Omega);
    // Class_PID_Pitch_Omega_IMU.TIM_Calculate_PeriodElapsedCallback();
    // float pitch_torque = Class_PID_Pitch_Omega_IMU.Get_Out();
    // pitch_torque = limit(pitch_torque, -Pitch_Torque_Limit, Pitch_Torque_Limit);

    Yaw_Motor_MIT.Set_K_P(0.0f);
    Yaw_Motor_MIT.Set_K_D(0.0f);
    Pitch_Motor_MIT.Set_K_P(0.0f);
    Pitch_Motor_MIT.Set_K_D(0.0f);

    Yaw_Motor_MIT.Set_Control_Torque(Yaw_Motor_Direction*yaw_torque);
    // Pitch_Motor_MIT.Set_Control_Torque(pitch_torque);

    Yaw_Motor_MIT.Set_Control_Angle(0.0f);
    // Pitch_Motor_MIT.Set_Control_Angle(0.0f);
    Yaw_Motor_MIT.Set_Control_Omega(0.0f);
    // Pitch_Motor_MIT.Set_Control_Omega(0.0f);

}


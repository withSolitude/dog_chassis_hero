//
// Created by Lenovo on 2026/1/12.
//
#include "ita_robot.h"

// 爆发优先热量打表
static const float shoot_burst_first_heat_max[10] = {200.0f, 250.0f, 300.0f, 350.0f, 400.0f,
                                                     450.0f, 500.0f, 550.0f, 600.0f, 650.0f};

// 爆发优先冷却打表
static const float shoot_burst_first_heat_cd[10] = {10.0f, 15.0f, 20.0f, 25.0f, 30.0f,
                                                    35.0f, 40.0f, 45.0f, 50.0f, 60.0f};

// 冷却优先热量打表
static const float shoot_cd_first_heat_max[10] = {50.0f, 85.0f, 120.0f, 155.0f, 190.0f,
                                                  225.0f, 260.0f, 295.0f, 330.0f, 400.0f};

// 冷却优先冷却打表
static const float shoot_cd_first_heat_cd[10] = {40.0f, 45.0f, 50.0f, 55.0f, 60.0f,
                                                 65.0f, 70.0f, 75.0f, 80.0f, 80.0f};

/**
 *@brief 控制交互端初始化
 *
 */
void Class_Robot::Init()
{

    // 裁判系统初始化
    //referee.Init(&huart6);

    // 底盘斜坡函数初始化
    // Slope_Speed_X.Init(0.003f,0.006f,Slope_First_TARGET);
    // Slope_Speed_Y.Init(0.003f,0.006f,Slope_First_TARGET);
    // Slope_Speed_Omega.Init(0.003f,0.006f,Slope_First_TARGET);
    Slope_Speed_X.Init(0.009f,0.012f,Slope_First_TARGET);
    Slope_Speed_Y.Init(0.009f,0.012f,Slope_First_TARGET);
    Slope_Speed_Omega.Init(0.020f,0.020f,Slope_First_TARGET);
    Slope_Track_Omega.Init(0.012f, 0.020f, Slope_First_REAL);
    Slope_Leg_Left_Angle.Init(0.0010f, 0.0015f, Slope_First_REAL);
    Slope_Leg_Right_Angle.Init(0.0010f, 0.0015f, Slope_First_REAL);
    // 遥控器初始化
    VT13.Init(&huart1);

    // 底盘初始化
    Chassis.Init();

    // 云台初始化
    Gimbal.Init();

    // 发射初始化

    // 姿态感知器初始化

    // 最大底盘功率限制初始化

}


/**
 *@brief TIM定时器中断定时检测模块是否存活
 *
 */
void Class_Robot::TIM_1000ms_Alive_PeriodElapsedCallback()
{
    //referee.TIM_1000ms_Alive_PeriodElapsedCallback();
}

/**
 *@brief TIM定时器定时检测模块是否存活
 *
 */
void Class_Robot::TIM_100ms_Alive_PeriodElapsedCallback()
{
    VT13.TIM_100ms_Alive_PeriodElapsedCallback();

    Chassis.TIM_100ms_Alive_PeriodElapsedCallback();
    Gimbal.TIM_100ms_Alive_PeriodElapedCallback();
}

/**
 *@brief 定时器计算函数
 *
 */
void Class_Robot::TIM_100ms_Calculate_Callback()
{

}

/**
 *@brief
 *
 */
void Class_Robot::TIM_10ms_Calculate_PeriodElapsedCallback()
{

}

/**
 *@brief
 *
 */
void Class_Robot::TIM_2ms_Calculate_PeriodElapsedCallback()
{
    Chassis.TIM_2ms_Resolution_PeriodElapsedCallback();
    Chassis.TIM_2ms_Control_PeriodElapsedCallback();
}

/**
 *@brief
 *
 */
void Class_Robot::TIM_1ms_Calculate_Callback()
{



    // 遥控器处理上升沿下降沿
    VT13.TIM_1ms_Calculate_PeriodElapsedCallback();

    _Chassis_Control();

    Gimbal.TIM_1ms_Resolution_PeriodElapedCallback();

    _Gimbal_Control();
    Gimbal.TIM_1ms_Control_PeriodElapedCallback();

    _Shooter_Control();
    Shooter.TIM_1ms_Calculate_PeriodElapsedCallback();
}

/**
 *@brief 状态控制逻辑
 *
 */
void Class_Robot::_Status_Control()
{
    // 判断遥控器是否正常，不在线或急停直接断控
    if (VT13.Get_Status()==Image_Transform_Status_DISABLE )
        return;

    // 整车状态控制
    if (VT13.Get_Keyboard_Key_CTRL() == Image_Transform_Key_Status_FREE)
    {

    }
}

/**
 *@brief 底盘控制逻辑
 *
 */
void Class_Robot::_Chassis_Control()
{
    // 速度上限与加减速规划控制
    float tmp_chassis_velocity_max,tmp_chassis_omega_max;
    // if (Supercap_Accelerate_Status == true)
    // {
    //
    // }
    // else
    // {
    //
    // }
    tmp_chassis_velocity_max = 3.0f;//3.0f;
    tmp_chassis_omega_max = 3.0f*PI;

    //Slope_Speed_X.Set_Increase_Value(5.0f / 1000.0f)

    // 判断遥控器状态是否正常，不在线或急停直接断控
    if (VT13.Get_Status()==Image_Transform_Status_DISABLE || VT13.Get_Mode_Switch_Raw()==Image_Transform_Mode_SW_C)
    {
        Chassis.Set_Chassis_Control_Typer(Chassis_Control_Type_DISABLE);
        return;
    }
    else
    {
        // 比赛模式
        if (VT13.Get_Mode_Switch_Raw()==Image_Transform_Mode_SW_N)
            Chassis.Set_Chassis_Control_Typer(Chassis_Control_Type_Wheel);
        if (VT13.Get_Mode_Switch_Raw()==Image_Transform_Mode_SW_S)
            Chassis.Set_Chassis_Control_Typer((Chassis_Control_Type_Wheel_Track_Leg));

        // 底盘速度期望值
        float tmp_expect_direction_velocity_x,tmp_expect_direction_velocity_y,tmp_expect_direction_omega;
        // 底盘速度经过斜坡函数的值
        float tmp_planning_chassis_velocity_x,tmp_planning_chassis_velocity_y,tmp_planning_chassis_omega,tmp_planning_leg_angle;
        // 底盘速度传参值
        float tmp_chassis_velocity_x,tmp_chassis_velocity_y,tmp_chassis_omega;

        // 履带转动标志位
        uint8_t Track_Move_Flag;
        // 履带电机速度期望值
        float tmp_expect_track_wheel_omega;

        // 腿电机角度期望值
        float tmp_expect_leg_angle;

        // 遥控器摇杆值
        float vt13_left_x = VT13.Get_Left_X();
        float vt13_left_y = VT13.Get_Left_Y();
        float vt13_yaw = - VT13.Get_Right_Y();
        // float vt13_yaw = VT13.Get_Left_Y();
        // float vt13_left_y = VT13.Get_Right_Y();
        // float vt13_yaw = VT13.Yaw();

        static uint8_t tarck_on_sw = 0;
        // if (VT13.Get_Trigger_Raw()==Image_Transform_BTN_PRESSED)
        if (VT13.Get_Trigger()==Image_Transform_Key_Status_TRIG_FREE_PRESSED)
        {
            tarck_on_sw ^= 1;
        }


        float vt13_dial = VT13.Get_Dial() * dial_to_angle;

        // 排除死区
        vt13_left_x = Math_Abs(vt13_left_x) > VT13_Dead_Zone ? vt13_left_x : 0.0f;
        vt13_left_y = Math_Abs(vt13_left_y) > VT13_Dead_Zone ? vt13_left_y : 0.0f;
        vt13_yaw = Math_Abs(vt13_yaw) >  VT13_Dead_Zone ? vt13_yaw : 0.0f;

        vt13_dial = Math_Abs(vt13_dial) > VT13_Dead_Zone ? vt13_dial  : 0.0f;

        // 摇杆控制
        tmp_expect_direction_velocity_x = vt13_left_x * tmp_chassis_velocity_max;
        tmp_expect_direction_velocity_y =  vt13_left_y * tmp_chassis_velocity_max;
        tmp_expect_direction_omega = vt13_yaw * tmp_chassis_omega_max;
        tmp_expect_track_wheel_omega = tarck_on_sw ? 5.0f : 0.0f;
        tmp_expect_leg_angle = -vt13_dial;

        float track_now = 0.5f * (Chassis.Get_Now_Left_Track_Wheel_Omega() + Chassis.Get_Now_Right_Track_Wheel_Omega());

        Slope_Track_Omega.Set_Now_Real(track_now);
        Slope_Track_Omega.Set_Target(tmp_expect_track_wheel_omega);
        Slope_Track_Omega.TIM_Calculate_PeriodElapsedCallback();

        float tmp_planning_track_wheel_omega = Slope_Track_Omega.Get_Out();

        // 键鼠
        if (VT13.Get_Keyboard_Key_CTRL() == Image_Transform_Key_Status_FREE)
        {
            // 没有按下Ctrl
            if (VT13.Get_Keyboard_Key_W() == Image_Transform_Key_Status_PRESSED)
            {
                tmp_expect_direction_velocity_x += tmp_chassis_velocity_max;
            }
            if (VT13.Get_Keyboard_Key_S() == Image_Transform_Key_Status_PRESSED)
            {
                tmp_expect_direction_velocity_x -= tmp_chassis_velocity_max;
            }
            if (VT13.Get_Keyboard_Key_A() == Image_Transform_Key_Status_PRESSED)
            {
                tmp_expect_direction_velocity_y += tmp_chassis_velocity_max;
            }
            if (VT13.Get_Keyboard_Key_D() == Image_Transform_Key_Status_PRESSED)
            {
                tmp_expect_direction_velocity_y -= tmp_chassis_velocity_max;
            }
        }

        // 小陀螺状态控制
        if (Chassis_Gyroscope_Type == Robot_Gyroscope_Type_CLOCKWISE)
        {
            tmp_expect_direction_omega += tmp_chassis_omega_max;
        }
        if (Chassis_Gyroscope_Type == Robot_Gyroscope_Type_ANTICLOCKWISE)
        {
            tmp_expect_direction_omega -= tmp_chassis_omega_max;
        }

        // 非小陀螺则底盘跟随



        // 速度斜坡函数
        Slope_Speed_X.Set_Target(tmp_expect_direction_velocity_x);
        Slope_Speed_Y.Set_Target(tmp_expect_direction_velocity_y);
        Slope_Speed_Omega.Set_Target(tmp_expect_direction_omega);

        Slope_Speed_X.TIM_Calculate_PeriodElapsedCallback();
        Slope_Speed_Y.TIM_Calculate_PeriodElapsedCallback();
        Slope_Speed_Omega.TIM_Calculate_PeriodElapsedCallback();

        // 腿电机斜坡函数

        float leg_l_now = Chassis.Leg_Wheel[0].Get_Now_Angle();
        float leg_r_now = Chassis.Leg_Wheel[1].Get_Now_Angle();
        Slope_Leg_Left_Angle.Set_Now_Real(leg_l_now);
        Slope_Leg_Right_Angle.Set_Now_Real(leg_r_now);

        Slope_Leg_Left_Angle.Set_Target(tmp_expect_leg_angle);
        Slope_Leg_Right_Angle.Set_Target(tmp_expect_leg_angle);

        Slope_Leg_Left_Angle.TIM_Calculate_PeriodElapsedCallback();
        Slope_Leg_Right_Angle.TIM_Calculate_PeriodElapsedCallback();

        tmp_planning_leg_angle = (Slope_Leg_Left_Angle.Get_Out()+Slope_Leg_Right_Angle.Get_Out())/2;



        // 底盘相对于云台角度
        float cos_yaw,sin_yaw;



        // 底盘相对于云台运动方向
        float chassis_now_vx,chassis_now_vy,chassis_now_omega;


        // 规划速度
        // Slope_Speed_X.Set_Now_Real(chassis_now_vx);
        // Slope_Speed_Y.Set_Now_Real(chassis_now_vy);
        // //Slope_Speed_Omega.Set_Now_Real(chassis_now_omega);
        //
        // Slope_Speed_X.Set_Target(tmp_expect_direction_velocity_x);
        // Slope_Speed_Y.Set_Target(tmp_expect_direction_velocity_y);
        // Slope_Speed_X.TIM_Calculate_PeriodElapsedCallback();
        // Slope_Speed_Y.TIM_Calculate_PeriodElapsedCallback();

        // 保存规划后结果
        tmp_planning_chassis_velocity_x = Slope_Speed_X.Get_Out();
        tmp_planning_chassis_velocity_y = Slope_Speed_Y.Get_Out();
        tmp_planning_chassis_omega = Slope_Speed_Omega.Get_Out();
        // 小陀螺模式前馈适配平动速度

        // 云台相对底盘角速度加前馈三角函数



        // 确保第一人称直线，乘上旋转矩阵


        // 角速度斜坡函数





        Chassis.Set_Target_Velocity_X(tmp_planning_chassis_velocity_x);
        Chassis.Set_Target_Velocity_Y(tmp_planning_chassis_velocity_y);
        Chassis.Set_Target_Omega(tmp_planning_chassis_omega);
        //Chassis.Set_Target_Omega(tmp_expect_direction_omega);
        Chassis.Set_Track_Motor_Omega(tmp_planning_track_wheel_omega);
        Chassis.Set_Leg_Motor_Angle(tmp_planning_leg_angle);
        //Chassis.Set_Target_Chassis_Pitch_Angle(tmp_expect_leg_angle);
    }
}

/**
 *@brief 云台控制逻辑
 *
 */
void Class_Robot::_Gimbal_Control()
{
    if (VT13.Get_Status()==Image_Transform_Status_DISABLE || VT13.Get_Mode_Switch_Raw()==Image_Transform_Mode_SW_C)
    {
        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
        return;
    }
    else
    {
        // 竞赛模式
        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_POSITION);

        // 云台角度传参值
        float tmp_gimbal_yaw = Gimbal.Get_Target_Yaw_Angle();
        float tmp_gimbal_pitch = Gimbal.Get_Target_Pitch_Angle();
        // 前馈值
        float tmp_gimbal_yaw_feedforward_omega = 0.0f;
        float tmp_gimbal_pitch_feedforward_omega = 0.0f;
        float tmp_gimbal_pitch_feedforward_current = 0.0f;

        // 遥控器摇杆值
        float vt13_right_x = VT13.Get_Right_X();
        float vt13_right_y = VT13.Get_Right_Y();
        // 死区
        vt13_right_x = Math_Abs(vt13_right_x) > VT13_Dead_Zone ? vt13_right_x : 0.0f;
        vt13_right_y = Math_Abs(vt13_right_y) > VT13_Dead_Zone ? vt13_right_y : 0.0f;

        // 遥控控制
        const float dt = 0.001f;

        const float yaw_rate_max = 4.5f;      // rad/s，先取 2~6 之间调手感
        tmp_gimbal_yaw -= vt13_right_y * yaw_rate_max * dt;
        tmp_gimbal_yaw_feedforward_omega -= vt13_right_y*1;
        tmp_gimbal_pitch = vt13_right_x*1;
        tmp_gimbal_pitch_feedforward_omega = vt13_right_x*1;

        // 键鼠控制

        // if (VT13.Get_Keyboard_Key_Ctrl() == DR16_Key_Status_FREE)
        // {
        //     // 没有按下Ctrl
        //
        //     tmp_gimbal_yaw -= DR16.Get_Mouse_X() * DR16_Mouse_Yaw_Angle_Resolution;
        //     tmp_gimbal_yaw_feedforward_omega -= DR16.Get_Mouse_X() * DR16_Mouse_Yaw_Angle_Resolution * 1000.0f;
        //     tmp_gimbal_pitch += DR16.Get_Mouse_Y() * DR16_Mouse_Pitch_Angle_Resolution;
        //     tmp_gimbal_pitch_feedforward_omega -= DR16.Get_Mouse_Y() * DR16_Mouse_Pitch_Angle_Resolution * 1000.0f;
        //
        //     if (DR16.Get_Keyboard_Key_G() == DR16_Key_Status_TRIG_FREE_PRESSED)
        //     {
        //         Timer_Turn_About.Set_Delay(1000);
        //         tmp_gimbal_yaw += PI;
        //     }
        //     if (DR16.Get_Keyboard_Key_Z() == DR16_Key_Status_TRIG_FREE_PRESSED)
        //     {
        //         Timer_Turn_About.Set_Delay(500);
        //         tmp_gimbal_yaw += PI / 4.0f;
        //     }
        //     if (DR16.Get_Keyboard_Key_C() == DR16_Key_Status_TRIG_FREE_PRESSED)
        //     {
        //         Timer_Turn_About.Set_Delay(500);
        //         tmp_gimbal_yaw -= PI / 4.0f;
        //     }
        // }

        // 视觉控制


        // 底盘旋转设配

        // tmp_gimbal_yaw -=

        // 重力补偿拟合

        // tmp_gimbal_pitch_feedforwad_current +=
         Gimbal.Set_Target_Yaw_Angle(tmp_gimbal_yaw);
        // Gimbal.Set_Target_Pitch_Angle(tmp_gimbal_pitch);

    }
}


void Class_Robot::_Shooter_Control()
{
    if (VT13.Get_Status()==Image_Transform_Status_DISABLE || VT13.Get_Mode_Switch_Raw()==Image_Transform_Mode_SW_C)
    {
        Shooter.Set_Shoot_Control_Type(Shoot_Control_Type_DISABLE);
        return;
    }
    else
    {
         // 1) 先更新本地长按计数
    if (VT13.Get_Mouse_Left_Key() == Image_Transform_Key_Status_PRESSED)
    {
        if (Shoot_Left_Mouse_Hold_Count < 1000)
        {
            Shoot_Left_Mouse_Hold_Count++;
        }
    }
    else
    {
        Shoot_Left_Mouse_Hold_Count = 0;
    }

    // 2) 如果你已经有本地热量估计状态机，就在这里跑
    // FSM_Heat_Detector.TIM_1ms_Calculate_PeriodElapsedCallback();

    // // 3) 先根据裁判系统/本地估计设置热量参数
    // // if (Referee.Get_Status() == Referee_Status_DISABLE ||
    // //     Referee.Get_Referee_Trust_Status() == Referee_Data_Status_DISABLE)
    // // {
    //     if (Robot_Shoot_Type == Robot_Shoot_Type_BURST)
    //     {
    //         Shooter.Set_Heat_Limit_Max(shoot_burst_first_heat_max[Robot_Level - 1]);
    //         Shooter.Set_Heat_CD(shoot_burst_first_heat_cd[Robot_Level - 1]);
    //     }
    //     else
    //     {
    //         Shooter.Set_Heat_Limit_Max(shoot_cd_first_heat_max[Robot_Level - 1]);
    //         Shooter.Set_Heat_CD(shoot_cd_first_heat_cd[Robot_Level - 1]);
    //     }
    //
    //     // 如果你已经做了本地热量估计，用它
    //     // Shoot.Set_Now_Heat(FSM_Heat_Detector.Get_Now_Heat());
    //
    //     // 暂时没有本地热量估计就先置 0
    //     Shooter.Set_Now_Heat(0.0f);
    // }
    // // else
    // // {
    //     Shooter.Set_Heat_Limit_Max(Referee.Get_Self_Booster_Heat_Max());
    //     Shooter.Set_Heat_CD(Referee.Get_Self_Booster_Heat_CD());
    //     Shooter.Set_Now_Heat(Referee.Get_Booster_17mm_1_Heat());
    // // }

    // 4) 遥控掉线 / 急停直接断控
    if (VT13.Get_Status() == Image_Transform_Status_DISABLE ||
        VT13.Get_Mode_Switch_Raw() == Image_Transform_Mode_SW_C)
    {
        Shooter.Set_Shoot_Control_Type(Shoot_Control_Type_DISABLE);
        return;
    }

    // 5) Ctrl 模式：只调摩擦轮转速，不发射
    if (VT13.Get_Keyboard_Key_CTRL() == Image_Transform_Key_Status_PRESSED)
    {
        float tmp_friction_omega = Shooter.Get_Friction_Omega();

        // 你当前是鼠标滚轮就用 Mouse_Z，若更习惯拨轮就换成 Dial
        tmp_friction_omega += VT13.Get_Mouse_Z() * 500.0f * 0.001f;

        Basic_Math_Constrain(&tmp_friction_omega, 100.0f, 1200.0f);
        Shooter.Set_Friction_Omega(tmp_friction_omega);

        return;
    }

    // 6) R 键直接关闭摩擦轮和拨弹
    if (VT13.Get_Keyboard_Key_R() == Image_Transform_Key_Status_TRIG_FREE_PRESSED)
    {
        Shooter.Set_Shoot_Control_Type(Shoot_Control_Type_DISABLE);
        return;
    }

    // 7) 根据左键和视觉状态决定 单发 / 连发 / 停火
    bool allow_fire = true;

    // 如果你现在也有视觉和切目标定时器，就保留这个条件；
    // 没有的话可以直接 allow_fire = true;
    // if (Manifold_Autoaiming_Status == true)
    // {
    //     if (Manifold.Get_Status() == Manifold_Status_DISABLE)
    //     {
    //         allow_fire = false;
    //     }
    //     else if (!(Timer_Switch_Target.Get_Now_Status() == Timer_Status_RESET ||
    //                Timer_Switch_Target.Get_Now_Status() == Timer_Status_WAIT))
    //     {
    //         allow_fire = false;
    //     }
    // }

    allow_fire = true;

    if (allow_fire == true)
    {
        // 鼠标左键单击：单发
        if (VT13.Get_Mouse_Left_Key() == Image_Transform_Key_Status_TRIG_FREE_PRESSED)
        {
            Shooter.Set_Shoot_Control_Type(Shoot_Control_Type_SPOT);
        }
        // 鼠标左键长按：连发
        else if (Shoot_Left_Mouse_Hold_Count >= Shoot_Left_Mouse_Hold_Threshold)
        {
            Shooter.Set_Shoot_Control_Type(Shoot_Control_Type_AUTO);

            // if (Referee.Get_Status() == Referee_Status_ENABLE &&
            //     Referee.Get_Referee_Trust_Status() == Referee_Data_Status_ENABLE)
            // {
            //     float tmp_frequency = Referee.Get_Self_Booster_Heat_Max() / 10.0f;
            //     Basic_Math_Constrain(&tmp_frequency, 5.0f, 18.0f);
            //     Shooter.Set_Target_Ammo_Shoot_Frequency(tmp_frequency);
            // }
            // else
            // {
            //     Shooter.Set_Target_Ammo_Shoot_Frequency(15.0f);
            // }
            Shooter.Set_Target_Ammo_Shoot_Frequency(1.0f);
        }
        // 鼠标左键松开：停火，但摩擦轮保持
        else
        {
            Shooter.Set_Shoot_Control_Type(Shoot_Control_Type_CEASEFIRE);
        }
    }
    else
    {
        Shooter.Set_Shoot_Control_Type(Shoot_Control_Type_CEASEFIRE);
    }

    }
}
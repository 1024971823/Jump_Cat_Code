/**
 * @file dvc_motor_stw.h
 * @author xylm
 * @brief 伺泰威8115-36电机MIT协议驱动
 * @version 0.1
 * @date 2026-03-30 0.1 新增
 *
 * @copyright USTC-RoboWalker (c) 2026
 *
 */

#ifndef DVC_MOTOR_STW_H
#define DVC_MOTOR_STW_H

/* Includes ------------------------------------------------------------------*/

#include "alg_pid.h"
#include "drv_can.h"
#include "alg_basic.h"

/* Exported macros -----------------------------------------------------------*/

// 伺泰威8115-36电机MIT协议参数范围 (必须与GDM810驱动板上位机设置一致)
// 位置范围: -95.5 ~ 95.5 rad
#define STW_P_MIN (-95.5f)
#define STW_P_MAX (95.5f)
// 速度范围: -45.0 ~ 45.0 rad/s
#define STW_V_MIN (-45.0f)
#define STW_V_MAX (45.0f)
// 扭矩范围: -18.0 ~ 18.0 Nm
#define STW_T_MIN (-18.0f)
#define STW_T_MAX (18.0f)
// Kp范围: 0 ~ 500
#define STW_KP_MIN (0.0f)
#define STW_KP_MAX (500.0f)
// Kd范围: 0 ~ 5
#define STW_KD_MIN (0.0f)
#define STW_KD_MAX (5.0f)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 伺泰威电机状态
 *
 */
enum Enum_Motor_STW_Status
{
    Motor_STW_Status_DISABLE = 0,
    Motor_STW_Status_ENABLE,
};

/**
 * @brief 伺泰威电机控制方式
 *
 */
enum Enum_Motor_STW_Control_Method
{
    Motor_STW_Control_Method_MIT = 0,       // MIT模式 (位置-速度-力矩混合控制)
    Motor_STW_Control_Method_ANGLE,         // 基于PID的角度闭环控制
    Motor_STW_Control_Method_OMEGA,         // 基于PID的速度闭环控制
    Motor_STW_Control_Method_TORQUE,        // 直接力矩控制
};

/**
 * @brief 伺泰威电机MIT协议接收原始数据
 *
 */
struct Struct_Motor_STW_CAN_Rx_Data_MIT
{
    uint8_t Motor_ID;
    uint16_t Angle_Raw;         // 16bit 位置
    uint16_t Omega_Raw;         // 12bit 速度
    uint16_t Torque_Raw;        // 12bit 扭矩
} __attribute__((packed));

/**
 * @brief 伺泰威电机经过处理的接收数据
 *
 */
struct Struct_Motor_STW_Rx_Data
{
    float Now_Angle;            // 当前角度, rad
    float Now_Omega;            // 当前角速度, rad/s
    float Now_Torque;           // 当前扭矩, Nm
};

/**
 * @brief Reusable, 伺泰威8115-36电机 MIT协议驱动
 *
 * MIT协议帧格式 (CAN标准帧, 8字节):
 *
 * 发送帧 (Tx):
 *   CAN_ID = Motor_ID (如0x01)
 *   Data[0~1]: 位置 (16bit, 映射到 P_MIN~P_MAX)
 *   Data[2]:   速度高8位
 *   Data[3]:   速度低4位 | Kp高4位
 *   Data[4]:   Kp低8位
 *   Data[5]:   Kd高8位
 *   Data[6]:   Kd低4位 | 扭矩高4位
 *   Data[7]:   扭矩低8位
 *
 * 接收帧 (Rx):
 *   CAN_ID = Motor_ID (如0x01)
 *   Data[0]:   Motor ID
 *   Data[1~2]: 位置 (16bit, 映射到 P_MIN~P_MAX)
 *   Data[3]:   速度高8位
 *   Data[4]:   速度低4位 | 扭矩高4位
 *   Data[5]:   扭矩低8位
 *
 */
class Class_Motor_STW
{
public:
    // PID角度环控制
    Class_PID PID_Angle;
    // PID角速度环控制
    Class_PID PID_Omega;

    void Init(const FDCAN_HandleTypeDef *hcan, const uint8_t &__Motor_ID = 0x01,
              const Enum_Motor_STW_Control_Method &__Control_Method = Motor_STW_Control_Method_MIT,
              const float &__P_Max = STW_P_MAX, const float &__V_Max = STW_V_MAX,
              const float &__T_Max = STW_T_MAX);

    inline Enum_Motor_STW_Status Get_Status() const;

    inline float Get_Now_Angle() const;

    inline float Get_Now_Omega() const;

    inline float Get_Now_Torque() const;

    inline Enum_Motor_STW_Control_Method Get_Control_Method() const;

    inline float Get_Control_Angle() const;

    inline float Get_Control_Omega() const;

    inline float Get_Control_Torque() const;

    inline float Get_Target_Angle() const;

    inline float Get_Target_Omega() const;

    inline float Get_Target_Torque() const;

    inline float Get_K_P() const;

    inline float Get_K_D() const;

    inline void Set_Control_Method(const Enum_Motor_STW_Control_Method &__Control_Method);

    inline void Set_Control_Angle(const float &__Control_Angle);

    inline void Set_Control_Omega(const float &__Control_Omega);

    inline void Set_Control_Torque(const float &__Control_Torque);

    inline void Set_Target_Angle(const float &__Target_Angle);

    inline void Set_Target_Omega(const float &__Target_Omega);

    inline void Set_Target_Torque(const float &__Target_Torque);

    inline void Set_K_P(const float &__K_P);

    inline void Set_K_D(const float &__K_D);

    inline void Set_Feedforward_Omega(const float &__Feedforward_Omega);

    inline void Set_Feedforward_Torque(const float &__Feedforward_Torque);

    // 获取发送缓冲区指针 (调试用)
    inline const uint8_t* Get_Tx_Data() const;

    void CAN_RxCpltCallback();

    void CAN_Send_Enter() const;

    void CAN_Send_Exit() const;

    void CAN_Send_Save_Zero() const;

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_Send_PeriodElapsedCallback();

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    // 初始化相关变量

    // 绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    // 电机ID, 即CAN发送/接收ID
    uint8_t Motor_ID;
    // 最大位置, rad
    float P_Max;
    // 最大速度, rad/s
    float V_Max;
    // 最大扭矩, Nm
    float T_Max;

    // 内部变量

    // 当前时刻的电机接收flag
    uint32_t Flag = 0;
    // 前一时刻的电机接收flag
    uint32_t Pre_Flag = 0;
    // 发送缓冲区
    uint8_t Tx_Data[8];

    // 读变量

    // 电机状态
    Enum_Motor_STW_Status Motor_STW_Status = Motor_STW_Status_DISABLE;
    // 电机对外接口信息
    Struct_Motor_STW_Rx_Data Rx_Data;

    // 写变量

    // 电机控制方式
    Enum_Motor_STW_Control_Method Control_Method = Motor_STW_Control_Method_MIT;

    // MIT模式直接控制量
    // 控制角度, rad
    float Control_Angle = 0.0f;
    // 控制角速度, rad/s
    float Control_Omega = 0.0f;
    // 控制扭矩, Nm
    float Control_Torque = 0.0f;
    // K_P, 0~500, MIT模式有效
    float K_P = 0.0f;
    // K_D, 0~5, MIT模式有效
    float K_D = 0.0f;

    // PID闭环控制目标量
    // 目标角度
    float Target_Angle = 0.0f;
    // 目标速度, rad/s
    float Target_Omega = 0.0f;
    // 目标扭矩, Nm
    float Target_Torque = 0.0f;
    // 前馈速度, rad/s
    float Feedforward_Omega = 0.0f;
    // 前馈扭矩, Nm
    float Feedforward_Torque = 0.0f;

    // 内部函数

    void Data_Process();

    void PID_Calculate();

    void Output();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取电机状态
 *
 * @return Enum_Motor_STW_Status 电机状态
 */
inline Enum_Motor_STW_Status Class_Motor_STW::Get_Status() const
{
    return (Motor_STW_Status);
}

/**
 * @brief 获取当前角度
 *
 * @return float 当前角度, rad
 */
inline float Class_Motor_STW::Get_Now_Angle() const
{
    return (Rx_Data.Now_Angle);
}

/**
 * @brief 获取当前角速度
 *
 * @return float 当前角速度, rad/s
 */
inline float Class_Motor_STW::Get_Now_Omega() const
{
    return (Rx_Data.Now_Omega);
}

/**
 * @brief 获取当前扭矩
 *
 * @return float 当前扭矩, Nm
 */
inline float Class_Motor_STW::Get_Now_Torque() const
{
    return (Rx_Data.Now_Torque);
}

/**
 * @brief 获取电机控制方式
 *
 * @return Enum_Motor_STW_Control_Method 电机控制方式
 */
inline Enum_Motor_STW_Control_Method Class_Motor_STW::Get_Control_Method() const
{
    return (Control_Method);
}

/**
 * @brief 获取MIT模式控制角度
 *
 * @return float 控制角度, rad
 */
inline float Class_Motor_STW::Get_Control_Angle() const
{
    return (Control_Angle);
}

/**
 * @brief 获取MIT模式控制角速度
 *
 * @return float 控制角速度, rad/s
 */
inline float Class_Motor_STW::Get_Control_Omega() const
{
    return (Control_Omega);
}

/**
 * @brief 获取MIT模式控制扭矩
 *
 * @return float 控制扭矩, Nm
 */
inline float Class_Motor_STW::Get_Control_Torque() const
{
    return (Control_Torque);
}

/**
 * @brief 获取PID闭环目标角度
 *
 * @return float 目标角度
 */
inline float Class_Motor_STW::Get_Target_Angle() const
{
    return (Target_Angle);
}

/**
 * @brief 获取PID闭环目标角速度
 *
 * @return float 目标角速度, rad/s
 */
inline float Class_Motor_STW::Get_Target_Omega() const
{
    return (Target_Omega);
}

/**
 * @brief 获取PID闭环目标扭矩
 *
 * @return float 目标扭矩, Nm
 */
inline float Class_Motor_STW::Get_Target_Torque() const
{
    return (Target_Torque);
}

/**
 * @brief 获取K_P, 0~500, MIT模式有效
 *
 * @return float K_P
 */
inline float Class_Motor_STW::Get_K_P() const
{
    return (K_P);
}

/**
 * @brief 获取K_D, 0~5, MIT模式有效
 *
 * @return float K_D
 */
inline float Class_Motor_STW::Get_K_D() const
{
    return (K_D);
}

/**
 * @brief 设定电机控制方式
 *
 * @param __Control_Method 电机控制方式
 */
inline void Class_Motor_STW::Set_Control_Method(const Enum_Motor_STW_Control_Method &__Control_Method)
{
    Control_Method = __Control_Method;
}

/**
 * @brief 设定MIT模式控制角度, rad
 *
 * @param __Control_Angle 控制角度, rad
 */
inline void Class_Motor_STW::Set_Control_Angle(const float &__Control_Angle)
{
    Control_Angle = __Control_Angle;
}

/**
 * @brief 设定MIT模式控制角速度, rad/s
 *
 * @param __Control_Omega 控制角速度, rad/s
 */
inline void Class_Motor_STW::Set_Control_Omega(const float &__Control_Omega)
{
    Control_Omega = __Control_Omega;
}

/**
 * @brief 设定MIT模式控制扭矩, Nm
 *
 * @param __Control_Torque 控制扭矩, Nm
 */
inline void Class_Motor_STW::Set_Control_Torque(const float &__Control_Torque)
{
    Control_Torque = __Control_Torque;
}

/**
 * @brief 设定PID闭环目标角度
 *
 * @param __Target_Angle 目标角度
 */
inline void Class_Motor_STW::Set_Target_Angle(const float &__Target_Angle)
{
    Target_Angle = __Target_Angle;
}

/**
 * @brief 设定PID闭环目标角速度, rad/s
 *
 * @param __Target_Omega 目标角速度, rad/s
 */
inline void Class_Motor_STW::Set_Target_Omega(const float &__Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定PID闭环目标扭矩, Nm
 *
 * @param __Target_Torque 目标扭矩, Nm
 */
inline void Class_Motor_STW::Set_Target_Torque(const float &__Target_Torque)
{
    Target_Torque = __Target_Torque;
}

/**
 * @brief 设定K_P, 0~500, MIT模式有效
 *
 * @param __K_P K_P
 */
inline void Class_Motor_STW::Set_K_P(const float &__K_P)
{
    K_P = __K_P;
}

/**
 * @brief 设定K_D, 0~5, MIT模式有效
 *
 * @param __K_D K_D
 */
inline void Class_Motor_STW::Set_K_D(const float &__K_D)
{
    K_D = __K_D;
}

/**
 * @brief 设定前馈速度, rad/s
 *
 * @param __Feedforward_Omega 前馈速度, rad/s
 */
inline void Class_Motor_STW::Set_Feedforward_Omega(const float &__Feedforward_Omega)
{
    Feedforward_Omega = __Feedforward_Omega;
}

/**
 * @brief 设定前馈扭矩, Nm
 *
 * @param __Feedforward_Torque 前馈扭矩, Nm
 */
inline void Class_Motor_STW::Set_Feedforward_Torque(const float &__Feedforward_Torque)
{
    Feedforward_Torque = __Feedforward_Torque;
}

/**
 * @brief 获取发送缓冲区 (调试用, 可查看实际发送字节)
 *
 * @return const uint8_t* 发送缓冲区指针, 8字节
 */
inline const uint8_t* Class_Motor_STW::Get_Tx_Data() const
{
    return (Tx_Data);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

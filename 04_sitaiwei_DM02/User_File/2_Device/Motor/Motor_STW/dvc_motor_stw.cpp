/**
 * @file dvc_motor_stw.cpp
 * @author xylm
 * @brief 伺泰威8115-36电机MIT协议驱动
 * @version 0.1
 * @date 2026-03-30 0.1 新增
 *
 * @copyright USTC-RoboWalker (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_motor_stw.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 伺泰威电机使能命令 (MIT协议进入电机控制模式)
uint8_t STW_Motor_CAN_Message_Enter[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
// 伺泰威电机失能命令 (MIT协议退出电机控制模式)
uint8_t STW_Motor_CAN_Message_Exit[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
// 伺泰威电机保存零点命令
uint8_t STW_Motor_CAN_Message_Save_Zero[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};

/* Private function declarations ---------------------------------------------*/

/**
 * @brief 浮点数转换为整数 (MIT协议编码)
 *
 * @param x 浮点数输入
 * @param x_min 浮点数最小值
 * @param x_max 浮点数最大值
 * @param bits 整数位数
 * @return uint16_t 转换后的整数
 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    // 限幅
    if (x < x_min) x = x_min;
    else if (x > x_max) x = x_max;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief 整数转换为浮点数 (MIT协议解码)
 *
 * @param x_int 整数输入
 * @param x_min 浮点数最小值
 * @param x_max 浮点数最大值
 * @param bits 整数位数
 * @return float 转换后的浮点数
 */
static float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 电机初始化
 *
 * @param hcan 绑定的CAN总线
 * @param __Motor_ID 电机ID, 即CAN帧ID, 默认0x01
 * @param __Control_Method 电机控制方式, 默认MIT模式
 * @param __P_Max 最大位置, 与电机参数PMAX保持一致, 默认12.5 rad
 * @param __V_Max 最大速度, 与电机参数VMAX保持一致, 默认45.0 rad/s
 * @param __T_Max 最大扭矩, 与电机参数TMAX保持一致, 默认54.0 Nm
 */
void Class_Motor_STW::Init(const FDCAN_HandleTypeDef *hcan, const uint8_t &__Motor_ID,
                            const Enum_Motor_STW_Control_Method &__Control_Method,
                            const float &__P_Max, const float &__V_Max,
                            const float &__T_Max)
{
    if (hcan->Instance == FDCAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == FDCAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }
    else if (hcan->Instance == FDCAN3)
    {
        CAN_Manage_Object = &CAN3_Manage_Object;
    }

    Motor_ID = __Motor_ID;
    Control_Method = __Control_Method;
    P_Max = __P_Max;
    V_Max = __V_Max;
    T_Max = __T_Max;
}

/**
 * @brief CAN通信接收回调函数
 *
 */
void Class_Motor_STW::CAN_RxCpltCallback()
{
    // 滑动窗口, 判断电机是否在线
    Flag += 1;

    Data_Process();
}

/**
 * @brief 发送使能电机命令 (进入电机控制模式)
 *
 */
void Class_Motor_STW::CAN_Send_Enter() const
{
    CAN_Transmit_Data(CAN_Manage_Object->CAN_Handler, Motor_ID, STW_Motor_CAN_Message_Enter, 8);
}

/**
 * @brief 发送失能电机命令 (退出电机控制模式)
 *
 */
void Class_Motor_STW::CAN_Send_Exit() const
{
    CAN_Transmit_Data(CAN_Manage_Object->CAN_Handler, Motor_ID, STW_Motor_CAN_Message_Exit, 8);
}

/**
 * @brief 发送保存当前位置为零点命令
 *
 */
void Class_Motor_STW::CAN_Send_Save_Zero() const
{
    CAN_Transmit_Data(CAN_Manage_Object->CAN_Handler, Motor_ID, STW_Motor_CAN_Message_Save_Zero, 8);
}

/**
 * @brief TIM定时器中断定期检测电机是否存活, 检测周期100ms
 *
 */
void Class_Motor_STW::TIM_100ms_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否接收过电机数据
    if (Flag == Pre_Flag)
    {
        // 电机断开连接
        Motor_STW_Status = Motor_STW_Status_DISABLE;
        PID_Angle.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
    }
    else
    {
        // 电机保持连接
        Motor_STW_Status = Motor_STW_Status_ENABLE;
    }

    Pre_Flag = Flag;

    // 电机掉线, 尝试重新使能
    if (Motor_STW_Status == Motor_STW_Status_DISABLE)
    {
        CAN_Send_Enter();
    }
}

/**
 * @brief TIM定时器中断发送回调函数 (MIT模式直接发送)
 *        适用于直接使用MIT模式控制时调用
 *
 */
void Class_Motor_STW::TIM_Send_PeriodElapsedCallback()
{
    // 无论电机状态如何, 都发送控制帧
    // MIT协议下, 使能后必须持续发送控制帧电机才会响应
    // 如果电机未使能, 控制帧会被电机忽略, 不会有副作用
    Output();
}

/**
 * @brief TIM定时器中断计算回调函数 (PID闭环模式)
 *        适用于使用PID角度/速度/扭矩闭环控制时调用
 *
 */
void Class_Motor_STW::TIM_Calculate_PeriodElapsedCallback()
{
    PID_Calculate();

    // 无论电机状态如何, 都发送控制帧
    Output();

    // 清除前馈量
    Feedforward_Omega = 0.0f;
    Feedforward_Torque = 0.0f;
}

/**
 * @brief 数据处理过程 (解析MIT协议接收帧)
 *
 * 接收帧格式 (8字节):
 *   Data[0]:    Motor ID
 *   Data[1~2]:  位置 (16bit)
 *   Data[3]:    速度[11:4]
 *   Data[4]:    速度[3:0] | 扭矩[11:8]
 *   Data[5]:    扭矩[7:0]
 *
 */
void Class_Motor_STW::Data_Process()
{
    uint8_t *rx_buf = CAN_Manage_Object->Rx_Buffer;

    // 检查Motor ID是否匹配
    uint8_t rx_motor_id = rx_buf[0];
    if (rx_motor_id != Motor_ID)
    {
        return;
    }

    // 解析位置 (16bit)
    uint16_t pos_raw = ((uint16_t)rx_buf[1] << 8) | rx_buf[2];
    // 解析速度 (12bit)
    uint16_t vel_raw = ((uint16_t)rx_buf[3] << 4) | (rx_buf[4] >> 4);
    // 解析扭矩 (12bit)
    uint16_t torque_raw = ((uint16_t)(rx_buf[4] & 0x0f) << 8) | rx_buf[5];

    // 转换为物理量
    float raw_angle = uint_to_float(pos_raw, -P_Max, P_Max, 16);
    float raw_omega = uint_to_float(vel_raw, -V_Max, V_Max, 12);
    float raw_torque = uint_to_float(torque_raw, -T_Max, T_Max, 12);

    // 对反馈数据做一阶低通滤波, 平滑12bit量化噪声
    // 反馈滤波系数稍大一些(0.3), 保证跟踪性能
    const float rx_alpha = 0.3f;
    if (Flag <= 1)
    {
        // 首帧直接赋值
        Rx_Data.Now_Angle = raw_angle;
        Rx_Data.Now_Omega = raw_omega;
        Rx_Data.Now_Torque = raw_torque;
    }
    else
    {
        Rx_Data.Now_Angle = rx_alpha * raw_angle + (1.0f - rx_alpha) * Rx_Data.Now_Angle;
        Rx_Data.Now_Omega = rx_alpha * raw_omega + (1.0f - rx_alpha) * Rx_Data.Now_Omega;
        Rx_Data.Now_Torque = rx_alpha * raw_torque + (1.0f - rx_alpha) * Rx_Data.Now_Torque;
    }
}

/**
 * @brief PID闭环计算
 *
 */
void Class_Motor_STW::PID_Calculate()
{
    switch (Control_Method)
    {
    case (Motor_STW_Control_Method_MIT):
    {
        // MIT模式: 不需要PID, 直接使用Control_Angle/Omega/Torque/K_P/K_D
        break;
    }
    case (Motor_STW_Control_Method_TORQUE):
    {
        // 直接力矩模式: MIT帧中 K_P=0, K_D=0, position=0, velocity=0, 仅发送扭矩
        Control_Angle = 0.0f;
        Control_Omega = 0.0f;
        K_P = 0.0f;
        K_D = 0.0f;
        Control_Torque = Target_Torque + Feedforward_Torque;
        break;
    }
    case (Motor_STW_Control_Method_OMEGA):
    {
        // 速度闭环: PID_Omega 输出扭矩
        PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
        PID_Omega.Set_Now(Rx_Data.Now_Omega);
        PID_Omega.TIM_Calculate_PeriodElapsedCallback();

        // MIT帧中 K_P=0, position=0, K_D=0, velocity=0, 仅发送扭矩
        Control_Angle = 0.0f;
        Control_Omega = 0.0f;
        K_P = 0.0f;
        K_D = 0.0f;
        Control_Torque = PID_Omega.Get_Out() + Feedforward_Torque;
        break;
    }
    case (Motor_STW_Control_Method_ANGLE):
    {
        // 角度闭环: 串级PID
        PID_Angle.Set_Target(Target_Angle);
        PID_Angle.Set_Now(Rx_Data.Now_Angle);
        PID_Angle.TIM_Calculate_PeriodElapsedCallback();

        Target_Omega = PID_Angle.Get_Out();

        PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
        PID_Omega.Set_Now(Rx_Data.Now_Omega);
        PID_Omega.TIM_Calculate_PeriodElapsedCallback();

        // MIT帧中 K_P=0, position=0, K_D=0, velocity=0, 仅发送扭矩
        Control_Angle = 0.0f;
        Control_Omega = 0.0f;
        K_P = 0.0f;
        K_D = 0.0f;
        Control_Torque = PID_Omega.Get_Out() + Feedforward_Torque;
        break;
    }
    }
}

/**
 * @brief 电机数据输出到CAN总线 (MIT协议发送帧)
 *
 * 发送帧格式 (8字节):
 *   Data[0~1]: 位置 (16bit, 映射到 -P_MAX ~ P_MAX)
 *   Data[2]:   速度[11:4]
 *   Data[3]:   速度[3:0] | Kp[11:8]
 *   Data[4]:   Kp[7:0]
 *   Data[5]:   Kd[11:4]
 *   Data[6]:   Kd[3:0] | 扭矩[11:8]
 *   Data[7]:   扭矩[7:0]
 *
 */
void Class_Motor_STW::Output()
{
    // 限幅
    Basic_Math_Constrain(&Control_Angle, -P_Max, P_Max);
    Basic_Math_Constrain(&Control_Omega, -V_Max, V_Max);
    Basic_Math_Constrain(&Control_Torque, -T_Max, T_Max);
    Basic_Math_Constrain(&K_P, STW_KP_MIN, STW_KP_MAX);
    Basic_Math_Constrain(&K_D, STW_KD_MIN, STW_KD_MAX);

    // 直接编码发送, 不做发送端低通滤波
    // MIT模式下电机侧闭环: τ = Kp*(p_des-p) + Kd*(v_des-v) + τ_ff
    // 对常数目标值做LPF会导致启动阶段缓慢爬升, 编码整数逐帧跳变, 反而引入抖动
    uint16_t p_int = float_to_uint(Control_Angle, -P_Max, P_Max, 16);
    uint16_t v_int = float_to_uint(Control_Omega, -V_Max, V_Max, 12);
    uint16_t kp_int = float_to_uint(K_P, STW_KP_MIN, STW_KP_MAX, 12);
    uint16_t kd_int = float_to_uint(K_D, STW_KD_MIN, STW_KD_MAX, 12);
    uint16_t t_int = float_to_uint(Control_Torque, -T_Max, T_Max, 12);

    // 打包MIT协议发送帧
    Tx_Data[0] = (p_int >> 8) & 0xff;
    Tx_Data[1] = p_int & 0xff;
    Tx_Data[2] = (v_int >> 4) & 0xff;
    Tx_Data[3] = ((v_int & 0x0f) << 4) | ((kp_int >> 8) & 0x0f);
    Tx_Data[4] = kp_int & 0xff;
    Tx_Data[5] = (kd_int >> 4) & 0xff;
    Tx_Data[6] = ((kd_int & 0x0f) << 4) | ((t_int >> 8) & 0x0f);
    Tx_Data[7] = t_int & 0xff;

    // 发送CAN帧
    CAN_Transmit_Data(CAN_Manage_Object->CAN_Handler, Motor_ID, Tx_Data, 8);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

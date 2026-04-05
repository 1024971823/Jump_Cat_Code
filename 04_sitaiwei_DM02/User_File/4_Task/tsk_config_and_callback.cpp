/**
 * @file tsk_config_and_callback.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-01-17 1.1 调试到机器人层
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"

#include "dvc_motor_dji.h"
#include "bsp_bmi088.h"
#include "dvc_vofa.h"
#include "bsp_w25q64jv.h"
#include "bsp_ws2812.h"
#include "bsp_buzzer.h"
#include "bsp_power.h"
#include "bsp_key.h"
#include "alg_filter_kalman.h"
#include "alg_matrix.h"
#include "drv_wdg.h"
#include "sys_timestamp.h"
#include "dvc_serialplot.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 串口绘图
Class_Vofa_USB Vofa_USB;
char Vofa_Variable_Assignment_List[][VOFA_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] = {"q00", "q11", "r00", "r11",};

// LED灯
int32_t red = 0;
int32_t green = 12;
int32_t blue = 12;
bool red_minus_flag = false;
bool green_minus_flag = false;
bool blue_minus_flag = true;

// 大疆电机3508
Class_Motor_DJI_GM6020 motor;
// Kalman滤波器
Class_Filter_Kalman filter_kalman;
// 相关矩阵
Class_Matrix_f32<2, 2> A;
Class_Matrix_f32<2, 1> B;
Class_Matrix_f32<2, 2> H;
Class_Matrix_f32<2, 2> Q;
Class_Matrix_f32<2, 2> R;
Class_Matrix_f32<2, 2> P;

// 全局初始化完成标志位
bool init_finished = false;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief USB虚拟串口接收回调函数
 *
 * @param Buffer 接收缓冲区
 * @param Length 接收数据长度
 */
void Serial_USB_Call_Back(uint8_t *Buffer, uint16_t Length)
{
    Vofa_USB.USB_RxCallback(Buffer, Length);
    int32_t index = Vofa_USB.Get_Variable_Index();
    switch (index)
    {
    case (0):
    {
        filter_kalman.Matrix_Q[0][0] = Vofa_USB.Get_Variable_Value();
        break;
    }
    case (1):
    {
        filter_kalman.Matrix_Q[1][1] = Vofa_USB.Get_Variable_Value();
        break;
    }
    case (2):
    {
        filter_kalman.Matrix_R[0][0] = Vofa_USB.Get_Variable_Value();
        break;
    }
    case (3):
    {
        filter_kalman.Matrix_R[1][1] = Vofa_USB.Get_Variable_Value();
        break;
    }
    default:
    {
        break;
    }
    }
}

/**
 * @brief SPI2任务回调函数
 *
 */
void SPI2_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length)
{
    if (SPI2_Manage_Object.Activate_GPIOx == BMI088_ACCEL__SPI_CS_GPIO_Port && SPI2_Manage_Object.Activate_GPIO_Pin == BMI088_ACCEL__SPI_CS_Pin || SPI2_Manage_Object.Activate_GPIOx == BMI088_GYRO__SPI_CS_GPIO_Port && SPI2_Manage_Object.Activate_GPIO_Pin == BMI088_GYRO__SPI_CS_Pin)
    {
        BSP_BMI088.SPI_RxCpltCallback();
    }
}

/**
 * @brief CAN1回调函数
 *
 *
 */
void CAN1_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    switch (Header.Identifier)
    {
    case (0x206):
    {
        motor.CAN_RxCpltCallback();

        break;
    }
    }
}

/**
 * @brief OSPI2轮询回调函数
 *
 */
void OSPI2_Polling_Callback()
{
    BSP_W25Q64JV.OSPI_StatusMatchCallback();
}

/**
 * @brief OSPI2接收回调函数
 *
 */
void OSPI2_Rx_Callback(uint8_t *Buffer)
{
    BSP_W25Q64JV.OSPI_RxCallback();
}

/**
 *@brief OSPI2发送回调函数
 */
void OSPI2_Tx_Callback(uint8_t *Buffer)
{
    BSP_W25Q64JV.OSPI_TxCallback();
}

/**
 * @brief 每3600s调用一次
 *
 */
void Task3600s_Callback()
{
    SYS_Timestamp.TIM_3600s_PeriodElapsedCallback();
}

/**
 * @brief 每1s调用一次
 *
 */
void Task1s_Callback()
{
}

/**
 * @brief 每1ms调用一次
 *
 */
void Task1ms_Callback()
{

    // static int16_t mod100 = 0;
    // mod100++;
    // if (mod100 == 100)
    // {
    //     mod100 = 0;

    //     // 喂狗
    //     Motor_4310.TIM_100ms_Alive_PeriodElapsedCallback();
    // }
    // static int counter = 0;
    // counter++;
    // Motor_4310.Set_K_P(5.0f);
    // Motor_4310.Set_K_D(1.0f);
    // Motor_4310.Set_Control_Angle(PI * sinf(0.5f * counter / 100.0f));
    // //前馈加速度是角度的一阶导数
    // Motor_4310.Set_Feedforward_Omega(PI * 0.5f * cosf(0.5f * counter / 100.0f));
    // //前馈力矩是角度的二阶导数乘以转动惯量（经过测试发现转动惯量为0.003kg）
    // Motor_4310.Set_Feedforward_Torque(PI * 0.25f * -sinf(0.5f * counter / 100.0f) * 0.003f);

    // float target_angle = Motor_4310.Get_Control_Angle();
    // float now_omega = Motor_4310.Get_Now_Omega();
    // float target_omega = Motor_4310.Get_Control_Omega();
    // float now_angle = Motor_4310.Get_Now_Angle();
    // float target_torque = Motor_4310.Get_Control_Torque();
    // float now_torque = Motor_4310.Get_Now_Torque();
    // Serialplot_USB.Set_Data(6, &target_angle, &now_angle, &target_omega, &now_omega, &target_torque, &now_torque);


    static int mod10 = 0;
    mod10++;
    if (mod10 == 10)
    {
        mod10 = 0;

        if (red >= 18)
        {
            red_minus_flag = true;
        }
        else if (red == 0)
        {
            red_minus_flag = false;
        }
        if (green >= 18)
        {
            green_minus_flag = true;
        }
        else if (green == 0)
        {
            green_minus_flag = false;
        }
        if (blue >= 18)
        {
            blue_minus_flag = true;
        }
        else if (blue == 0)
        {
            blue_minus_flag = false;
        }

        if (red_minus_flag)
        {
            red--;
        }
        else
        {
            red++;
        }
        if (green_minus_flag)
        {
            green--;
        }
        else
        {
            green++;
        }
        if (blue_minus_flag)
        {
            blue--;
        }
        else
        {
            blue++;
        }

        BSP_WS2812.Set_RGB(red, green, blue);
        // BSP_WS2812.Set_RGB(0, 0, 0);

        // 发送实例
        BSP_WS2812.TIM_10ms_Write_PeriodElapsedCallback();
    }

    BSP_Buzzer.Set_Sound(0.0f, 0.0f);

    BSP_Key.TIM_1ms_Process_PeriodElapsedCallback();
    static int mod50 = 0;
    mod50++;
    if (mod50 == 50)
    {
        mod50 = 0;

        // 处理按键状态
        BSP_Key.TIM_50ms_Read_PeriodElapsedCallback();
    }

    static int mod100 = 0;
    mod100++;
    if (mod100 == 100)
    {
        mod100 = 0;

        motor.TIM_100ms_Alive_PeriodElapsedCallback();
    }
    motor.Set_Target_Angle(1.0f * PI);
    motor.TIM_Calculate_PeriodElapsedCallback();

    static int mod128 = 0;
    mod128++;
    if (mod128 == 128)
    {
        mod128 = 0;

        BSP_BMI088.TIM_128ms_Calculate_PeriodElapsedCallback();
    }

    filter_kalman.Vector_Z[0][0] = motor.Get_Now_Angle();
    filter_kalman.Vector_Z[1][0] = motor.Get_Now_Omega();
    filter_kalman.TIM_Predict_PeriodElapsedCallback();
    filter_kalman.TIM_Update_PeriodElapsedCallback();

    float yaw = BSP_BMI088.Get_Euler_Angle()[0][0] / BASIC_MATH_DEG_TO_RAD;
    float pitch = BSP_BMI088.Get_Euler_Angle()[1][0] / BASIC_MATH_DEG_TO_RAD;
    float roll = BSP_BMI088.Get_Euler_Angle()[2][0] / BASIC_MATH_DEG_TO_RAD;
    float q0 = BSP_BMI088.Get_Quaternion()[0];
    float q1 = BSP_BMI088.Get_Quaternion()[1];
    float q2 = BSP_BMI088.Get_Quaternion()[2];
    float q3 = BSP_BMI088.Get_Quaternion()[3];
    float temperature = BSP_BMI088.BMI088_Accel.Get_Now_Temperature();
    float calculating_time = BSP_BMI088.Get_Calculating_Time();
    float loss = BSP_BMI088.Get_Accel_Chi_Square_Loss();
    float origin_accel_x = BSP_BMI088.Get_Original_Accel()[0][0];
    float origin_accel_y = BSP_BMI088.Get_Original_Accel()[1][0];
    float origin_accel_z = BSP_BMI088.Get_Original_Accel()[2][0];
    float origin_gyro_x = BSP_BMI088.Get_Original_Gyro()[0][0];
    float origin_gyro_y = BSP_BMI088.Get_Original_Gyro()[1][0];
    float origin_gyro_z = BSP_BMI088.Get_Original_Gyro()[2][0];
    float now_time = SYS_Timestamp.Get_Now_Microsecond() / 1000000.0f;
    float accel_x = BSP_BMI088.Get_Accel()[0][0];
    float accel_y = BSP_BMI088.Get_Accel()[1][0];
    float accel_z = BSP_BMI088.Get_Accel()[2][0];
    float gyro_x = BSP_BMI088.Get_Gyro()[0][0];
    float gyro_y = BSP_BMI088.Get_Gyro()[1][0];
    float gyro_z = BSP_BMI088.Get_Gyro()[2][0];
    float rotation_matrix_r00 = BSP_BMI088.Get_Rotation_Matrix()[0][0];
    float rotation_matrix_r01 = BSP_BMI088.Get_Rotation_Matrix()[0][1];
    float rotation_matrix_r02 = BSP_BMI088.Get_Rotation_Matrix()[0][2];
    float rotation_matrix_r10 = BSP_BMI088.Get_Rotation_Matrix()[1][0];
    float rotation_matrix_r11 = BSP_BMI088.Get_Rotation_Matrix()[1][1];
    float rotation_matrix_r12 = BSP_BMI088.Get_Rotation_Matrix()[1][2];
    float rotation_matrix_r20 = BSP_BMI088.Get_Rotation_Matrix()[2][0];
    float rotation_matrix_r21 = BSP_BMI088.Get_Rotation_Matrix()[2][1];
    float rotation_matrix_r22 = BSP_BMI088.Get_Rotation_Matrix()[2][2];
    float motor_target_angle = motor.Get_Target_Angle();
    float motor_now_angle = motor.Get_Now_Angle();
    float motor_target_omega = motor.Get_Target_Omega();
    float motor_now_omega = motor.Get_Now_Omega();
    float motor_target_torque = motor.Get_Target_Torque();
    float motor_now_torque = motor.Get_Now_Torque();
    float filter_omega = filter_kalman.Vector_X[1][0];
    float float_red = static_cast<float>(red);
    float float_green = static_cast<float>(green);
    float float_blue = static_cast<float>(blue);

    // 串口绘图
    // IMU常规显示
    Vofa_USB.Set_Data(23, &origin_accel_x, &origin_accel_y, &origin_accel_z, &origin_gyro_x, &origin_gyro_y, &origin_gyro_z, &q0, &q1, &q2, &q3, &yaw, &pitch, &roll, &temperature, &accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z, &loss, &calculating_time, &now_time);
    // Vofa_USB.Set_Data(7, &motor_target_angle, &motor_now_angle, &motor_target_omega, &motor_now_omega, &motor_target_torque, &motor_now_torque, &filter_omega);
    Vofa_USB.TIM_1ms_Write_PeriodElapsedCallback();

    TIM_1ms_CAN_PeriodElapsedCallback();
    // 喂狗
    TIM_1ms_IWDG_PeriodElapsedCallback();
}

/**
 * @brief 每125us调用一次
 *
 */
void Task125us_Callback()
{
    BSP_BMI088.TIM_125us_Calculate_PeriodElapsedCallback();
}

/**
 * @brief 每10us调用一次
 *
 */
void Task10us_Callback()
{
    BSP_BMI088.TIM_10us_Calculate_PeriodElapsedCallback();
}

/**
 * @brief 初始化任务
 *
 */
void Task_Init()
{
    SYS_Timestamp.Init(&htim5);
    // 串口绘图的USB
    USB_Init(Serial_USB_Call_Back);
    // 陀螺仪的SPI
    SPI_Init(&hspi2, SPI2_Callback);
    // WS2812的SPI
    SPI_Init(&hspi6, nullptr);
    // 电机的CAN
    CAN_Init(&hfdcan1, CAN1_Callback);
    // 电源的ADC
    ADC_Init(&hadc1, 1);
    // flash的OSPI
    OSPI_Init(&hospi2, OSPI2_Polling_Callback, OSPI2_Rx_Callback, OSPI2_Tx_Callback);

    // 定时器中断初始化
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim8);

    Vofa_USB.Init(4, reinterpret_cast<const char **>(Vofa_Variable_Assignment_List));

    BSP_WS2812.Init(0, 0, 0);

    BSP_Buzzer.Init();

    BSP_Power.Init();

    BSP_Key.Init();

    BSP_BMI088.Init();

    motor.PID_Angle.Init(12.0f, 0.0f, 0.0f, 0.0f, 10.0f, 10.0f);
    motor.PID_Omega.Init(0.03f, 5.0f, 0.0f, 0.0f, 0.2f, 0.2f);
    motor.Init(&hfdcan1, Motor_DJI_ID_0x206, Motor_DJI_Control_Method_ANGLE, 0, PI / 6);
    A[0][0] = 1.0f;
    A[0][1] = 0.001f;
    A[1][0] = 0.0f;
    A[1][1] = 1.0f;
    B[0][0] = 0.0f;
    B[1][0] = 0.0f;
    H[0][0] = 1.0f;
    H[0][1] = 0.0f;
    H[1][0] = 0.0f;
    H[1][1] = 1.0f;

    // 调参侠
    Q[0][0] = 0.001f;
    Q[0][1] = 0.0f;
    Q[1][0] = 0.0f;
    Q[1][1] = 0.1f;
    R[0][0] = 0.001f;
    R[0][1] = 0.0f;
    R[1][0] = 0.0f;
    R[1][1] = 1.0f;
    P[0][0] = 1.0f;
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 1.0f;
    filter_kalman.Init(A, B, H, Q, R, P);

    BSP_W25Q64JV.Init();

    // 标记初始化完成
    init_finished = true;
}

/**
 * @brief 前台循环任务
 *
 */
void Task_Loop()
{
    Namespace_SYS_Timestamp::Delay_Millisecond(1);
}

/**
 * @brief GPIO中断回调函数
 *
 * @param GPIO_Pin 中断引脚
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (!init_finished)
    {
        return;
    }

    if (GPIO_Pin == BMI088_ACCEL__INTERRUPT_Pin || GPIO_Pin == BMI088_GYRO__INTERRUPT_Pin)
    {
        BSP_BMI088.EXTI_Flag_Callback(GPIO_Pin);
    }
}

/**
 * @brief 定时器中断回调函数
 *
 * @param htim
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (!init_finished)
    {
        return;
    }

    // 选择回调函数
    if (htim->Instance == TIM4)
    {
        Task10us_Callback();
    }
    else if (htim->Instance == TIM5)
    {
        Task3600s_Callback();
    }
    else if (htim->Instance == TIM6)
    {
        Task1s_Callback();
    }
    else if (htim->Instance == TIM7)
    {
        Task1ms_Callback();
    }
    else if (htim->Instance == TIM8)
    {
        Task125us_Callback();
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

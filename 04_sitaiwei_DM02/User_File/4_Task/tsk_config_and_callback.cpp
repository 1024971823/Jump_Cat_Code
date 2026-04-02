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
#include "dvc_motor_stw.h"
#include "i6x.h"

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
// 伺泰威8115-36电机 (MIT协议), 四电机数组
// 索引: [0]=左前(0x01), [1]=左后(0x02), [2]=右前(0x03), [3]=右后(0x04)
Class_Motor_STW motor_stw[4];
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

// VOFA+ 四电机监控通道定义: 每个电机 6 个量
enum Enum_Vofa_STW_Channel
{
    Vofa_STW_Channel_Target_Omega = 0,
    Vofa_STW_Channel_Now_Omega,
    Vofa_STW_Channel_Control_Torque,
    Vofa_STW_Channel_Now_Torque,
    Vofa_STW_Channel_Now_Angle,
    Vofa_STW_Channel_Status,
    Vofa_STW_Channel_Num,
};

static float vofa_stw_monitor_data[4][Vofa_STW_Channel_Num] = {0.0f};

/* Private function declarations ---------------------------------------------*/

static void Update_Vofa_STW_Monitor_Data(void);
static void Send_Vofa_STW_Monitor_Data(void);

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
// 调试用: 记录CAN帧信息
uint32_t debug_last_unknown_can_id = 0;
uint32_t debug_unknown_can_count = 0;

void CAN1_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    switch (Header.Identifier)
    {
    case (0x206):
    {
        motor.CAN_RxCpltCallback();

        break;
    }
    case (0x00):
    {
        // 伺泰威8115-36反馈帧 CAN ID=0x00, Data[0]携带电机ID
        // 每个电机对象内部在Data_Process()中自行过滤Motor_ID, 全部调用即可
        debug_unknown_can_count++;
        for (int i = 0; i < 4; i++)
        {
            motor_stw[i].CAN_RxCpltCallback();
        }
        break;
    }
    default:
    {
        debug_last_unknown_can_id = Header.Identifier;
        break;
    }
    }
}

/**
 * @brief OSPI2轮询回调函数
 *
 * @brief UART5 SBUS接收回调 (由drv_uart DMA空闲中断触发)
 * @note  接收机信号需反相后接入UART5_RX:
 *        - 有硬件反相器(74HC14等): 直接连接
 *        - 无反相器: 在CubeMX中为UART5开启 "RX pin active level inversion"
 */
static void UART5_SBUS_Callback(uint8_t *Buffer, uint16_t Length)
{
    if (Length == I6X_FRAME_LENGTH)
    {
        sbus_to_i6x(get_i6x_point(), Buffer);
    }
}

/**
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
 * @brief 更新 VOFA+ 四电机监控缓存
 *
 * 通道顺序:
 *   电机0: 目标速度 / 当前速度 / 控制扭矩 / 当前扭矩 / 当前角度 / 使能状态
 *   电机1: 同上
 *   电机2: 同上
 *   电机3: 同上
 */
static void Update_Vofa_STW_Monitor_Data(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (int i = 0; i < 4; i++)
    {
        vofa_stw_monitor_data[i][Vofa_STW_Channel_Target_Omega] = motor_stw[i].Get_Target_Omega();
        vofa_stw_monitor_data[i][Vofa_STW_Channel_Now_Omega] = motor_stw[i].Get_Now_Omega();
        vofa_stw_monitor_data[i][Vofa_STW_Channel_Control_Torque] = motor_stw[i].Get_Control_Torque();
        vofa_stw_monitor_data[i][Vofa_STW_Channel_Now_Torque] = motor_stw[i].Get_Now_Torque();
        vofa_stw_monitor_data[i][Vofa_STW_Channel_Now_Angle] = motor_stw[i].Get_Now_Angle();
        vofa_stw_monitor_data[i][Vofa_STW_Channel_Status] = static_cast<float>(motor_stw[i].Get_Status());
    }
    __set_PRIMASK(primask);
}

/**
 * @brief 发送 VOFA+ 四电机监控数据
 *
 * @note Class_Vofa_USB 的发送接口虽然名为 TIM_1ms_Write_PeriodElapsedCallback，
 *       实际只是打包并发送一帧 justfloat 数据，不要求必须在 1ms 中断内调用。
 */
static void Send_Vofa_STW_Monitor_Data(void)
{
    Update_Vofa_STW_Monitor_Data();

    Vofa_USB.Set_Data(24,
                      &vofa_stw_monitor_data[0][0], &vofa_stw_monitor_data[0][1], &vofa_stw_monitor_data[0][2],
                      &vofa_stw_monitor_data[0][3], &vofa_stw_monitor_data[0][4], &vofa_stw_monitor_data[0][5],
                      &vofa_stw_monitor_data[1][0], &vofa_stw_monitor_data[1][1], &vofa_stw_monitor_data[1][2],
                      &vofa_stw_monitor_data[1][3], &vofa_stw_monitor_data[1][4], &vofa_stw_monitor_data[1][5],
                      &vofa_stw_monitor_data[2][0], &vofa_stw_monitor_data[2][1], &vofa_stw_monitor_data[2][2],
                      &vofa_stw_monitor_data[2][3], &vofa_stw_monitor_data[2][4], &vofa_stw_monitor_data[2][5],
                      &vofa_stw_monitor_data[3][0], &vofa_stw_monitor_data[3][1], &vofa_stw_monitor_data[3][2],
                      &vofa_stw_monitor_data[3][3], &vofa_stw_monitor_data[3][4], &vofa_stw_monitor_data[3][5]);
    Vofa_USB.TIM_1ms_Write_PeriodElapsedCallback();
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
        // STW电机存活检测已移至RTOS ctrl_task, 避免与ctrl_task并发发送CAN帧
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

    // STW四电机 VOFA+ 监控已迁移到 RTOS monitor_task, 避免在中断里直接占用 USB 发送

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

    // 四个8115-36电机初始化 (速度闭环模式)
    // CAN ID: 0x01=左前, 0x02=左后, 0x03=右前, 0x04=右后
    // PID_Omega 参数为保守初值, 需根据实际负载调整 Kp
    {
        const uint8_t motor_ids[4] = {0x01, 0x02, 0x03, 0x04};
        for (int i = 0; i < 4; i++)
        {
            motor_stw[i].PID_Omega.Init(0.5f, 0.0f, 0.0f, 0.0f, 5.0f, 18.0f);
            motor_stw[i].Init(&hfdcan1, motor_ids[i],
                              Motor_STW_Control_Method_OMEGA, 95.5f, 45.0f, 18.0f);
            motor_stw[i].CAN_Send_Enter();
        }
    }
    // UART5: SBUS接收机 (100kbaud 9E2 RX-only, 已在CubeMX中配置)
    UART_Init(&huart5, UART5_SBUS_Callback);
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
    // HAL时基(TIM13)必须始终递增, 否则初始化阶段可能卡死在依赖HAL tick的流程
    if (htim->Instance == TIM13)
    {
        HAL_IncTick();
        return;
    }

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

/* ============================================================
 * RTOS 任务实现
 * 由 freertos.c 的对应任务调用, 声明在 tsk_config_and_callback.h
 * ============================================================ */

/**
 * @brief RTOS 控制任务主循环 (ctrl_task, 1ms)
 *
 * 功能: SBUS遥控 → 差速混控 → 四轮速度指令 → CAN输出
 *
 * 遥控通道:
 *   ch[1] 右摇杆上下 = 前进/后退 (speed)
 *   ch[3] 左摇杆左右 = 左转/右转 (turn)
 *
 * 差速公式:
 *   left_omega  = speed + turn
 *   right_omega = speed - turn
 *
 * 电机方向:
 *   右侧电机物理安装方向与左侧相反, 因此 right_omega 取反
 *   若实测方向不对, 修改下方正负号或调换右侧电机CAN ID
 */
extern "C" void RTOS_Ctrl_Task_Loop(void)
{
    /* 使用 PRIMASK 短暂关全局中断来原子读取 16 字节 SBUS 数据结构
     * 避免在 UART DMA 回调写入过程中读到撕裂帧 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    i6x_ctrl_t rc = *get_i6x_point();
    __set_PRIMASK(primask);

    /* Failsafe: 遥控器断连 → 立即清零所有电机目标速度 */
    if (rc.failsafe || rc.frame_lost)
    {
        for (int i = 0; i < 4; i++)
        {
            motor_stw[i].Set_Target_Omega(0.0f);
            motor_stw[i].TIM_Calculate_PeriodElapsedCallback();
        }
        return;
    }

    /* 死区处理: 摇杆在中心区域输出为零 */
    const int16_t DEADBAND = 20;
    int16_t ch_speed = (rc.ch[1] > DEADBAND || rc.ch[1] < -DEADBAND) ? rc.ch[1] : 0;
    int16_t ch_turn  = (rc.ch[3] > DEADBAND || rc.ch[3] < -DEADBAND) ? rc.ch[3] : 0;

    /* 归一化: ch范围±660 → rad/s */
    const float MAX_OMEGA = STW_V_MAX;   /* rad/s, 放开到驱动配置允许的最大速度 */
    float speed = (float)ch_speed / 660.0f * MAX_OMEGA;
    float turn  = (float)ch_turn  / 660.0f * MAX_OMEGA;

    /* 差速混控 */
    float left_omega  = speed + turn;
    float right_omega = speed - turn;

    /* 限幅 */
    if (left_omega  >  MAX_OMEGA) left_omega  =  MAX_OMEGA;
    if (left_omega  < -MAX_OMEGA) left_omega  = -MAX_OMEGA;
    if (right_omega >  MAX_OMEGA) right_omega =  MAX_OMEGA;
    if (right_omega < -MAX_OMEGA) right_omega = -MAX_OMEGA;

    /* 设置四电机目标速度 */
    motor_stw[0].Set_Target_Omega( left_omega);   /* 左前 */
    motor_stw[1].Set_Target_Omega( left_omega);   /* 左后 */
    motor_stw[2].Set_Target_Omega(-right_omega);  /* 右前 (安装方向相反) */
    motor_stw[3].Set_Target_Omega(-right_omega);  /* 右后 (安装方向相反) */

    /* 执行PID计算并发送CAN帧 */
    for (int i = 0; i < 4; i++)
    {
        motor_stw[i].TIM_Calculate_PeriodElapsedCallback();
    }

    /* 电机存活检测: 每100次 ≈ 100ms 执行一次
     * 离线电机会自动重发使能帧, 全部在ctrl_task中操作避免ISR竞争 */
    static uint32_t alive_counter = 0U;
    if (++alive_counter >= 100U)
    {
        alive_counter = 0U;
        for (int i = 0; i < 4; i++)
        {
            motor_stw[i].TIM_100ms_Alive_PeriodElapsedCallback();
        }
    }
}

/**
 * @brief RTOS 遥控任务主循环 (remote_task, 20ms)
 *
 * 预留位置: 可在此添加基于拨杆的模式切换、急停逻辑等
 * 示例: if (i6x_switch_is_down(rc->s[0])) {  急停 
 */
extern "C" void RTOS_Remote_Task_Loop(void)
{       static int mod10 = 0;
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
    /* 预留: 通过 get_i6x_point()->s[x] 读取拨杆状态做模式控制 */
    (void)get_i6x_point();
}

/**
 * @brief RTOS 监控任务主循环 (monitor_task, 20ms)
 *
 * 功能: 通过 USB CDC 虚拟串口向 VOFA+ 持续输出四个 STW 电机的状态
 */
extern "C" void RTOS_Monitor_Task_Loop(void)
{
    Send_Vofa_STW_Monitor_Data();
}

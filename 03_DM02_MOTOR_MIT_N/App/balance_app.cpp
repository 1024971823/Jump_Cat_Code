#include "balance_app.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "balance_config.h"
#include "balance_motor_if.h"
#include "balance_imu_if.h"
#include "balance_observer.h"
#include "balance_controller.h"
#include "balance_tool.h"

#include "fdcan.h"
#include "dvc_motor_dm.h"
#include "dvc_motor_dji.h"
#include "drv_can.h"
#include "bsp_jy61p.h"

// =====================================================
// 全局机器人对象
// =====================================================
BalanceRobot g_balance_robot;

// =====================================================
// 电机对象
// 关节：达妙（CAN1）
// 轮子：DJI 3508 + C620（CAN2）
// =====================================================
static Class_Motor_DM_Normal g_motor_joint_0;
static Class_Motor_DM_Normal g_motor_joint_1;
static Class_Motor_DM_Normal g_motor_joint_2;
static Class_Motor_DM_Normal g_motor_joint_3;

static Class_Motor_DJI_C620 g_motor_wheel_0;
static Class_Motor_DJI_C620 g_motor_wheel_1;

// =====================================================
// 任务句柄
// =====================================================
static TaskHandle_t xBalanceControlTaskHandle = NULL;
static TaskHandle_t xBalanceMotorSendTaskHandle = NULL;
static TaskHandle_t xBalanceAliveTaskHandle = NULL;
static TaskHandle_t xBalancePrintTaskHandle = NULL;

// =====================================================
// 运行标志
// =====================================================
static bool g_balance_app_inited = false;
static bool g_balance_app_enabled = false;

// =====================================================
// 内部参数
// =====================================================
namespace
{
    // =========================
    // 轮子 DJI 参数
    // =========================
    static constexpr Enum_Motor_DJI_ID kWheelLeftId  = Motor_DJI_ID_0x201;
    static constexpr Enum_Motor_DJI_ID kWheelRightId = Motor_DJI_ID_0x202;

    // 3508 常见减速比 19:1
    static constexpr float kWheelGearRatio = 19.0f;

    static inline void ClearMotorCmd(BalanceMotorCmd* cmd)
    {
        if (cmd == nullptr)
        {
            return;
        }

        cmd->pos = 0.0f;
        cmd->vel = 0.0f;
        cmd->tor = 0.0f;
        cmd->kp = 0.0f;
        cmd->kd = 0.0f;
        cmd->enable = false;
    }

    static inline void ClearLegCmd(BalanceLegCmd* cmd)
    {
        if (cmd == nullptr)
        {
            return;
        }

        cmd->rod_f = 0.0f;
        cmd->rod_tp = 0.0f;
        cmd->joint_t[0] = 0.0f;
        cmd->joint_t[1] = 0.0f;
        cmd->wheel_t = 0.0f;
    }
}

// =====================================================
// 内部函数声明
// =====================================================
static void BalanceApp_InitRobot(void);
static void BalanceApp_InitImu(void);
static void BalanceApp_InitMotors(void);

static void vBalanceControlTask(void *pvParameters);
static void vBalanceMotorSendTask(void *pvParameters);
static void vBalanceAliveTask(void *pvParameters);
static void vBalancePrintTask(void *pvParameters);

// =====================================================
// CAN1 回调：关节达妙
// =====================================================
void CAN1_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    (void)Buffer;

    switch (Header.Identifier)
    {
    case (0x05):
        g_motor_joint_0.CAN_RxCpltCallback();
        break;
    case (0x04):
        g_motor_joint_1.CAN_RxCpltCallback();
        break;
    case (0x02):
        g_motor_joint_2.CAN_RxCpltCallback();
        break;
    case (0x03):
        g_motor_joint_3.CAN_RxCpltCallback();
        break;
    default:
        break;
    }
}

// =====================================================
// CAN2 回调：轮子 DJI
// =====================================================
void CAN2_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    (void)Buffer;

    switch (Header.Identifier)
    {
    case (0x201):
        g_motor_wheel_0.CAN_RxCpltCallback();
        break;
    case (0x202):
        g_motor_wheel_1.CAN_RxCpltCallback();
        break;
    default:
        break;
    }
}

// =====================================================
// 初始化
// =====================================================
static void BalanceApp_InitRobot(void)
{
    memset(&g_balance_robot, 0, sizeof(g_balance_robot));
    BalanceObserver_Init(&g_balance_robot);
    BalanceController_Init(&g_balance_robot);
}

static void BalanceApp_InitImu(void)
{
    BalanceImuIf_Init();
}

static void BalanceApp_InitMotors(void)
{
    // CAN1 给关节
    CAN_Init(&hfdcan1, CAN1_Callback);

    // CAN2 给轮子
    CAN_Init(&hfdcan2, CAN2_Callback);

    // =========================
    // 关节：达妙（CAN1）
    // =========================
    g_motor_joint_0.Init(&hfdcan1, 0x05, 0x05,
                         Motor_DM_Control_Method_NORMAL_MIT,
                         12.5f, 25.0f, 10.0f, 10.261194f);
    g_motor_joint_1.Init(&hfdcan1, 0x04, 0x04,
                         Motor_DM_Control_Method_NORMAL_MIT,
                         12.5f, 25.0f, 10.0f, 10.261194f);
    g_motor_joint_2.Init(&hfdcan1, 0x02, 0x02,
                         Motor_DM_Control_Method_NORMAL_MIT,
                         12.5f, 25.0f, 10.0f, 10.261194f);
    g_motor_joint_3.Init(&hfdcan1, 0x03, 0x03,
                         Motor_DM_Control_Method_NORMAL_MIT,
                         12.5f, 25.0f, 10.0f, 10.261194f);

    // =========================
    // 轮子：DJI 3508 + C620（CAN2）
    // =========================
    g_motor_wheel_0.Init(&hfdcan2,
                         kWheelLeftId,
                         Motor_DJI_Control_Method_TORQUE,
                         kWheelGearRatio);

    g_motor_wheel_1.Init(&hfdcan2,
                         kWheelRightId,
                         Motor_DJI_Control_Method_TORQUE,
                         kWheelGearRatio);

    BalanceMotorIf_Init();

    // =========================
    // 注册关节
    // =========================
    BalanceMotorIf_RegisterJoint(BAL_JOINT_L_0, &g_motor_joint_1);
    BalanceMotorIf_RegisterJoint(BAL_JOINT_L_1, &g_motor_joint_0);
    BalanceMotorIf_RegisterJoint(BAL_JOINT_R_0, &g_motor_joint_2);
    BalanceMotorIf_RegisterJoint(BAL_JOINT_R_1, &g_motor_joint_3);

    // =========================
    // 注册轮子
    // =========================
    BalanceMotorIf_RegisterWheel(BAL_WHEEL_L, &g_motor_wheel_0);
    BalanceMotorIf_RegisterWheel(BAL_WHEEL_R, &g_motor_wheel_1);
}

void BalanceApp_Init(void)
{
    if (g_balance_app_inited)
    {
        return;
    }

    BalanceApp_InitRobot();
    BalanceApp_InitImu();
    BalanceApp_InitMotors();

    g_balance_app_inited = true;
}

// =====================================================
// 使能 / 失能
// =====================================================
void BalanceApp_Enable(void)
{
    if (!g_balance_app_inited)
    {
        BalanceApp_Init();
    }

    g_balance_app_enabled = true;
    g_balance_robot.enable = true;
    g_balance_robot.safe = false;

    BalanceMotorIf_SendEnterAll();
}

void BalanceApp_Disable(void)
{
    g_balance_app_enabled = false;
    g_balance_robot.enable = false;
    g_balance_robot.safe = true;

    BalanceController_Stop(&g_balance_robot);
    BalanceMotorIf_SendCommand(&g_balance_robot);
    BalanceMotorIf_SendExitAll();
}

// =====================================================
// 控制任务
// =====================================================
static void vBalanceControlTask(void *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(500));

    if (!g_balance_app_inited)
    {
        BalanceApp_Init();
    }

    BalanceApp_Enable();

    for (;;)
    {
        BalanceMotorIf_UpdateFeedback(&g_balance_robot);                        // 更新电机反馈
        BalanceImuIf_Update(&g_balance_robot.imu);                              // 更新IMU数据
        BalanceObserver_UpdateAll(&g_balance_robot);                            // 更新观察器，计算出身体状态和腿状态等

        if (g_balance_robot.enable && !g_balance_robot.safe)
        {
            BalanceController_SetRef(&g_balance_robot);                         // 设置控制器参考值
            BalanceController_LegLength(&g_balance_robot);                      // 腿长控制 -> rod_f
            BalanceController_LegAngle(&g_balance_robot);                       // 腿角控制 -> rod_tp
            BalanceController_Output(&g_balance_robot);                         // 输出最终电机命令
        }
        else
        {
            BalanceController_Stop(&g_balance_robot);
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// =====================================================
// 电机发送任务
// =====================================================
static void vBalanceMotorSendTask(void *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(600));

    for (;;)
    {
        BalanceMotorIf_SendCommand(&g_balance_robot);
        BalanceMotorIf_TxAllPeriodic();
        CAN_Transmit_Data(&hfdcan2, 0x200, CAN2_0x200_Tx_Data, FDCAN_DLC_BYTES_8);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// =====================================================
// 保活任务
// =====================================================
static void vBalanceAliveTask(void *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(1000));

    for (;;)
    {
        BalanceMotorIf_AliveAllPeriodic();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// =====================================================
// 打印任务
// =====================================================
static void vBalancePrintTask(void *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(1200));

    for (;;)
    {
        const float l0_l = g_balance_robot.leg[0].rod.l0;
        const float l0_r = g_balance_robot.leg[1].rod.l0;

        const float phi0_l = g_balance_robot.leg[0].rod.phi0;
        const float phi0_r = g_balance_robot.leg[1].rod.phi0;

        const float q0_l = g_balance_robot.leg[0].joint.phi1;
        const float q1_l = g_balance_robot.leg[0].joint.phi4;

        BalanceTool_PrintRaw("\r\n[balance]\r\n");

        BalanceTool_PrintFloat4Line("l0_l",
                                    l0_l,
                                    "l0_r",
                                    l0_r);

        BalanceTool_PrintFloat4Line("phi0_l",
                                    phi0_l,
                                    "phi0_r",
                                    phi0_r);

        BalanceTool_PrintFloat4Line("q0_l",
                                    q0_l,
                                    "q1_l",
                                    q1_l);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// =====================================================
// 创建任务
// =====================================================
void BalanceApp_Task_Create(void)
{
    if (!g_balance_app_inited)
    {
        BalanceApp_Init();
    }

    BaseType_t ret;

    ret = xTaskCreate(vBalanceControlTask,
                      "vBalanceControlTask",
                      512,
                      NULL,
                      3,
                      &xBalanceControlTaskHandle);
    if (ret != pdPASS)
    {
        while (1) {}
    }

    ret = xTaskCreate(vBalanceMotorSendTask,
                      "vBalanceMotorSendTask",
                      384,
                      NULL,
                      4,
                      &xBalanceMotorSendTaskHandle);
    if (ret != pdPASS)
    {
        while (1) {}
    }

    ret = xTaskCreate(vBalanceAliveTask,
                      "vBalanceAliveTask",
                      256,
                      NULL,
                      2,
                      &xBalanceAliveTaskHandle);
    if (ret != pdPASS)
    {
        while (1) {}
    }

    ret = xTaskCreate(vBalancePrintTask,
                      "vBalancePrintTask",
                      512,
                      NULL,
                      1,
                      &xBalancePrintTaskHandle);
    if (ret != pdPASS)
    {
        while (1) {}
    }
}
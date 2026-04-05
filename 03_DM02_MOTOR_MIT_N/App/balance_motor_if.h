#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "balance_types.h"

// 关节：达妙
#include "dvc_motor_dm.h"

// 轮子：大疆 3508 + C620
#include "dvc_motor_dji.h"

#ifdef __cplusplus
extern "C" {
#endif

// 类型别名：关节与轮子分开
typedef Class_Motor_DM_Normal BalanceDmMotor;
typedef Class_Motor_DJI_C620  BalanceDjiWheelMotor;

// 关节电机绑定
typedef struct BalanceJointMotorBinding
{
    BalanceDmMotor* motor;
    bool registered;
} BalanceJointMotorBinding;

// 轮子电机绑定
typedef struct BalanceWheelMotorBinding
{
    BalanceDjiWheelMotor* motor;
    bool registered;
} BalanceWheelMotorBinding;

// 初始化接口层
void BalanceMotorIf_Init(void);

// 注册关节 / 轮子电机
bool BalanceMotorIf_RegisterJoint(uint8_t index, BalanceDmMotor* motor);
bool BalanceMotorIf_RegisterWheel(uint8_t index, BalanceDjiWheelMotor* motor);

// 更新反馈到 BalanceRobot
void BalanceMotorIf_UpdateFeedback(BalanceRobot* robot);

// 根据 robot->xxx_motor_cmd 下发命令
void BalanceMotorIf_SendCommand(const BalanceRobot* robot);

// 立即清零所有输出
void BalanceMotorIf_DisableAll(void);

// 进入 / 退出
void BalanceMotorIf_SendEnterAll(void);
void BalanceMotorIf_SendExitAll(void);

// 周期发送 / 在线检测
void BalanceMotorIf_TxAllPeriodic(void);
void BalanceMotorIf_AliveAllPeriodic(void);

#ifdef __cplusplus
}
#endif
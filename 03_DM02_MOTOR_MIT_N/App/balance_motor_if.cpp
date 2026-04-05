#include "balance_motor_if.h"

#include <stddef.h>

namespace
{
    BalanceJointMotorBinding g_joint_binding[BALANCE_JOINT_NUM];
    BalanceWheelMotorBinding g_wheel_binding[BALANCE_WHEEL_NUM];

    static inline float BalanceClamp(float x, float min_v, float max_v)
    {
        if (x < min_v) return min_v;
        if (x > max_v) return max_v;
        return x;
    }

    // =========================
    // 关节达妙
    // =========================

    static inline bool BalanceJointMotorOnline(const BalanceDmMotor* motor)
    {
        if (motor == nullptr)
        {
            return false;
        }

        return (motor->Get_Status() == Motor_DM_Status_ENABLE);
    }

    static inline void BalanceFillFdbFromDm(BalanceMotorFdb* out,
                                            const BalanceDmMotor* motor)
    {
        if (out == nullptr)
        {
            return;
        }

        if (motor == nullptr)
        {
            out->pos = 0.0f;
            out->vel = 0.0f;
            out->tor = 0.0f;
            out->online = false;
            return;
        }

        out->pos = motor->Get_Now_Angle();
        out->vel = motor->Get_Now_Omega();
        out->tor = motor->Get_Now_Torque();
        out->online = BalanceJointMotorOnline(motor);
    }

    static inline void BalanceApplyCmdToDm(BalanceDmMotor* motor,
                                           const BalanceMotorCmd* cmd,
                                           float torque_limit,
                                           float kp_limit,
                                           float kd_limit)
    {
        if ((motor == nullptr) || (cmd == nullptr))
        {
            return;
        }

        if (!cmd->enable)
        {
            motor->Set_Control_Angle(0.0f);
            motor->Set_Control_Omega(0.0f);
            motor->Set_Control_Torque(0.0f);
            motor->Set_K_P(0.0f);
            motor->Set_K_D(0.0f);
            return;
        }

        const float pos = cmd->pos;
        const float vel = cmd->vel;
        const float tor = BalanceClamp(cmd->tor, -torque_limit, torque_limit);
        const float kp  = BalanceClamp(cmd->kp,  0.0f, kp_limit);
        const float kd  = BalanceClamp(cmd->kd,  0.0f, kd_limit);

        motor->Set_Control_Angle(pos);
        motor->Set_Control_Omega(vel);
        motor->Set_Control_Torque(tor);
        motor->Set_K_P(kp);
        motor->Set_K_D(kd);
    }

    // =========================
    // 轮子大疆 C620
    // 只使用力矩模式
    // =========================

    static inline bool BalanceWheelMotorOnline(const BalanceDjiWheelMotor* motor)
    {
        if (motor == nullptr)
        {
            return false;
        }

        return (motor->Get_Status() == Motor_DJI_Status_ENABLE);
    }

    static inline void BalanceFillFdbFromDji(BalanceMotorFdb* out,
                                             const BalanceDjiWheelMotor* motor)
    {
        if (out == nullptr)
        {
            return;
        }

        if (motor == nullptr)
        {
            out->pos = 0.0f;
            out->vel = 0.0f;
            out->tor = 0.0f;
            out->online = false;
            return;
        }

        out->pos = motor->Get_Now_Angle();
        out->vel = motor->Get_Now_Omega();
        out->tor = motor->Get_Now_Torque();
        out->online = BalanceWheelMotorOnline(motor);
    }

    static inline void BalanceApplyCmdToDji(BalanceDjiWheelMotor* motor,
                                            const BalanceMotorCmd* cmd,
                                            float torque_limit)
    {
        if ((motor == nullptr) || (cmd == nullptr))
        {
            return;
        }

        const float tor = cmd->enable
                            ? BalanceClamp(cmd->tor, -torque_limit, torque_limit)
                            : 0.0f;

        // 轮子只走力矩模式
        // 这里不使用 pos / vel / kp / kd
        motor->Set_Target_Torque(tor);
    }

    static inline void BalanceClearFdb(BalanceMotorFdb* fdb)
    {
        if (fdb == nullptr)
        {
            return;
        }

        fdb->pos = 0.0f;
        fdb->vel = 0.0f;
        fdb->tor = 0.0f;
        fdb->online = false;
    }
}

void BalanceMotorIf_Init(void)
{
    for (uint8_t i = 0; i < BALANCE_JOINT_NUM; ++i)
    {
        g_joint_binding[i].motor = nullptr;
        g_joint_binding[i].registered = false;
    }

    for (uint8_t i = 0; i < BALANCE_WHEEL_NUM; ++i)
    {
        g_wheel_binding[i].motor = nullptr;
        g_wheel_binding[i].registered = false;
    }
}

bool BalanceMotorIf_RegisterJoint(uint8_t index, BalanceDmMotor* motor)
{
    if (index >= BALANCE_JOINT_NUM)
    {
        return false;
    }

    if (motor == nullptr)
    {
        return false;
    }

    g_joint_binding[index].motor = motor;
    g_joint_binding[index].registered = true;
    return true;
}

bool BalanceMotorIf_RegisterWheel(uint8_t index, BalanceDjiWheelMotor* motor)
{
    if (index >= BALANCE_WHEEL_NUM)
    {
        return false;
    }

    if (motor == nullptr)
    {
        return false;
    }

    g_wheel_binding[index].motor = motor;
    g_wheel_binding[index].registered = true;
    return true;
}

void BalanceMotorIf_UpdateFeedback(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    // 关节：达妙
    for (uint8_t i = 0; i < BALANCE_JOINT_NUM; ++i)
    {
        if (g_joint_binding[i].registered && g_joint_binding[i].motor != nullptr)
        {
            BalanceFillFdbFromDm(&robot->joint_motor_fdb[i], g_joint_binding[i].motor);
        }
        else
        {
            BalanceClearFdb(&robot->joint_motor_fdb[i]);
        }
    }

    // 轮子：大疆
    for (uint8_t i = 0; i < BALANCE_WHEEL_NUM; ++i)
    {
        if (g_wheel_binding[i].registered && g_wheel_binding[i].motor != nullptr)
        {
            BalanceFillFdbFromDji(&robot->wheel_motor_fdb[i], g_wheel_binding[i].motor);
        }
        else
        {
            BalanceClearFdb(&robot->wheel_motor_fdb[i]);
        }
    }
}

void BalanceMotorIf_SendCommand(const BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    // 关节：达妙，保留 MIT 风格接口
    for (uint8_t i = 0; i < BALANCE_JOINT_NUM; ++i)
    {
        if (!g_joint_binding[i].registered || g_joint_binding[i].motor == nullptr)
        {
            continue;
        }

        BalanceApplyCmdToDm(g_joint_binding[i].motor,
                            &robot->joint_motor_cmd[i],
                            BALANCE_DEFAULT_JOINT_TORQUE_LIMIT,
                            BALANCE_DEFAULT_JOINT_KP_LIMIT,
                            BALANCE_DEFAULT_JOINT_KD_LIMIT);
    }

    // 轮子：大疆，只发力矩
    for (uint8_t i = 0; i < BALANCE_WHEEL_NUM; ++i)
    {
        if (!g_wheel_binding[i].registered || g_wheel_binding[i].motor == nullptr)
        {
            continue;
        }

        BalanceApplyCmdToDji(g_wheel_binding[i].motor,
                             &robot->wheel_motor_cmd[i],
                             BALANCE_DEFAULT_WHEEL_TORQUE_LIMIT);
    }
}

void BalanceMotorIf_DisableAll(void)
{
    // 关节：清零 MIT 控制量
    for (uint8_t i = 0; i < BALANCE_JOINT_NUM; ++i)
    {
        if (!g_joint_binding[i].registered || g_joint_binding[i].motor == nullptr)
        {
            continue;
        }

        g_joint_binding[i].motor->Set_Control_Angle(0.0f);
        g_joint_binding[i].motor->Set_Control_Omega(0.0f);
        g_joint_binding[i].motor->Set_Control_Torque(0.0f);
        g_joint_binding[i].motor->Set_K_P(0.0f);
        g_joint_binding[i].motor->Set_K_D(0.0f);
    }

    // 轮子：只清零目标力矩
    for (uint8_t i = 0; i < BALANCE_WHEEL_NUM; ++i)
    {
        if (!g_wheel_binding[i].registered || g_wheel_binding[i].motor == nullptr)
        {
            continue;
        }

        g_wheel_binding[i].motor->Set_Target_Torque(0.0f);
    }
}

void BalanceMotorIf_SendExitAll(void)
{
    // 关节：继续保留 DM 的 enter / exit
    for (uint8_t i = 0; i < BALANCE_JOINT_NUM; ++i)
    {
        if (g_joint_binding[i].registered && g_joint_binding[i].motor != nullptr)
        {
            g_joint_binding[i].motor->CAN_Send_Exit();
        }
    }

    // 轮子：没有必要发 DM 风格 Exit，直接清零力矩更稳
    for (uint8_t i = 0; i < BALANCE_WHEEL_NUM; ++i)
    {
        if (g_wheel_binding[i].registered && g_wheel_binding[i].motor != nullptr)
        {
            g_wheel_binding[i].motor->Set_Target_Torque(0.0f);
        }
    }
}

void BalanceMotorIf_SendEnterAll(void)
{
    // 关节：继续保留 DM 的 enter
    for (uint8_t i = 0; i < BALANCE_JOINT_NUM; ++i)
    {
        if (g_joint_binding[i].registered && g_joint_binding[i].motor != nullptr)
        {
            g_joint_binding[i].motor->CAN_Send_Enter();
        }
    }

    // 轮子：DJI 这里不需要做 DM 风格 enter
    // 保持空操作即可
}

void BalanceMotorIf_TxAllPeriodic(void)
{
    for (uint8_t i = 0; i < BALANCE_JOINT_NUM; ++i)
    {
        if (g_joint_binding[i].registered && g_joint_binding[i].motor != nullptr)
        {
            g_joint_binding[i].motor->TIM_Send_PeriodElapsedCallback();
        }
    }

    for (uint8_t i = 0; i < BALANCE_WHEEL_NUM; ++i)
    {
        if (g_wheel_binding[i].registered && g_wheel_binding[i].motor != nullptr)
        {
            g_wheel_binding[i].motor->TIM_Calculate_PeriodElapsedCallback();
        }
    }
}

void BalanceMotorIf_AliveAllPeriodic(void)
{
    for (uint8_t i = 0; i < BALANCE_JOINT_NUM; ++i)
    {
        if (g_joint_binding[i].registered && g_joint_binding[i].motor != nullptr)
        {
            g_joint_binding[i].motor->TIM_100ms_Alive_PeriodElapsedCallback();
        }
    }

    for (uint8_t i = 0; i < BALANCE_WHEEL_NUM; ++i)
    {
        if (g_wheel_binding[i].registered && g_wheel_binding[i].motor != nullptr)
        {
            g_wheel_binding[i].motor->TIM_100ms_Alive_PeriodElapsedCallback();
        }
    }
}
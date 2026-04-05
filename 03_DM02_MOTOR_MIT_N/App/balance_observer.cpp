#include "balance_observer.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "balance_config.h"
#include "balance_kinematics.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923f
#endif

namespace
{
    // 参考姿态标定后的关节角映射：
    // phi = sign * (continuous - cont_ref) + phi_ref
    static inline float BalanceApplyJointMount(float continuous,
                                               float sign,
                                               float cont_ref,
                                               float phi_ref)
    {
        return sign * (continuous - cont_ref) + phi_ref;
    }

    static inline float BalanceWrapPi(float angle)
    {
        while (angle > M_PI)  angle -= 2.0f * M_PI;
        while (angle < -M_PI) angle += 2.0f * M_PI;
        return angle;
    }

    static inline float BalanceClamp(float x, float min_v, float max_v)
    {
        if (x < min_v) return min_v;
        if (x > max_v) return max_v;
        return x;
    }

    static inline void BalanceClearLeg(BalanceLegState* leg)
    {
        if (leg == nullptr)
        {
            return;
        }

        memset(leg, 0, sizeof(BalanceLegState));
    }

    static inline void BalanceAngleUnwrapReset(BalanceAngleUnwrap* unwrap)
    {
        if (unwrap == nullptr)
        {
            return;
        }

        unwrap->initialized = false;    // 表示这个解包器是否已经接收过第一帧数据
        unwrap->raw_last = 0.0f;        // 上一次读取到的原始角度
        unwrap->continuous = 0.0f;      // 解包后的连续角度
    }

    static inline float BalanceAngleUnwrapUpdate(BalanceAngleUnwrap* unwrap, float raw_now)
    {
        if (unwrap == nullptr)
        {
            return raw_now;
        }

        // 达妙当前位置反馈范围近似 [-12.5, 12.5]
        constexpr float kHalfRange = 12.5f;
        constexpr float kFullRange = 25.0f;

        if (!unwrap->initialized)
        {
            unwrap->initialized = true;
            unwrap->raw_last = raw_now;
            unwrap->continuous = raw_now;
            return unwrap->continuous;
        }

        float delta = raw_now - unwrap->raw_last;

        if (delta > kHalfRange)
        {
            delta -= kFullRange;
        }
        else if (delta < -kHalfRange)
        {
            delta += kFullRange;
        }

        unwrap->continuous += delta;
        unwrap->raw_last = raw_now;

        return unwrap->continuous;
    }

    // =====================================================
    // 启动姿态自动捕获：
    // 每个关节第一次拿到 continuous 时，记录：
    // cont_ref = continuous_init
    // phi_ref  = sign * continuous_init
    //
    // 这样后续：
    // phi = sign * (continuous - cont_ref) + phi_ref
    // 就表示：
    // 当前模型角 = 电机相对启动姿态的变化角 + 启动时模型角
    // =====================================================
    static bool  g_joint_start_ref_inited[BALANCE_JOINT_NUM] = {false, false, false, false};
    static float g_joint_cont_ref[BALANCE_JOINT_NUM] = {0.0f, 0.0f, 0.0f, 0.0f};
    static float g_joint_phi_ref[BALANCE_JOINT_NUM]  = {0.0f, 0.0f, 0.0f, 0.0f};

    static inline void BalanceJointStartRefReset(void)
    {
        for (int i = 0; i < BALANCE_JOINT_NUM; ++i)
        {
            g_joint_start_ref_inited[i] = false;
            g_joint_cont_ref[i] = 0.0f;
            g_joint_phi_ref[i] = 0.0f;
        }
    }

    static inline void BalanceJointStartRefTryInit(int joint_index,
                                                   float continuous,
                                                   float sign)
    {
        if (joint_index < 0 || joint_index >= BALANCE_JOINT_NUM)
        {
            return;
        }

        if (g_joint_start_ref_inited[joint_index])
        {
            return;
        }

        g_joint_cont_ref[joint_index] = continuous;
        g_joint_phi_ref[joint_index] = sign * continuous;
        g_joint_start_ref_inited[joint_index] = true;                           // 初始化完毕
    }

    // =====================================================
    // 轮子位移参考零点
    // 用于把轮子绝对角度转换成相对位移
    // =====================================================
    static bool g_wheel_pos_ref_inited = false;
    static float g_wheel_pos_l_ref = 0.0f;
    static float g_wheel_pos_r_ref = 0.0f;

    static inline void BalanceWheelPosRefReset(void)
    {
        g_wheel_pos_ref_inited = false;
        g_wheel_pos_l_ref = 0.0f;
        g_wheel_pos_r_ref = 0.0f;
    }
}

void BalanceObserver_Init(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    robot->dt = BALANCE_CTRL_DT;                                // 设置控制周期
    robot->enable = false;                                      // 未使能
    robot->safe = true;                                         // 安全状态

    robot->body.x = 0.0f;                                       // 机体位移
    robot->body.x_dot = 0.0f;                                   // 机体速度
    robot->body.x_acc = 0.0f;                                   // 加速度
    robot->body.x_dot_obv = 0.0f;                               // 观测速
    robot->body.x_acc_obv = 0.0f;                               // 观测加速度

    // 重置轮子位移参考零点
    BalanceWheelPosRefReset();

    // 重置启动姿态参考
    BalanceJointStartRefReset();

    // 初始化每个电机的角度解包器
    for (int i = 0; i < BALANCE_JOINT_NUM; ++i)
    {
        BalanceAngleUnwrapReset(&robot->joint_angle_unwrap[i]);
    }

    for (int i = 0; i < BALANCE_LEG_NUM; ++i)
    {
        BalanceClearLeg(&robot->leg[i]);
        memset(&robot->leg_state[i], 0, sizeof(BalanceLegLqrState));
        robot->ref.target_leg_length[i] = BALANCE_DEFAULT_LEG_LEN_STAND;
    }

    robot->ref.target_vx = 0.0f;
    robot->ref.target_wz = 0.0f;
    robot->ref.target_roll = 0.0f;
}

// 从 IMU 读取机身姿态和角速度，构造 body 状态
void BalanceObserver_UpdateBody(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    // 直接从统一 IMU 结构取数据
    robot->body.roll = robot->imu.roll;
    robot->body.pitch = robot->imu.pitch;
    robot->body.yaw = robot->imu.yaw;

    robot->body.roll_dot = robot->imu.roll_dot;
    robot->body.pitch_dot = robot->imu.pitch_dot;
    robot->body.yaw_dot = robot->imu.yaw_dot;

    // 平衡主平面暂用 pitch
    robot->body.phi = BalanceWrapPi(robot->body.pitch);
    robot->body.phi_dot = robot->body.pitch_dot;

    // 线加速度简单取 IMU x 方向
    robot->body.x_acc = robot->imu.ax;
}

void BalanceObserver_UpdateLeg(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    // ----- 左腿 -----
    {
        BalanceLegState& leg = robot->leg[0];

        const float joint0_raw = robot->joint_motor_fdb[BAL_JOINT_L_0].pos;
        const float joint1_raw = robot->joint_motor_fdb[BAL_JOINT_L_1].pos;

        const float joint0_cont =
            BalanceAngleUnwrapUpdate(&robot->joint_angle_unwrap[BAL_JOINT_L_0], joint0_raw);
        const float joint1_cont =
            BalanceAngleUnwrapUpdate(&robot->joint_angle_unwrap[BAL_JOINT_L_1], joint1_raw);

        // 启动姿态自动捕获参考
        BalanceJointStartRefTryInit(BAL_JOINT_L_0, joint0_cont, BALANCE_JOINT_L0_SIGN);
        BalanceJointStartRefTryInit(BAL_JOINT_L_1, joint1_cont, BALANCE_JOINT_L1_SIGN);

        // 启动姿态映射：
        // 当前模型角 = 电机相对启动姿态变化角 + 启动时模型角
        leg.joint.phi1 = BalanceApplyJointMount(joint0_cont,
                                                BALANCE_JOINT_L0_SIGN,
                                                g_joint_cont_ref[BAL_JOINT_L_0],
                                                g_joint_phi_ref[BAL_JOINT_L_0]);
        leg.joint.phi4 = BalanceApplyJointMount(joint1_cont,
                                                BALANCE_JOINT_L1_SIGN,
                                                g_joint_cont_ref[BAL_JOINT_L_1],
                                                g_joint_phi_ref[BAL_JOINT_L_1]);

        // 速度、力矩也统一到同一个关节坐标系
        leg.joint.dphi1 = BALANCE_JOINT_L0_SIGN * robot->joint_motor_fdb[BAL_JOINT_L_0].vel;
        leg.joint.dphi4 = BALANCE_JOINT_L1_SIGN * robot->joint_motor_fdb[BAL_JOINT_L_1].vel;
        leg.joint.t1    = BALANCE_JOINT_L0_SIGN * robot->joint_motor_fdb[BAL_JOINT_L_0].tor;
        leg.joint.t2    = BALANCE_JOINT_L1_SIGN * robot->joint_motor_fdb[BAL_JOINT_L_1].tor;

        leg.wheel_vel = robot->wheel_motor_fdb[BAL_WHEEL_L].vel;

        float l0_phi0[2] = {0.0f, 0.0f};
        BalanceCalcL0Phi0(leg.joint.phi1, leg.joint.phi4, l0_phi0);             // 获得虚拟腿长和虚拟腿角度

        leg.rod.l0 = l0_phi0[0];                                                // 虚拟腿长
        leg.rod.phi0 = l0_phi0[1];                                              // 虚拟腿角度
        leg.rod.theta = M_PI_2 - leg.rod.phi0 - robot->body.phi;                // 虚拟腿角度 - 机体仰角

        float J[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
        BalanceCalcJacobian(leg.joint.phi1, leg.joint.phi4, J);                 // 计算雅可比矩阵

        float d_l0_d_phi0[2] = {0.0f, 0.0f};
        BalanceCalcdL0dPhi0(J, leg.joint.dphi1, leg.joint.dphi4, d_l0_d_phi0);  // 获得虚拟腿长变化率和虚拟腿角速度

        leg.rod.dl0 = d_l0_d_phi0[0];                                           // 虚拟腿长变化率
        leg.rod.dphi0 = d_l0_d_phi0[1];                                         // 虚拟腿角速度
        leg.rod.dtheta = -leg.rod.dphi0 - robot->body.phi_dot;                  // 虚拟腿角速度 - 机体角速度

        // 第一版先固定不做离地判定
        leg.is_take_off = false;

        // 腿长限制
        leg.rod.l0 = BalanceClamp(leg.rod.l0,
                                  BALANCE_DEFAULT_LEG_LEN_MIN,
                                  BALANCE_DEFAULT_LEG_LEN_MAX);
    }

    // ----- 右腿 -----
    {
        BalanceLegState& leg = robot->leg[1];

        const float joint0_raw = robot->joint_motor_fdb[BAL_JOINT_R_0].pos;
        const float joint1_raw = robot->joint_motor_fdb[BAL_JOINT_R_1].pos;

        const float joint0_cont =
            BalanceAngleUnwrapUpdate(&robot->joint_angle_unwrap[BAL_JOINT_R_0], joint0_raw);
        const float joint1_cont =
            BalanceAngleUnwrapUpdate(&robot->joint_angle_unwrap[BAL_JOINT_R_1], joint1_raw);

        // 启动姿态自动捕获参考
        BalanceJointStartRefTryInit(BAL_JOINT_R_0, joint0_cont, BALANCE_JOINT_R0_SIGN);
        BalanceJointStartRefTryInit(BAL_JOINT_R_1, joint1_cont, BALANCE_JOINT_R1_SIGN);

        // 启动姿态映射：
        // 当前模型角 = 电机相对启动姿态变化角 + 启动时模型角
        leg.joint.phi1 = BalanceApplyJointMount(joint0_cont,
                                                BALANCE_JOINT_R0_SIGN,
                                                g_joint_cont_ref[BAL_JOINT_R_0],
                                                g_joint_phi_ref[BAL_JOINT_R_0]);
        leg.joint.phi4 = BalanceApplyJointMount(joint1_cont,
                                                BALANCE_JOINT_R1_SIGN,
                                                g_joint_cont_ref[BAL_JOINT_R_1],
                                                g_joint_phi_ref[BAL_JOINT_R_1]);

        // 速度、力矩也统一到同一个关节坐标系
        leg.joint.dphi1 = BALANCE_JOINT_R0_SIGN * robot->joint_motor_fdb[BAL_JOINT_R_0].vel;
        leg.joint.dphi4 = BALANCE_JOINT_R1_SIGN * robot->joint_motor_fdb[BAL_JOINT_R_1].vel;
        leg.joint.t1    = BALANCE_JOINT_R0_SIGN * robot->joint_motor_fdb[BAL_JOINT_R_0].tor;
        leg.joint.t2    = BALANCE_JOINT_R1_SIGN * robot->joint_motor_fdb[BAL_JOINT_R_1].tor;

        leg.wheel_vel = robot->wheel_motor_fdb[BAL_WHEEL_R].vel;

        float l0_phi0[2] = {0.0f, 0.0f};
        BalanceCalcL0Phi0(leg.joint.phi1, leg.joint.phi4, l0_phi0);

        leg.rod.l0 = l0_phi0[0];
        leg.rod.phi0 = l0_phi0[1];
        leg.rod.theta = M_PI_2 - leg.rod.phi0 - robot->body.phi;

        float J[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
        BalanceCalcJacobian(leg.joint.phi1, leg.joint.phi4, J);

        float d_l0_d_phi0[2] = {0.0f, 0.0f};
        BalanceCalcdL0dPhi0(J, leg.joint.dphi1, leg.joint.dphi4, d_l0_d_phi0);

        leg.rod.dl0 = d_l0_d_phi0[0];
        leg.rod.dphi0 = d_l0_d_phi0[1];
        leg.rod.dtheta = -leg.rod.dphi0 - robot->body.phi_dot;

        // 第一版先固定不做离地判定
        leg.is_take_off = false;

        // 腿长限制
        leg.rod.l0 = BalanceClamp(leg.rod.l0,
                                  BALANCE_DEFAULT_LEG_LEN_MIN,
                                  BALANCE_DEFAULT_LEG_LEN_MAX);
    }
}

// 位移与速度观测：
// x_dot 继续用左右轮角速度平均 * 轮半径
// x 改为左右轮相对角位移平均 * 轮半径
void BalanceObserver_UpdateVelocity(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    const float wheel_radius = BALANCE_DEFAULT_WHEEL_RADIUS;                // 轮足半径

    // 当前左右轮连续角度 / 角速度
    const float pos_l = robot->wheel_motor_fdb[BAL_WHEEL_L].pos;            // 这里用绝对角度，后面再减去参考零点
    const float pos_r = robot->wheel_motor_fdb[BAL_WHEEL_R].pos;
    const float vel_l = robot->wheel_motor_fdb[BAL_WHEEL_L].vel;            // 这里直接用轮速，不需要参考零点
    const float vel_r = robot->wheel_motor_fdb[BAL_WHEEL_R].vel;

    const bool online_l = robot->wheel_motor_fdb[BAL_WHEEL_L].online;
    const bool online_r = robot->wheel_motor_fdb[BAL_WHEEL_R].online;

    // =========================
    // x_dot：继续由轮速得到
    // =========================
    const float speed = wheel_radius * (vel_l + vel_r) * 0.5f;              // 轮速平均 * 轮半径 = 车体前向速度

    robot->body.x_dot_obv = speed;                                          // 观测速度
    robot->body.x_acc_obv = robot->body.x_acc;
    robot->body.x_dot = robot->body.x_dot_obv;                              // 直接把观测速度赋给 x_dot，后续如果要做滤波或者其他处理，可以在观测速度的基础上改这里

    // =========================
    // x：改由轮角相对位移得到
    // 只有左右轮都在线时，才初始化参考零点
    // =========================
    if (!g_wheel_pos_ref_inited)                                            // 第一次进入
    {
        if (online_l && online_r)
        {
            g_wheel_pos_l_ref = pos_l;                                      // 以第一次进入时的轮子绝对角度作为参考零点
            g_wheel_pos_r_ref = pos_r;
            g_wheel_pos_ref_inited = true;
        }

        // 参考零点还没准备好时，先保持 x = 0
        robot->body.x = 0.0f;
        return;
    }

    const float delta_l = pos_l - g_wheel_pos_l_ref;                        // 轮子绝对角度 - 参考零点 = 轮子相对角位移
    const float delta_r = pos_r - g_wheel_pos_r_ref;

    robot->body.x = wheel_radius * (delta_l + delta_r) * 0.5f;              // 轮子相对角位移平均 * 轮半径 = 车体前向位移
}

void BalanceObserver_UpdateLqrState(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    for (int i = 0; i < BALANCE_LEG_NUM; ++i)
    {
        robot->leg_state[i].theta     = robot->leg[i].rod.theta;
        robot->leg_state[i].theta_dot = robot->leg[i].rod.dtheta;
        robot->leg_state[i].x         = robot->body.x;
        robot->leg_state[i].x_dot     = robot->body.x_dot_obv;
        robot->leg_state[i].phi       = robot->body.phi;
        robot->leg_state[i].phi_dot   = robot->body.phi_dot;
    }
}

void BalanceObserver_UpdateAll(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    BalanceObserver_UpdateBody(robot);
    BalanceObserver_UpdateLeg(robot);
    BalanceObserver_UpdateVelocity(robot);
    BalanceObserver_UpdateLqrState(robot);
}
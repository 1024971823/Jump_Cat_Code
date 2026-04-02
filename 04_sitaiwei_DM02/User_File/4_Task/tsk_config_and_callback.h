/**
 * @file tsk_config_and_callback.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-01-17 1.1 调试到机器人层
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

#ifndef TSK_CONFIG_AND_CALLBACK_H
#define TSK_CONFIG_AND_CALLBACK_H

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

void Task_Init();

void Task_Loop();

/** 由 freertos.c 的 ctrl_task 每 1ms 调用一次: 遥控→混控→四电机速度控制 */
void RTOS_Ctrl_Task_Loop(void);

/** 由 freertos.c 的 remote_task 每 20ms 调用一次: 遥控状态机/模式切换 */
void RTOS_Remote_Task_Loop(void);

/** 由 freertos.c 的 monitor_task 每 20ms 调用一次: USB VOFA+四电机状态输出 */
void RTOS_Monitor_Task_Loop(void);

#ifdef __cplusplus
};
#endif

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

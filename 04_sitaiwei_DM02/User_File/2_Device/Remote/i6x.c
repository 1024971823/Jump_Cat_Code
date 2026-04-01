/******************************************************************************
 * @file    i6x.c
 * @author  X.yu (原始), 本工程适配
 * @brief   FS-i6x遥控器搭配iA6B接收机SBUS数据解包
 * @note    原始代码bug修复: failsafe/frame_lost字段移至#if MAPPING_ENABLE块外,
 *          确保无论映射是否开启都能正确读取失控标志
 ******************************************************************************/
#include "i6x.h"
#include <math.h>

/* 三档拨杆归一化: 负值→-1, 零→0, 正值→+1 */
#define TO_STICK(v) (((v) < 0) - ((v) > 0))

/* 是否将原始通道值(-784~783)映射到(-660~660), 1=开启 */
#define MAPPING_ENABLE 1

/* 全局遥控器数据 */
static i6x_ctrl_t i6x_ctrl;

/**
 * @brief 将原始通道值映射到 -660~660
 * @param val 解包后的原始值(-784~783)
 * @return 映射后的值(-660~660)
 */
static int16_t map_to_660(const int16_t val)
{
    if (val >= 0)
        return (int16_t)floorf((660.0f / 783.0f) * (float)val + 0.5f);
    else
        return (int16_t)floorf((660.0f / 784.0f) * (float)val + 0.5f);
}

/**
 * @brief 将25字节SBUS原始帧解包为i6x_ctrl_t结构体
 * @param i6x_ctrl  输出数据结构体指针
 * @param sbus_data 25字节SBUS原始帧 (UART5 DMA接收缓冲区)
 */
void sbus_to_i6x(i6x_ctrl_t *i6x_ctrl, const uint8_t *sbus_data)
{
    /* 帧头0x0F, 帧尾0x00校验 */
    if (sbus_data[0] != 0x0F || sbus_data[24] != 0x00)
    {
        return;
    }

    /* 解包6个模拟通道 (每通道11bit) */
    i6x_ctrl->ch[0] = (int16_t)(((sbus_data[1]         | (sbus_data[2]  << 8))                              & 0x07FF) - 1024);
    i6x_ctrl->ch[1] = (int16_t)((((sbus_data[2]  >> 3)  | (sbus_data[3]  << 5))                             & 0x07FF) - 1024);
    i6x_ctrl->ch[2] = (int16_t)((((sbus_data[3]  >> 6)  | (sbus_data[4]  << 2) | (sbus_data[5]  << 10))    & 0x07FF) - 1024);
    i6x_ctrl->ch[3] = (int16_t)((((sbus_data[5]  >> 1)  | (sbus_data[6]  << 7))                             & 0x07FF) - 1024);
    i6x_ctrl->ch[4] = (int16_t)((((sbus_data[6]  >> 4)  | (sbus_data[7]  << 4))                             & 0x07FF) - 1024);
    i6x_ctrl->ch[5] = (int16_t)((((sbus_data[7]  >> 7)  | (sbus_data[8]  << 1) | (sbus_data[9]  << 9))     & 0x07FF) - 1024);

    /* 解包4个拨杆 (三档归一化为-1/0/+1) */
    i6x_ctrl->s[0]  = (int8_t)TO_STICK((((sbus_data[9]  >> 2)  | (sbus_data[10] << 6)) & 0x07FF) - 1024);
    i6x_ctrl->s[1]  = (int8_t)TO_STICK((((sbus_data[10] >> 5)  | (sbus_data[11] << 3)) & 0x07FF) - 1024);
    i6x_ctrl->s[2]  = (int8_t)TO_STICK(( (sbus_data[12]        | (sbus_data[13] << 8)) & 0x07FF) - 1024);
    i6x_ctrl->s[3]  = (int8_t)TO_STICK((((sbus_data[13] >> 3)  | (sbus_data[14] << 5)) & 0x07FF) - 1024);

    /* 失控/丢帧标志位: 必须在#if MAPPING_ENABLE块之外解析, 始终有效 */
    const uint8_t flag      = sbus_data[23];
    i6x_ctrl->frame_lost    = (flag >> 2) & 0x01;
    i6x_ctrl->failsafe      = (flag >> 3) & 0x01;

#if MAPPING_ENABLE
    /* 将通道值映射到 -660~660 以匹配原有DT7代码习惯 */
    for (int i = 0; i < 6; i++)
    {
        i6x_ctrl->ch[i] = map_to_660(i6x_ctrl->ch[i]);
    }
#endif
}

/**
 * @brief 获取全局遥控器数据结构体指针
 * @return i6x_ctrl_t* 遥控器数据指针
 */
i6x_ctrl_t *get_i6x_point(void)
{
    return &i6x_ctrl;
}

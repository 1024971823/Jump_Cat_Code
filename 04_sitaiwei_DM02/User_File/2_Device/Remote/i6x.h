/******************************************************************************
 * @file    i6x.h
 * @author  X.yu
 * @brief   FS-i6x遥控器搭配iA6B接收机SBUS数据解包头文件
 * @note    UART5已配置为100000baud 9E2 RX-only, 与SBUS协议完全匹配
 *          若使用硬件信号反相器(如74HC14)则无需额外配置;
 *          若无反相器则需在CubeMX中为UART5开启 "RX pin active level inversion"
 ******************************************************************************/
#ifndef I6X_H
#define I6X_H

#include <stdint.h>

/* 每帧固定25字节 */
#define I6X_FRAME_LENGTH 25u

/* 拨杆状态定义 */
#define I6X_SW_UP   ((int8_t) 1)
#define I6X_SW_MID  ((int8_t) 0)
#define I6X_SW_DOWN ((int8_t)-1)

#define i6x_switch_is_up(s)   ((s) == I6X_SW_UP)
#define i6x_switch_is_mid(s)  ((s) == I6X_SW_MID)
#define i6x_switch_is_down(s) ((s) == I6X_SW_DOWN)

/* 遥控器数据结构体 */
typedef struct
{
    int16_t ch[6];       /**< 6个模拟通道: ch[0]=右H, ch[1]=右V, ch[2]=左V(油门), ch[3]=左H, ch[4~5]=旋钮 */
    int8_t  s[4];        /**< 4个拨杆: s[0]~s[2]两档(-1/1), s[3]三档(-1/0/1) */
    uint8_t failsafe;    /**< 失控保护标志位: 1=失控 */
    uint8_t frame_lost;  /**< 丢帧标志位:     1=丢帧 */
} __attribute__((packed)) i6x_ctrl_t;

#ifdef __cplusplus
extern "C" {
#endif

void        sbus_to_i6x(i6x_ctrl_t *i6x_ctrl, const uint8_t *sbus_data);
i6x_ctrl_t *get_i6x_point(void);

#ifdef __cplusplus
}
#endif

#endif /* I6X_H */

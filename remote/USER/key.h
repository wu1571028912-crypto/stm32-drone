#ifndef __KEY_H
#define __KEY_H

#include "main.h"
#include <stdint.h>

/* Key_Scan10ms() 的固定调用周期 */
#define KEY_SCAN_PERIOD_MS           10U

/* 连续稳定 30 ms 才确认按键状态 */
#define KEY_DEBOUNCE_COUNT           3U

/* 长按时间 */
#define KEY1_CALIBRATE_TIME_MS       1000U
#define KEY3_EMERGENCY_TIME_MS       500U

#define KEY1_CALIBRATE_COUNT \
    (KEY1_CALIBRATE_TIME_MS / KEY_SCAN_PERIOD_MS)

#define KEY3_EMERGENCY_COUNT \
    (KEY3_EMERGENCY_TIME_MS / KEY_SCAN_PERIOD_MS)

/* 控制精度模式 */
typedef enum
{
    KEY_CONTROL_STANDARD = 0,  /* 标准模式 */
    KEY_CONTROL_FINE     = 1   /* 精细模式 */
} KeyControlMode_t;

/* 一次性按键事件 */
typedef enum
{
    KEY_EVENT_NONE              = 0x00,
    KEY_EVENT_CALIBRATE_REQUEST = 0x01,
    KEY_EVENT_MODE_CHANGED      = 0x02,
    KEY_EVENT_EMERGENCY_STOP    = 0x04
} KeyEvent_t;

/*
 * 初始化按键状态。
 * GPIO 初始化完成以后调用一次。
 */
void Key_Init(void);

/*
 * 每 10 ms 调用一次。
 * 函数内部不延时、不阻塞。
 */
void Key_Scan10ms(void);

/*
 * 获取并清除一次性事件。
 *
 * 返回值由 KeyEvent_t 的一个或多个标志组成。
 */
uint8_t Key_GetAndClearEvents(void);

/* KEY2：当前是否一直按住缓降键 */
uint8_t Key_IsDescentHeld(void);

/* KEY3：紧急停机是否已经锁存 */
uint8_t Key_IsEmergencyStopLatched(void);

/* KEY4：获取当前控制精度模式 */
KeyControlMode_t Key_GetControlMode(void);

#endif



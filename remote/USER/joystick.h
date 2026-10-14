#ifndef __JOYSTICK_H
#define __JOYSTICK_H

#include <stdint.h>

/*
 * ADC_Value[] 与原理图通道的对应关系。
 */
#define JOYSTICK_ADC_YAW_INDEX       0U
#define JOYSTICK_ADC_THR_INDEX       1U
#define JOYSTICK_ADC_PITCH_INDEX     2U
#define JOYSTICK_ADC_ROLL_INDEX      3U

/*
 * 当前实测校准值。
 */
#define JOYSTICK_THR_MIN             19U
#define JOYSTICK_THR_MAX             4077U

#define JOYSTICK_PITCH_MIN           1U
#define JOYSTICK_PITCH_CENTER        2038U
#define JOYSTICK_PITCH_MAX           4092U

#define JOYSTICK_ROLL_MIN            0U
#define JOYSTICK_ROLL_CENTER         1970U
#define JOYSTICK_ROLL_MAX            4094U

/*
 * Pitch和Roll中位死区。
 * 中位附近±30个ADC单位都输出1500。
 */
#define JOYSTICK_DEADZONE            30U

/*
 * 如果某一路方向相反，将对应宏改成1。
 */
#define JOYSTICK_THR_REVERSED        0U
#define JOYSTICK_PITCH_REVERSED      0U
#define JOYSTICK_ROLL_REVERSED       0U

typedef struct
{
    uint16_t throttle;
    uint16_t yaw;
    uint16_t pitch;
    uint16_t roll;
} JoystickData_t;

/*
 * 处理四路ADC原始数据。
 *
 * fine_mode：
 * 0：标准模式，Pitch/Roll输出1000～2000
 * 1：精细模式，Pitch/Roll输出1250～1750
 */
void Joystick_Update(
    const uint16_t adc_value[4],
    uint8_t fine_mode,
    JoystickData_t *output
);

/* 油门单向映射 */
uint16_t Joystick_MapThrottle(
    uint16_t raw,
    uint16_t min_value,
    uint16_t max_value
);

/* Pitch/Roll带中位的分段映射 */
uint16_t Joystick_MapAxis(
    uint16_t raw,
    uint16_t min_value,
    uint16_t center_value,
    uint16_t max_value,
    uint16_t deadzone
);

#endif


#include "joystick.h"
//摇杆
//限幅
static uint16_t Joystick_ClampU16(
    uint16_t value,
    uint16_t min_value,
    uint16_t max_value
)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

uint16_t Joystick_MapThrottle(
    uint16_t raw,
    uint16_t min_value,
    uint16_t max_value
)
{
    uint32_t result;

    if (max_value <= min_value)
    {
        return 1000U;
    }

    raw = Joystick_ClampU16(
        raw,
        min_value,
        max_value
    );

    /*
     * min～max映射到1000～2000。
     */
    result =
        1000U +
        ((uint32_t)(raw - min_value) * 1000U) /
        (uint32_t)(max_value - min_value);

    if (result > 2000U)
    {
        result = 2000U;
    }

    return (uint16_t)result;
}

uint16_t Joystick_MapAxis(
    uint16_t raw,
    uint16_t min_value,
    uint16_t center_value,
    uint16_t max_value,
    uint16_t deadzone
)
{
    uint16_t center_low;
    uint16_t center_high;
    uint32_t result;
    uint32_t denominator;

    /*
     * 校准参数异常时输出中位，防止除零。
     */
    if ((center_value <= min_value) ||
        (max_value <= center_value))
    {
        return 1500U;
    }

    if (center_value > deadzone)
    {
        center_low = center_value - deadzone;
    }
    else
    {
        center_low = center_value;
    }

    center_high = center_value + deadzone;

    if (center_high > max_value)
    {
        center_high = center_value;
    }

    raw = Joystick_ClampU16(
        raw,
        min_value,
        max_value
    );

    /*
     * 中位死区。
     */
    if ((raw >= center_low) &&
        (raw <= center_high))
    {
        return 1500U;
    }

    /*
     * 中位以下映射到1000～1500。
     */
    if (raw < center_low)
    {
        denominator = center_low - min_value;

        if (denominator == 0U)
        {
            return 1000U;
        }

        result =
            1000U +
            ((uint32_t)(raw - min_value) * 500U) /
            denominator;

        if (result > 1500U)
        {
            result = 1500U;
        }

        return (uint16_t)result;
    }

    /*
     * 中位以上映射到1500～2000。
     */
    denominator = max_value - center_high;

    if (denominator == 0U)
    {
        return 2000U;
    }

    result =
        1500U +
        ((uint32_t)(raw - center_high) * 500U) /
        denominator;

    if (result > 2000U)
    {
        result = 2000U;
    }

    return (uint16_t)result;
}

void Joystick_Update(
    const uint16_t adc_value[4],
    uint8_t fine_mode,
    JoystickData_t *output
)
{
    uint16_t throttle;
    uint16_t pitch;
    uint16_t roll;

    if ((adc_value == 0) || (output == 0))
    {
        return;
    }

    throttle = Joystick_MapThrottle(
        adc_value[JOYSTICK_ADC_THR_INDEX],
        JOYSTICK_THR_MIN,
        JOYSTICK_THR_MAX
    );

    pitch = Joystick_MapAxis(
        adc_value[JOYSTICK_ADC_PITCH_INDEX],
        JOYSTICK_PITCH_MIN,
        JOYSTICK_PITCH_CENTER,
        JOYSTICK_PITCH_MAX,
        JOYSTICK_DEADZONE
    );

    roll = Joystick_MapAxis(
        adc_value[JOYSTICK_ADC_ROLL_INDEX],
        JOYSTICK_ROLL_MIN,
        JOYSTICK_ROLL_CENTER,
        JOYSTICK_ROLL_MAX,
        JOYSTICK_DEADZONE
    );

#if JOYSTICK_THR_REVERSED
    throttle = 3000U - throttle;
#endif

#if JOYSTICK_PITCH_REVERSED
    pitch = 3000U - pitch;
#endif

#if JOYSTICK_ROLL_REVERSED
    roll = 3000U - roll;
#endif

    /*
     * 精细模式：
     * 将Pitch/Roll相对1500的偏移缩小一半。
     *
     * 标准模式：1000～2000
     * 精细模式：1250～1750
     */
    if (fine_mode != 0U)
    {
        pitch = (uint16_t)(
            1500 + ((int32_t)pitch - 1500) / 2
        );

        roll = (uint16_t)(
            1500 + ((int32_t)roll - 1500) / 2
        );
    }

    output->throttle = throttle;

    /*
     * 没有Yaw物理摇杆，固定中位。
     */
    output->yaw = 1500U;

    output->pitch = pitch;
    output->roll = roll;
}



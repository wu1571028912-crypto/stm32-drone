#include "key.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"


/* 当前只启用前四个按键 */
#define KEY_USED_COUNT 4U

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;

    uint8_t raw_state;          /* 本次读取的原始状态 */
    uint8_t last_raw_state;     /* 上一次原始状态 */
    uint8_t stable_state;       /* 消抖后的稳定状态 */

    uint8_t debounce_count;     /* 消抖计数 */
    uint16_t press_count;       /* 按住时间计数 */

    uint8_t long_event_sent;    /* 本次长按事件是否已经发送 */
} KeyState_t;

/*
 * 原理图中按键按下后接地，因此：
 *
 * GPIO_PIN_RESET：按下
 * GPIO_PIN_SET：松开
 *
 * CubeMX 中必须将按键 GPIO 配置成 GPIO_PULLUP。
 */
static KeyState_t s_keys[KEY_USED_COUNT] =
{
    {KEY1_GPIO_Port, KEY1_Pin, 0U, 0U, 0U, 0U, 0U, 0U},
    {KEY2_GPIO_Port, KEY2_Pin, 0U, 0U, 0U, 0U, 0U, 0U},
    {KEY3_GPIO_Port, KEY3_Pin, 0U, 0U, 0U, 0U, 0U, 0U},
    {KEY4_GPIO_Port, KEY4_Pin, 0U, 0U, 0U, 0U, 0U, 0U}
};

/* 一次性事件标志 */
static volatile uint8_t s_key_events = KEY_EVENT_NONE;

/* KEY2 按住缓降状态 */
static volatile uint8_t s_descent_held = 0U;

/* KEY3 紧急停机锁存状态 */
static volatile uint8_t s_emergency_stop_latched = 0U;

/* KEY4 控制精度模式 */
static volatile KeyControlMode_t s_control_mode = KEY_CONTROL_STANDARD;

/*
 * 读取按键。
 *
 * 返回：
 * 1：按下
 * 0：松开
 */
static uint8_t Key_ReadPin(const KeyState_t *key)
{
    if (HAL_GPIO_ReadPin(key->port, key->pin) == GPIO_PIN_RESET)
    {
        return 1U;
    }

    return 0U;
}

/* 确认某个按键刚刚按下 */
static void Key_OnPressed(uint8_t key_index)
{
    KeyState_t *key = &s_keys[key_index];

    key->press_count = 0U;
    key->long_event_sent = 0U;

    switch (key_index)
    {
        case 0U:
            /*
             * KEY1：校准键。
             * 长按达到 1 秒后再产生事件。
             */
            break;

        case 1U:
            /*
             * KEY2：缓降键。
             * 按住期间一直为 1。
             */
            s_descent_held = 1U;
            break;

        case 2U:
            /*
             * KEY3：紧急停机键。
             * 长按达到 0.5 秒后锁存。
             */
            break;

        case 3U:
            /*
             * KEY4：精度切换。
             * 在松开时判断为一次短按。
             */
            break;

        default:
            break;
    }
}

/* 确认某个按键刚刚松开 */
static void Key_OnReleased(uint8_t key_index)
{
    KeyState_t *key = &s_keys[key_index];

    switch (key_index)
    {
        case 0U:/* KEY1：切换精度模式 */
				    if (key->press_count > 0U)
            {
                if (s_control_mode == KEY_CONTROL_STANDARD)
                {
                    s_control_mode = KEY_CONTROL_FINE;
                }
                else
                {
                    s_control_mode = KEY_CONTROL_STANDARD;
                }

                s_key_events |= KEY_EVENT_MODE_CHANGED;
            }
            break;

        case 1U:
            /* KEY2 松开，退出手动缓降命令 */
            s_descent_held = 0U;
            break;

        case 2U:
            /*
             * KEY3 松开不会清除紧急停机。
             * 紧急停机必须保持锁存。
             */
            break;

        case 3U: /* KEY4：暂时不用 */
            break;

        default:
            break;
    }

    key->press_count = 0U;
    key->long_event_sent = 0U;
}

/* 处理按键持续按住 */
static void Key_OnHolding(uint8_t key_index)
{
    KeyState_t *key = &s_keys[key_index];

    if (key->press_count < 0xFFFFU)
    {
        key->press_count++;
    }

    switch (key_index)
    {
        case 0U:
            /*
             * KEY1：按住 1 秒，产生一次校准请求。
             */
            if ((key->press_count >= KEY1_CALIBRATE_COUNT) &&
                (key->long_event_sent == 0U))
            {
                key->long_event_sent = 1U;
                s_key_events |= KEY_EVENT_CALIBRATE_REQUEST;
            }
            break;

        case 1U:
            /*
             * KEY2：按住缓降。
             * s_descent_held 在按下时已经置 1。
             */
            break;

        case 2U:
            /*
             * KEY3：按住 0.5 秒，锁存紧急停机。
             */
            if ((key->press_count >= KEY3_EMERGENCY_COUNT) &&
                (key->long_event_sent == 0U))
            {
                key->long_event_sent = 1U;

                s_emergency_stop_latched = 1U;
                s_key_events |= KEY_EVENT_EMERGENCY_STOP;
            }
            break;

        case 3U:
            /* KEY4 只在松开时切换精度模式 */
            break;

        default:
            break;
    }
}

void Key_Init(void)
{
    uint8_t i;

    s_key_events = KEY_EVENT_NONE;
    s_descent_held = 0U;
    s_emergency_stop_latched = 0U;
    s_control_mode = KEY_CONTROL_STANDARD;

    for (i = 0U; i < KEY_USED_COUNT; i++)
    {
        uint8_t state = Key_ReadPin(&s_keys[i]);

        s_keys[i].raw_state = state;
        s_keys[i].last_raw_state = state;
        s_keys[i].stable_state = state;
        s_keys[i].debounce_count = 0U;
        s_keys[i].press_count = 0U;
        s_keys[i].long_event_sent = 0U;
    }
}

void Key_Scan10ms(void)
{
    uint8_t i;

    for (i = 0U; i < KEY_USED_COUNT; i++)
    {
        KeyState_t *key = &s_keys[i];

        key->raw_state = Key_ReadPin(key);

        /*
         * 原始状态发生变化：
         * 重新开始消抖计数。
         */
        if (key->raw_state != key->last_raw_state)
        {
            key->last_raw_state = key->raw_state;
            key->debounce_count = 0U;
        }
        else
        {
            /*
             * 原始状态连续保持不变。
             */
            if (key->debounce_count < KEY_DEBOUNCE_COUNT)
            {
                key->debounce_count++;
            }

            /*
             * 连续稳定达到 30 ms，确认状态变化。
             */
            if ((key->debounce_count >= KEY_DEBOUNCE_COUNT) &&
                (key->stable_state != key->raw_state))
            {
                key->stable_state = key->raw_state;

                if (key->stable_state != 0U)
                {
                    Key_OnPressed(i);
                }
                else
                {
                    Key_OnReleased(i);
                }
            }
        }

        /*
         * 对已经消抖确认且仍然按住的按键进行计时。
         */
        if (key->stable_state != 0U)
        {
            Key_OnHolding(i);
        }
    }
}

uint8_t Key_GetAndClearEvents(void)
{
    uint8_t events;

    /*
     * KEYTask 和其他任务可能同时访问事件标志，
     * 使用临界区完成“读取并清除”。
     */
    taskENTER_CRITICAL();
    events = s_key_events;
    s_key_events = KEY_EVENT_NONE;
    taskEXIT_CRITICAL();

    return events;
}

uint8_t Key_IsDescentHeld(void)
{
    return s_descent_held;
}

uint8_t Key_IsEmergencyStopLatched(void)
{
    return s_emergency_stop_latched;
}

KeyControlMode_t Key_GetControlMode(void)
{
    return s_control_mode;
}





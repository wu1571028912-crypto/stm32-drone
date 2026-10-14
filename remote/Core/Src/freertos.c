/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "sys.h"
#include "stdio.h"
#include "adc.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

uint16_t ADC_Value[4];
uint16_t thr, yaw,  pit, rol;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// 校验和计算函数： 将数组前 len 个字节累加
uint8_t Calculate_Checksum(
    const uint8_t *data,
    uint8_t length
)
{
    uint8_t sum = 0U;
    uint8_t i;

    for (i = 0U; i < length; i++)
    {
        sum += data[i];
    }

    return sum;
}

// 遥控器发送函数：建议放在定时器中断中，每20ms(50Hz)调用一次
void Remote_Send_CtrlData(
  uint16_t thr, 
  uint16_t yaw, 
  uint16_t pit,
  uint16_t rol, 
  uint8_t sw_mode
)
{
    FC_Packet_t tx_data;
    
    // 1. 填充数据
    tx_data.header1 = 0xAA;
    tx_data.header2 = 0xAF;
    tx_data.throttle = thr;
    tx_data.yaw = yaw;
    tx_data.pitch = pit;
    tx_data.roll = rol;
    tx_data.switch_cfg = sw_mode;
    
    // 2. 计算校验和 (前11个字节相加，存入第12个字节)
    tx_data.checksum = Calculate_Checksum((uint8_t*)&tx_data, 11);
    
    // 3. 将结构体数据拷贝到江协科技驱动的发送缓冲区
    memcpy(NRF24L01_TxPacket, &tx_data, 12);
    
    // 4. 调用驱动发送
    NRF24L01_Send(); 
}

uint8_t Remote_BuildControlFlags(void)
{
    uint8_t flags = 0U;

    /* KEY2：按住时发送缓降命令 */
    if (Key_IsDescentHeld() != 0U)
    {
        flags |= CTRL_FLAG_DESCENT;
    }

    /* KEY3：紧急停机触发后持续发送 */
    if (Key_IsEmergencyStopLatched() != 0U)
    {
        flags |= CTRL_FLAG_EMERGENCY_STOP;
    }

    /* KEY4：精细模式 */
    if (Key_GetControlMode() == KEY_CONTROL_FINE)
    {
        flags |= CTRL_FLAG_FINE_MODE;
    }

    return flags;
}

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for NRF */
osThreadId_t NRFHandle;
const osThreadAttr_t NRF_attributes = {
  .name = "NRF",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for KEY */
osThreadId_t KEYHandle;
const osThreadAttr_t KEY_attributes = {
  .name = "KEY",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ADC */
osThreadId_t ADCHandle;
const osThreadAttr_t ADC_attributes = {
  .name = "ADC",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void NRFTask(void *argument);
void KEYTask(void *argument);
void StartADC(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of NRF */
  NRFHandle = osThreadNew(NRFTask, NULL, &NRF_attributes);

  /* creation of KEY */
  KEYHandle = osThreadNew(KEYTask, NULL, &KEY_attributes);

  /* creation of ADC */
  ADCHandle = osThreadNew(StartADC, NULL, &ADC_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_NRFTask */

JoystickData_t g_joystick_data;
/**
  * @brief  Function implementing the NRF thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_NRFTask */
void NRFTask(void *argument)
{
  /* USER CODE BEGIN NRFTask */
  TickType_t last_wake_tick;
  uint8_t control_flags;
  uint8_t fine_mode;

  last_wake_tick = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
        fine_mode =
            Key_GetControlMode() == KEY_CONTROL_FINE;

        Joystick_Update(
            ADC_Value,
            fine_mode,
            &g_joystick_data
        );

        control_flags = Remote_BuildControlFlags();

        Remote_Send_CtrlData(
            g_joystick_data.throttle,
            1500U,
            g_joystick_data.pitch,
            g_joystick_data.roll,
            control_flags
        );
        vTaskDelayUntil(
            &last_wake_tick,
            pdMS_TO_TICKS(20U)
        );
		
  }
  /* USER CODE END NRFTask */
}

/* USER CODE BEGIN Header_KEYTask */
/**
* @brief Function implementing the KEY thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_KEYTask */
void KEYTask(void *argument)
{
  /* USER CODE BEGIN KEYTask */
  uint8_t events;

  Key_Init();

  /* Infinite loop */
  for(;;)
  {
    /* 必须保持大约 10 ms 的固定扫描周期 */
    Key_Scan10ms();

    events = Key_GetAndClearEvents();

    if ((events & KEY_EVENT_CALIBRATE_REQUEST) != 0U)
    {
            /*
             * 在这里启动摇杆校准流程。
             *
             * 注意：
             * 这里只设置请求标志，不建议在按键扫描函数中直接
             * 执行耗时的 ADC 校准或 AT24C02 写入。
             */
            // Joystick_RequestCalibration();
    }

    if ((events & KEY_EVENT_EMERGENCY_STOP) != 0U)
    {
            /*
             * 紧急停机已经锁存。
             * 可以打开蜂鸣器或者状态灯提示。
             */
            // Buzzer_EmergencyAlarm();
    }

    if ((events & KEY_EVENT_MODE_CHANGED) != 0U)
    {
      if (Key_GetControlMode() == KEY_CONTROL_FINE)
      {
          /* 精细模式指示 */
         // LED_FineMode_On();
      }
      else
      {
        /* 标准模式指示 */
        // LED_FineMode_Off();
      }
    }
    osDelay(KEY_SCAN_PERIOD_MS);
  }
  /* USER CODE END KEYTask */
}

/* USER CODE BEGIN Header_StartADC */

volatile uint8_t adc_conversion_done = 0U;

/**
* @brief Function implementing the ADC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartADC */
void StartADC(void *argument)
{
  /* USER CODE BEGIN StartADC */

    TickType_t last_wake_tick;
    uint32_t last_print_time;
    uint32_t wait_start;

    last_wake_tick = xTaskGetTickCount();
    last_print_time = HAL_GetTick();

    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

  /* Infinite loop */
  for(;;)
  {
        adc_conversion_done = 0U;

        if (HAL_ADC_Start_DMA(
                &hadc1,
                (uint32_t *)ADC_Value,
                4U
            ) != HAL_OK)
        {
            Error_Handler();
        }

        wait_start = HAL_GetTick();

        while (adc_conversion_done == 0U)
        {
            if ((HAL_GetTick() - wait_start) > 5U)
            {
                break;
            }

            osDelay(1U);
        }

        HAL_ADC_Stop_DMA(&hadc1);

        if ((HAL_GetTick() - last_print_time) >= 100U)
        {
            last_print_time = HAL_GetTick();

            thr=ADC_Value[1];
            yaw=ADC_Value[0];
            pit=ADC_Value[2];
            rol=ADC_Value[3];

        }

        vTaskDelayUntil(
            &last_wake_tick,
            pdMS_TO_TICKS(10U)
        );
		
  }
  /* USER CODE END StartADC */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc_conversion_done = 1U;
    }
}

/* USER CODE END Application */


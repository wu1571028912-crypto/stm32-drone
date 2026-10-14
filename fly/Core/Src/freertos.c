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

#include "usart.h"
#include "i2c.h"
#include "sys.h"
#include "stdio.h"
#include "tim.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//读取MPU6050的数据

uint8_t descent_command = 0U;
uint8_t emergency_stop = 0U;
uint8_t fine_mode = 0U;
//按键状态-模式
void FC_ParseControlFlags(uint8_t switch_cfg)
{
    descent_command =
        (switch_cfg & CTRL_FLAG_DESCENT) != 0U;

    emergency_stop =
        (switch_cfg & CTRL_FLAG_EMERGENCY_STOP) != 0U;

    fine_mode =
        (switch_cfg & CTRL_FLAG_FINE_MODE) != 0U;

}

FC_Packet_t g_CtrlData; 

// 记录最后一次收到有效数据的时间
uint32_t Last_Rx_Time = 0;

uint8_t Calculate_Checksum(uint8_t *data, uint8_t len) 
{
    uint8_t sum = 0;
    for(uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}
//接收数据
uint8_t FC_Receive_Task(void)
{
    uint8_t calc_sum;

    if (NRF24L01_Receive() != 1U)
    {
        return 0U;
    }

    /* 检查帧头 */
    if ((NRF24L01_RxPacket[0] != 0xAAU) ||
        (NRF24L01_RxPacket[1] != 0xAFU))
    {
        return 0U;
    }

    /* 检查校验和 */
    calc_sum = Calculate_Checksum(
        NRF24L01_RxPacket,
        sizeof(FC_Packet_t) - 1U
    );

    if (calc_sum !=
        NRF24L01_RxPacket[sizeof(FC_Packet_t) - 1U])
    {
        return 0U;
    }

    /* 数据包有效，复制到控制结构体 */
    memcpy(
        &g_CtrlData,
        NRF24L01_RxPacket,
        sizeof(FC_Packet_t)
    );

    /* 解析 switch_cfg 中的按键状态 */
    FC_ParseControlFlags(g_CtrlData.switch_cfg);

    /* 更新最后一次收到有效数据的时间 */
    Last_Rx_Time = HAL_GetTick();

    return 1U;
}

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for MPU6050 */
osThreadId_t MPU6050Handle;
const osThreadAttr_t MPU6050_attributes = {
  .name = "MPU6050",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for NRF */
osThreadId_t NRFHandle;
const osThreadAttr_t NRF_attributes = {
  .name = "NRF",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for PID */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

//积分时间
uint32_t last_time = 0;
uint8_t dir=1;

// 控制变量
float throttle = 0.0f;      // 油门 0-100%
float target_pitch = 0.0f;  // 目标俯仰角
float target_roll = 0.0f;   // 目标横滚角

uint8_t NRF_ok=0;
uint8_t MPU_ok=0;
uint8_t fly=0;

/* USER CODE END FunctionPrototypes */

void MPU6050Task(void *argument);
void NRFTask(void *argument);


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
  /* creation of MPU6050 */
  MPU6050Handle = osThreadNew(MPU6050Task, NULL, &MPU6050_attributes);

  /* creation of NRF */
  NRFHandle = osThreadNew(NRFTask, NULL, &NRF_attributes);

  /* creation of PID */

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_MPU6050Task */
int16_t motor1, motor2, motor3, motor4;
float motor1_f;
float motor2_f;
float motor3_f;
float motor4_f;
float roll_pwm,pitch_pwm,yaw_pwm;

float base_throttle;
float target_pitch;
float target_roll;
float target_yaw_rate;

uint8_t level_calibrated = 0U;

float control_roll = 0.0f;
float control_pitch = 0.0f;



uint8_t ladrc_active = 0U;
float actual_roll_rate;
float actual_pitch_rate;
float actual_yaw_rate;

/**
  * @brief  Function implementing the MPU6050 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_MPU6050Task */
void MPU6050Task(void *argument)
{
  /* USER CODE BEGIN MPU6050Task */
    TickType_t last_wake_tick;
    uint32_t last_update_ms;
    uint32_t now_ms;
    uint32_t elapsed_ms;
    float dt;

    /*
     * 保存任务启动时刻。
     */
    last_wake_tick = xTaskGetTickCount();
    last_update_ms = HAL_GetTick();
		
		uint16_t settle_count = 0U;
		uint16_t level_sample_count = 0U;

		float level_roll_sum = 0.0f;
		float level_pitch_sum = 0.0f;

		float level_roll_offset = 0.0f;
		float level_pitch_offset = 0.0f;
		
    float current_applied_roll_pwm=0.0f;
    float current_applied_pitch_pwm=0.0f;
    float current_applied_yaw_pwm=0.0f;
	
  /* Infinite loop */
  for(;;)
  {
        /*
         * 以绝对时间延时到下一个 5ms 节拍。
         * 与 osDelay(5) 不同，vTaskDelayUntil 会补偿前面
         * I2C读取和姿态计算消耗的时间。
         */
        vTaskDelayUntil(
            &last_wake_tick,
            pdMS_TO_TICKS(5U)
        );
        /*
         * 计算时间差
         */
        now_ms = HAL_GetTick();
        elapsed_ms = now_ms - last_update_ms;
        last_update_ms = now_ms;

        dt = (float)elapsed_ms * 0.001f;

        /*
         * 正常情况下 dt 应在 0.004～0.006 秒附近。
         *
         * 如果因为调试打印、I2C异常或任务阻塞导致 dt 明显异常，
         * 本次不允许使用过大的时间进行积分。
         */
        if ((dt < 0.001f) || (dt > 0.020f))
        {
            dt = 0.005f;
        }

        /*
         * 读取 MPU6050 并更新姿态。
         */
        IMU_UpdateAttitude(dt);

        /* 保存上一周期实际施加的差动量 */
        static float previous_applied_roll_pwm  = 0.0f;
        static float previous_applied_pitch_pwm = 0.0f;
        static float previous_applied_yaw_pwm   = 0.0f;
					//初始化校准
				if (level_calibrated == 0U)
					{
							/*
							 * 前400次只等待Mahony收敛：
							 * 400 × 5ms = 2秒。
							 */
							if (settle_count < 400U)
							{
									settle_count++;
							}
							else
							{
									/*
									 * 再采集400个姿态值求平均。
									 */
									level_roll_sum += attitude.roll;
									level_pitch_sum += attitude.pitch;
									level_sample_count++;

									if (level_sample_count >= 400U)
									{
											level_roll_offset =
													level_roll_sum / 400.0f;

											level_pitch_offset =
													level_pitch_sum / 400.0f;

											level_calibrated = 1U;

									}
							}
							
					}
				control_roll  =	attitude.roll   - level_roll_offset;
				control_pitch =	attitude.pitch  - level_pitch_offset;
					
				/*
         * 2. 解析遥控器数据 (映射为期望角度和角速度)
         * 假设摇杆中位1500，转换系数需根据实际手感调整
         */
					
				target_pitch =
						-((int32_t)g_CtrlData.pitch - 1500) * 0.04f;

				target_roll =
						-((int32_t)g_CtrlData.roll - 1500) * 0.04f;
        target_yaw_rate = 0.0f;
					
				base_throttle = ((float)g_CtrlData.throttle - 1000.0f) * 3.0;
				base_throttle = constrain(base_throttle, 0.0f, 3000.0f);
				
				actual_roll_rate  =  (Mpu6050_Data.Gyro_X - MpuOffset.Gyro_X) / 131.0f;
				actual_pitch_rate =  (Mpu6050_Data.Gyro_Y - MpuOffset.Gyro_Y) / 131.0f;
				actual_yaw_rate   =	 (Mpu6050_Data.Gyro_Z - MpuOffset.Gyro_Z) / 131.0f;
				
        /*
         * 4. 混控器 (Mixer) 与 动力分配
         */

				if ((level_calibrated == 0U) ||
            (emergency_stop != 0U) ||
						(g_CtrlData.throttle < 1050U) ||
						((HAL_GetTick() - Last_Rx_Time) > 300U)
           )
				{

						motor1 = motor2 = motor3 = motor4 = 0;
					
						yaw_pwm = roll_pwm = pitch_pwm = 0.0f;

						ladrc_active = 0U;
            current_applied_roll_pwm   = 0.0f;
            current_applied_pitch_pwm  = 0.0f;
            current_applied_yaw_pwm    = 0.0f;
            previous_applied_roll_pwm  = 0.0f;
            previous_applied_pitch_pwm = 0.0f;
            previous_applied_yaw_pwm   = 0.0f;
					
					CascadeLADRC_Reset(
							&flight_ladrc,
							control_roll,
							control_pitch,
							actual_roll_rate,
							actual_pitch_rate,
							actual_yaw_rate
					);
				}
				else
				{	
					if (g_CtrlData.throttle <1200U)
							{
									ladrc_active = 0U;
					      	yaw_pwm = roll_pwm = pitch_pwm = 0.0f;
									CascadeLADRC_Reset(
											&flight_ladrc,
											control_roll,
											control_pitch,
											actual_roll_rate,
											actual_pitch_rate,
											actual_yaw_rate
									);
							}
					else
							{
								/*
								 * 第一次进入控制状态时同步观测器。
								 */
								if (ladrc_active == 0U)
								{
										ladrc_active = 1U;

										CascadeLADRC_Reset(
										&flight_ladrc,
										control_roll,
										control_pitch,
										actual_roll_rate,
										actual_pitch_rate,
										actual_yaw_rate
										);
								}

										CascadeLADRC_Update(
										&flight_ladrc,
										target_roll,
										target_pitch,
										target_yaw_rate,
										control_roll,
										control_pitch,
										actual_roll_rate,
										actual_pitch_rate,
										actual_yaw_rate,
                    previous_applied_roll_pwm,
                    previous_applied_pitch_pwm,
										previous_applied_yaw_pwm,
										dt,
										&roll_pwm,
										&pitch_pwm,
										&yaw_pwm
										);
							 }
											
						/*
						 * X型四旋翼混控。
						 */
						/* 右前 */
						motor1_f = base_throttle + roll_pwm + pitch_pwm + yaw_pwm;

						/* 左前 */
						motor2_f = base_throttle - roll_pwm + pitch_pwm - yaw_pwm;

						/* 右后 */
						motor3_f = base_throttle + roll_pwm - pitch_pwm - yaw_pwm;

						/* 左后 */
						motor4_f = base_throttle - roll_pwm - pitch_pwm + yaw_pwm;

						/*
						 * 最终PWM必须限制在TIM2的0～4000范围。
						 */
						motor1_f = constrain(motor1_f, 0.0f, 4000.0f);

						motor2_f = constrain(motor2_f, 0.0f, 4000.0f);

						motor3_f = constrain(motor3_f, 0.0f, 4000.0f);

						motor4_f = constrain(motor4_f, 0.0f, 4000.0f);



						current_applied_roll_pwm =
								(motor1_f + motor3_f - motor2_f - motor4_f) * 0.25f;

						current_applied_pitch_pwm =
								(motor1_f + motor2_f - motor3_f - motor4_f) * 0.25f;

						current_applied_yaw_pwm =
								(motor1_f + motor4_f - motor2_f - motor3_f) * 0.25f;
		
		
						motor1 = (int16_t)motor1_f;//右边上
						motor2 = (int16_t)motor2_f;//左边上
						motor3 = (int16_t)motor3_f;//右边下
						motor4 = (int16_t)motor4_f;//左边下
						

				}
        /*
         * 更新定时器 PWM 占空比输出给电调
         */
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, motor1);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, motor2);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, motor3);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, motor4);
        
        previous_applied_roll_pwm  = current_applied_roll_pwm;
        previous_applied_pitch_pwm = current_applied_pitch_pwm;
        previous_applied_yaw_pwm   = current_applied_yaw_pwm;
  }
  /* USER CODE END MPU6050Task */
}

/* USER CODE BEGIN Header_NRFTask */


/**
* @brief Function implementing the NRF thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_NRFTask */
void NRFTask(void *argument)
{
  /* USER CODE BEGIN NRFTask */
  /* Infinite loop */
  for(;;)
  {

    if (FC_Receive_Task() != 0U)
    {
//				printf(
//						"ADC P:%u R:%u  TARGET P:%.2f R:%.2f  OUT P:%.2f R:%.2f\r\n",
//						g_CtrlData.pitch,4
//						g_CtrlData.roll,
//						target_pitch,
//						target_roll,
//						pitch_pwm,
//						roll_pwm
//				);
//			 printf(
//       "motor%d,%d\r\n",
//            motor2,
//            motor1
//       );
//			 printf(
//       "motor%d,%d\r\n",
//            motor4,
//						motor3
//       );
    }
    osDelay(20);
  }
  /* USER CODE END NRFTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


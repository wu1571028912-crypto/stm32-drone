#include "sys.h"
#include "stdio.h"

//系统初始化
void sys_init(void)
{ 
	HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
	HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
	HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
	HAL_GPIO_TogglePin(LED4_GPIO_Port,LED4_Pin); 
    
	// MPU6050初始化
  MPU6050_Init(Sensor_I2C2_Serch());
    //校准数据
  MPU6050_Calibrate(1000);
    //开启无线通信
	NRF24L01_Init();
	
	
	CascadeLADRC_Init(&flight_ladrc);
	
	
	HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
	HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
	HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
	HAL_GPIO_TogglePin(LED4_GPIO_Port,LED4_Pin);
}

//float 拆分为 4个 uint8_t
void float_to_uint8s(float f, uint8_t *byte0, uint8_t *byte1, uint8_t *byte2, uint8_t *byte3) 
{
    // 使用联合体进行安全类型转换
    union {
        float float_val;
        uint8_t bytes[4];
    } converter;
    
    converter.float_val = f;
    
    // 分别输出4个字节
    *byte0 = converter.bytes[0];
    *byte1 = converter.bytes[1];
    *byte2 = converter.bytes[2];
    *byte3 = converter.bytes[3];
}

//4个 uint8_t 合成为 float
float uint8s_to_float(uint8_t byte0, uint8_t byte1, uint8_t byte2, uint8_t byte3) 
{
    // 使用联合体进行安全类型转换
    union {
        float float_val;
        uint8_t bytes[4];
    } converter;
    
    // 分别设置4个字节
    converter.bytes[0] = byte0;
    converter.bytes[1] = byte1;
    converter.bytes[2] = byte2;
    converter.bytes[3] = byte3;
    
    return converter.float_val;
}



// 将uint16_t分割为两个uint8_t
void split_uint16(uint16_t data, uint8_t *high_byte, uint8_t *low_byte) {
    *high_byte = (data >> 8) & 0xFF;  // 获取高8位
    *low_byte = data & 0xFF;          // 获取低8位
}

// 将两个uint8_t合并为uint16_t
uint16_t merge_uint8(uint8_t high_byte, uint8_t low_byte) {
    return (high_byte << 8) | low_byte;
}

float constrain(
    float value,
    float min_value,
    float max_value
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

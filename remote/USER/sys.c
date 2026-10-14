#include "sys.h"
#include "stdio.h"

//系统初始化
void sys_init(void)
{ 
		NRF24L01_TxPacket[0] = 0;
		NRF24L01_TxPacket[1] = 0;
		NRF24L01_TxPacket[2] = 0;
		NRF24L01_TxPacket[3] = 0;
		NRF24L01_Init();
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
void split_uint16(uint16_t data, uint8_t *high_byte, uint8_t *low_byte)
{
    *high_byte = (data >> 8) & 0xFF;  // 获取高8位
    *low_byte = data & 0xFF;          // 获取低8位
}

// 将两个uint8_t合并为uint16_t
uint16_t merge_uint8(uint8_t high_byte, uint8_t low_byte) 
{
    return (high_byte << 8) | low_byte;
}


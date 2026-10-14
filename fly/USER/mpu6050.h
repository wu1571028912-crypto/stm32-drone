#ifndef __MPU6050_H
#define __MPU6050_H
#include "main.h"

#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIGL				0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_ADDRESS	0x3B
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define GYRO_ADDRESS  0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		  0x75	//IIC地址寄存器(默认数值0x68，只读)
#define MPU6050_PRODUCT_ID 0x68
#define MPU6050_ADDRESS 0xD0//0x68



typedef struct{
	// 加速度
	float Accel_X;
	float Accel_Y;
	float Accel_Z;
	//	角速度
	float Gyro_X;
	float Gyro_Y;
	float Gyro_Z;
	// 温度值
	float Temp;
}MPU6050DATATYPE;

/*
Pitch → 控制前后运动
Roll → 控制左右运动
Yaw → 控制航向
*/
// 姿态结构体
typedef struct {
    float pitch;  // 俯仰角（前后倾斜）
    float roll;   // 横滚角（左右倾斜）  
    float yaw;    // 偏航角（左右转向）
} EulerAngles;

extern MPU6050DATATYPE Mpu6050_Data;
//静态校准数据
extern MPU6050DATATYPE MpuOffset;

extern EulerAngles attitude;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
int16_t Sensor_I2C2_Serch(void);
int8_t MPU6050_Init(int16_t Addr);
int8_t Sensor_I2C2_ReadOneByte(uint16_t DevAddr, uint16_t MemAddr, uint8_t *oData);
int8_t Sensor_I2C2_WriteOneByte(uint16_t DevAddr, uint16_t MemAddr, uint8_t *iData);
void MPU6050_Read_Accel(void);
void MPU6050_Read_Gyro(void);
void MPU6050_Read_Temp(void);
void MPU6050_Calibrate(uint16_t sample_count);
void IMU_UpdateAttitude(float dt);
float fast_atan2(float y, float x);


#endif



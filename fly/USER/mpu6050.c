#include "mpu6050.h"
#include "mahony.h"
#include "stdio.h"


// 地址
static int16_t Mpu6050Addr = 0x68;
//读取MPU6050的数据
MPU6050DATATYPE Mpu6050_Data;
//静态校准数据
MPU6050DATATYPE MpuOffset;
//计算出的角度
EulerAngles attitude;

int8_t Sensor_I2C2_Read(uint16_t DevAddr, uint16_t MemAddr, uint8_t *oData, uint8_t DataLen)
{
	return HAL_I2C_Mem_Read(&hi2c1,DevAddr,MemAddr,1,oData,DataLen,1000);
}

int8_t Sensor_I2C2_Write(uint16_t DevAddr, uint16_t MemAddr, uint8_t *iData, uint8_t DataLen)
{
	return HAL_I2C_Mem_Write(&hi2c1,DevAddr,MemAddr,1,iData,DataLen,1000);
}

int16_t Sensor_I2C2_Serch(void)
{
	for(uint8_t i = 1; i < 255; i++)
	{
		if(HAL_I2C_IsDeviceReady(&hi2c1, i, 1, 1000) == HAL_OK)
		{
			Mpu6050Addr = i;
			return i;
		}
	}
	return 0xD1;
}

// MPU6050初始化
int8_t MPU6050_Init(int16_t Addr)
{
	uint8_t check;
	HAL_I2C_Mem_Read(&hi2c1,Addr,WHO_AM_I,1,&check,1,1000);
	if(check == 0x68) // 确认设备用 地址寄存器
	{	
		/* 唤醒MPU6050 */
		check = 0x00;
		Sensor_I2C2_Write(Addr, PWR_MGMT_1, &check, 1);

		/* 数字低通滤波器：约44Hz带宽 */
		check = 0x04;
		Sensor_I2C2_Write(Addr, CONFIGL, &check, 1);

		/* 1kHz / (1 + 4) = 200Hz */
		check = 0x04;
		Sensor_I2C2_Write(Addr, SMPLRT_DIV, &check, 1);

		/* 加速度计：±2g */
		check = 0x00;
		Sensor_I2C2_Write(Addr, ACCEL_CONFIG, &check, 1);

		/* 陀螺仪：±250°/s */
		check = 0x00;
		Sensor_I2C2_Write(Addr, GYRO_CONFIG, &check, 1);

		return 0;
	}

	return -1;
}

//MPU6050读取到原始加速度
void MPU6050_Read_Accel(void)
{
	uint8_t Read_Buf[6];

	// 寄存器依次是加速度X高 - 加速度X低 - 加速度Y高位 - 加速度Y低位 - 加速度Z高位 - 加速度度Z低位
	Sensor_I2C2_Read(Mpu6050Addr, ACCEL_XOUT_H, Read_Buf, 6); 
	
	Mpu6050_Data.Accel_X = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
	
	Mpu6050_Data.Accel_Y = (int16_t)(Read_Buf[2] << 8 | Read_Buf[3]);
	
	Mpu6050_Data.Accel_Z = (int16_t)(Read_Buf[4] << 8 | Read_Buf[5]);
}

//MPU6050读取到原始角速度
void MPU6050_Read_Gyro(void)
{
	uint8_t Read_Buf[6];

	// 寄存器依次是角度X高 - 角度X低 - 角度Y高位 - 角度Y低位 - 角度Z高位 - 角度Z低位
	Sensor_I2C2_Read(Mpu6050Addr, GYRO_XOUT_H, Read_Buf, 6); 
	
	Mpu6050_Data.Gyro_X = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
	Mpu6050_Data.Gyro_Y = (int16_t)(Read_Buf[2] << 8 | Read_Buf[3]);
	Mpu6050_Data.Gyro_Z = (int16_t)(Read_Buf[4] << 8 | Read_Buf[5]);

}


//MPU6050读取温度
void MPU6050_Read_Temp(void)
{
  uint8_t Read_Buf[2];
	
	Sensor_I2C2_Read(Mpu6050Addr, TEMP_OUT_H, Read_Buf, 2); 
	
	Mpu6050_Data.Temp = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
	
	Mpu6050_Data.Temp = 36.53f + (Mpu6050_Data.Temp / 340.0f);

}

// MPU6050 静态零偏校准
void MPU6050_Calibrate(uint16_t sample_count)
{
    int32_t gyro_sum_x = 0;
    int32_t gyro_sum_y = 0;
    int32_t gyro_sum_z = 0;
	
    int32_t accel_sum_x = 0;
    int32_t accel_sum_y = 0;
    int32_t accel_sum_z = 0;
    if (sample_count == 0U)
    {
        return;
    }

    /*
     * MPU6050唤醒后先等待稳定。
     */
    HAL_Delay(100);

    /*
     * 丢弃刚启动的前100个样本。
     */
    for (uint16_t i = 0U; i < 100U; i++)
    {
        MPU6050_Read_Gyro();
				MPU6050_Read_Accel();
        HAL_Delay(2);
    }

    /*
     * 正式计算陀螺仪零偏。
     */
    for (uint16_t i = 0U; i < sample_count; i++)
    {
        MPU6050_Read_Gyro();

        gyro_sum_x += (int32_t)Mpu6050_Data.Gyro_X;
        gyro_sum_y += (int32_t)Mpu6050_Data.Gyro_Y;
        gyro_sum_z += (int32_t)Mpu6050_Data.Gyro_Z;
			
				MPU6050_Read_Accel();			
        accel_sum_x += (int32_t)Mpu6050_Data.Accel_X;
        accel_sum_y += (int32_t)Mpu6050_Data.Accel_Y;
        accel_sum_z += (int32_t)Mpu6050_Data.Accel_Z;
        HAL_Delay(5);
    }

    MpuOffset.Gyro_X =
        (float)gyro_sum_x / (float)sample_count;

    MpuOffset.Gyro_Y =
        (float)gyro_sum_y / (float)sample_count;

    MpuOffset.Gyro_Z =
        (float)gyro_sum_z / (float)sample_count;

		
		MpuOffset.Accel_X =
        (float)accel_sum_x / (float)sample_count;

    MpuOffset.Accel_Y =
        (float)accel_sum_y / (float)sample_count;

    MpuOffset.Accel_Z =
        ((float)accel_sum_z / (float)sample_count)-16384;
    printf(
        "Calibration Done X:%.1f Y:%.1f Z:%.1f\r\n",
        MpuOffset.Gyro_X,
        MpuOffset.Gyro_Y,
        MpuOffset.Gyro_Z
    );
}


// mpu6050.c

// 更新姿态解算 (参数 dt 为执行周期，单位：秒)
void IMU_UpdateAttitude(float dt)
{
    // 1. 读取最新的 6 个原始传感器数据
    MPU6050_Read_Accel();
    MPU6050_Read_Gyro();

    // 2. 陀螺仪处理：减去零偏误差，并转为 弧度/秒 (rad/s)
    float gx = (Mpu6050_Data.Gyro_X - MpuOffset.Gyro_X) / 131.0f * 0.0174533f;
    float gy = (Mpu6050_Data.Gyro_Y - MpuOffset.Gyro_Y) / 131.0f * 0.0174533f;
    float gz = (Mpu6050_Data.Gyro_Z - MpuOffset.Gyro_Z) / 131.0f * 0.0174533f;

    // 3. 加速度计处理：直接取原始浮点数
    float ax = Mpu6050_Data.Accel_X - MpuOffset.Accel_X;
    float ay = Mpu6050_Data.Accel_Y - MpuOffset.Accel_Y;
    float az = Mpu6050_Data.Accel_Z - MpuOffset.Accel_Z;

    // 4. 将处理好的数据和 dt 喂入互补滤波算法
    MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az, dt);

    // 5. 取出计算结果，存入全局姿态结构体中
    attitude.pitch = Flight_Attitude.Pitch;
    attitude.roll  = Flight_Attitude.Roll;
    attitude.yaw   = Flight_Attitude.Yaw;
}



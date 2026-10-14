#ifndef __MAHONY_H
#define __MAHONY_H

#include <math.h>

// Mahony 滤波器的比例和积分增益
#define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.0f) // 2 * integral gain

// 存放解算后的欧拉角 (单位：度)
typedef struct {
    float Roll;
    float Pitch;
    float Yaw;
} Attitude_t;

extern Attitude_t Flight_Attitude;

// 核心更新函数
// 参数：
// gx, gy, gz: 陀螺仪数据 (必须转换为 弧度/秒 rad/s)
// ax, ay, az: 加速度计数据 (单位可以是 g，因为内部会归一化)
// dt: 两次调用的时间间隔 (单位：秒，比如 2ms 也就是 0.002f)
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);

#endif



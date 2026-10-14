#include "mahony.h"

Attitude_t Flight_Attitude;

// 全局四元数，初始化为 [1, 0, 0, 0]
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
// 误差积分
float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;

// 快速计算 1/Sqrt(x) 的倒数平方根函数 (卡马克算法)
float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) 
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // 1. 如果加速度计各轴均为0，则不处理（防止除零异常）
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
    {
        // 2. 将加速度计的三轴向量归一化
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 3. 提取当前四元数的重力分量 (把上一次四元数推导出的重力方向拿出来)
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // 4. 计算加速度计测量的重力向量与四元数推导的重力向量的叉积误差
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // 5. 积分误差比例 (Ki)
        if(twoKiDef > 0.0f) {
            integralFBx += twoKiDef * halfex * dt;
            integralFBy += twoKiDef * halfey * dt;
            integralFBz += twoKiDef * halfez * dt;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // 6. 比例误差补偿 (Kp)
        gx += twoKpDef * halfex;
        gy += twoKpDef * halfey;
        gz += twoKpDef * halfez;
    }

    // 7. 融合修正后的角速度，更新四元数
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // 8. 归一化四元数
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
    
    // 9. 将四元数转换为欧拉角 (单位: 弧度 -> 角度)
    Flight_Attitude.Pitch = asin(-2.0f * q1 * q3 + 2.0f * q0* q2)* 57.29578f; 
    Flight_Attitude.Roll  = atan2(2.0f * q2 * q3 + 2.0f * q0 * q1, -2.0f * q1 * q1 - 2.0f * q2* q2 + 1.0f)* 57.29578f;
    Flight_Attitude.Yaw   = atan2(2.0f * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3) * 57.29578f;
}



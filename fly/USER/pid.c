#include "pid.h"
#include "math.h"


CascadeLADRC_TypeDef flight_ladrc;



static float LimitFloat(
    float value,
    float minimum,
    float maximum
)
{
    if (value > maximum)
    {
        return maximum;
    }

    if (value < minimum)
    {
        return minimum;
    }

    return value;
}

void LADRC1_Init(
    LADRC1_TypeDef *adrc,
    float wc,
    float wo,
    float b0,
    float limit
)
{
    adrc->wc = wc;
    adrc->wo = wo;
    adrc->b0 = b0;
    adrc->limit = limit;

    adrc->z1 = 0.0f;
    adrc->z2 = 0.0f;
    adrc->u = 0.0f;

    adrc->saturated = 0U;
}

void LADRC1_Reset(
    LADRC1_TypeDef *adrc,
    float measurement
)
{
    /*
     * z1直接同步当前测量值，防止启动瞬间跳变。
     */
    adrc->z1 = measurement;
    adrc->z2 = 0.0f;
    adrc->u = 0.0f;

    adrc->saturated = 0U;
}

float LADRC1_Update(
    LADRC1_TypeDef *adrc,
    float target,
    float measurement,
    float applied_input,
    float dt
)
{
    float beta1;
    float beta2;
    float observer_error;
    float u0;
    float u_raw;

    if ((dt < 0.001f) || (dt > 0.020f))
    {
        dt = 0.005f;
    }

    if (adrc->b0 == 0.0f)
    {
        adrc->u = 0.0f;
        return 0.0f;
    }

    /*
     * 一阶LADRC的二阶线性ESO增益。
     */
    beta1 = 2.0f * adrc->wo;
    beta2 = adrc->wo * adrc->wo;

    observer_error =
        adrc->z1 - measurement;

    /*
     * 一阶对象：
     *
     * y_dot = b0*u + disturbance
     *
     * z1：被控量估计
     * z2：总扰动估计
     *
     * adrc->u是上一个周期真正作用的输出。
     */
    adrc->z1 += dt * (
        adrc->z2
        - beta1 * observer_error
        + adrc->b0 * applied_input
    );

    adrc->z2 += dt * (
        -beta2 * observer_error
    );

    /*
     * 状态误差反馈。
     */
    u0 = adrc->wc * (target - adrc->z1);
		
			float disturbance_limit;

			/*
			 * 防止长时间无法响应时，ESO扰动估计无限积累。
			 * 最大可补偿扰动与执行器最大输出对应。
			 */
			disturbance_limit =
					fabsf(adrc->b0) * adrc->limit;

			adrc->z2 = LimitFloat(
					adrc->z2,
					-disturbance_limit,
					disturbance_limit
			);		
		
			
    /*
     * 扰动补偿。
     */
    u_raw = (u0 - adrc->z2) / adrc->b0;
    /*
     * 这里只限制物理输出：
     *
     * 外环限制最大目标角速度；
     * 内环限制最大电机PWM差动。
     */
    adrc->u = LimitFloat(
        u_raw,
        -adrc->limit,
        adrc->limit
    );
    adrc->saturated =
        (u_raw > adrc->limit) ||
        (u_raw < -adrc->limit);

    return adrc->u;
}

void CascadeLADRC_Init(
    CascadeLADRC_TypeDef *controller
)
{
    /*
     * 角度外环：
     * 输入单位：degree
     * 输出单位：degree/s
     *
     * 对象近似为：
     * angle_dot = angular_rate
     *
     * 因此b0可以先取1。
     */
		/* 角度外环 */
		LADRC1_Init(
				&controller->roll_angle,
				1.2f,       // wc
				6.0f,       // wo
				1.0f,       // angle_dot = rate，所以b0应从1开始
				40.0f       // 最大目标角速度
		);

		LADRC1_Init(
				&controller->pitch_angle,
				1.2f,
				6.0f,
				1.0f,
				40.0f
		);

		/* 角速度内环 */
		LADRC1_Init(
				&controller->roll_rate,
				3.0f,
				15.0f,
				0.8f,
				150.0f
		);

		LADRC1_Init(
				&controller->pitch_rate,
				3.0f,
				15.0f,
				0.8f,
				150.0f
		);

        /* Yaw只做角速度内环；当前目标为0，用于抑制原地旋转。 */
        LADRC1_Init(
                &controller->yaw_rate,
                3.0f,
                15.0f,
                0.8f,
                150.0f
        );
}

void CascadeLADRC_Reset(
    CascadeLADRC_TypeDef *controller,
    float roll_angle,
    float pitch_angle,
    float roll_rate,
    float pitch_rate,
    float yaw_rate
)
{
    LADRC1_Reset(
        &controller->roll_angle,
        roll_angle
    );

    LADRC1_Reset(
        &controller->pitch_angle,
        pitch_angle
    );

    LADRC1_Reset(
        &controller->roll_rate,
        roll_rate
    );

    LADRC1_Reset(
        &controller->pitch_rate,
        pitch_rate
    );

    LADRC1_Reset(
        &controller->yaw_rate,
        yaw_rate
    );
}

void CascadeLADRC_Update(
    CascadeLADRC_TypeDef *controller,
    float target_roll,
    float target_pitch,
    float target_yaw_rate,
    float actual_roll,
    float actual_pitch,
    float actual_roll_rate,
    float actual_pitch_rate,
    float actual_yaw_rate,
    float previous_applied_roll_pwm,
    float previous_applied_pitch_pwm,
    float previous_applied_yaw_pwm,
    float dt,
    float *roll_pwm,
    float *pitch_pwm,
    float *yaw_pwm
)
{
    float target_roll_rate;
    float target_pitch_rate;

    /*
     * 第一级：角度外环。
     * 输出目标角速度。
     */
    target_roll_rate = LADRC1_Update(
        &controller->roll_angle,
        target_roll,
        actual_roll,
        actual_roll_rate,       // 真正引起角度变化的输入
        dt
    );

    target_pitch_rate = LADRC1_Update(
        &controller->pitch_angle,
        target_pitch,
        actual_pitch,
        actual_pitch_rate,      // 真正引起角度变化的输入
        dt
    );

    /*
     * 第二级：角速度内环。
     * 输出电机PWM差动。
     */
    *roll_pwm = LADRC1_Update(
        &controller->roll_rate,
        target_roll_rate,
        actual_roll_rate,
        previous_applied_roll_pwm,
        dt
    );
    *pitch_pwm = LADRC1_Update(
        &controller->pitch_rate,
        target_pitch_rate,
        actual_pitch_rate,
        previous_applied_pitch_pwm,
        dt
    );
    *yaw_pwm = LADRC1_Update(
        &controller->yaw_rate,
        target_yaw_rate,
        actual_yaw_rate,
        previous_applied_yaw_pwm,
        dt
    );
}




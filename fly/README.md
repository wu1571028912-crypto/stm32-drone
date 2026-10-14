# STM32 四旋翼飞控实验程序（fly）

基于 STM32F103C8T6、STM32 HAL、FreeRTOS / CMSIS-RTOS 的四旋翼控制实验工程，配合 `remote` 遥控端使用。该工程为独立 STM32 应用，不是 PX4 固件。

## 从源码确认的功能

- MPU6050 数据读取、启动校准与 Mahony 姿态解算。
- 横滚/俯仰角度外环和角速度内环采用串级 LADRC；偏航采用角速度控制。实现位于 `USER/pid.c`，文件名保留原工程名称。
- 姿态任务目标周期为 5 ms（200 Hz），实际运行周期需要在硬件上测量。
- NRF24L01 接收 12 字节遥控数据帧，包含油门、偏航、俯仰、横滚、状态位及累加校验。
- 源码包含超过 300 ms 未收到有效控制帧时停止电机输出的处理，以及急停、低油门和启动校准处理；
- TIM2_CH1～CH4 输出四路 PWM，对应 PA0～PA3。

## 电机接口说明

当前 `Core/Src/tim.c` 设置 TIM2 预分频为 1、周期为 3999；在 TIM2 时钟为 72 MHz 时，PWM 频率约为 9 kHz。控制器直接写入占空比计数值。这不能据此认定为标准无刷电调的 1～2 ms PWM、DShot 或 FOC 接口。


## 关键源码

- `Core/Src/freertos.c`：姿态任务、遥控接收、状态处理与混控。
- `USER/pid.c` / `USER/pid.h`：LADRC 控制器。
- `USER/mahony.c`：姿态融合。
- `USER/mpu6050.c`：IMU 驱动。
- `USER/NRF24L01.c`：无线通信驱动。
- `USER/Protocol.h`：遥控数据帧。

## 编译与烧录

1. 安装 Keil MDK-ARM 5。原工程记录的编译器为 ARM Compiler 5.06 update 5（build 528），设备包为 `Keil.STM32F1xx_DFP.2.2.0`。
2. 打开 `MDK-ARM/fly.uvprojx`，确认 STM32F103C8 器件和编译器配置。
3. 执行 Rebuild，确认 Flash / RAM 使用量符合实际芯片容量，再使用适配的 SWD 调试器烧录。
4. `fly.ioc` 为 STM32CubeMX 配置源文件；重新生成代码前请备份自定义代码。


## 目录

| 路径 | 内容 |
| --- | --- |
| `Core/` | 初始化、外设配置、中断和 FreeRTOS 任务 |
| `USER/` | 应用逻辑与设备驱动 |
| `Drivers/` | STM32 HAL 与 CMSIS |
| `Middlewares/` | FreeRTOS 与 CMSIS-RTOS 适配 |
| `MDK-ARM/` | Keil 工程与启动文件 |

## 版权与依赖

第三方代码保留各自的版权和许可，见 `THIRD_PARTY_NOTICES.md`。自有代码尚未指定统一的开源许可证，当前公开源码供查看；其他使用与再分发权利以适用的原有许可或作者后续授权为准。不能将第三方代码视为作者原创或统一重新授权。

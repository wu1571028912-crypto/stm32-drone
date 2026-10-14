# STM32 无人机（stm32-drone）

基于 **STM32F103C8T6 + FreeRTOS + NRF24L01** 的四旋翼飞控与遥控器配套工程，使用 Keil MDK-ARM 开发。

一个仓库包含两个独立烧录的嵌入式程序：`fly/` 部署在飞控端，`remote/` 部署在遥控端。两个工程的源码、依赖和相对路径保持完整。

## 工程入口

| 工程 | 用途 | Keil 工程 | 详细说明 |
| --- | --- | --- | --- |
| `fly/` | MPU6050 采集、Mahony 姿态解算、串级 LADRC、无线接收和四路 PWM 输出 | [fly.uvprojx](fly/MDK-ARM/fly.uvprojx) | [飞控说明](fly/README.md) |
| `remote/` | ADC / DMA 摇杆采集、按键处理和 NRF24L01 控制帧发送 | [remote.uvprojx](remote/MDK-ARM/remote.uvprojx) | [遥控说明](remote/README.md) |

## 获取与编译

```sh
git clone https://github.com/wu1571028912-crypto/stm32-drone.git
```

1. 安装 Keil MDK-ARM 5 及 STM32F1 器件支持。原工程记录的编译器为 ARM Compiler 5.06 update 5（build 528），器件包为 `Keil.STM32F1xx_DFP.2.2.0`。
2. 分别打开 `fly/MDK-ARM/fly.uvprojx` 与 `remote/MDK-ARM/remote.uvprojx`，选择匹配的编译器并执行 Rebuild。
3. 确认编译结果和实际芯片资源容量，然后将两个固件分别烧录到对应硬件。
4. 接线与外设配置见各自 `.ioc` 文件、`Core/Inc/main.h` 及外设初始化源码。

## 配套通信

遥控端以目标 50 Hz 周期发送 12 字节控制帧，包含帧头、油门、偏航、俯仰、横滚、状态位和累加校验。两端协议定义位于 `USER/Protocol.h`；无线地址、通道、数据速率和包长度须一致。

飞控姿态任务目标周期为 5 ms（200 Hz）。具体控制实现位于 `fly/USER/pid.c`，当前为 LADRC，文件名沿用原工程命名。

## 当前状态

本仓库整理保留原有 C 源码、头文件、CubeMX 配置和 Keil 工程，排除编译产物与本机调试缓存。本次发布未重新编译、烧录或验证实飞效果。

当前飞控输出为 TIM2 四路占空比 PWM，按 72 MHz TIM2 时钟计算约为 9 kHz；未据此声明支持标准无刷电调、DShot 或 FOC。电机驱动接口和已有实现限制详见两个子工程 README。

## 版权与第三方依赖

自有代码尚未指定统一的开源许可证。STM32 HAL、CMSIS、FreeRTOS 和江协科技 NRF24L01 驱动保留原有版权和许可声明，见 [fly 依赖说明](fly/THIRD_PARTY_NOTICES.md) 与 [remote 依赖说明](remote/THIRD_PARTY_NOTICES.md)。

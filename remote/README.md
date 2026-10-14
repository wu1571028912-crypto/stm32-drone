# STM32 NRF24L01 遥控端（remote）

基于 STM32F103C8T6、STM32 HAL、FreeRTOS / CMSIS-RTOS 的四通道遥控器工程，与 `fly` 飞控实验程序配套。

## 从源码确认的功能

- ADC / DMA 采集四通道摇杆数据，进行摇杆量映射。
- NRF24L01 发送控制帧，任务目标周期为 20 ms（50 Hz）。
- 控制包为 12 字节：帧头 `0xAA 0xAF`，四个 `uint16_t` 通道，状态位和 8 位累加校验。
- 状态位包含下降请求、锁存急停和精细模式。
- 按键任务以约 10 ms 周期扫描；摇杆校准事件的执行回调目前是注释占位。
- 当前发送任务将偏航通道固定为 `1500`，没有直接发送摇杆偏航值。

## 关键源码

- `Core/Src/freertos.c`：摇杆、按键任务和无线帧发送。
- `USER/joystick.c`：摇杆映射。
- `USER/key.c`：按键扫描与事件。
- `USER/NRF24L01.c`：无线通信驱动。
- `USER/Protocol.h`：控制帧结构。

接线以 `remote.ioc`、`Core/Inc/main.h` 和外设初始化源码为准。无线通信时，双方地址、通道、数据速率和帧长度必须一致。

## 编译与烧录

1. 安装 Keil MDK-ARM 5。原工程记录的编译器为 ARM Compiler 5.06 update 5（build 528），设备包为 `Keil.STM32F1xx_DFP.2.2.0`。
2. 打开 `MDK-ARM/remote.uvprojx`，确认 STM32F103C8 器件和编译器配置。
3. 执行 Rebuild，确认 Flash / RAM 使用量符合实际芯片容量，再使用适配的 SWD 调试器烧录。
4. `remote.ioc` 为 STM32CubeMX 配置源文件；重新生成代码前请备份自定义代码。


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

## 电路原理图

[遥控原理图.pdf](遥控原理图.pdf)

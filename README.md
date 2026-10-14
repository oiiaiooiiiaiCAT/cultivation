# Balance_Car

STM32F103C8T6，全部外设驱动使用 HAL，FreeRTOS CMSIS-RTOS v1 调度。

## 硬件

| 模块 | 接口 |
| --- | --- |
| OLED | I2C1，PB8/PB9 |
| MPU6050 | I2C2，PB10/PB11 |
| ZDT X 固件两电机 | CAN1 PA11/PA12，800 kbps，ID 1/2，扩展帧 |
| PS2 接收器 | DATA PA4、CMD PA5、CS PA6、CLK PA7 |

旧 PWM 电机控制链路已移除。SELECT 循环切换状态、MPU Pitch/Yaw/Roll、手柄页面。摇杆 X 减 128、Y 减 127，死区 ±5，显示限幅 ±120。

## 当前固件

上电等待 1 秒后开始外设/应用初始化，MPU 还会静止校准。Pitch 融合输入加 4° 安装补偿，OLED 和控制使用相同的校正角度。

已接入四状态离散 LQR，目标控制周期 4 ms（250 Hz），查询电机实际转速和位置。CAN 命令采用有界邮箱/发送等待，不等待电机命令应答；每轮查询一项反馈，由 CAN 中断解析。反馈超过 50 ms 时输出零电流并继续查询，不再锁存 FAULT 05；反馈恢复后可继续控制。任一轮电流命令发送失败则双轮停机，需重新按 START。上电不自动转轮，`g_lqr.outputAllowed=1` 只表示允许启动。车体近水平静止、四个摇杆居中，按一次 START 开始控制，再按一次 START 停止。CROSS 急停、手柄失联、IMU、倾角和速度/位移保护仍保留。

LQR 使用 X 固件 F5 力矩命令，初始电流限幅 ±500 mA。F5 没有硬件限速；模型中的惯量、轮子质量和力矩系数仍是估值，方向和动态响应尚需实测，不能认为已完成实车平衡。CAN 偶发发送错误的根因尚未确认。

已使能时左摇杆 Y 控制前后位置目标，死区 ±5、输入限幅 ±120，上推前进、下推后退、回中保持目标。目标速度为 `-归一化LeftY * joystickSpeedScale`，每次按目标速度乘 `0.004` 累加位置，计算中间量不再保存在结构体中。`g_lqr.joystickSpeedScale` 可监测调节，默认 `0.0005`，满杆目标增长速度 0.06 m/s，每 4 ms 增减 0.00024 m；默认宏 `LQR_JOYSTICK_SPEED_SCALE` 在 `Algorithm/lqr.h`。原有实际速度及 0.30 m 位置偏差保护保持不变。

反馈观察及 LQR 建模步骤见 `Docs/MOTOR_LQR_DEBUG.md`；根据参考 F4 提炼的编码规范见 `Docs/F4_CODING_STANDARD.md`。旧独立电流测试分支已移除，不再通过测试宏启动电机。

## 调试变量

- MPU：`g_mpu6050`，定义在 `Hardware/mpu6050.c`。
- PS2：`g_ps2`，定义在 `Hardware/ps2.c`。
- OLED：`g_oled`，定义在 `Hardware/oled.c`。
- 电机：`g_motor_left`、`g_motor_right`，定义在 `Hardware/motor.c`；仅保留 ID、速度 rpm、累计位置 deg、有效标志、反馈时间戳和 `commandedCurrentMa`。后者是发送指令，不是实际电流；不保存原始帧或通信统计，不新增 OLED 反馈页。
- LQR：`g_lqr`，定义在 `Algorithm/lqr.c`；`state` 单位为 m、m/s、rad、rad/s，`currentMa` 为计算电流。
- 系统：`g_car`，定义在 `Algorithm/balance.c`；`faultCode` 为 LQR 故障码，`targetL/targetR` 为 mA。

## 构建

使用 Keil ARM Compiler 5 打开 `MDK-ARM/Balance_Car.uvprojx` 编译并下载。修改工程不会关闭 Keil，也不会自动下载。

LQR 增益须根据实测机械参数、电流输入模型和实际采样周期离线计算；本仓库固件不求解 Riccati 方程，现有 K 未完成实车验证。数值验证和编译不能替代硬件验证。

当前任务栈字数为 default=64、MPU=128、motor=168、OLED=128、hand=128，FreeRTOS 堆 3072 字节，本次未改这些配置。`.ioc` 中 CAN AutoBusOff 配置仍与生成源码不一致；以当前 `can.c` 为准，自动重发开启、自动 bus-off 恢复关闭。发送等待受 `MOTOR_TX_TIMEOUT_MS=1` 和 `MOTOR_TX_WAIT_LOOPS` 限制；这不表示整个控制函数必定在 1 ms 内完成，整个控制周期及任务栈余量仍需实测，固件不再保存超周期统计。重新生成前应人工核对并保留自定义驱动、LQR 文件及 `INCLUDE_vTaskDelayUntil=1`。

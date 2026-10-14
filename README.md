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

已接入四状态离散 LQR，目标控制周期 4 ms（250 Hz），查询电机实际转速和位置。CAN 运行环节采用非阻塞提交，不让反馈等待拖住电流控制。上电不自动转轮：`MOTOR_CURRENT_TEST_ENABLED=0`、`g_lqr.outputAllowed=1`。车体近水平静止、四个摇杆居中，按一次 START 开始控制，再按一次 START 停止；不需要修改调试变量或按住 L1。CROSS 保留紧急停机，手柄失联、倾角和其他故障保护仍有效；启动条件不满足时不使能，故障不会自动重启。

LQR 使用 X 固件 F5 力矩命令，初始电流限幅 ±500 mA。F5 没有硬件限速；模型中的惯量、轮子质量和力矩系数仍是估值，方向和动态响应尚需实测，不能认为已完成实车平衡。CAN 偶发发送错误的根因尚未确认。

已使能时左摇杆 Y 控制前后位置目标，死区 ±5、输入限幅 ±120，上推前进、下推后退、回中保持目标。`referenceSpeedMps = -归一化LeftY * joystickSpeedScale`，再每次按 `referenceSpeedMps * 0.004` 累加位置。`g_lqr.joystickSpeedScale` 可监测调节，默认 `0.0005`，满杆目标增长速度 0.06 m/s，每 4 ms 增减 0.00024 m；默认宏 `LQR_JOYSTICK_SPEED_SCALE` 在 `Algorithm/lqr.h`。原有实际速度及 0.30 m 位置偏差保护保持不变。

详见 `../Docs/LQR调试.md`；原 1000 mA、3 秒、C5 限速方向测试见 `../Docs/电流方向测试.md`，仅架空轮子后专门打开测试宏使用。

## 调试变量

- MPU：`g_mpu6050`，定义在 `Hardware/mpu6050.c`。
- PS2：`g_ps2`，定义在 `Hardware/ps2.c`。
- OLED：`g_oled`，定义在 `Hardware/oled.c`。
- 电机：`g_motor_left`、`g_motor_right`、`g_motor_current_test`、`g_motor_balance_diag`，定义在 `Hardware/motor.c`。06/07 故障的阶段、HAL 状态、应答和错误在停机前锁存，并显示在 OLED 状态页。
- LQR：`g_lqr`，定义在 `Algorithm/lqr.c`；`state` 单位为 m、m/s、rad、rad/s，`currentMa` 为计算电流。
- 系统：`g_car`，定义在 `Algorithm/balance.c`；`faultCode` 为 LQR 故障码，`targetL/targetR` 为 mA。

## 构建

使用 Keil ARM Compiler 5 打开 `MDK-ARM/Balance_Car.uvprojx` 编译并下载。修改工程不会关闭 Keil，也不会自动下载。

`python ../Docs/calculate_lqr.py` 可重算增益并检查标称模型，需要 NumPy 和 SciPy。数值验证和编译不能替代硬件验证。

当前任务栈字数为 default=64、MPU=128、motor=168、OLED=128、hand=128，FreeRTOS 堆 3072 字节，本次未改这些配置。`.ioc` 中 CAN AutoBusOff 配置仍与生成源码不一致；以当前 `can.c` 为准，自动重发开启但每包仍受 10 ms 发送超时约束，自动 bus-off 恢复关闭。重新生成前应人工核对并保留自定义驱动、LQR 文件及 `INCLUDE_vTaskDelayUntil=1`。

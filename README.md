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

上电等待 1 秒后开始外设/应用初始化。Pitch 融合输入使用 +6.4° 安装补偿（`Hardware/mpu6050.h` 的 `MPU6050_PITCH_OFFSET_DEG`）；LQR 平衡参考改为 0°。这不是对原 +5° 补偿、+3.7° 参考的等价换零：同一物理姿态的角度误差增加 5.1°，需要重新验证平衡点。加速度计为 ±4 g（8192 LSB/g），陀螺仪保持 ±2000°/s（16.4 LSB/(°/s)）。

上电现在实际执行三轴陀螺仪零偏校准：先初始化电机总线并请求停机/失能，MPU配置后稳定等待1秒，再按5 ms间隔读取500组数据，约2.5秒；OLED提示 `KEEP CAR STILL`。静止检查失败最多重试3次，成功后显示 `CALIBRATION OK`，零偏保存到 `g_mpu6050.gyroBias[0..2]` 的RAM中，每次上电重测。车体只需固定静止，不要求水平或处于平衡角，不能拿手晃动校准，也不把当时Pitch强制归零。

`g_mpu6050.gyroCalibrated=1` 才允许START使能；初始化或校准失败时状态页显示 `IMU INIT FAILED` 或 `IMU CAL FAILED`，固定车体后复位重试。它只增加启动校准有效性门槛，不增加运行期自动停机。参数、静止判据和漂移限制见 `Docs/LQR_PARAMETERS.md`。

启动停机路径的临界区已改为保存/恢复PRIMASK，避免调度器启动前使用FreeRTOS任务临界区而屏蔽TIM1时基、卡死HAL_Delay。OLED第三行依次显示 `IMU INIT...`、`STABILIZING...`、`CALIBRATING...`，便于区分阻塞阶段；校准阈值和采样数未因此放宽。

OLED 角度显示一位小数。状态页 `P` 为校正后 Pitch，`E` 为 Pitch 减合成参考；MPU 页首行 `REF` 显示合成参考角，另三行保留 Pitch/Yaw/Roll。`g_lqr.balancePitchDeg` 可在 Watch 微调，默认值 `LQR_BALANCE_PITCH_DEG=0.0f`。

当前 `LQR_OUTPUT_SCALE=1.0f`，初始化 `g_lqr.outputScale`。此前按要求放大到2.5倍，用户反馈纠正猛烈、需手压抑制抖动后，先回退整体倍率到1.0，保留用户调整的平衡参考+3.7°及±1500 mA上限。原K、4 ms周期、摇杆和航向PID不变；`rawCurrentA`仍为乘倍率后、限幅前的值。回退只减少同状态下的电流请求，不能据此认定振荡已解决；手扶改变受力和运动条件，不能当作独立闭环稳定性证明。不要继续加整体倍率或用手压车调参；在限制倾倒/位移的支架下观察，准备物理断电。

模型采用整车 1.0415 kg、两轮约 0.100 kg，车体 0.9415 kg；根据整车质心坐标 (-10,16.44) mm 的径向距离暂推车体质心到轴距离约 21.286 mm，重新计算 K。几何坐标对应的姿态与 +3.7° 实测参考尚不一致，惯量和转矩常数仍为估计，详见 `Docs/LQR_PARAMETERS.md`。不把几何推算的 31.31° 直接写入角度参考。

已接入四状态离散 LQR，目标控制周期 4 ms（250 Hz），查询电机实际转速和位置。当前为手动调参模式：上电默认失能，启动校准成功后，START 每次按下切换一次电机使能/失能，长按不重复切换。不要求当前姿态稳定、低速、反馈及时或摇杆居中才能使能。CROSS、手柄失联、运行期IMU异常、倾角、速度/位移越限、反馈过期及 CAN 发送失败均不自动失能，不建立 LQR 错误锁存，也不自动启动。

失能时仍计算 LQR 供 Watch 观察；使能后持续发送限幅电流并轮询反馈。反馈无效/过期仅更新观察标志，算法使用现存测量值；手柄失联时停止推进位置参考，但仍运行 LQR。非有限数导致本次输出零电流，不切换使能。电流限幅和 CAN 有界等待保留。START 失能命令发送失败时继续重试，确保用户的停止请求不会被一次发送失败丢弃。

**本模式不具备应用层自动停机保护：陈旧反馈、单轮发送失败或断开的手柄都可能导致电机继续驱动，手柄 START 无法替代物理断电。仅架空车轮、固定车体并准备物理断电后进行调参。`g_car.enabled` 是软件请求状态，不能证明驱动已执行使能/失能命令；电机自身固件保护不由此工程撤销。**

LQR 使用 X 固件 F5 力矩命令，当前电流限幅 ±1500 mA。F5 没有硬件限速；模型中的惯量和力矩系数仍是估值，轮子质量为约测值，方向和动态响应尚需实测，不能认为已完成实车平衡。CAN 偶发发送错误的根因尚未确认。

已使能时左摇杆 X 增量调整 Yaw 目标：满杆每秒改变 30°，每 4 ms 改变 0.12°，半杆速率减半；回中或失联后停止累加并保持最后目标，不返回使能时朝向。目标在 ±180° 处环绕，可连续转向，不再受原 ±90° 范围约束；每次重新使能以当前 Yaw 初始化。左摇杆 Y 位置式映射 Pitch 目标 ±5°，不累加，上推增加参考角、下推减小参考角，回中或失联恢复 0° 基准。两轴均使用死区 ±5、输入限幅 ±120。位置参考仍为使能时记录的位置，原有位置和速度反馈保留，因此持续偏置 Pitch 不等于保证持续前进或精确跟踪倾角。

比例宏：`Algorithm/balance.h` 的 `JOYSTICK_YAW_RATE_DPS=30.0f`（满杆目标变化速率，deg/s），`Algorithm/lqr.h` 的 `LQR_JOYSTICK_PITCH_MAX_DEG=5.0f`（约 0.04167°/输入单位）。Watch 查看 `g_car.yawPid.referenceDeg` 和 `g_lqr.joystickPitchOffsetDeg`；实际 Pitch 参考为 `balancePitchDeg + joystickPitchOffsetDeg`，OLED 的 REF 显示此合成目标。原 `joystickSpeedScale` 与 `LQR_AdvanceReference` 接口保留但不再用于摇杆控制；本次不改变现有 LQR/PID 增益、整体倍率和电流限幅。

反馈观察及 LQR 建模步骤见 `Docs/MOTOR_LQR_DEBUG.md`；根据参考 F4 提炼的编码规范见 `Docs/F4_CODING_STANDARD.md`。旧独立电流测试分支已移除，不再通过测试宏启动电机。

## 调试变量

- MPU：`g_mpu6050`，定义在 `Hardware/mpu6050.c`；`initialized`为硬件初始化结果，`gyroCalibrated`为启动校准结果，`gyroBias`为三轴零偏deg/s。`gyro_dps`保留未扣零偏的工程量，查看残余角速度用 `gyro_dps[axis]-gyroBias[axis]`。
- PS2：`g_ps2`，定义在 `Hardware/ps2.c`。`PS2_Init()` 启动 TIM4 后额外等待 100 ms，再允许首次查询；运行期间的手柄轮询周期不变。该延时尚需实机验证，不能保证解决所有偶发无响应问题。
- OLED：`g_oled`，定义在 `Hardware/oled.c`。
- 电机：`g_motor_left`、`g_motor_right`，定义在 `Hardware/motor.c`；仅保留 ID、速度 rpm、累计位置 deg、有效标志、反馈时间戳和 `commandedCurrentMa`。后者是发送指令，不是实际电流；不保存原始帧或通信统计，不新增 OLED 反馈页。
- LQR：`g_lqr`，定义在 `Algorithm/lqr.c`；`state` 依次为位置误差 m、实际速度 m/s、相对平衡角的误差 rad、角速度 rad/s；`balancePitchDeg` 为校正后 Pitch 基准下的平衡参考，`currentMa` 为计算电流。
- 系统：`g_car`，定义在 `Algorithm/balance.c`；`enabled` 为 START 请求的使能状态，`targetL/targetR` 为 mA；故障码及锁存逻辑已删除。

## 构建

使用 Keil ARM Compiler 5 打开 `MDK-ARM/Balance_Car.uvprojx` 编译并下载。修改工程不会关闭 Keil，也不会自动下载。

LQR 增益须根据实测机械参数、电流输入模型和实际采样周期离线计算；本仓库固件不求解 Riccati 方程，现有 K 未完成实车验证。数值验证和编译不能替代硬件验证。

当前任务栈字数为 default=64、MPU=128、motor=168、OLED=128、hand=128，FreeRTOS 堆 3072 字节，本次未改这些配置。`.ioc` 中 CAN AutoBusOff 配置仍与生成源码不一致；以当前 `can.c` 为准，自动重发开启、自动 bus-off 恢复关闭。发送等待受 `MOTOR_TX_TIMEOUT_MS=1` 和 `MOTOR_TX_WAIT_LOOPS` 限制；这不表示整个控制函数必定在 1 ms 内完成，整个控制周期及任务栈余量仍需实测，固件不再保存超周期统计。重新生成前应人工核对并保留自定义驱动、LQR 文件及 `INCLUDE_vTaskDelayUntil=1`。

## yaw 航向保持

START 每次使能时，以当时 `g_mpu6050.yaw` 锁定朝向；失能清空积分，再使能重新锁定。不增加 OLED 页，不添加自动停机条件。

Keil Watch 展开 `g_car.yawPid`：`referenceDeg` 是目标角度，`errorDeg` 是最短角差（±180°），`integralMa` 是积分电流，`correctionMa` 是实际差动修正电流；`kp/ki/kd` 可实时修改，单位分别为 mA/deg、mA/(deg·s)、mA/(deg/s)。默认 8、0.5、0.15，仅为待实测的起始值；`outputSign` 默认 +1，实车反馈方向相反时改为 -1，不能在未核对方向时继续加大增益。

算法为 `correction = Kp*角差 + 积分 - Kd*去零偏yaw角速度`。积分按实际控制间隔累加，超过 50 ms 的间隔跳过积分；积分限幅 ±120 mA，输出限幅 ±250 mA，并进一步限制到 `1500-abs(LQR电流)` 的剩余电流空间，饱和方向不继续积分。两轮先按车辆前进方向混合：左轮 `LQR-correction`，右轮 `LQR+correction`，最后分别乘原有电机方向符号 -1/+1，因此两侧电流不再要求等幅。

建议先将 `ki=0`，确认轻微转动车体时修正方向是回到目标，再调 P、D，最后逐步增加 I 消除持续偏转。当前 yaw 来自 Z 轴陀螺积分，没有绝对航向校正，因此保持的是估计 yaw，长期真实朝向可能漂移；PID 不能补救单侧命令未执行。编译通过不代表实车闭环已调好，测试时准备物理断电。

## 软件电流上限

根据用户提供的ZDT X固件手册及驱动已设置4400 mA的反馈，将LQR限流由500 mA、发送层限流由1000 mA统一提高到每轮±1500 mA，暂不直接放到4400 mA。驱动设置不是额定电流证明，亦未由本程序读回或自动修改。OLED目标电流显示四位，如+1500；`commandedCurrentMa`不是实际电流。航向仍最多±250 mA，混合后每轮不超过1500 mA；编译检查防止LQR上限超过发送层。当前倍率1.0、平衡参考3.7°；控制周期及启停逻辑不变。仅在原输出确实触发限幅时，这次修改才会增加输出；不保证解决往返，需限制位移和倾倒并准备物理断电。

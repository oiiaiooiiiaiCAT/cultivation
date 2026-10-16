/**
 * @file    balance.h
 * @brief   平衡车应用状态及任务接口，负责传感器、控制器和电机的编排。
 */
#ifndef BALANCE_H
#define BALANCE_H

#include "main.h"
#include "mpu6050.h"
#include "oled.h"
#include "motor.h"
#include "ps2.h"
#include "filter.h"
#include "lqr.h"

#if LQR_CURRENT_LIMIT_MA > MOTOR_CURRENT_LIMIT_MA
#error LQR_CURRENT_LIMIT_MA_exceeds_motor_current_limit
#endif

#define YAW_PID_MAX_CORRECTION_MA 250
#define YAW_PID_KP_DEFAULT 8.0f
#define YAW_PID_KI_DEFAULT 0.5f
#define YAW_PID_KD_DEFAULT 0.15f
#define YAW_PID_INTEGRAL_LIMIT_MA 120.0f
#define JOYSTICK_YAW_RATE_DPS 75.0f

typedef struct
{
    volatile float referenceDeg;
    volatile float errorDeg;
    volatile float integralMa;
    volatile float correctionMa;
    volatile float kp;
    volatile float ki;
    volatile float kd;
    volatile int8_t outputSign;   /* +1 or -1, set from the actual IMU/motor yaw direction. */
} YawPid_TypeDef;

typedef struct
{
    LowPassFilter_TypeDef lpfPitchAcc;
    LowPassFilter_TypeDef lpfRollAcc;
    int16_t targetL;              /* 左轮目标电流，mA；不是实际电流反馈。 */
    int16_t targetR;              /* 右轮目标电流，mA。 */
    uint8_t canReady;
    uint8_t enabled;             /* START 请求的使能状态，不是电机实际状态反馈。 */
    uint8_t stopPending;         /* 仅初始化或 START 失能请求发送失败时重试。 */
    uint8_t feedbackPriming;     /* 使能后等待新反馈，用当前位置重置参考值。 */
    uint32_t lastStopAttempt;
    YawPid_TypeDef yawPid;        /* Heading target, PID gains and differential current. */
} BalanceCar_TypeDef;

extern BalanceCar_TypeDef g_car;
/** @brief 调度器启动前初始化，电机默认失能，START 手动切换。 */
void Car_Init(void);
/** @brief 由 MPU 任务采样、融合并更新姿态，角度单位为 deg。 */
void Car_ReadMPU(void);
/** @brief 由电机任务按 LQR_PERIOD_MS 调用；唯一的运行期电机控制入口。 */
void Car_Control(void);
/** @brief 控制任务内使能/失能，不使用 LQR 状态作为使能门槛。 */
void Car_Enable(uint8_t enable);
/** @brief 手柄任务提交按键事件，控制任务消费事件并决策。 */
void Car_SetJoystick(const PS2_StateTypeDef *state);
/** @brief OLED 任务刷新现有三页，不执行电机控制。 */
void Car_Display(void);

#endif

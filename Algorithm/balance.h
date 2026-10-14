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

typedef struct
{
    LowPassFilter_TypeDef lpfPitchAcc;
    LowPassFilter_TypeDef lpfRollAcc;
    int16_t targetL;              /* 左轮目标电流，mA；不是实际电流反馈。 */
    int16_t targetR;              /* 右轮目标电流，mA。 */
    uint8_t canReady;
    uint8_t enabled;
    uint8_t faultCode;           /* 0 表示无锁存故障，不再另存重复的 fault 标志。 */
    uint8_t stopPending;         /* 停机帧发送失败时，由控制任务定期重试。 */
    uint8_t feedbackPriming;     /* 使能后等待新反馈，用当前位置重置参考值。 */
    uint32_t lastStopAttempt;
} BalanceCar_TypeDef;

extern BalanceCar_TypeDef g_car;
/** @brief 调度器启动前初始化应用和外设，不自动使能电机。 */
void Car_Init(void);
/** @brief 由 MPU 任务采样、融合并更新姿态，角度单位为 deg。 */
void Car_ReadMPU(void);
/** @brief 由电机任务按 LQR_PERIOD_MS 调用；唯一的运行期电机控制入口。 */
void Car_Control(void);
/** @brief 控制任务内使能/停机；使能须满足姿态及反馈条件。 */
void Car_Enable(uint8_t enable);
void Car_ClearFault(void);
/** @brief 手柄任务提交按键事件，控制任务消费事件并决策。 */
void Car_SetJoystick(const PS2_StateTypeDef *state);
/** @brief OLED 任务刷新现有三页，不执行电机控制。 */
void Car_Display(void);

#endif

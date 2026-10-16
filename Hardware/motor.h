/**
 * @file    motor.h
 * @brief   EMM-V5 电机 CAN 驱动及反馈数据定义。
 *
 * 发送路径只负责下发控制命令，接收路径在 CAN 中断中解析反馈，
 * 解析后的数据保存在对应电机结构体中，便于控制任务和 Keil 调试器观察。
 */
#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

#define MOTOR_LEFT_ID 1U
#define MOTOR_RIGHT_ID 2U
#define MOTOR_LEFT_SIGN -1
#define MOTOR_RIGHT_SIGN 1
#define MOTOR_MAX_RPM 120
#define MOTOR_ACCELERATION 10U
#define MOTOR_CURRENT_LIMIT_MA 1500
#define MOTOR_CURRENT_MAX_RPM 120U
#define MOTOR_CURRENT_SLOPE_MA_S 10000U
#define MOTOR_TX_TIMEOUT_MS 1U
#define MOTOR_TX_WAIT_LOOPS 10000U
#define MOTOR_STOP_TX_ATTEMPTS 2U
#define MOTOR_FEEDBACK_WARN_AGE_MS 24U

/* EMM-V5 指令码和反馈帧格式。 */
#define MOTOR_CMD_ENABLE        0xF3U
#define MOTOR_CMD_SPEED         0xF6U
#define MOTOR_CMD_CURRENT       0xF5U
#define MOTOR_CMD_STOP          0xFEU
#define MOTOR_CMD_HEARTBEAT     0x68U
#define MOTOR_CMD_SPEED_QUERY   0x35U
#define MOTOR_CMD_POSITION_QUERY 0x36U
#define MOTOR_FRAME_END         0x6BU

/* 参照 F4：只公开控制需要的量，不混入原始帧、应答或收发统计。 */
typedef struct
{
    uint8_t address;                       /* CAN 电机 ID：左 1，右 2。 */
    volatile uint8_t speedValid;           /* 已收到有效速度反馈。 */
    volatile uint8_t positionValid;        /* 已收到有效位置反馈。 */
    int16_t commandedCurrentMa;            /* 发送电流，mA；不是实测电流。 */
    volatile float measuredRpm;            /* 有符号轴转速，rpm。 */
    volatile float measuredPositionDeg;    /* 有符号累计轴位置，deg；供 LQR 换算位移。 */
    volatile uint32_t speedUpdatedAt;      /* 速度反馈时间，HAL tick/ms。 */
    volatile uint32_t positionUpdatedAt;   /* 位置反馈时间，HAL tick/ms。 */
} Motor_TypeDef;

extern Motor_TypeDef g_motor_left, g_motor_right;

/** @brief 调度器启动前配置 CAN 及收发通知；返回 HAL 初始化结果。 */
HAL_StatusTypeDef MotorBus_Init(CAN_HandleTypeDef *handle);
void Motor_Init(Motor_TypeDef *motor, uint8_t address);
void Motor_ResetFeedback(void);
HAL_StatusTypeDef MotorBus_CancelPending(void);
HAL_StatusTypeDef Motor_Enable(Motor_TypeDef *motor, uint8_t enable);
HAL_StatusTypeDef Motor_SetSpeed(Motor_TypeDef *motor, int16_t rpm);
HAL_StatusTypeDef Motor_SetCurrent(Motor_TypeDef *motor, int16_t currentMa);
/** @brief Clamp signed current (mA) and send the X-firmware F5 command. */
HAL_StatusTypeDef Motor_SetBalanceCurrent(Motor_TypeDef *motor, int16_t currentMa);
HAL_StatusTypeDef Motor_SetHeartbeat(Motor_TypeDef *motor, uint32_t timeoutMs);
/** @brief 查询速度/位置，仅等待有界发送完成，反馈由 CAN ISR 更新。 */
HAL_StatusTypeDef Motor_RequestFeedback(Motor_TypeDef *motor, uint8_t function);
/** @brief 控制任务每次查询一项，按左速、右速、左位置、右位置轮询。 */
HAL_StatusTypeDef MotorBus_PollFeedback(void);
HAL_StatusTypeDef Motor_StartBalance(void);
HAL_StatusTypeDef Motor_Stop(Motor_TypeDef *motor);
/** @brief 取消待发帧并停止/禁用双轮；返回发送结果，不代表机械已停止。 */
HAL_StatusTypeDef Motor_StopPair(void);
HAL_StatusTypeDef Motor_SetPosition(Motor_TypeDef *motor, uint8_t direction, uint16_t rpm, uint8_t acceleration, uint32_t pulses, uint8_t absolute);

#endif



/**
 * @file    balance.c
 * @brief   传感器快照、LQR 状态构造、安全检查和双轮输出编排。
 */
#include "balance.h"
#include "i2c.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

BalanceCar_TypeDef g_car;
static uint16_t pendingButtons;
static uint8_t pendingDisconnect;

static uint8_t Car_SensorsSafe(void)
{
    uint8_t safe;
    taskENTER_CRITICAL();
    safe = g_mpu6050.healthy && HAL_GetTick() - g_mpu6050.lastUpdate < LQR_MPU_TIMEOUT_MS &&
           isfinite(g_mpu6050.pitch) && isfinite(g_mpu6050.roll) &&
           fabsf(g_mpu6050.pitch) < LQR_STOP_ANGLE_DEG && fabsf(g_mpu6050.roll) < LQR_STOP_ANGLE_DEG;
    taskEXIT_CRITICAL();
    return safe;
}

static HAL_StatusTypeDef Car_StopMotors(void)
{
    HAL_StatusTypeDef status;
    g_car.enabled = 0U;
    g_car.feedbackPriming = 0U;
    g_car.targetL = g_car.targetR = 0;
    g_lqr.positionStepM = 0.0f;
    g_lqr.referenceSpeedMps = 0.0f;
    status = Motor_StopPair();
    g_car.lastStopAttempt = HAL_GetTick();
    g_car.stopPending = status != HAL_OK;
    return status;
}

static void Car_RetryStop(void)
{
    if (g_car.stopPending && g_car.canReady && HAL_GetTick() - g_car.lastStopAttempt >= 200U)
        Car_StopMotors();
}

static void Car_Fault(uint8_t code)
{
    if (!g_car.fault) g_car.faultCode = code;
    g_car.fault = 1U;
    Car_StopMotors();
}

void Car_Init(void)
{
    memset(&g_car, 0, sizeof(g_car));
    LQR_Init();
    Motor_Init(&g_motor_left, MOTOR_LEFT_ID);
    Motor_Init(&g_motor_right, MOTOR_RIGHT_ID);
    OLED_Init(&g_oled, &hi2c1);
    PS2_Init();
    Filter_LPF_InitCutoff(&g_car.lpfPitchAcc, 10.0f, 200.0f);
    Filter_LPF_InitCutoff(&g_car.lpfRollAcc, 10.0f, 200.0f);
    if (MPU6050_Init(&g_mpu6050, &hi2c2, MPU6050_ADDR) != HAL_OK)
    {
        g_car.fault = 1U;
        if (g_car.faultCode == 0U) g_car.faultCode = 3U;
    }
    g_car.canReady = MotorBus_Init(&hcan) == HAL_OK;
    if (!g_car.canReady)
    {
        g_car.fault = 1U;
        g_car.faultCode = 1U;
    }
    else if (Car_StopMotors() != HAL_OK)
    {
        g_car.fault = 1U;
        g_car.faultCode = 2U;
    }
}

void Car_ReadMPU(void)
{
    float ax;
    float ay;
    float az;
    float pitchAcc;
    float rollAcc;
    float pitch;
    float roll;
    float yaw;
    float elapsed;
    uint32_t updated;
    uint8_t healthy;
    taskENTER_CRITICAL();
    healthy = g_mpu6050.healthy;
    updated = g_mpu6050.lastUpdate;
    pitch = g_mpu6050.pitch;
    roll = g_mpu6050.roll;
    yaw = g_mpu6050.yaw;
    taskEXIT_CRITICAL();
    if (!g_mpu6050.initialized || MPU6050_ReadAll(&g_mpu6050) != HAL_OK)
    {
        taskENTER_CRITICAL();
        g_mpu6050.healthy = 0U;
        taskEXIT_CRITICAL();
        return;
    }
    elapsed = (float)(HAL_GetTick() - updated) * 0.001f;
    ax = g_mpu6050.acc_g[0];
    ay = g_mpu6050.acc_g[1];
    az = g_mpu6050.acc_g[2];
    pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2957795f + MPU6050_PITCH_OFFSET_DEG;
    rollAcc = atan2f(ay, az) * 57.2957795f;
    if (!healthy || elapsed > 0.1f)
    {
        Filter_LPF_Reset(&g_car.lpfPitchAcc, pitchAcc);
        Filter_LPF_Reset(&g_car.lpfRollAcc, rollAcc);
        pitch = pitchAcc;
        roll = rollAcc;
    }
    else
    {
        float alpha = 0.5f / (0.5f + elapsed);
        pitchAcc = Filter_LPF_Update(&g_car.lpfPitchAcc, pitchAcc);
        rollAcc = Filter_LPF_Update(&g_car.lpfRollAcc, rollAcc);
        pitch = alpha * (pitch + (g_mpu6050.gyro_dps[1] - g_mpu6050.gyroBias[1]) * elapsed) +
                (1.0f - alpha) * pitchAcc;
        roll = alpha * (roll + (g_mpu6050.gyro_dps[0] - g_mpu6050.gyroBias[0]) * elapsed) +
               (1.0f - alpha) * rollAcc;
        yaw += (g_mpu6050.gyro_dps[2] - g_mpu6050.gyroBias[2]) * elapsed;
        if (yaw > 180.0f) yaw -= 360.0f;
        if (yaw < -180.0f) yaw += 360.0f;
    }
    taskENTER_CRITICAL();
    g_mpu6050.pitch = pitch;
    g_mpu6050.roll = roll;
    g_mpu6050.yaw = yaw;
    g_mpu6050.lastUpdate = HAL_GetTick();
    g_mpu6050.healthy = 1U;
    taskEXIT_CRITICAL();
}

void Car_SetJoystick(const PS2_StateTypeDef *state)
{
    taskENTER_CRITICAL();
    g_ps2 = *state;
    pendingButtons |= state->pressed;
    if (!state->connected) pendingDisconnect = 1U;
    taskEXIT_CRITICAL();
}

void Car_ClearFault(void)
{
    g_car.fault = 0U;
    g_car.faultCode = 0U;
}

void Car_Enable(uint8_t enable)
{
    if (!enable)
    {
        Car_StopMotors();
        return;
    }
    if (g_car.enabled || !g_lqr.outputAllowed || !g_car.canReady ||
        !g_lqr.feedbackReady || !g_lqr.valid || !Car_SensorsSafe()) return;
    if (fabsf(g_lqr.state[2]) >= LQR_ARM_ANGLE_DEG * LQR_DEG_TO_RAD ||
        fabsf(g_lqr.state[1]) >= 0.05f || fabsf(g_lqr.state[3]) >= 0.25f) return;
    if (Motor_StartBalance() != HAL_OK)
    {
        Car_StopMotors();
        return;
    }
    g_car.enabled = 1U;
    Motor_ResetFeedback();
    g_car.feedbackPriming = 1U;
    g_lqr.lastControlTick = HAL_GetTick();
}

/* 双轮发送不是原子操作；任一失败都进入统一停机，避免单轮持续输出。 */
static uint8_t Car_SendBalancePair(int16_t leftCurrent, int16_t rightCurrent)
{
    HAL_StatusTypeDef leftStatus = Motor_SetBalanceCurrent(&g_motor_left, leftCurrent);
    HAL_StatusTypeDef rightStatus = Motor_SetBalanceCurrent(&g_motor_right, rightCurrent);
    if (leftStatus == HAL_OK && rightStatus == HAL_OK) return 1U;
    /* 复用停机重试路径，不锁存通信故障；再次启动仍需按 START。 */
    Car_StopMotors();
    return 0U;
}

void Car_Control(void)
{
    PS2_StateTypeDef state;
    uint8_t disconnected;
    uint8_t feedbackReady;
    uint8_t feedbackProblem = 0U;
    uint8_t index;
    uint32_t now = HAL_GetTick();
    float pitch;
    float pitchRate;
    float gyroRateDps;
    float gyroBiasDps;
    float position;
    float speed;
    float leftPositionDeg;
    float rightPositionDeg;
    float leftSpeedRpm;
    float rightSpeedRpm;
    uint32_t feedbackUpdatedAt[4];
    uint8_t feedbackValid[4];
    g_lqr.positionStepM = 0.0f;
    g_lqr.referenceSpeedMps = 0.0f;
    taskENTER_CRITICAL();
    state = g_ps2;
    state.pressed = pendingButtons;
    pendingButtons = 0U;
    disconnected = pendingDisconnect;
    pendingDisconnect = 0U;
    taskEXIT_CRITICAL();
    if (state.pressed & PS2_SELECT)
        g_oled.displayMode = (uint8_t)((g_oled.displayMode + 1U) % 3U);
    if (g_car.enabled && ((state.pressed & PS2_START) ||
        ((state.buttons | state.pressed) & PS2_CROSS)))
    {
        Car_Enable(0U);
        return;
    }
    if (!g_car.canReady || g_car.stopPending)
    {
        g_lqr.feedbackReady = 0U;
        g_lqr.currentMa = 0;
        Car_RetryStop();
        return;
    }
    g_lqr.lastControlTick = now;
    taskENTER_CRITICAL();
    now = HAL_GetTick();
    pitch = g_mpu6050.pitch;
    gyroRateDps = g_mpu6050.gyro_dps[1];
    gyroBiasDps = g_mpu6050.gyroBias[1];
    leftPositionDeg = g_motor_left.measuredPositionDeg;
    rightPositionDeg = g_motor_right.measuredPositionDeg;
    leftSpeedRpm = g_motor_left.measuredRpm;
    rightSpeedRpm = g_motor_right.measuredRpm;
    for (index = 0U; index < 4U; index++)
    {
        Motor_TypeDef *motor = index & 1U ? &g_motor_right : &g_motor_left;
        feedbackValid[index] = index < 2U ? motor->speedValid : motor->positionValid;
        feedbackUpdatedAt[index] = index < 2U ? motor->speedUpdatedAt : motor->positionUpdatedAt;
    }
    taskEXIT_CRITICAL();
    /* 临界区只复制一致快照；浮点换算放在区外，缩短 CAN 中断屏蔽时间。 */
    pitchRate = LQR_PITCH_RATE_SIGN * (gyroRateDps - gyroBiasDps);
    /* 电机位置/速度相对车体测量，加入车体 pitch 得到轮子对地运动。 */
    position = LQR_WHEEL_RADIUS_M * LQR_DEG_TO_RAD *
        (0.5f * (MOTOR_LEFT_SIGN * leftPositionDeg +
                 MOTOR_RIGHT_SIGN * rightPositionDeg) + pitch);
    speed = LQR_WHEEL_RADIUS_M *
        (LQR_RPM_TO_RAD_S * 0.5f * (MOTOR_LEFT_SIGN * leftSpeedRpm +
                                   MOTOR_RIGHT_SIGN * rightSpeedRpm) +
         pitchRate * LQR_DEG_TO_RAD);
    g_lqr.feedbackDegraded = 0U;
    for (index = 0U; index < 4U; index++)
    {
        g_lqr.feedbackAgeMs[index] = now - feedbackUpdatedAt[index];
        if (!feedbackValid[index] || g_lqr.feedbackAgeMs[index] >= LQR_FEEDBACK_TIMEOUT_MS)
            feedbackProblem |= (uint8_t)(1U << index);
        if (!feedbackValid[index] || g_lqr.feedbackAgeMs[index] >= MOTOR_FEEDBACK_WARN_AGE_MS)
            g_lqr.feedbackDegraded = 1U;
    }
    feedbackReady = feedbackProblem == 0U;
    g_lqr.feedbackReady = feedbackReady;
    g_lqr.feedbackProblem = feedbackProblem;
    if (g_car.enabled)
    {
        if (!g_lqr.outputAllowed) { Car_Enable(0U); return; }
        if (disconnected || !state.connected || now - state.lastUpdate >= PS2_TIMEOUT_MS)
        { Car_Fault(4U); return; }
        if (!Car_SensorsSafe()) { Car_Fault(3U); return; }
    }
    /* 反馈过期先输出零电流并继续查询；新反馈恢复后继续控制，不锁存 05。 */
    if (!feedbackReady)
    {
        g_lqr.currentMa = 0;
        g_lqr.valid = 0U;
        g_car.targetL = g_car.targetR = 0;
        MotorBus_PollFeedback();
        if (g_car.enabled) Car_SendBalancePair(0, 0);
        return;
    }
    if (g_car.feedbackPriming)
    {
        LQR_ResetReference(position);
        g_car.feedbackPriming = 0U;
    }
    if (!g_car.enabled) LQR_ResetReference(position);
    if (!LQR_Update(position, speed, pitch, pitchRate))
    {
        if (g_car.enabled) Car_Fault(9U);
        else MotorBus_PollFeedback();
        return;
    }
    if ((state.pressed & PS2_START) && state.connected && !disconnected &&
        now - state.lastUpdate < PS2_TIMEOUT_MS &&
        PS2_NormalizeAxis(state.leftX, 128U) == 0 &&
        PS2_NormalizeAxis(state.leftY, 127U) == 0 &&
        PS2_NormalizeAxis(state.rightX, 128U) == 0 &&
        PS2_NormalizeAxis(state.rightY, 127U) == 0 &&
        !(state.buttons & (PS2_UP | PS2_DOWN | PS2_LEFT | PS2_RIGHT)))
    {
        Car_ClearFault();
        Car_Enable(1U);
        return;
    }
    if (!g_car.enabled)
    {
        MotorBus_PollFeedback();
        return;
    }
    if (!g_lqr.feedbackDegraded &&
        (!LQR_AdvanceReference((int16_t)-PS2_NormalizeAxis(state.leftY, 127U)) ||
         !LQR_Update(position, speed, pitch, pitchRate)))
    {
        Car_Fault(9U);
        return;
    }
    if (fabsf(g_lqr.state[0]) > LQR_MAX_POSITION_ERROR_M || fabsf(speed) > LQR_MAX_SPEED_M_S)
    {
        Car_Fault(12U);
        return;
    }
    if ((state.buttons | state.pressed) & PS2_CROSS)
    {
        MotorBus_PollFeedback();
        return;
    }
    g_car.targetL = (int16_t)(MOTOR_LEFT_SIGN * g_lqr.currentMa);
    g_car.targetR = (int16_t)(MOTOR_RIGHT_SIGN * g_lqr.currentMa);
    MotorBus_PollFeedback();
    Car_SendBalancePair(g_car.targetL, g_car.targetR);
}
void Car_Display(void)
{
    float pitch;
    float roll;
    float yaw;
    int16_t left;
    int16_t right;
    uint8_t connected;
    uint8_t enabled;
    uint8_t fault;
    uint16_t buttons;
    int16_t rightX;
    int16_t rightY;
    int16_t leftX;
    int16_t leftY;
    if (!g_oled.initialized) return;
    taskENTER_CRITICAL();
    pitch = g_mpu6050.pitch;
    roll = g_mpu6050.roll;
    yaw = g_mpu6050.yaw;
    left = g_car.targetL;
    right = g_car.targetR;
    enabled = g_car.enabled;
    fault = g_car.fault;
    connected = g_ps2.connected && HAL_GetTick() - g_ps2.lastUpdate < PS2_TIMEOUT_MS;
    buttons = g_ps2.buttons;
    rightX = PS2_NormalizeAxis(g_ps2.rightX, 128U);
    rightY = PS2_NormalizeAxis(g_ps2.rightY, 127U);
    leftX = PS2_NormalizeAxis(g_ps2.leftX, 128U);
    leftY = PS2_NormalizeAxis(g_ps2.leftY, 127U);
    taskEXIT_CRITICAL();
    if (g_oled.displayMode == 1U)
    {
        OLED_ShowString(&g_oled, 1U, 1U, "MPU ANGLE MODE   ");
        OLED_ShowString(&g_oled, 2U, 1U, "PITCH:          ");
        OLED_ShowSignedNum(&g_oled, 2U, 8U, (int32_t)pitch, 3U);
        OLED_ShowString(&g_oled, 3U, 1U, "YAW:            ");
        OLED_ShowSignedNum(&g_oled, 3U, 6U, (int32_t)yaw, 3U);
        OLED_ShowString(&g_oled, 4U, 1U, "ROLL:           ");
        OLED_ShowSignedNum(&g_oled, 4U, 7U, (int32_t)roll, 3U);
        return;
    }
    if (g_oled.displayMode == 2U)
    {
        OLED_ShowString(&g_oled, 1U, 1U, "PS2 CONTROL      ");
        OLED_ShowString(&g_oled, 2U, 1U, "RX:     RY:      ");
        OLED_ShowSignedNum(&g_oled, 2U, 4U, rightX, 3U);
        OLED_ShowSignedNum(&g_oled, 2U, 12U, rightY, 3U);
        OLED_ShowString(&g_oled, 3U, 1U, "LX:     LY:      ");
        OLED_ShowSignedNum(&g_oled, 3U, 4U, leftX, 3U);
        OLED_ShowSignedNum(&g_oled, 3U, 12U, leftY, 3U);
        OLED_ShowString(&g_oled, 4U, 1U, "L2R2L1R1:0000    ");
        OLED_ShowChar(&g_oled, 4U, 10U, (buttons & PS2_L2) ? '1' : '0');
        OLED_ShowChar(&g_oled, 4U, 11U, (buttons & PS2_R2) ? '1' : '0');
        OLED_ShowChar(&g_oled, 4U, 12U, (buttons & PS2_L1) ? '1' : '0');
        OLED_ShowChar(&g_oled, 4U, 13U, (buttons & PS2_R1) ? '1' : '0');
        return;
    }
    OLED_ShowString(&g_oled, 1U, 1U,
        fault ? "LQR FAULT:00    " : enabled ?
        (g_car.feedbackPriming ? "LQR WAIT FB     " : "LQR RUN         ") :
        g_lqr.feedbackReady ? (g_lqr.outputAllowed ? "LQR READY START " : "LQR SHADOW READY") :
                              "LQR WAIT FB     ");
    OLED_ShowString(&g_oled, 2U, 1U, "P:      R:      ");
    OLED_ShowSignedNum(&g_oled, 2U, 3U, (int32_t)pitch, 3U);
    OLED_ShowSignedNum(&g_oled, 2U, 11U, (int32_t)roll, 3U);
    OLED_ShowString(&g_oled, 3U, 1U, "I1:     I2:     ");
    OLED_ShowSignedNum(&g_oled, 3U, 4U, left, 3U);
    OLED_ShowSignedNum(&g_oled, 3U, 12U, right, 3U);
    OLED_ShowString(&g_oled, 4U, 1U,
        connected ? (g_lqr.feedbackReady ? (g_lqr.feedbackDegraded ? "PS2:OK  FB:WARN " : "PS2:OK  FB:OK   ") : "PS2:OK  FB:OFF  ") :
                    (g_lqr.feedbackReady ? (g_lqr.feedbackDegraded ? "PS2:OFF FB:WARN " : "PS2:OFF FB:OK   ") : "PS2:OFF FB:OFF  "));
}

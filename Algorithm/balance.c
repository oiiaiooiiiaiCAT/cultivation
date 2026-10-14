/**
 * @file    balance.c
 * @brief   传感器快照、LQR 状态构造及 START 手动使能的调试控制。
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
static HAL_StatusTypeDef Car_StopMotors(void)
{
    HAL_StatusTypeDef status;
    g_car.enabled = 0U;
    g_car.feedbackPriming = 0U;
    g_car.targetL = g_car.targetR = 0;
    g_lqr.currentMa = 0;
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
    /* IMU 初始化结果由 initialized/healthy 表示，不建立 LQR 错误锁存。 */
    MPU6050_Init(&g_mpu6050, &hi2c2, MPU6050_ADDR);
    g_car.canReady = MotorBus_Init(&hcan) == HAL_OK;
    if (g_car.canReady) Car_StopMotors();
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
    taskEXIT_CRITICAL();
}

void Car_Enable(uint8_t enable)
{
    if (!enable)
    {
        Car_StopMotors();
        return;
    }
    /* enabled 表示软件请求状态；发送成功也不等于电机实际执行。 */
    if (g_car.enabled || !g_car.canReady || g_car.stopPending) return;
    g_car.enabled = 1U;
    g_car.feedbackPriming = 1U;
    Motor_StartBalance();
}

/* 两轮分别发送，失败不触发禁用或错误锁存，下一周期继续发送。 */
static void Car_SendBalancePair(int16_t leftCurrent, int16_t rightCurrent)
{
    Motor_SetBalanceCurrent(&g_motor_left, leftCurrent);
    Motor_SetBalanceCurrent(&g_motor_right, rightCurrent);
}

void Car_Control(void)
{
    PS2_StateTypeDef state;
    uint8_t feedbackReady = 1U;
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
    taskENTER_CRITICAL();
    state = g_ps2;
    state.pressed = pendingButtons;
    pendingButtons = 0U;
    taskEXIT_CRITICAL();
    if (state.pressed & PS2_SELECT)
        g_oled.displayMode = (uint8_t)((g_oled.displayMode + 1U) % 3U);
    /* 只有 START 按下边沿改变使能状态；长按、CROSS 和失联不切换。 */
    if (state.pressed & PS2_START)
    {
        Car_Enable(!g_car.enabled);
        if (g_car.canReady && !g_car.stopPending) MotorBus_PollFeedback();
        return;
    }
    if (!g_car.canReady || g_car.stopPending)
    {
        g_lqr.feedbackReady = 0U;
        g_lqr.currentMa = 0;
        Car_RetryStop();
        return;
    }
    taskENTER_CRITICAL();
    now = HAL_GetTick();
    pitch = g_mpu6050.pitch;
    gyroRateDps = g_mpu6050.gyro_dps[1];
    gyroBiasDps = g_mpu6050.gyroBias[1];
    leftPositionDeg = g_motor_left.measuredPositionDeg;
    rightPositionDeg = g_motor_right.measuredPositionDeg;
    leftSpeedRpm = g_motor_left.measuredRpm;
    rightSpeedRpm = g_motor_right.measuredRpm;
    g_lqr.feedbackDegraded = 0U;
    for (index = 0U; index < 4U; index++)
    {
        Motor_TypeDef *motor = index & 1U ? &g_motor_right : &g_motor_left;
        uint8_t valid = index < 2U ? motor->speedValid : motor->positionValid;
        uint32_t age = now - (index < 2U ? motor->speedUpdatedAt : motor->positionUpdatedAt);
        if (!valid || age >= LQR_FEEDBACK_TIMEOUT_MS) feedbackReady = 0U;
        if (!valid || age >= MOTOR_FEEDBACK_WARN_AGE_MS) g_lqr.feedbackDegraded = 1U;
    }
    taskEXIT_CRITICAL();
    /* 临界区仅复制反馈并检查整数时间差；浮点换算放在区外。 */
    pitchRate = LQR_PITCH_RATE_SIGN * (gyroRateDps - gyroBiasDps);
    /* 电机位置/速度相对车体测量，加入车体 pitch 得到轮子对地运动。 */
    position = LQR_WHEEL_RADIUS_M * LQR_DEG_TO_RAD *
        (0.5f * (MOTOR_LEFT_SIGN * leftPositionDeg +
                 MOTOR_RIGHT_SIGN * rightPositionDeg) + pitch);
    speed = LQR_WHEEL_RADIUS_M *
        (LQR_RPM_TO_RAD_S * 0.5f * (MOTOR_LEFT_SIGN * leftSpeedRpm +
                                   MOTOR_RIGHT_SIGN * rightSpeedRpm) +
         pitchRate * LQR_DEG_TO_RAD);
    g_lqr.feedbackReady = feedbackReady;
    /* 反馈状态仅供观察，不再作为运行门槛；未更新时使用现存测量值。 */
    if (g_car.feedbackPriming)
    {
        LQR_ResetReference(position);
        g_car.feedbackPriming = 0U;
    }
    if (!g_car.enabled) LQR_ResetReference(position);
    /* 失能时也计算 LQR，便于先在 Watch 中观察状态和增益。 */
    if (!LQR_Update(position, speed, pitch, pitchRate))
    {
        g_car.targetL = g_car.targetR = 0;
        MotorBus_PollFeedback();
        if (g_car.enabled) Car_SendBalancePair(0, 0);
        return;
    }
    if (!g_car.enabled)
    {
        MotorBus_PollFeedback();
        return;
    }
    /* 失联不保留最后一个摇杆运动命令，但不停止 LQR 或禁用电机。 */
    if (!LQR_AdvanceReference(state.connected && now - state.lastUpdate < PS2_TIMEOUT_MS ?
        (int16_t)-PS2_NormalizeAxis(state.leftY, 127U) : 0) ||
        !LQR_Update(position, speed, pitch, pitchRate))
    {
        g_car.targetL = g_car.targetR = 0;
        g_lqr.currentMa = 0;
        MotorBus_PollFeedback();
        Car_SendBalancePair(0, 0);
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
    uint16_t buttons;
    int16_t rightX;
    int16_t rightY;
    int16_t leftX;
    int16_t leftY;
    taskENTER_CRITICAL();
    pitch = g_mpu6050.pitch;
    roll = g_mpu6050.roll;
    yaw = g_mpu6050.yaw;
    left = g_car.targetL;
    right = g_car.targetR;
    enabled = g_car.enabled;
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
        enabled ? "LQR RUN         " : "MOTOR DISABLED  ");
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

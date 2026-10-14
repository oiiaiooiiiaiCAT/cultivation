/**
 * @file    lqr.h
 * @brief   四状态 LQR 接口，不依赖 HAL/CAN，状态与增益单位必须匹配。
 */
#ifndef LQR_H
#define LQR_H

#include <stdint.h>

#define LQR_PERIOD_MS 4U             /* Must match discrete model, ms. */
#define LQR_JOYSTICK_SPEED_SCALE 0.0005f
#define LQR_DEFAULT_OUTPUT_ALLOWED 1U
#define LQR_WHEEL_RADIUS_M 0.0335f
#define LQR_CURRENT_LIMIT_MA 500
#define LQR_ARM_ANGLE_DEG 5.0f
#define LQR_STOP_ANGLE_DEG 15.0f
#define LQR_FEEDBACK_TIMEOUT_MS 50U
#define LQR_MPU_TIMEOUT_MS 30U
#define LQR_MAX_SPEED_M_S 0.30f
#define LQR_MAX_POSITION_ERROR_M 0.30f
#define LQR_RPM_TO_RAD_S 0.1047197551f
#define LQR_PITCH_RATE_SIGN 1.0f
#define LQR_DEG_TO_RAD 0.01745329252f

typedef struct
{
    float gain[4];                /* K uses current input in A; verify on the real car. */
    float state[4];               /* Position error m, speed m/s, pitch rad, pitch rate rad/s. */
    float positionReference;      /* Position reference, m. */
    volatile float joystickSpeedScale;
    float referenceSpeedMps;
    float positionStepM;
    float rawCurrentA;
    int16_t currentMa;
    uint8_t saturated;
    uint8_t feedbackReady;
    uint8_t feedbackDegraded;
    uint8_t feedbackProblem;      /* Bits follow feedbackAgeMs ordering. */
    uint32_t feedbackAgeMs[4];     /* Left/right speed, left/right position, ms. */
    uint8_t valid;
    volatile uint8_t outputAllowed;
    uint32_t lastControlTick;
    uint32_t overrunCount;
} LQR_TypeDef;

extern LQR_TypeDef g_lqr;

void LQR_Init(void);
void LQR_ResetReference(float positionM);
uint8_t LQR_AdvanceReference(int16_t forwardCommand);
/**
 * @brief 输入单位为 m、m/s、deg、deg/s；内部角度转为 rad，输出为 mA。
 * @return 状态与计算有效返回 1；非有限值返回 0，应用层另行检查安全限值。
 */
uint8_t LQR_Update(float positionM, float speedMps, float pitchDeg, float pitchRateDps);

#endif

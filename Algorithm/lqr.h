/**
 * @file    lqr.h
 * @brief   四状态 LQR 接口，不依赖 HAL/CAN，状态与增益单位必须匹配。
 */
#ifndef LQR_H
#define LQR_H

#include <stdint.h>

#define LQR_PERIOD_MS 4U             /* Must match discrete model, ms. */
#define LQR_BALANCE_PITCH_DEG 0.0f
#define LQR_JOYSTICK_PITCH_MAX_DEG 7.5f
#define LQR_OUTPUT_SCALE 1.72f
#define LQR_JOYSTICK_SPEED_SCALE 0.004f
#define LQR_WHEEL_RADIUS_M 0.0335f
#define LQR_CURRENT_LIMIT_MA 1500
#define LQR_FEEDBACK_TIMEOUT_MS 50U
#define LQR_RPM_TO_RAD_S 0.1047197551f
#define LQR_PITCH_RATE_SIGN 1.0f
#define LQR_DEG_TO_RAD 0.01745329252f

typedef struct
{
    float gain[4];                /* K uses current input in A; verify on the real car. */
    float state[4];               /* Position error m, speed m/s, pitch error rad, pitch rate rad/s. */
    float positionReference;      /* Position reference, m. */
    volatile float balancePitchDeg;
    volatile float joystickPitchOffsetDeg;
    volatile float outputScale;
    volatile float joystickSpeedScale;
    float rawCurrentA;
    int16_t currentMa;
    uint8_t saturated;
    uint8_t feedbackReady;
    uint8_t feedbackDegraded;
    uint8_t valid;
} LQR_TypeDef;

extern LQR_TypeDef g_lqr;

void LQR_Init(void);
void LQR_ResetReference(float positionM);
uint8_t LQR_AdvanceReference(int16_t forwardCommand);
/**
 * @brief 输入为 m、m/s、校正后 Pitch deg、deg/s；减去平衡参考再转 rad，输出 mA。
 * @return 状态与计算有效返回 1；非有限值返回 0，不切换电机使能状态。
 */
uint8_t LQR_Update(float positionM, float speedMps, float pitchDeg, float pitchRateDps);

#endif

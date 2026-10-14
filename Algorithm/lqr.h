#ifndef LQR_H
#define LQR_H

#include <stdint.h>

#define LQR_PERIOD_MS 4U
#define LQR_JOYSTICK_SPEED_SCALE 0.0005f
#define LQR_DEFAULT_OUTPUT_ALLOWED 1U
#define LQR_WHEEL_RADIUS_M 0.0335f
#define LQR_CURRENT_LIMIT_MA 500
#define LQR_ARM_ANGLE_DEG 5.0f
#define LQR_STOP_ANGLE_DEG 15.0f
#define LQR_FEEDBACK_TIMEOUT_MS 50U
#define LQR_MPU_TIMEOUT_MS 30U
#define LQR_CURRENT_ACK_TIMEOUT_MS 50U
#define LQR_MAX_SPEED_M_S 0.30f
#define LQR_PITCH_RATE_SIGN 1.0f
#define LQR_DEG_TO_RAD 0.01745329252f

typedef struct
{
    float gain[4];
    float state[4];
    float positionReference;
    volatile float joystickSpeedScale;
    float referenceSpeedMps;
    float positionStepM;
    float rawCurrentA;
    int16_t currentMa;
    uint8_t saturated;
    uint8_t feedbackReady;
    uint8_t feedbackDegraded;
    uint8_t feedbackProblem;
    uint32_t feedbackAgeMs[4];
    uint8_t valid;
    volatile uint8_t outputAllowed;
    uint32_t lastControlTick;
    uint32_t overrunCount;
} LQR_TypeDef;

extern LQR_TypeDef g_lqr;

void LQR_Init(void);
void LQR_ResetReference(float positionM);
uint8_t LQR_AdvanceReference(int16_t forwardCommand);
uint8_t LQR_Update(float positionM, float speedMps, float pitchDeg, float pitchRateDps);

#endif

#include "lqr.h"
#include <math.h>
#include <string.h>

LQR_TypeDef g_lqr;

void LQR_Init(void)
{
    memset(&g_lqr, 0, sizeof(g_lqr));
    g_lqr.gain[0] = -0.12822586f;
    g_lqr.gain[1] = -0.25494726f;
    g_lqr.gain[2] = -1.33878598f;
    g_lqr.gain[3] = -0.11150894f;
    g_lqr.joystickSpeedScale = LQR_JOYSTICK_SPEED_SCALE;
    g_lqr.outputAllowed = LQR_DEFAULT_OUTPUT_ALLOWED;
}

void LQR_ResetReference(float positionM)
{
    g_lqr.positionReference = positionM;
    g_lqr.referenceSpeedMps = 0.0f;
    g_lqr.positionStepM = 0.0f;
    g_lqr.currentMa = 0;
    g_lqr.rawCurrentA = 0.0f;
}

uint8_t LQR_AdvanceReference(int16_t forwardCommand)
{
    float scale = g_lqr.joystickSpeedScale;
    float referenceSpeed;
    float step;
    float reference;
    g_lqr.positionStepM = 0.0f;
    g_lqr.referenceSpeedMps = 0.0f;
    if (!isfinite(scale) || scale < 0.0f || !isfinite(g_lqr.positionReference))
    {
        g_lqr.valid = 0U;
        return 0U;
    }
    if (forwardCommand > 120) forwardCommand = 120;
    if (forwardCommand < -120) forwardCommand = -120;
    referenceSpeed = (float)forwardCommand * scale;
    step = referenceSpeed * (float)LQR_PERIOD_MS * 0.001f;
    reference = g_lqr.positionReference + step;
    if (!isfinite(referenceSpeed) || !isfinite(step) || !isfinite(reference))
    {
        g_lqr.valid = 0U;
        return 0U;
    }
    g_lqr.positionStepM = step;
    g_lqr.referenceSpeedMps = referenceSpeed;
    g_lqr.positionReference = reference;
    return 1U;
}

uint8_t LQR_Update(float positionM, float speedMps, float pitchDeg, float pitchRateDps)
{
    float currentA = 0.0f;
    uint8_t index;
    g_lqr.valid = 0U;
    g_lqr.currentMa = 0;
    g_lqr.rawCurrentA = 0.0f;
    g_lqr.saturated = 0U;
    if (!isfinite(positionM) || !isfinite(speedMps) ||
        !isfinite(pitchDeg) || !isfinite(pitchRateDps)) return 0U;
    g_lqr.state[0] = positionM - g_lqr.positionReference;
    g_lqr.state[1] = speedMps;
    g_lqr.state[2] = pitchDeg * LQR_DEG_TO_RAD;
    g_lqr.state[3] = pitchRateDps * LQR_DEG_TO_RAD;
    for (index = 0U; index < 4U; index++)
    {
        if (!isfinite(g_lqr.state[index])) return 0U;
    }
    for (index = 0U; index < 4U; index++)
    {
        if (!isfinite(g_lqr.gain[index])) return 0U;
        currentA -= g_lqr.gain[index] * g_lqr.state[index];
    }
    if (!isfinite(currentA)) return 0U;
    g_lqr.rawCurrentA = currentA;
    if (currentA > (float)LQR_CURRENT_LIMIT_MA * 0.001f)
    {
        currentA = (float)LQR_CURRENT_LIMIT_MA * 0.001f;
        g_lqr.saturated = 1U;
    }
    if (currentA < -(float)LQR_CURRENT_LIMIT_MA * 0.001f)
    {
        currentA = -(float)LQR_CURRENT_LIMIT_MA * 0.001f;
        g_lqr.saturated = 1U;
    }
    g_lqr.currentMa = (int16_t)(currentA * 1000.0f);
    g_lqr.valid = 1U;
    return 1U;
}

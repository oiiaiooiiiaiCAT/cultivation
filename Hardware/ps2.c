#include "ps2.h"
#include "tim.h"
#include <string.h>

static uint16_t previousButtons;
static uint8_t ready;
static uint8_t previousValid;
PS2_StateTypeDef g_ps2;
PS2_DiagnosticTypeDef g_ps2_diag;

static void PS2_DelayUs(uint16_t duration)
{
    uint16_t started = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    while ((uint16_t)((uint16_t)__HAL_TIM_GET_COUNTER(&htim4) - started) < duration) {}
}

static uint8_t PS2_Transfer(uint8_t command)
{
    uint8_t received = 0U;
    uint8_t bitIndex;
    for (bitIndex = 0U; bitIndex < 8U; bitIndex++)
    {
        HAL_GPIO_WritePin(PS2_CMD_GPIO_Port, PS2_CMD_Pin,
            (command & (1U << bitIndex)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_RESET);
        PS2_DelayUs(16U);
        if (HAL_GPIO_ReadPin(PS2_DATA_GPIO_Port, PS2_DATA_Pin) == GPIO_PIN_SET)
            received |= (uint8_t)(1U << bitIndex);
        HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_SET);
        PS2_DelayUs(16U);
    }
    PS2_DelayUs(20U);
    return received;
}

HAL_StatusTypeDef PS2_Init(void)
{
    memset(&g_ps2, 0, sizeof(g_ps2));
    memset(&g_ps2_diag, 0, sizeof(g_ps2_diag));
    HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PS2_CMD_GPIO_Port, PS2_CMD_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_SET);
    previousButtons = 0U;
    previousValid = 0U;
    ready = HAL_TIM_Base_Start(&htim4) == HAL_OK;
    return ready ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef PS2_Poll(PS2_StateTypeDef *state)
{
    const uint8_t command[9] = {0x01U, 0x42U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t response[9];
    uint8_t index;
    if (state == NULL) return HAL_ERROR;
    g_ps2_diag.pollCount++;
    g_ps2_diag.lastPollAt = HAL_GetTick();
    memset(state, 0, sizeof(*state));
    state->leftX = state->leftY = state->rightX = state->rightY = 128U;
    if (!ready)
    {
        previousButtons = 0U;
        previousValid = 0U;
        g_ps2_diag.lastStatus = HAL_ERROR;
        return HAL_ERROR;
    }
    HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_RESET);
    PS2_DelayUs(20U);
    for (index = 0U; index < 9U; index++) response[index] = PS2_Transfer(command[index]);
    HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_SET);
    memcpy(g_ps2_diag.lastResponse, response, sizeof(response));
    if (response[2] != 0x5AU || (response[1] != 0x41U && response[1] != 0x73U))
    {
        previousButtons = 0U;
        previousValid = 0U;
        g_ps2_diag.invalidFrameCount++;
        g_ps2_diag.consecutiveErrors++;
        g_ps2_diag.lastStatus = HAL_ERROR;
        return HAL_ERROR;
    }
    state->mode = response[1];
    state->buttons = (uint16_t)~((uint16_t)response[3] | ((uint16_t)response[4] << 8));
    state->pressed = previousValid ? state->buttons & (uint16_t)~previousButtons : 0U;
    previousValid = 1U;
    previousButtons = state->buttons;
    if (state->mode == 0x73U)
    {
        state->rightX = response[5];
        state->rightY = response[6];
        state->leftX = response[7];
        state->leftY = response[8];
    }
    state->connected = 1U;
    state->lastUpdate = HAL_GetTick();
    g_ps2_diag.validFrameCount++;
    g_ps2_diag.consecutiveErrors = 0U;
    g_ps2_diag.lastValidAt = state->lastUpdate;
    g_ps2_diag.lastStatus = HAL_OK;
    return HAL_OK;
}

int16_t PS2_NormalizeAxis(uint8_t value, uint8_t center)
{
    int16_t centered = (int16_t)value - (int16_t)center;
    if (centered >= -PS2_DEADZONE && centered <= PS2_DEADZONE) return 0;
    if (centered < -120) return -120;
    if (centered > 120) return 120;
    return centered;
}

int16_t PS2_Axis(uint8_t value)
{
    int16_t centered = (int16_t)value - 128;
    if (centered >= -PS2_DEADZONE && centered <= PS2_DEADZONE) return 0;
    return centered > 0 ? centered - PS2_DEADZONE : centered + PS2_DEADZONE;
}

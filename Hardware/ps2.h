#ifndef PS2_H
#define PS2_H

#include "main.h"

#define PS2_SELECT (1U << 0)
#define PS2_R3 (1U << 1)
#define PS2_L3 (1U << 2)
#define PS2_START (1U << 3)
#define PS2_UP (1U << 4)
#define PS2_RIGHT (1U << 5)
#define PS2_DOWN (1U << 6)
#define PS2_LEFT (1U << 7)
#define PS2_L2 (1U << 8)
#define PS2_R2 (1U << 9)
#define PS2_L1 (1U << 10)
#define PS2_R1 (1U << 11)
#define PS2_TRIANGLE (1U << 12)
#define PS2_CIRCLE (1U << 13)
#define PS2_CROSS (1U << 14)
#define PS2_SQUARE (1U << 15)
#define PS2_TIMEOUT_MS 200U
#define PS2_DEADZONE 5

typedef struct
{
    uint16_t buttons;
    uint16_t pressed;
    uint8_t rightX;
    uint8_t rightY;
    uint8_t leftX;
    uint8_t leftY;
    uint8_t connected;
    uint32_t lastUpdate;
} PS2_StateTypeDef;

extern PS2_StateTypeDef g_ps2;

HAL_StatusTypeDef PS2_Init(void);
HAL_StatusTypeDef PS2_Poll(PS2_StateTypeDef *state);
int16_t PS2_Axis(uint8_t value);
int16_t PS2_NormalizeAxis(uint8_t value, uint8_t center);

#endif

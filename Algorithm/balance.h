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
    int16_t targetL;
    int16_t targetR;
    uint8_t canReady;
    uint8_t enabled;
    uint8_t fault;
    uint8_t faultCode;
    uint8_t feedbackSlot;
    uint8_t stopPending;
    uint8_t feedbackPriming;
    uint32_t feedbackPrimingStartedAt;
    uint32_t lastStopAttempt;
} BalanceCar_TypeDef;

extern BalanceCar_TypeDef g_car;
void Car_Init(void);
void Car_ReadMPU(void);
void Car_Control(void);
void Car_Enable(uint8_t enable);
void Car_ClearFault(void);
void Car_SetJoystick(const PS2_StateTypeDef *state);
void Car_Display(void);

#endif

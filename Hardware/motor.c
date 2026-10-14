/**
 * @file    motor.c
 * @brief   EMM-V5 CAN command TX and interrupt feedback decoding; no LQR.
 */
#include <string.h>

#include "motor.h"
#include "FreeRTOS.h"
#include "task.h"

Motor_TypeDef g_motor_left;
Motor_TypeDef g_motor_right;
Motor_BusTypeDef g_motor_bus;
static CAN_HandleTypeDef *motorCan;
static Motor_TypeDef *registeredMotors[2];
static volatile uint32_t completedMailboxes;
static volatile uint32_t failedMailboxes;
static volatile uint32_t mailboxErrors[3];

static HAL_StatusTypeDef Motor_RecordTxFailure(Motor_TypeDef *motor,
                                               uint8_t function, HAL_StatusTypeDef status)
{
    motor->lastStatus = status;
    motor->lastTxError = motor->commandTxError | HAL_CAN_GetError(motorCan);
    motor->commandTxError = motor->lastTxError;
    motor->lastTxFailedFunction = function;
    motor->txFailureCount++;
    return status;
}

/* Extended ID = (address << 8) | packet; payload starts with function.
 * Control-task only: waits for CAN TX completion, not motor acknowledgement.
 * Tick and loop limits bound the wait; never call inside a critical section.
 */
static HAL_StatusTypeDef Motor_Send(Motor_TypeDef *motor,
                                    const uint8_t *command, uint8_t length)
{
    CAN_TxHeaderTypeDef header = {0};
    uint8_t payload[8] = {0};
    uint8_t offset = 2U;
    uint8_t packet = 0U;
    uint32_t mailbox;
    uint32_t started;
    uint32_t waitLoops;
    HAL_StatusTypeDef status;
    if (motor == NULL || motorCan == NULL || motor->address == 0U || command == NULL || length < 3U)
        return HAL_ERROR;
    motor->commandTxError = HAL_CAN_ERROR_NONE;
    while (offset < length)
    {
        uint8_t count = (uint8_t)(length - offset);
        if (count > 7U) count = 7U;
        header.ExtId = ((uint32_t)motor->address << 8) | packet;
        header.IDE = CAN_ID_EXT;
        header.RTR = CAN_RTR_DATA;
        header.DLC = (uint32_t)count + 1U;
        payload[0] = command[1];
        memcpy(&payload[1], &command[offset], count);
        waitLoops = 0U;
        started = HAL_GetTick();
        while (HAL_CAN_GetTxMailboxesFreeLevel(motorCan) == 0U)
        {
            if (HAL_GetTick() - started >= MOTOR_TX_TIMEOUT_MS ||
                ++waitLoops >= MOTOR_TX_WAIT_LOOPS)
                return Motor_RecordTxFailure(motor, command[1], HAL_TIMEOUT);
        }
        completedMailboxes = 0U;
        failedMailboxes = 0U;
        mailboxErrors[0] = mailboxErrors[1] = mailboxErrors[2] = 0U;
        HAL_CAN_ResetError(motorCan);
        status = HAL_CAN_AddTxMessage(motorCan, &header, payload, &mailbox);
        if (status != HAL_OK)
        {
            return Motor_RecordTxFailure(motor, command[1], status);
        }
        started = HAL_GetTick();
        waitLoops = 0U;
        while ((completedMailboxes & mailbox) == 0U)
        {
            if ((failedMailboxes & mailbox) != 0U)
            {
                uint8_t slot = mailbox == CAN_TX_MAILBOX0 ? 0U : mailbox == CAN_TX_MAILBOX1 ? 1U : 2U;
                motor->commandTxError = mailboxErrors[slot];
                return Motor_RecordTxFailure(motor, command[1], HAL_ERROR);
            }
            if (HAL_GetTick() - started >= MOTOR_TX_TIMEOUT_MS ||
                ++waitLoops >= MOTOR_TX_WAIT_LOOPS)
            {
                status = Motor_RecordTxFailure(motor, command[1], HAL_TIMEOUT);
                HAL_CAN_AbortTxRequest(motorCan, mailbox);
                return status;
            }
            if (!HAL_CAN_IsTxMessagePending(motorCan, mailbox))
            {
                completedMailboxes |= mailbox;
                break;
            }
        }
        offset = (uint8_t)(offset + count);
        packet++;
    }
    motor->lastStatus = HAL_OK;
    return HAL_OK;
}

HAL_StatusTypeDef MotorBus_Init(CAN_HandleTypeDef *handle)
{
    CAN_FilterTypeDef filter = {0};
    if (handle == NULL) return HAL_ERROR;
    if (HAL_CAN_GetState(handle) != HAL_CAN_STATE_READY) return HAL_ERROR;
    if (handle->Init.AutoRetransmission != ENABLE)
    {
        handle->Init.AutoRetransmission = ENABLE;
        if (HAL_CAN_Init(handle) != HAL_OK) return HAL_ERROR;
    }
    motorCan = handle;
    memset(&g_motor_bus, 0, sizeof(g_motor_bus));
    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdLow = CAN_ID_EXT;
    filter.FilterMaskIdLow = CAN_ID_EXT | CAN_RTR_REMOTE;
    filter.FilterFIFOAssignment = CAN_RX_FIFO1;
    filter.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(handle, &filter) != HAL_OK) return HAL_ERROR;
    if (HAL_CAN_Start(handle) != HAL_OK) return HAL_ERROR;
    return HAL_CAN_ActivateNotification(handle,
        CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_RX_FIFO1_OVERRUN | CAN_IT_TX_MAILBOX_EMPTY);
}


HAL_StatusTypeDef MotorBus_CancelPending(void)
{
    uint32_t mask = 0U;
    uint32_t started;
    uint32_t waitLoops = 0U;
    uint8_t index;
    if (motorCan == NULL) return HAL_ERROR;
    /* Check hardware mailboxes; normal TX no longer marks legacy active flags. */
    for (index = 0U; index < 3U; index++)
        if (HAL_CAN_IsTxMessagePending(motorCan, 1UL << index)) mask |= 1UL << index;
    if (mask != 0U && HAL_CAN_IsTxMessagePending(motorCan, mask))
    {
        if (HAL_CAN_AbortTxRequest(motorCan, mask) != HAL_OK) return HAL_ERROR;
        started = HAL_GetTick();
        while (HAL_CAN_IsTxMessagePending(motorCan, mask))
        {
            if (HAL_GetTick() - started >= MOTOR_TX_TIMEOUT_MS ||
                ++waitLoops >= MOTOR_TX_WAIT_LOOPS)
                return HAL_TIMEOUT;
        }
    }
    taskENTER_CRITICAL();
    completedMailboxes &= ~mask;
    failedMailboxes &= ~mask;
    taskEXIT_CRITICAL();
    return HAL_OK;
}

void Motor_ResetFeedback(void)
{
    taskENTER_CRITICAL();
    g_motor_left.speedValid = g_motor_right.speedValid = 0U;
    g_motor_left.positionValid = g_motor_right.positionValid = 0U;
    taskEXIT_CRITICAL();
    g_motor_bus.feedbackCursor = 0U;
}

/* One query per call: left speed, right speed, left position, right position. */
HAL_StatusTypeDef MotorBus_PollFeedback(void)
{
    uint8_t slot = g_motor_bus.feedbackCursor;
    HAL_StatusTypeDef status = Motor_RequestFeedback(
        slot & 1U ? &g_motor_right : &g_motor_left,
        slot < 2U ? MOTOR_CMD_SPEED_QUERY : MOTOR_CMD_POSITION_QUERY);
    if (status == HAL_OK)
        g_motor_bus.feedbackCursor = (uint8_t)((slot + 1U) % 4U);
    return status;
}

void Motor_Init(Motor_TypeDef *motor, uint8_t address)
{
    uint8_t index;
    if (motor == NULL || address == 0U) return;
    memset(motor, 0, sizeof(*motor));
    motor->address = address;
    motor->forwardSign = address == MOTOR_LEFT_ID ? MOTOR_LEFT_SIGN : MOTOR_RIGHT_SIGN;
    for (index = 0U; index < 2U; index++)
    {
        if (registeredMotors[index] == NULL)
        {
            registeredMotors[index] = motor;
            break;
        }
    }
}

HAL_StatusTypeDef Motor_Enable(Motor_TypeDef *motor, uint8_t enable)
{
    const uint8_t command[] = {0U, MOTOR_CMD_ENABLE, 0xABU, enable ? 1U : 0U, 0U, MOTOR_FRAME_END};
    return Motor_Send(motor, command, sizeof(command));
}

HAL_StatusTypeDef Motor_SetSpeed(Motor_TypeDef *motor, int16_t rpm)
{
    uint16_t magnitude;
    uint8_t command[] = {0U, MOTOR_CMD_SPEED, 0U, 0U, 0U, MOTOR_ACCELERATION, 0U, MOTOR_FRAME_END};
    HAL_StatusTypeDef status;
    if (rpm > MOTOR_MAX_RPM) rpm = MOTOR_MAX_RPM;
    if (rpm < -MOTOR_MAX_RPM) rpm = -MOTOR_MAX_RPM;
    magnitude = (uint16_t)(rpm < 0 ? -rpm : rpm);
    command[2] = rpm < 0 ? 1U : 0U;
    command[3] = (uint8_t)(magnitude >> 8);
    command[4] = (uint8_t)magnitude;
    status = Motor_Send(motor, command, sizeof(command));
    if (status == HAL_OK) motor->commandedRpm = rpm;
    return status;
}

HAL_StatusTypeDef Motor_SetCurrent(Motor_TypeDef *motor, int16_t currentMa)
{
    uint16_t magnitude;
    uint16_t speedLimit = MOTOR_CURRENT_MAX_RPM * 10U;
    uint8_t command[] = {0U, 0xC5U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, MOTOR_FRAME_END};
    HAL_StatusTypeDef status;
    if (currentMa > MOTOR_CURRENT_LIMIT_MA) currentMa = MOTOR_CURRENT_LIMIT_MA;
    if (currentMa < -MOTOR_CURRENT_LIMIT_MA) currentMa = -MOTOR_CURRENT_LIMIT_MA;
    magnitude = (uint16_t)(currentMa < 0 ? -currentMa : currentMa);
    command[2] = currentMa < 0 ? 1U : 0U;
    command[3] = (uint8_t)(MOTOR_CURRENT_SLOPE_MA_S >> 8);
    command[4] = (uint8_t)MOTOR_CURRENT_SLOPE_MA_S;
    command[5] = (uint8_t)(magnitude >> 8);
    command[6] = (uint8_t)magnitude;
    command[8] = (uint8_t)(speedLimit >> 8);
    command[9] = (uint8_t)speedLimit;
    status = Motor_Send(motor, command, sizeof(command));
    if (status == HAL_OK) motor->commandedCurrentMa = currentMa;
    return status;
}

HAL_StatusTypeDef Motor_SetHeartbeat(Motor_TypeDef *motor, uint32_t timeoutMs)
{
    uint8_t command[] = {0U, MOTOR_CMD_HEARTBEAT, 0x38U, 0U, 0U, 0U, 0U, 0U, MOTOR_FRAME_END};
    command[4] = (uint8_t)(timeoutMs >> 24);
    command[5] = (uint8_t)(timeoutMs >> 16);
    command[6] = (uint8_t)(timeoutMs >> 8);
    command[7] = (uint8_t)timeoutMs;
    return Motor_Send(motor, command, sizeof(command));
}

HAL_StatusTypeDef Motor_SetBalanceCurrent(Motor_TypeDef *motor, int16_t currentMa)
{
    uint16_t magnitude;
    uint8_t command[] = {0U, MOTOR_CMD_CURRENT, 0U, 0xFFU, 0xFFU, 0U, 0U, 0U, MOTOR_FRAME_END};
    HAL_StatusTypeDef status;
    if (currentMa > MOTOR_CURRENT_LIMIT_MA) currentMa = MOTOR_CURRENT_LIMIT_MA;
    if (currentMa < -MOTOR_CURRENT_LIMIT_MA) currentMa = -MOTOR_CURRENT_LIMIT_MA;
    magnitude = (uint16_t)(currentMa < 0 ? -currentMa : currentMa);
    command[2] = currentMa < 0 ? 1U : 0U;
    command[5] = (uint8_t)(magnitude >> 8);
    command[6] = (uint8_t)magnitude;
    status = Motor_Send(motor, command, sizeof(command));
    if (status == HAL_OK)
    {
        motor->commandedCurrentMa = currentMa;
        motor->currentSubmittedAt = HAL_GetTick();
    }
    return status;
}

HAL_StatusTypeDef Motor_RequestFeedback(Motor_TypeDef *motor, uint8_t function)
{
    uint8_t command[] = {0U, function, MOTOR_FRAME_END};
    Motor_FeedbackTypeDef *feedback;
    HAL_StatusTypeDef status;
    if (motor == NULL || (function != MOTOR_CMD_SPEED_QUERY && function != MOTOR_CMD_POSITION_QUERY)) return HAL_ERROR;
    feedback = &motor->feedback[function == MOTOR_CMD_SPEED_QUERY ? 0U : 1U];
    feedback->requestedAt = HAL_GetTick();
    status = Motor_Send(motor, command, sizeof(command));
    if (status == HAL_OK) feedback->requestCount++;
    else
    {
        if (status != HAL_BUSY) feedback->txFailureCount++;
    }
    return status;
}

HAL_StatusTypeDef Motor_StartBalance(void)
{
    HAL_StatusTypeDef leftStatus;
    HAL_StatusTypeDef rightStatus;
    Motor_ResetFeedback();
    g_motor_left.commandedCurrentMa = 0;
    g_motor_right.commandedCurrentMa = 0;
    leftStatus = Motor_SetHeartbeat(&g_motor_left, MOTOR_HEARTBEAT_MS);
    rightStatus = Motor_SetHeartbeat(&g_motor_right, MOTOR_HEARTBEAT_MS);
    if (leftStatus != HAL_OK || rightStatus != HAL_OK) return HAL_ERROR;
    leftStatus = Motor_Enable(&g_motor_left, 1U);
    rightStatus = Motor_Enable(&g_motor_right, 1U);
    return leftStatus == HAL_OK && rightStatus == HAL_OK ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef Motor_Stop(Motor_TypeDef *motor)
{
    const uint8_t command[] = {0U, MOTOR_CMD_STOP, 0x98U, 0U, MOTOR_FRAME_END};
    HAL_StatusTypeDef status = Motor_Send(motor, command, sizeof(command));
    if (status == HAL_OK)
    {
        motor->commandedRpm = 0;
        motor->commandedCurrentMa = 0;
    }
    return status;
}

static HAL_StatusTypeDef Motor_StopOrDisableRetry(Motor_TypeDef *motor, uint8_t disable)
{
    uint8_t attempt;
    HAL_StatusTypeDef status = HAL_ERROR;
    for (attempt = 0U; attempt < MOTOR_STOP_TX_ATTEMPTS; attempt++)
    {
        status = disable ? Motor_Enable(motor, 0U) : Motor_Stop(motor);
        if (status == HAL_OK) return status;
        if (attempt + 1U < MOTOR_STOP_TX_ATTEMPTS) HAL_Delay(2U);
    }
    return status;
}

/** @brief Abort queued TX, stop both motors, then disable both drivers. */
HAL_StatusTypeDef Motor_StopPair(void)
{
    HAL_StatusTypeDef leftStop;
    HAL_StatusTypeDef rightStop;
    HAL_StatusTypeDef leftDisable;
    HAL_StatusTypeDef rightDisable;
    if (MotorBus_CancelPending() != HAL_OK) return HAL_ERROR;
    leftStop = Motor_StopOrDisableRetry(&g_motor_left, 0U);
    rightStop = Motor_StopOrDisableRetry(&g_motor_right, 0U);
    leftDisable = Motor_StopOrDisableRetry(&g_motor_left, 1U);
    rightDisable = Motor_StopOrDisableRetry(&g_motor_right, 1U);
    return leftStop == HAL_OK && rightStop == HAL_OK &&
           leftDisable == HAL_OK && rightDisable == HAL_OK ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef Motor_SetPosition(Motor_TypeDef *motor, uint8_t direction,
                                   uint16_t rpm, uint8_t acceleration,
                                   uint32_t pulses, uint8_t absolute)
{
    uint8_t command[] = {0U, 0xFDU, direction, 0U, 0U, acceleration,
                        0U, 0U, 0U, 0U, absolute, 0U, MOTOR_FRAME_END};
    if (direction > 1U || absolute > 1U || rpm > MOTOR_MAX_RPM) return HAL_ERROR;
    command[3] = (uint8_t)(rpm >> 8);
    command[4] = (uint8_t)rpm;
    command[6] = (uint8_t)(pulses >> 24);
    command[7] = (uint8_t)(pulses >> 16);
    command[8] = (uint8_t)(pulses >> 8);
    command[9] = (uint8_t)pulses;
    return Motor_Send(motor, command, sizeof(command));
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *handle)
{
    if (handle == motorCan) completedMailboxes |= CAN_TX_MAILBOX0;
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *handle)
{
    if (handle == motorCan) completedMailboxes |= CAN_TX_MAILBOX1;
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *handle)
{
    if (handle == motorCan) completedMailboxes |= CAN_TX_MAILBOX2;
}

void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *handle)
{
    if (handle == motorCan) failedMailboxes |= CAN_TX_MAILBOX0;
}

void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *handle)
{
    if (handle == motorCan) failedMailboxes |= CAN_TX_MAILBOX1;
}

void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *handle)
{
    if (handle == motorCan) failedMailboxes |= CAN_TX_MAILBOX2;
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *handle)
{
    uint32_t error;
    uint8_t index;
    const uint32_t arbitration[3] = {HAL_CAN_ERROR_TX_ALST0, HAL_CAN_ERROR_TX_ALST1, HAL_CAN_ERROR_TX_ALST2};
    const uint32_t transmit[3] = {HAL_CAN_ERROR_TX_TERR0, HAL_CAN_ERROR_TX_TERR1, HAL_CAN_ERROR_TX_TERR2};
    if (handle != motorCan) return;
    error = HAL_CAN_GetError(handle);
    for (index = 0U; index < 3U; index++)
    {
        if (error & arbitration[index]) g_motor_bus.arbitrationLossCount++;
        if (error & (arbitration[index] | transmit[index]))
        {
            mailboxErrors[index] |= error & (arbitration[index] | transmit[index]);
            failedMailboxes |= 1UL << index;
        }
    }
    if (error & HAL_CAN_ERROR_RX_FOV1) g_motor_bus.rxOverrunCount++;
    HAL_CAN_ResetError(handle);
}

static void Motor_FeedbackReceived(Motor_TypeDef *motor, uint8_t index)
{
    Motor_FeedbackTypeDef *feedback = &motor->feedback[index];
    uint32_t now = HAL_GetTick();
    feedback->replyCount++;
    feedback->receivedAt = now;

}

/* ISR validates frames and updates measurements only; no control or TX waits. */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *handle)
{
    CAN_RxHeaderTypeDef header;
    uint8_t payload[8];
    uint8_t index;
    uint8_t offset;
    if (handle != motorCan) return;
    while (HAL_CAN_GetRxFifoFillLevel(handle, CAN_RX_FIFO1) != 0U)
    {
        if (HAL_CAN_GetRxMessage(handle, CAN_RX_FIFO1, &header, payload) != HAL_OK) break;
        if (header.IDE != CAN_ID_EXT || header.RTR != CAN_RTR_DATA || header.DLC > 8U) continue;
        for (index = 0U; index < 2U; index++)
        {
            Motor_TypeDef *motor = registeredMotors[index];
            if (motor != NULL && (header.ExtId >> 8) == motor->address)
            {
                for (offset = 0U; offset < header.DLC; offset++) motor->lastRxData[offset] = payload[offset];
                motor->lastRxLength = (uint8_t)header.DLC;
                motor->rxCount++;
                motor->lastRxAt = HAL_GetTick();
                motor->lastRxPacket = header.ExtId & 0xFFU;
                if (header.DLC && (payload[0] == MOTOR_CMD_SPEED_QUERY || payload[0] == MOTOR_CMD_POSITION_QUERY) &&
                    !((header.ExtId & 0xFFU) == 0U &&
                      ((payload[0] == MOTOR_CMD_SPEED_QUERY && header.DLC == 5U && payload[1] <= 1U && payload[4] == MOTOR_FRAME_END) ||
                       (payload[0] == MOTOR_CMD_POSITION_QUERY && header.DLC == 7U && payload[1] <= 1U && payload[6] == MOTOR_FRAME_END))))
                    motor->malformedRxCount++;
                if ((header.ExtId & 0xFFU) == 0U && header.DLC == 5U &&
                    payload[0] == MOTOR_CMD_SPEED_QUERY && payload[1] <= 1U && payload[4] == MOTOR_FRAME_END)
                {
                    uint16_t magnitude = ((uint16_t)payload[2] << 8) | payload[3];
                    motor->measuredRpm = (float)magnitude * (payload[1] ? -0.1f : 0.1f);
                    motor->feedbackSpeedRpm = motor->measuredRpm;
                    motor->speedUpdatedAt = HAL_GetTick();
                    motor->feedbackSpeedAt = motor->speedUpdatedAt;
                    motor->feedbackFrameCount++;
                    motor->speedValid = 1U;
                    Motor_FeedbackReceived(motor, 0U);
                }
                if ((header.ExtId & 0xFFU) == 0U && header.DLC == 7U &&
                    payload[0] == MOTOR_CMD_POSITION_QUERY && payload[1] <= 1U && payload[6] == MOTOR_FRAME_END)
                {
                    uint32_t magnitude = ((uint32_t)payload[2] << 24) |
                        ((uint32_t)payload[3] << 16) | ((uint32_t)payload[4] << 8) | payload[5];
                    motor->measuredPositionDeg = (float)magnitude * (payload[1] ? -0.1f : 0.1f);
                    motor->feedbackPositionDeg = motor->measuredPositionDeg;
                    motor->positionUpdatedAt = HAL_GetTick();
                    motor->feedbackPositionAt = motor->positionUpdatedAt;
                    motor->feedbackFrameCount++;
                    motor->positionValid = 1U;
                    Motor_FeedbackReceived(motor, 1U);
                }
                if ((header.ExtId & 0xFFU) == 0U && header.DLC == 3U && payload[2] == MOTOR_FRAME_END)
                {
                    motor->replyFunction = payload[0];
                    motor->replyCode = payload[1];
                    motor->replyCount++;
                    if (payload[0] == MOTOR_CMD_CURRENT || payload[0] == 0xC5U)
                    {
                        if (payload[1] == 0x02U)
                        {
                            if (!motor->commandRejected) motor->currentReplyCode = payload[1];
                            motor->currentAcceptedAt = HAL_GetTick();
                            motor->currentAccepted = 1U;
                            motor->currentReplyCount++;
                        }
                        else if (payload[1] == 0xE2U || payload[1] == 0xEEU ||
                                 payload[1] == 0x12U || payload[1] == 0x22U)
                        {
                            motor->currentReplyCode = payload[1];
                            motor->commandRejected = 1U;
                            motor->lastRejectedFunction = payload[0];
                        }
                    }
                }
            }
        }
    }
}



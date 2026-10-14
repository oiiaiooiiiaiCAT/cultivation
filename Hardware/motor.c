#include "motor.h"
#include "FreeRTOS.h"
#include "task.h"

Motor_TypeDef g_motor_left;
Motor_TypeDef g_motor_right;
Motor_BalanceDiagnosticTypeDef g_motor_balance_diag;
Motor_BusTypeDef g_motor_bus;
Motor_CurrentTestTypeDef g_motor_current_test = {
    .leftCurrentMa = MOTOR_CURRENT_TEST_LEFT_MA,
    .rightCurrentMa = MOTOR_CURRENT_TEST_RIGHT_MA
};
#include <string.h>

static CAN_HandleTypeDef *motorCan;
static Motor_TypeDef *registeredMotors[2];
static volatile uint32_t completedMailboxes;
static volatile uint32_t failedMailboxes;

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
    if (motor == NULL || motorCan == NULL || motor->address == 0U || length < 3U)
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
        g_motor_bus.tx[0].error = g_motor_bus.tx[1].error = g_motor_bus.tx[2].error = 0U;
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
                motor->commandTxError = g_motor_bus.tx[slot].error;
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

static HAL_StatusTypeDef Motor_SendConfirmed(Motor_TypeDef *motor,
                                             const uint8_t *command, uint8_t length)
{
    uint32_t started;
    HAL_StatusTypeDef status;
    motor->confirmFunction = command[1];
    motor->confirmCode = 0U;
    motor->confirmReceived = 0U;
    motor->confirmWaiting = 1U;
    status = Motor_Send(motor, command, length);
    if (status == HAL_OK)
    {
        started = HAL_GetTick();
        status = HAL_TIMEOUT;
        while (HAL_GetTick() - started < MOTOR_CONFIRM_TIMEOUT_MS)
        {
            if (motor->confirmReceived)
            {
                status = motor->confirmCode == 0x02U ? HAL_OK : HAL_ERROR;
                break;
            }
        }
    }
    motor->confirmWaiting = 0U;
    motor->lastStatus = status;
    return status;
}

static HAL_StatusTypeDef Motor_StartCommandConfirmed(Motor_TypeDef *motor,
                                                     const uint8_t *command, uint8_t length)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    motor->confirmAttempts = 0U;
    while (motor->confirmAttempts < MOTOR_START_CONFIRM_ATTEMPTS)
    {
        motor->confirmAttempts++;
        status = Motor_SendConfirmed(motor, command, length);
        if (status != HAL_TIMEOUT || motor->commandTxError != HAL_CAN_ERROR_NONE) break;
        if (motor->confirmAttempts < MOTOR_START_CONFIRM_ATTEMPTS) HAL_Delay(2U);
    }
    return status;
}

static HAL_StatusTypeDef Motor_EnableConfirmed(Motor_TypeDef *motor)
{
    const uint8_t command[] = {0U, 0xF3U, 0xABU, 1U, 0U, 0x6BU};
    HAL_StatusTypeDef status = Motor_StartCommandConfirmed(motor, command, sizeof(command));
    motor->enableAttempts = motor->confirmAttempts;
    return status;
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
    for (index = 0U; index < 3U; index++)
        if (g_motor_bus.tx[index].active) mask |= 1UL << index;
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
    for (index = 0U; index < 3U; index++) g_motor_bus.tx[index].active = 0U;
    completedMailboxes &= ~mask;
    failedMailboxes &= ~mask;
    taskEXIT_CRITICAL();
    g_motor_left.feedback[0].pending = g_motor_left.feedback[1].pending = 0U;
    g_motor_right.feedback[0].pending = g_motor_right.feedback[1].pending = 0U;
    return HAL_OK;
}

void Motor_ResetFeedback(void)
{
    taskENTER_CRITICAL();
    g_motor_left.speedValid = g_motor_right.speedValid = 0U;
    g_motor_left.positionValid = g_motor_right.positionValid = 0U;
    g_motor_left.feedback[0].pending = g_motor_left.feedback[1].pending = 0U;
    g_motor_right.feedback[0].pending = g_motor_right.feedback[1].pending = 0U;
    taskEXIT_CRITICAL();
    g_motor_bus.feedbackCursor = 0U;
}

HAL_StatusTypeDef MotorBus_Service(void)
{
    return HAL_OK;
}

HAL_StatusTypeDef MotorBus_PollFeedback(void)
{
    uint8_t slot = g_motor_bus.feedbackCursor;
    HAL_StatusTypeDef status = Motor_RequestFeedback(
        slot & 1U ? &g_motor_right : &g_motor_left,
        slot < 2U ? 0x35U : 0x36U);
    if (status == HAL_OK)
        g_motor_bus.feedbackCursor = (uint8_t)((slot + 1U) % 4U);
    return status;
}

void Motor_Init(Motor_TypeDef *motor, uint8_t address)
{
    uint8_t index;
    memset(motor, 0, sizeof(*motor));
    motor->address = address;
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
    const uint8_t command[] = {0U, 0xF3U, 0xABU, enable ? 1U : 0U, 0U, 0x6BU};
    return Motor_Send(motor, command, sizeof(command));
}

HAL_StatusTypeDef Motor_SetSpeed(Motor_TypeDef *motor, int16_t rpm)
{
    uint16_t magnitude;
    uint8_t command[] = {0U, 0xF6U, 0U, 0U, 0U, MOTOR_ACCELERATION, 0U, 0x6BU};
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
    uint8_t command[] = {0U, 0xC5U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0x6BU};
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
    uint8_t command[] = {0U, 0x68U, 0x38U, 0U, 0U, 0U, 0U, 0U, 0x6BU};
    command[4] = (uint8_t)(timeoutMs >> 24);
    command[5] = (uint8_t)(timeoutMs >> 16);
    command[6] = (uint8_t)(timeoutMs >> 8);
    command[7] = (uint8_t)timeoutMs;
    return Motor_Send(motor, command, sizeof(command));
}

HAL_StatusTypeDef Motor_SetBalanceCurrent(Motor_TypeDef *motor, int16_t currentMa)
{
    uint16_t magnitude;
    uint8_t command[] = {0U, 0xF5U, 0U, 0xFFU, 0xFFU, 0U, 0U, 0U, 0x6BU};
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
    uint8_t command[] = {0U, function, 0x6BU};
    Motor_FeedbackTypeDef *feedback;
    HAL_StatusTypeDef status;
    if (motor == NULL || (function != 0x35U && function != 0x36U)) return HAL_ERROR;
    feedback = &motor->feedback[function == 0x35U ? 0U : 1U];
    feedback->requestedAt = HAL_GetTick();
    status = Motor_Send(motor, command, sizeof(command));
    if (status == HAL_OK) feedback->requestCount++;
    else
    {
        feedback->pending = 0U;
        if (status != HAL_BUSY) feedback->txFailureCount++;
    }
    return status;
}

void Motor_RecordBalanceFailure(uint8_t faultCode, uint8_t stage, uint8_t function,
                               HAL_StatusTypeDef leftStatus, HAL_StatusTypeDef rightStatus,
                               uint8_t reason)
{
    uint32_t now = HAL_GetTick();
    uint8_t index;
    g_motor_balance_diag.capturedAt = now;
    g_motor_balance_diag.faultCode = faultCode;
    g_motor_balance_diag.stage = stage;
    g_motor_balance_diag.function = function;
    g_motor_balance_diag.reason = reason;
    g_motor_balance_diag.leftStatus = leftStatus;
    g_motor_balance_diag.rightStatus = rightStatus;
    g_motor_balance_diag.leftTxError = g_motor_left.commandTxError;
    g_motor_balance_diag.rightTxError = g_motor_right.commandTxError;
    g_motor_balance_diag.leftAckAgeMs = now - g_motor_left.currentAcceptedAt;
    g_motor_balance_diag.rightAckAgeMs = now - g_motor_right.currentAcceptedAt;
    g_motor_balance_diag.leftSpeedAgeMs = now - g_motor_left.speedUpdatedAt;
    g_motor_balance_diag.rightSpeedAgeMs = now - g_motor_right.speedUpdatedAt;
    g_motor_balance_diag.leftPositionAgeMs = now - g_motor_left.positionUpdatedAt;
    g_motor_balance_diag.rightPositionAgeMs = now - g_motor_right.positionUpdatedAt;
    g_motor_balance_diag.leftEnableAttempts = g_motor_left.enableAttempts;
    g_motor_balance_diag.rightEnableAttempts = g_motor_right.enableAttempts;
    g_motor_balance_diag.leftConfirmAttempts = g_motor_left.confirmAttempts;
    g_motor_balance_diag.rightConfirmAttempts = g_motor_right.confirmAttempts;
    for (index = 0U; index < 4U; index++)
    {
        Motor_TypeDef *motor = index & 1U ? &g_motor_right : &g_motor_left;
        Motor_FeedbackTypeDef *feedback = &motor->feedback[index < 2U ? 0U : 1U];
        g_motor_balance_diag.feedbackRequests[index] = feedback->requestCount;
        g_motor_balance_diag.feedbackReplies[index] = feedback->replyCount;
        g_motor_balance_diag.feedbackMisses[index] = feedback->missedReplyCount;
        g_motor_balance_diag.feedbackTxFailures[index] = feedback->txFailureCount;
    }
    if (stage <= MOTOR_BALANCE_STAGE_ZERO_CURRENT)
    {
        g_motor_balance_diag.leftReplyCode = g_motor_left.confirmCode;
        g_motor_balance_diag.rightReplyCode = g_motor_right.confirmCode;
    }
    else if (function == 0xF5U)
    {
        g_motor_balance_diag.leftReplyCode = g_motor_left.currentReplyCode;
        g_motor_balance_diag.rightReplyCode = g_motor_right.currentReplyCode;
    }
    else
    {
        g_motor_balance_diag.leftReplyCode = g_motor_left.replyCode;
        g_motor_balance_diag.rightReplyCode = g_motor_right.replyCode;
    }
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
    const uint8_t command[] = {0U, 0xFEU, 0x98U, 0U, 0x6BU};
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

HAL_StatusTypeDef Motor_StopPair(void)
{
    HAL_StatusTypeDef leftStop;
    HAL_StatusTypeDef rightStop;
    HAL_StatusTypeDef leftDisable;
    HAL_StatusTypeDef rightDisable;
    if (MotorBus_CancelPending() != HAL_OK)
    {
        g_motor_current_test.stopPending = 1U;
        return HAL_ERROR;
    }
    leftStop = Motor_StopOrDisableRetry(&g_motor_left, 0U);
    rightStop = Motor_StopOrDisableRetry(&g_motor_right, 0U);
    leftDisable = Motor_StopOrDisableRetry(&g_motor_left, 1U);
    rightDisable = Motor_StopOrDisableRetry(&g_motor_right, 1U);
    g_motor_current_test.lastStopAttempt = HAL_GetTick();
    g_motor_current_test.stopPending = leftStop != HAL_OK || rightStop != HAL_OK ||
                                      leftDisable != HAL_OK || rightDisable != HAL_OK;
    if (g_motor_current_test.stopPending)
    {
        g_motor_current_test.fault = 1U;
        if (g_motor_current_test.failureStage == 0U)
        {
            g_motor_current_test.failureStage = 5U;
            g_motor_current_test.failureLeftStopStatus = leftStop;
            g_motor_current_test.failureRightStopStatus = rightStop;
            g_motor_current_test.failureLeftDisableStatus = leftDisable;
            g_motor_current_test.failureRightDisableStatus = rightDisable;
            g_motor_current_test.failureLeftStatus = leftStop != HAL_OK ? leftStop : leftDisable;
            g_motor_current_test.failureRightStatus = rightStop != HAL_OK ? rightStop : rightDisable;
        }
    }
    return g_motor_current_test.stopPending ? HAL_ERROR : HAL_OK;
}

void Motor_CurrentTestStop(void)
{
    g_motor_current_test.active = 0U;
    g_motor_current_test.finished = 1U;
    Motor_StopPair();
}

void Motor_CurrentTestUpdate(uint8_t busReady)
{
    HAL_StatusTypeDef leftStatus;
    HAL_StatusTypeDef rightStatus;
    if (g_motor_current_test.finished)
    {
        if (busReady && g_motor_current_test.stopPending &&
            HAL_GetTick() - g_motor_current_test.lastStopAttempt >= 200U)
            Motor_CurrentTestStop();
        return;
    }
    if (!busReady)
    {
        g_motor_current_test.failureStage = 1U;
        g_motor_current_test.failureLeftStatus = HAL_ERROR;
        g_motor_current_test.failureRightStatus = HAL_ERROR;
        g_motor_current_test.fault = 1U;
        g_motor_current_test.finished = 1U;
        return;
    }
    if (g_motor_current_test.fault) goto failed;
    if (!g_motor_current_test.active)
    {
        leftStatus = Motor_SetHeartbeat(&g_motor_left, MOTOR_CURRENT_HEARTBEAT_MS);
        rightStatus = Motor_SetHeartbeat(&g_motor_right, MOTOR_CURRENT_HEARTBEAT_MS);
        if (leftStatus != HAL_OK || rightStatus != HAL_OK)
        {
            g_motor_current_test.failureStage = 2U;
            g_motor_current_test.failureLeftStatus = leftStatus;
            g_motor_current_test.failureRightStatus = rightStatus;
            goto failed;
        }
        leftStatus = Motor_EnableConfirmed(&g_motor_left);
        rightStatus = Motor_EnableConfirmed(&g_motor_right);
        if (leftStatus != HAL_OK || rightStatus != HAL_OK)
        {
            g_motor_current_test.failureStage = 3U;
            g_motor_current_test.failureLeftStatus = leftStatus;
            g_motor_current_test.failureRightStatus = rightStatus;
            goto failed;
        }
        g_motor_current_test.startedAt = HAL_GetTick();
        g_motor_current_test.active = 1U;
    }
    if (HAL_GetTick() - g_motor_current_test.startedAt >= MOTOR_CURRENT_TEST_DURATION_MS)
    {
        Motor_CurrentTestStop();
        return;
    }
    leftStatus = Motor_SetCurrent(&g_motor_left, g_motor_current_test.leftCurrentMa);
    rightStatus = Motor_SetCurrent(&g_motor_right, g_motor_current_test.rightCurrentMa);
    if (leftStatus == HAL_OK && rightStatus == HAL_OK) return;
    if (g_motor_current_test.failureStage == 0U)
    {
        g_motor_current_test.failureStage = 4U;
        g_motor_current_test.failureLeftStatus = leftStatus;
        g_motor_current_test.failureRightStatus = rightStatus;
    }
failed:
    g_motor_current_test.fault = 1U;
    Motor_CurrentTestStop();
}

HAL_StatusTypeDef Motor_SetPosition(Motor_TypeDef *motor, uint8_t direction,
                                   uint16_t rpm, uint8_t acceleration,
                                   uint32_t pulses, uint8_t absolute)
{
    uint8_t command[] = {0U, 0xFDU, direction, 0U, 0U, acceleration,
                        0U, 0U, 0U, 0U, absolute, 0U, 0x6BU};
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
            g_motor_bus.tx[index].error |= error & (arbitration[index] | transmit[index]);
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
    if (feedback->pending)
    {
        feedback->lastRoundTripMs = now - feedback->requestedAt;
        if (feedback->lastRoundTripMs > feedback->maxRoundTripMs)
            feedback->maxRoundTripMs = feedback->lastRoundTripMs;
        feedback->pending = 0U;
    }
}

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
                if (header.DLC && (payload[0] == 0x35U || payload[0] == 0x36U) &&
                    !((header.ExtId & 0xFFU) == 0U &&
                      ((payload[0] == 0x35U && header.DLC == 5U && payload[1] <= 1U && payload[4] == 0x6BU) ||
                       (payload[0] == 0x36U && header.DLC == 7U && payload[1] <= 1U && payload[6] == 0x6BU))))
                    motor->malformedRxCount++;
                if ((header.ExtId & 0xFFU) == 0U && header.DLC == 5U &&
                    payload[0] == 0x35U && payload[1] <= 1U && payload[4] == 0x6BU)
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
                    payload[0] == 0x36U && payload[1] <= 1U && payload[6] == 0x6BU)
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
                if ((header.ExtId & 0xFFU) == 0U && header.DLC == 3U && payload[2] == 0x6BU)
                {
                    motor->replyFunction = payload[0];
                    motor->replyCode = payload[1];
                    motor->replyCount++;
                    if (motor->confirmWaiting && payload[0] == motor->confirmFunction &&
                        (payload[1] == 0x02U || payload[1] == 0x12U || payload[1] == 0x22U ||
                         payload[1] == 0xE2U || payload[1] == 0xEEU) &&
                        (!motor->confirmReceived || payload[1] != 0x02U))
                    {
                        motor->confirmCode = payload[1];
                        motor->confirmReceived = 1U;
                    }
                    if (payload[0] == 0xF5U || payload[0] == 0xC5U)
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
                    if (payload[0] == 0xC5U &&
                        (payload[1] == 0xE2U || payload[1] == 0xEEU))
                    {
                        if (g_motor_current_test.failureStage == 0U)
                            g_motor_current_test.failureStage = 4U;
                        if (motor == &g_motor_left)
                            g_motor_current_test.failureLeftStatus = HAL_ERROR;
                        else
                            g_motor_current_test.failureRightStatus = HAL_ERROR;
                        g_motor_current_test.fault = 1U;
                    }
                }
            }
        }
    }
}



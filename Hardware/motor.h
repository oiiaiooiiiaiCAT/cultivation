#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

#define MOTOR_LEFT_ID 1U
#define MOTOR_RIGHT_ID 2U
#define MOTOR_LEFT_SIGN -1
#define MOTOR_RIGHT_SIGN 1
#define MOTOR_MAX_RPM 120
#define MOTOR_ACCELERATION 10U
#define MOTOR_CURRENT_LIMIT_MA 1000
#define MOTOR_CURRENT_MAX_RPM 120U
#define MOTOR_CURRENT_SLOPE_MA_S 10000U
#define MOTOR_CURRENT_HEARTBEAT_MS 200U
#define MOTOR_HEARTBEAT_MS 200U
#define MOTOR_TX_TIMEOUT_MS 1U
#define MOTOR_TX_WAIT_LOOPS 10000U
#define MOTOR_STOP_TX_ATTEMPTS 2U
#define MOTOR_START_CONFIRM_ATTEMPTS 2U
#define MOTOR_CONFIRM_TIMEOUT_MS 20U
#define MOTOR_FEEDBACK_PERIOD_MS 4U
#define MOTOR_FEEDBACK_QUERY_INTERVAL_MS 4U
#define MOTOR_FEEDBACK_REPLY_TIMEOUT_MS 25U
#define MOTOR_FEEDBACK_WARN_AGE_MS 24U
#define MOTOR_CURRENT_TEST_ENABLED 0U
#define MOTOR_CURRENT_TEST_LEFT_MA 100
#define MOTOR_CURRENT_TEST_RIGHT_MA 100
#define MOTOR_CURRENT_TEST_DURATION_MS 3000U
#define MOTOR_BALANCE_STAGE_ZERO_CURRENT 2U
#define MOTOR_BALANCE_STAGE_CURRENT_TX 7U

typedef struct {
    volatile uint8_t pending;
    volatile uint32_t requestedAt;
    volatile uint32_t receivedAt;
    volatile uint32_t requestCount;
    volatile uint32_t replyCount;
    volatile uint32_t missedReplyCount;
    volatile uint32_t txFailureCount;
    volatile uint32_t lastRoundTripMs;
    volatile uint32_t maxRoundTripMs;
} Motor_FeedbackTypeDef;

typedef struct Motor_TypeDef Motor_TypeDef;
typedef struct {
    Motor_TypeDef *motor;
    uint8_t function;
    uint8_t active;
    uint32_t submittedAt;
    uint32_t error;
} Motor_AsyncTxTypeDef;
typedef struct {
    Motor_AsyncTxTypeDef tx[3];
    uint8_t feedbackCursor;
    uint32_t submittedCount;
    uint32_t completedCount;
    uint32_t failureCount;
    uint32_t busyCount;
    uint32_t queryFailureCount;
    uint32_t arbitrationLossCount;
    uint32_t rxOverrunCount;
    uint8_t lastFailureAddress;
    uint8_t lastFailureFunction;
    HAL_StatusTypeDef lastFailureStatus;
    uint32_t lastFailureError;
} Motor_BusTypeDef;

typedef struct {
    uint32_t capturedAt;
    uint8_t faultCode;
    uint8_t stage;
    uint8_t function;
    uint8_t reason;
    HAL_StatusTypeDef leftStatus;
    HAL_StatusTypeDef rightStatus;
    uint32_t leftTxError, rightTxError;
    uint32_t leftAckAgeMs, rightAckAgeMs;
    uint32_t leftSpeedAgeMs, rightSpeedAgeMs;
    uint32_t leftPositionAgeMs, rightPositionAgeMs;
    uint32_t leftEnableAttempts, rightEnableAttempts;
    uint32_t leftConfirmAttempts, rightConfirmAttempts;
    uint8_t leftReplyCode, rightReplyCode;
    uint32_t feedbackRequests[4], feedbackReplies[4];
    uint32_t feedbackMisses[4], feedbackTxFailures[4];
} Motor_BalanceDiagnosticTypeDef;

typedef struct {
    int16_t leftCurrentMa, rightCurrentMa;
    uint8_t active, finished, fault, stopPending;
    uint8_t failureStage;
    HAL_StatusTypeDef failureLeftStatus, failureRightStatus;
    HAL_StatusTypeDef failureLeftStopStatus, failureRightStopStatus;
    HAL_StatusTypeDef failureLeftDisableStatus, failureRightDisableStatus;
    uint32_t startedAt, lastStopAttempt;
} Motor_CurrentTestTypeDef;

typedef struct Motor_TypeDef {
    uint8_t address;
    int8_t forwardSign;
    int16_t commandedRpm;
    int16_t commandedCurrentMa;
    volatile float measuredRpm;
    volatile float measuredPositionDeg;
    volatile float feedbackSpeedRpm;
    volatile float feedbackPositionDeg;
    volatile uint32_t speedUpdatedAt, positionUpdatedAt;
    volatile uint32_t feedbackSpeedAt, feedbackPositionAt;
    volatile uint32_t feedbackFrameCount;
    volatile uint8_t speedValid, positionValid;
    Motor_FeedbackTypeDef feedback[2];
    volatile uint32_t rxCount, lastRxAt, lastRxPacket;
    volatile uint8_t lastRxLength, lastRxData[8];
    volatile uint8_t replyFunction, replyCode;
    volatile uint32_t replyCount;
    HAL_StatusTypeDef lastStatus;
    uint32_t commandTxError, lastTxError, lastTxFailedFunction, txFailureCount;
    uint32_t enableAttempts, confirmAttempts;
    uint8_t confirmFunction, confirmCode, confirmReceived, confirmWaiting;
    uint8_t currentAccepted, commandRejected;
    uint32_t currentAcceptedAt, currentSubmittedAt, currentReplyCount;
    uint8_t currentReplyCode, lastRejectedFunction;
    uint32_t malformedRxCount;
} Motor_TypeDef;

extern Motor_TypeDef g_motor_left, g_motor_right;
extern Motor_BalanceDiagnosticTypeDef g_motor_balance_diag;
extern Motor_BusTypeDef g_motor_bus;
extern Motor_CurrentTestTypeDef g_motor_current_test;

HAL_StatusTypeDef MotorBus_Init(CAN_HandleTypeDef *handle);
void Motor_Init(Motor_TypeDef *motor, uint8_t address);
void Motor_ResetFeedback(void);
HAL_StatusTypeDef MotorBus_Service(void);
HAL_StatusTypeDef MotorBus_CancelPending(void);
HAL_StatusTypeDef Motor_Enable(Motor_TypeDef *motor, uint8_t enable);
HAL_StatusTypeDef Motor_SetSpeed(Motor_TypeDef *motor, int16_t rpm);
HAL_StatusTypeDef Motor_SetCurrent(Motor_TypeDef *motor, int16_t currentMa);
HAL_StatusTypeDef Motor_SetBalanceCurrent(Motor_TypeDef *motor, int16_t currentMa);
HAL_StatusTypeDef Motor_SetHeartbeat(Motor_TypeDef *motor, uint32_t timeoutMs);
HAL_StatusTypeDef Motor_RequestFeedback(Motor_TypeDef *motor, uint8_t function);
HAL_StatusTypeDef MotorBus_PollFeedback(void);
HAL_StatusTypeDef Motor_StartBalance(void);
HAL_StatusTypeDef Motor_Stop(Motor_TypeDef *motor);
HAL_StatusTypeDef Motor_StopPair(void);
void Motor_RecordBalanceFailure(uint8_t faultCode, uint8_t stage, uint8_t function, HAL_StatusTypeDef leftStatus, HAL_StatusTypeDef rightStatus, uint8_t reason);
void Motor_CurrentTestStop(void);
void Motor_CurrentTestUpdate(uint8_t busReady);
HAL_StatusTypeDef Motor_SetPosition(Motor_TypeDef *motor, uint8_t direction, uint16_t rpm, uint8_t acceleration, uint32_t pulses, uint8_t absolute);

#endif



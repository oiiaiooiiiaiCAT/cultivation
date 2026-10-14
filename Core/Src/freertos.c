/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "balance.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId mpuTaskHandle;
osThreadId motorCtrlTaskHandle;
osThreadId oledTaskHandle;
osThreadId handTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void MPU_Task(void const * argument);
void MotorCtrl_Task(void const * argument);
void OLED_DisplayTask(void const * argument);
void HAND_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 64);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of mpuTask */
  osThreadDef(mpuTask, MPU_Task, osPriorityHigh, 0, 128);
  mpuTaskHandle = osThreadCreate(osThread(mpuTask), NULL);

  /* definition and creation of motorCtrlTask */
  osThreadDef(motorCtrlTask, MotorCtrl_Task, osPriorityAboveNormal, 0, 208);
  motorCtrlTaskHandle = osThreadCreate(osThread(motorCtrlTask), NULL);

  /* definition and creation of oledTask */
  osThreadDef(oledTask, OLED_DisplayTask, osPriorityBelowNormal, 0, 112);
  oledTaskHandle = osThreadCreate(osThread(oledTask), NULL);

  /* definition and creation of handTask */
  osThreadDef(handTask, HAND_Task, osPriorityAboveNormal, 0, 112);
  handTaskHandle = osThreadCreate(osThread(handTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  if (defaultTaskHandle == NULL || mpuTaskHandle == NULL ||
      motorCtrlTaskHandle == NULL || oledTaskHandle == NULL || handTaskHandle == NULL)
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_MPU_Task */
/**
  * @brief  Function implementing the mpuTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_MPU_Task */
void MPU_Task(void const * argument)
{
  /* USER CODE BEGIN MPU_Task */
  (void)argument;
  for(;;)
  {
    Car_ReadMPU();
    osDelay(5);
  }
  /* USER CODE END MPU_Task */
}

/* USER CODE BEGIN Header_MotorCtrl_Task */
/**
* @brief Function implementing the motorCtrlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorCtrl_Task */
void MotorCtrl_Task(void const * argument)
{
  /* USER CODE BEGIN MotorCtrl_Task */
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(LQR_PERIOD_MS);
  (void)argument;
  for(;;)
  {
    Car_Control();
    if (xTaskGetTickCount() - lastWake >= period)
    {
      g_lqr.overrunCount++;
      lastWake = xTaskGetTickCount();
    }
    vTaskDelayUntil(&lastWake, period);
  }
  /* USER CODE END MotorCtrl_Task */
}

/* USER CODE BEGIN Header_OLED_DisplayTask */
/**
* @brief Function implementing the oledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OLED_DisplayTask */
void OLED_DisplayTask(void const * argument)
{
  /* USER CODE BEGIN OLED_DisplayTask */
  (void)argument;
  for(;;)
  {
    Car_Display();
    osDelay(100);
  }
  /* USER CODE END OLED_DisplayTask */
}

/* USER CODE BEGIN Header_HAND_Task */
/**
* @brief Function implementing the handTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HAND_Task */
void HAND_Task(void const * argument)
{
  /* USER CODE BEGIN HAND_Task */
  PS2_StateTypeDef state;
  (void)argument;
  for(;;)
  {
    vTaskSuspendAll();
    PS2_Poll(&state);
    xTaskResumeAll();
    Car_SetJoystick(&state);
    osDelay(30);
  }
  /* USER CODE END HAND_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


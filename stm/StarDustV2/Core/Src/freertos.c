/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stdio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


    struct state
    {
      float ivme;
      int durum;
      float presure;
      float irtifa;
      
    };
    struct state state0;
    struct state state1;
    struct state state2;
    struct state state3;
    struct state state4;
    struct state state5;
    struct state state6;
    


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
osThreadId StateKontrolHandle;
osThreadId veriHandle;
osThreadId hataDHandle;
osThreadId flasMemoHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StateKontrolF(void const * argument);
void veriF(void const * argument);
void hataDF(void const * argument);
void flasMemoF(void const * argument);

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of StateKontrol */
  osThreadDef(StateKontrol, StateKontrolF, osPriorityIdle, 0, 128);
  StateKontrolHandle = osThreadCreate(osThread(StateKontrol), NULL);

  /* definition and creation of veri */
  osThreadDef(veri, veriF, osPriorityIdle, 0, 128);
  veriHandle = osThreadCreate(osThread(veri), NULL);

  /* definition and creation of hataD */
  osThreadDef(hataD, hataDF, osPriorityIdle, 0, 128);
  hataDHandle = osThreadCreate(osThread(hataD), NULL);

  /* definition and creation of flasMemo */
  osThreadDef(flasMemo, flasMemoF, osPriorityIdle, 0, 128);
  flasMemoHandle = osThreadCreate(osThread(flasMemo), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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

/* USER CODE BEGIN Header_StateKontrolF */
/**
* @brief Function implementing the StateKontrol thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StateKontrolF */
void StateKontrolF(void const * argument)
{
  /* USER CODE BEGIN StateKontrolF */
  /* Infinite loop */
  for(;;)
  {

    state0.durum=0;
    
    if(state1.ivme >= 4.5 ) {
        state0.durum = 1;

    }

    if (state1.durum==1){
      if(state2.ivme >= 0 ){
        state2.durum=1;
      }
    }





    osDelay(1);
  }
  /* USER CODE END StateKontrolF */
}

/* USER CODE BEGIN Header_veriF */
/**
* @brief Function implementing the veri thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_veriF */
void veriF(void const * argument)
{
  /* USER CODE BEGIN veriF */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END veriF */
}

/* USER CODE BEGIN Header_hataDF */
/**
* @brief Function implementing the hataD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_hataDF */
void hataDF(void const * argument)
{
  /* USER CODE BEGIN hataDF */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END hataDF */
}

/* USER CODE BEGIN Header_flasMemoF */
/**
* @brief Function implementing the flasMemo thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_flasMemoF */
void flasMemoF(void const * argument)
{
  /* USER CODE BEGIN flasMemoF */
  /* Infinite loop */
  for(;;)
  {

    
    osDelay(1);
  }
  /* USER CODE END flasMemoF */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "DJ_d16.h"
#include "DJ_Motor.h"
#include "duolun.h"
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
float x,y,w;

/* USER CODE END Variables */
/* Definitions for motor */
osThreadId_t motorHandle;
const osThreadAttr_t motor_attributes = {
  .name = "motor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal6,
};
/* Definitions for dipan */
osThreadId_t dipanHandle;
const osThreadAttr_t dipan_attributes = {
  .name = "dipan",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal7,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void motor_f(void *argument);
void dipan_f(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of motor */
  motorHandle = osThreadNew(motor_f, NULL, &motor_attributes);

  /* creation of dipan */
  dipanHandle = osThreadNew(dipan_f, NULL, &dipan_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_motor_f */
/**
  * @brief  Function implementing the motor thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_motor_f */
void motor_f(void *argument)
{
  /* USER CODE BEGIN motor_f */
  /* Infinite loop */
  for(;;)
  {
		Steer_Calculate_speed(&chassis_duolun,x,y,w);
    osDelay(5);
  }
  /* USER CODE END motor_f */
}

/* USER CODE BEGIN Header_dipan_f */
/**
* @brief Function implementing the dipan thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_dipan_f */
void dipan_f(void *argument)
{
  /* USER CODE BEGIN dipan_f */
  /* Infinite loop */
  for(;;)
  {
		y = ((float)yaokong.ch2-1024)/660*2000;
		x = ((float)yaokong.ch3-1024)/660*2000;
	if(yaokong.s1 == 1) w = 800;
	else if(yaokong.s1 == 2) w = -800;
	else w = 0;		
	if(yaokong.ch2<364) x = 0;
	if(yaokong.ch2>1684) x = 0;
	if(yaokong.ch0<364) y = 0;
	if(yaokong.ch0>1684)y = 0;
		Steer_Calculate_turn(&chassis_duolun,x,y,w);
    osDelay(5);
  }
  /* USER CODE END dipan_f */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


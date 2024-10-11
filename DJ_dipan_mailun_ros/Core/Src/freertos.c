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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct 
{
  float kp;
  float ki;
  float kd;
  /* data */
}Pid;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
	float x,y,z;
	int16_t motor1,motor2,motor3,motor4;
/* USER CODE END Variables */
/* Definitions for dipan */
osThreadId_t dipanHandle;
const osThreadAttr_t dipan_attributes = {
  .name = "dipan",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal7,
};
/* Definitions for yaokong */
osThreadId_t yaokongHandle;
const osThreadAttr_t yaokong_attributes = {
  .name = "yaokong",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal1,
};
/* Definitions for motor_state */
osThreadId_t motor_stateHandle;
const osThreadAttr_t motor_state_attributes = {
  .name = "motor_state",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
int16_t pid_v(float target,float current)
{
//	int i =1;
//	if(target <0)
//	{
//		target = target;
//		current = -current;
//		i = 0;
//	};
  Pid pid_v;
  pid_v.kp = 15;
  pid_v.ki = 0.7;
  pid_v.kd = 3;
  
  float c_err,l_err,i_err;
//  c_err = (yaokong.ch3-1024)/660*300 - motor_chassis.speed_rpm;
	c_err = target -current ;
  int16_t out_v = pid_v.kp * c_err + pid_v.kd *(c_err-l_err) + pid_v.ki *i_err;
  l_err = c_err;
  i_err += c_err;
	if(i_err >10000) i_err = 10000; 
	if(i_err <-10000) i_err = 10000;
	
//	if(i == 0) out = -out;
	return out_v;
};
void dipan_sport()
{
//	int16_t x,y,z;
//	int16_t motor1,motor2,motor3,motor4;
	x = ((float)yaokong.ch2-1024)/660*2000;
	y = ((float)yaokong.ch3-1024)/660*2000;
//  z = ((float)yaokong.ch0-1024)/660*1000;
	
	if(yaokong.s1 == 1) z = 3000;
	else if(yaokong.s1 == 2) z = -3000;
	else z = 0;
	
	if(yaokong.ch3<363) y = 0;
	else if(yaokong.ch3>1685) y = 0;
	
	
	if(yaokong.ch2<364) x = 0;
	if(yaokong.ch2>1684) x = 0;
	if(yaokong.ch0<364) z = 0;
	if(yaokong.ch0>1684) z = 0;
	
	z = z*(15+19)/50;
	
	motor1 = x + y + z;
	motor2 = x - y + z;
	motor3 = -x + y + z;
	motor4 = -x - y + z;
	CAN_Motor_Control_35058(pid_v(motor1,motor_chassis_3508_1.speed_rpm),pid_v(motor2,motor_chassis_3508_2.speed_rpm),pid_v(motor3,motor_chassis_3508_3.speed_rpm),pid_v(motor4,motor_chassis_3508_4.speed_rpm));
};
void yuntai_sport()
{
	float w = 1.414*z/24.207 + ((float)yaokong.ch1-1024)/660*1000 ;
	CAN_Motor_Control_6020(pid_v(w,motor_chassis_6020_1.speed_rpm));

};
/* USER CODE END FunctionPrototypes */

void dipan_f(void *argument);
void yaokong_f(void *argument);
void motor_state_f(void *argument);

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
  /* creation of dipan */
  dipanHandle = osThreadNew(dipan_f, NULL, &dipan_attributes);

  /* creation of yaokong */
  yaokongHandle = osThreadNew(yaokong_f, NULL, &yaokong_attributes);

  /* creation of motor_state */
  motor_stateHandle = osThreadNew(motor_state_f, NULL, &motor_state_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_dipan_f */
/**
  * @brief  Function implementing the dipan thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_dipan_f */
void dipan_f(void *argument)
{
  /* USER CODE BEGIN dipan_f */
  /* Infinite loop */
  for(;;)
  {
		dipan_sport();
//		yuntai_sport();
    osDelay(1);
  }
  /* USER CODE END dipan_f */
}

/* USER CODE BEGIN Header_yaokong_f */
/**
* @brief Function implementing the yaokong thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_yaokong_f */
void yaokong_f(void *argument)
{
  /* USER CODE BEGIN yaokong_f */
  /* Infinite loop */
  for(;;)
  {
		
    osDelay(1);
  }
  /* USER CODE END yaokong_f */
}

/* USER CODE BEGIN Header_motor_state_f */
/**
* @brief Function implementing the motor_state thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor_state_f */
void motor_state_f(void *argument)
{
  /* USER CODE BEGIN motor_state_f */
  /* Infinite loop */
  for(;;)
  {
//		CAN_RxHeaderTypeDef rx_header;  
//		uint8_t rx_data[8];
//		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data); 
//	 
//		 switch (rx_header.StdId) 
//		 { 
//	 
//				case 0x201:         //0x200 + Id
//				{				  
//					get_motor_measure(&motor_chassis_3508_1, rx_data); 
//					break; 
//				} 
//					case 0x202:         //0x200 + Id
//				{				  
//					get_motor_measure(&motor_chassis_3508_2, rx_data); 
//					break; 
//				} 
//				case 0x203:         //0x200 + Id
//				{				 
//					get_motor_measure(&motor_chassis_3508_3, rx_data); 
//					break; 
//				} 
//				case 0x204:         //0x200 + Id
//				{				  
//					get_motor_measure(&motor_chassis_3508_4, rx_data); 
//					break; 
//				} 
//				case 0x205:         //0x200 + Id
//				{				  
//					get_motor_measure(&motor_chassis_6020, rx_data); 
//					break; 
//				} 
//			default: 
//			{ 
//				break; 
//			} 
		//x } 
    osDelay(1);
  }
  /* USER CODE END motor_state_f */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


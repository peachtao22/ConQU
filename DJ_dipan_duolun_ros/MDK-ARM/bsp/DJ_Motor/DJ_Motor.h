#ifndef __DJ_MOTOR_H_
#define __DJ_MOTOR_H_

#include "main.h"
#include "can.h"


#define CAN_FILTER(x)((x)<<3)
#define CAN_FIFO_0 (0<<2)
#define CAN_FIFO_1 (1<<2)
#define CAN_STDID (0<<1)
#define CAN_EXTID (1<<1)
#define CAN_DATA_TYPE (0<<0)
#define CAN_REMOTE_TYPE (1<<0)

typedef struct 
{ 
 uint16_t ecd; 
 int16_t speed_rpm; 
 int16_t given_current; 
 uint8_t temperate; 
 int16_t last_ecd; 
} motor_measure_t;
typedef struct
{
	int16_t Motor_speed[4];
	int16_t Motor_turn[4];
	
	float jiesuan_jiaodu[4];
	float jiesuan_speed[4];

	int16_t bazi_init[4];
	int16_t zhibai_init[4];
	
	int16_t speed_out[4];
	int16_t turn_out[4];

}chassis_handle;

extern chassis_handle chassis_duolun;
extern motor_measure_t motor_chassis_3508_1;
extern motor_measure_t motor_chassis_3508_2;
extern motor_measure_t motor_chassis_3508_3;
extern motor_measure_t motor_chassis_3508_4;

extern motor_measure_t motor_chassis_6020_1;
extern motor_measure_t motor_chassis_6020_2;
extern motor_measure_t motor_chassis_6020_3;
extern motor_measure_t motor_chassis_6020_4;

void DJ_Motor_Init(CAN_HandleTypeDef *hcan);
void CAN_Motor_Control_3508(chassis_handle* hchassis);
void CAN_Motor_Control_6020(chassis_handle* hchassis);
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan,uint8_t object_param,uint32_t ID,uint32_t mask_ID);
void get_motor_measure(motor_measure_t *ptr, const uint8_t *data);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

#endif

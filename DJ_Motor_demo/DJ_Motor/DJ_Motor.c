#include "DJ_Motor.h"

motor_measure_t motor_chassis_3508_1;
motor_measure_t motor_chassis_3508_2;
motor_measure_t motor_chassis_6020;


//初始函数
void DJ_Motor_Init(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_Start(hcan);

    CAN_Filter_Mask_Config(hcan, CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
    __HAL_CAN_ENABLE_IT (hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
};

void CAN_Motor_Control_3508(int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4)
{
	static CAN_TxHeaderTypeDef Tx_header;
	Tx_header.StdId = 0x200;
    // Tx_header.StdId = 0x1FF;  0x200控制Id：1-4的电机//0x1FF控制Id：5-8的电机
	Tx_header.ExtId = 0;
	Tx_header.IDE = CAN_ID_STD;
	Tx_header.RTR = CAN_RTR_DATA;
	Tx_header.DLC = 8;
	
	uint8_t TxBuf[8];
	uint32_t mailbox;
	
	TxBuf[0] = motor1>>8;
	TxBuf[1] = motor1;
	TxBuf[2] = motor2>>8;
	TxBuf[3] = motor2;
	TxBuf[4] = motor3>>8;
	TxBuf[5] = motor3;
	TxBuf[6] = motor4>>8;
	TxBuf[7] = motor4;
	
	HAL_CAN_AddTxMessage(&hcan1,&Tx_header,TxBuf,&mailbox);

};

void CAN_Motor_Control_6020(int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4)
{
	static CAN_TxHeaderTypeDef Tx_header;
	Tx_header.StdId = 0x1FF;
    // Tx_header.StdId = 0x2FF;  0x1FF控制Id：1-4的电机//0x2FF控制Id：5-7的电机//一个总线只能控制7个电机
	Tx_header.ExtId = 0;
	Tx_header.IDE = CAN_ID_STD;
	Tx_header.RTR = CAN_RTR_DATA;
	Tx_header.DLC = 8;
	
	uint8_t TxBuf[8];
	uint32_t mailbox;
	
	TxBuf[0] = motor1>>8;
	TxBuf[1] = motor1;
	TxBuf[2] = motor2>>8;
	TxBuf[3] = motor2;
	TxBuf[4] = motor3>>8;
	TxBuf[5] = motor3;
	TxBuf[6] = motor4>>8;
	TxBuf[7] = motor4;
	
	HAL_CAN_AddTxMessage(&hcan1,&Tx_header,TxBuf,&mailbox);

};

//自定义掩码函数
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan,uint8_t object_param,uint32_t ID,uint32_t mask_ID){
	CAN_FilterTypeDef can_filter_init;
	assert_param(hcan!=NULL);
	if((object_param &0x02)){
	 	can_filter_init.FilterIdHigh =ID<<3<<16;																				
	 	can_filter_init.FilterIdLow =ID<<3|((object_param &0x03))<<1;
		can_filter_init.FilterMaskIdHigh =mask_ID <<3<<16;
		can_filter_init.FilterMaskIdLow =mask_ID<<3|((object_param &0x03))<<1; 
	}
	else{
		can_filter_init.FilterIdHigh =ID<<5;																				
	  	can_filter_init.FilterIdLow =ID<<3|((object_param &0x03))<<1;
		can_filter_init.FilterMaskIdHigh =mask_ID <<5;
		can_filter_init.FilterMaskIdLow =mask_ID<<3|((object_param &0x03))<<1; 
	}
	
	can_filter_init.FilterBank =object_param >>3;
	can_filter_init.FilterFIFOAssignment=((object_param >>2)&0x01);
	can_filter_init.FilterActivation =ENABLE ;
	can_filter_init.FilterMode =CAN_FILTERMODE_IDMASK;
	can_filter_init.FilterScale =CAN_FILTERSCALE_32BIT;
	can_filter_init.SlaveStartFilterBank =14;
	HAL_CAN_ConfigFilter (hcan,&can_filter_init);
}


//解码函数
void get_motor_measure(motor_measure_t *ptr, const uint8_t *data)
{   
	//将信息存储进结构体 motor_measure_t
    ptr->last_ecd = ptr->ecd; 
    ptr->ecd = (uint16_t)((data[0] << 8) | data[1]); 
    ptr->speed_rpm = (int16_t)((data[2] << 8) | data[3]);
    ptr->given_current = (int16_t)((data[4] << 8) | data[5]); 
    ptr->temperate = data[6];
}


//fif0中断回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) 
{ 
	 CAN_RxHeaderTypeDef rx_header;  
      uint8_t rx_data[8];
	 HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); 
	 
	 switch (rx_header.StdId) 
	 { 
 
			case 0x201:         //0x200 + Id
			{	
				//确认电机ID，调用函数解码信息
				get_motor_measure(&motor_chassis_3508_1, rx_data); 
				break; 
			} 
			case 0x202:         
			{	
				//确认电机ID，调用函数解码信息
				get_motor_measure(&motor_chassis_3508_2, rx_data); 
				break; 
			} 
			case 0x205:         //0x200 + Id
			{				  
				get_motor_measure(&motor_chassis_6020, rx_data); 
				break; 
			} 
		default: 
		{ 
			break; 
		} 
	 } 
	 	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}
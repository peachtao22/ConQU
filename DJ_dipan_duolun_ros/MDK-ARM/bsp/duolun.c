#include "duolun.h"
#include <math.h>
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
  pid_v.kp = 7;
  pid_v.ki = 0.5;
  pid_v.kd = 0.8;
  
  float c_err,l_err,i_err;
//  c_err = (yaokong.ch3-1024)/660*300 - motor_chassis.speed_rpm;
	c_err = target -current ;
  int16_t out_v = pid_v.kp * c_err + pid_v.kd *(c_err-l_err) + pid_v.ki *i_err;
  l_err = c_err;
  i_err += c_err;
	if(i_err >1000) i_err = 1000; 
	if(i_err <-1000) i_err = -1000;
	
//	if(i == 0) out = -out;
	return out_v;
};
int16_t pid_j(float target,float current)
{
  Pid pid_j;
  pid_j.kp = 9;
  pid_j.ki = 0.1;
  pid_j.kd = 0.5;
  
  float c_err,l_err,i_err;
//  c_err = (yaokong.ch3-1024)/660*300 - motor_chassis.speed_rpm;
	c_err = target -current ;
		if(c_err >4000)
		{
		c_err = (c_err - 8191);
		l_err = 0;
		i_err = 0;
	}
	else if(c_err < -4000)
	{
		c_err = (c_err + 8191);
		l_err = 0;
		i_err = 0;

	}
	else c_err = c_err;
  int16_t out_j = pid_j.kp * c_err + pid_j.kd *(c_err-l_err) + pid_j.ki *i_err;
  l_err = c_err;
  i_err += c_err;
//	if(c_err >4000)
//	{target = (target - 8191);
//		//c_err = (c_err - 8191);
//		l_err = 0;
//		i_err = 0;
//	}
//	else if(c_err < -4000)
//	{target = (target + 8191);
//		//c_err = (c_err + 8191);
//		l_err = 0;
//		i_err = 0;
//	}
//	else c_err = c_err;
	if(i_err >1000) i_err = 1000; 
	if(i_err <-1000) i_err = -1000;
	
//	if(i == 0) out = -out;
	return out_j;
};
void Steer_Speed_Calculate(chassis_handle* chassis_handle, double chassis_vx, double chassis_vy, double chassis_vw) 
{
    float theta = atan(1.0/1.0); //45度角的弧度制表达               
//    float steer_vw=chassis_vw*3.14/180;//角速度转线速度
		float steer_vw=chassis_vw;

    float wheel_rpm_ratio;//轮子的角速度转轮子的线速度
	
	

	
//	chassis_handle->jiesuan_speed[0]
//			  = sqrt(pow(chassis_vx - steer_vw*sin(theta),2)                
//			  + pow(chassis_vy - steer_vw*cos(theta),2)) ;     


			     																																																																							
			
			chassis_handle->jiesuan_speed[0]
			= sqrt(	pow(chassis_vx - steer_vw*sin(theta),2)
						+	pow(chassis_vy - steer_vw*cos(theta),2)) ;    
																																																																																
			chassis_handle->jiesuan_speed[1]
			= -sqrt(	pow(chassis_vx - steer_vw*sin(theta),2)
						+	pow(chassis_vy + steer_vw*cos(theta),2));
						
			chassis_handle->jiesuan_speed[2]
			= -sqrt(	pow(chassis_vx + steer_vw*sin(theta),2)
						+	pow(chassis_vy + steer_vw*cos(theta),2)) ;    
																																																																																
			chassis_handle->jiesuan_speed[3]
			= sqrt(	pow(chassis_vx + steer_vw*sin(theta),2)
						+	pow(chassis_vy - steer_vw*cos(theta),2));
						
	
			for(int i=0;i<4;i++) chassis_handle->Motor_speed[i] = chassis_handle->jiesuan_speed[i];
			chassis_handle->speed_out[0] = pid_v(chassis_handle->Motor_speed[0],motor_chassis_3508_1.speed_rpm);
			chassis_handle->speed_out[1] = pid_v(chassis_handle->Motor_speed[1],motor_chassis_3508_2.speed_rpm);
			chassis_handle->speed_out[2] = pid_v(chassis_handle->Motor_speed[2],motor_chassis_3508_3.speed_rpm);
			chassis_handle->speed_out[3] = pid_v(chassis_handle->Motor_speed[3],motor_chassis_3508_4.speed_rpm);
			//memcpy(chassis_handle->wheel_rpm, wheel_rpm, 4 * sizeof(fp32));
		}
 
void Steer_angle_change(chassis_handle* chassis_handle, float chassis_vx, float chassis_vy, float chassis_vw)  
{
	float theta = atan(1.0f/1.0f);    
//	float steer_vw=chassis_vw*3.14/180;
	float steer_vw=chassis_vw;
 

	if((chassis_vx==0)&&(chassis_vy==0)&&(chassis_vw==0))              
	{
			for(uint8_t i=0;i<4;i++)
		{
			chassis_handle->Motor_turn[i]=chassis_handle->bazi_init[i];       
		}	
	}
	else           
	{
		chassis_handle->jiesuan_jiaodu[0]=atan2((chassis_vy-steer_vw*sin(theta)),
							(chassis_vx-steer_vw*cos(theta)));       
		chassis_handle->jiesuan_jiaodu[3]=atan2((chassis_vy+steer_vw*sin(theta)),
							(chassis_vx-steer_vw*cos(theta)));       
		chassis_handle->jiesuan_jiaodu[1]=atan2((chassis_vy-steer_vw*sin(theta)),
							(chassis_vx+steer_vw*cos(theta)));      
		chassis_handle->jiesuan_jiaodu[2]=atan2((chassis_vy+steer_vw*sin(theta)),
							(chassis_vx+steer_vw*cos(theta)));       
			 
//		for(uint8_t i=0;i<4;i++)                                                
//		{
//			if(wheel_angle[i]-chassis_handle->lastSteeringAngletarget[i]>PI/2)
//			{
//				wheel_angle[i]=fmodf(wheel_angle[i]-PI,2*PI);
//				chassis_handle->turnFlag[i]=1;
//			}
//			else if(wheel_angle[i]-chassis_handle->lastSteeringAngletarget[i]<-PI/2)
//			{
//				wheel_angle[i]=fmodf(wheel_angle[i]+PI,2*PI);
//				chassis_handle->turnFlag[i]=1;
//			}
//			else
//			{
//				chassis_handle->turnFlag[i]=0;
//			}		
//		}
		for(uint8_t i=0;i<4;i++)
		{
			chassis_handle->jiesuan_jiaodu[i]=chassis_handle->jiesuan_jiaodu[i]*180/3.14;       
		}		 
		for(int i=0;i<4;i++) 
		{
			chassis_handle->Motor_turn[i] = chassis_handle->jiesuan_jiaodu[i]/360*8192+chassis_handle->zhibai_init[i];
		}
	}
	chassis_handle->turn_out[0] = pid_j(chassis_handle->Motor_turn[0],motor_chassis_6020_1.ecd);
	chassis_handle->turn_out[1] = pid_j(chassis_handle->Motor_turn[1],motor_chassis_6020_2.ecd);
	chassis_handle->turn_out[2] = pid_j(chassis_handle->Motor_turn[2],motor_chassis_6020_3.ecd);
	chassis_handle->turn_out[3] = pid_j(chassis_handle->Motor_turn[3],motor_chassis_6020_4.ecd);
		 
				 
//	memcpy(chassis_handle->lastSteeringAngletarget, wheel_angle, 4 * sizeof(fp32));

}
 
void Steer_Calculate_turn(chassis_handle* chassis_handle, float chassis_vx, float chassis_vy, float chassis_vw) 
{	
	Steer_angle_change(chassis_handle,chassis_vx,chassis_vy,chassis_vw);    
	
	CAN_Motor_Control_6020(chassis_handle);
	
}
void Steer_Calculate_speed(chassis_handle* chassis_handle, float chassis_vx, float chassis_vy, float chassis_vw) 
{	
	
	Steer_Speed_Calculate(chassis_handle,chassis_vx,chassis_vy,chassis_vw);

	CAN_Motor_Control_3508(chassis_handle);
}

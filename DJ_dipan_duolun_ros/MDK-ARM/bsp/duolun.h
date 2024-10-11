#ifndef __DUOLUN_H_
#define __DUOLUN_H_
#include "main.h"
#include "can.h"
#include "DJ_Motor.h"
typedef struct 
{
  float kp;
  float ki;
  float kd;
  /* data */
}Pid;

extern chassis_handle chassis_duolun;
extern motor_measure_t motor_chassis_3508_1;
extern motor_measure_t motor_chassis_3508_2;
extern motor_measure_t motor_chassis_3508_3;
extern motor_measure_t motor_chassis_3508_4;

extern motor_measure_t motor_chassis_6020_1;
extern motor_measure_t motor_chassis_6020_2;


int16_t pid_j(float target,float current);
int16_t pid_v(float target,float current);
void Steer_Speed_Calculate(chassis_handle* chassis_handle, double chassis_vx, double chassis_vy, double chassis_vw);
void Steer_angle_change(chassis_handle* chassis_handle, float chassis_vx, float chassis_vy, float chassis_vw);
void Steer_Calculate_turn(chassis_handle* chassis_handle, float chassis_vx, float chassis_vy, float chassis_vw);
void Steer_Calculate_speed(chassis_handle* chassis_handle, float chassis_vx, float chassis_vy, float chassis_vw);
#endif
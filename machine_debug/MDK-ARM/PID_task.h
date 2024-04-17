#ifndef _PID_TASK_H_
#define _PID_TASK_H_

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "RMLibHead.h"
#include "RoboModule_DRV.h"
#include "stm32f4xx_hal.h"
#include "Servos_task.h"
#include "Odrive_Set.h"
#include "CANDrive.h"
#include "PID.h"
#include "motor.h"
#include "ramp.h"
#include "Chassis.h"

#define PI                3.1415926f

#define SPEED_MODE 1      //速度模式
#define RM3510_MODE 0     //3510模式
typedef struct {
    int id;
    int x;
	int y;
}Data_Typedef;

typedef struct
{
 double Expect_Speed_X;
 double Expect_Speed_Y;
 double Expect_Speed_Yaw;
} Expect_Speed_Typedef;
extern Expect_Speed_Typedef Expect_Speed;

typedef struct {
	float l1;//机械臂关节一长度
	float l2;//机械臂关节二长度
	float l3;//机械臂关节三长度
	float l4;//机械臂关节四长度
	float d;//吸盘到地面的距离
	
	float a;//电机角度
	float b;//舵机角度
}Robotic_arm;    //机械臂角度

extern Chassis_Motor_expect expect_wheel_3508;
extern Chassis_Motor_expect expect_wheel_2006;
extern TaskHandle_t PID_Handler;
extern TaskHandle_t Angle_Handler;
extern TaskHandle_t vN5055_Handle;

int32_t RAMP_self( int32_t final, int32_t now, int32_t ramp );
float myslope(float current, float expect, float k);
void jiaqu(void);
void axis(int x, int y);
void setPos(int32_t *m, int32_t pos, int32_t ramp);
void mysetPos(int32_t *m, int32_t *n, int32_t pos, int32_t ramp);
void dissetPos(int32_t *m, int32_t *n, int32_t pos, int32_t ramp);
void back(void);
void put(void); 
void clamp(void);
void button(void);
void release(void);
void coordinate(int x, int y);
void PID_Task(void *pvParameters);
void Angle_Task(void *pvParameters);
void vN5055_task(void *parameters);
void Get_angle(float h,float x);

//void jiaqu();

#endif

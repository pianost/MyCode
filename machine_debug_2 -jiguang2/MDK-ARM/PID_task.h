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
#include "usart.h"

#define PI                3.1415926f

#define SPEED_MODE 1      //速度模式
#define RM3510_MODE 1     //3510模式
#define LASER_RX_ID 0X610 //¼¤¹â½ÓÊÕID



typedef struct {
    int id;
    int x;
	int y;
}Data_Typedef;
////激光接收结构体
#pragma pack(1)
typedef struct {
	uint16_t adc;
	uint16_t spi1;
	uint16_t spi2;
	uint16_t spi3;
}LASER_SEND_Typedef;
#pragma pack()

typedef struct {
    uint16_t X_left;
    uint16_t X_right;
    uint16_t behind;
} LASER_Typedef;
#pragma pack(1)
typedef struct 
{
	uint8_t Head1;
	uint8_t Head2;
	float posture_data[6];
//	float zangle;    航向
//	float xangle;    俯仰
//	float yangle;    横滚
//	float pos_x;      X
//	float pos_y;      Y
//	float w_z;     航向角速
	uint8_t Tail1;
	uint8_t Tail2;
}RxData_Typedef;
#pragma pack()
extern RxData_Typedef Deal_posture;


typedef struct
{
 double Expect_Speed_X;
 double Expect_Speed_Y;
 double Expect_Speed_Yaw;
} Expect_Speed_Typedef;

typedef struct {
	float l1;//机械臂关节一长度
	float l2;//机械臂关节二长度
	float l3;//机械臂关节三长度
	float l4;//机械臂关节四长度
	float d;//吸盘到地面的距离
	
	float a;//电机角度
	float b;//舵机角度
}Robotic_arm;    //机械臂角度



extern uint16_t LV53_distance;
extern uint8_t usart3_dma_buff[50];
extern Expect_Speed_Typedef Expect_Speed;
extern float pos_x;
extern float pos_y;
extern float zangle;
extern float xangle;
extern float yangle;
extern float w_z;
extern uint8_t RxData[28];



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

#ifndef _MYTASK_H_
#define _MYTASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include <WatchDog.h>
#include "cmsis_os.h"
#include "CANDrive.H"
#include "CRC.H"

#pragma pack(1)
typedef struct
{
  uint16_t Left_Key_Up : 1;
  uint16_t Left_Key_Down : 1;
  uint16_t Left_Key_Left : 1;
  uint16_t Left_Key_Right : 1;
  uint16_t Left_Rocker : 1;
  uint16_t Left_Encoder : 1;
  uint16_t Left_Switch_Up : 1;
  uint16_t Left_Switch_Down : 1;  
  uint16_t Right_Key_Up : 1;
  uint16_t Right_Key_Down : 1;
  uint16_t Right_Key_Left : 1;
  uint16_t Right_Key_Right : 1;
  uint16_t Right_Rocker : 1;
  uint16_t Right_Encoder : 1;
  uint16_t Right_Switch_Up : 1;
  uint16_t Right_Switch_Down : 1;  
  } hw_key_t;

typedef struct {
	uint8_t head;
	uint16_t rocker[4];
	hw_key_t Key;
	uint32_t Left_Encoder;
	uint32_t Right_Encoder;
  uint16_t crc;
} UART_DataPack;

#pragma pack()

typedef struct {
    int16_t Ex;
    int16_t Ey;
    int16_t Eangle;
    hw_key_t *Key_Control;
    hw_key_t First,Second;
} Remote_Handle_t;

typedef struct
{
 double Expect_Speed_X;
 double Expect_Speed_Y;
 double Expect_Speed_Yaw;
} Expect_Speed_Typedef;

void Task_Creat(void);
void Motor_Task(void const *argument);
void Dog_Task(void const *argument);


#endif

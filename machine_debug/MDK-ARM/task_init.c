#include "Task_Init.h"
#include "Servos_task.h"
#include "PID_task.h"
//#include "Grab_launch.h"



void task_Init() {
     /*CAN1_Init*/
    CanFilter_Init(&hcan1);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    
    /*CAN2_Init*/
    CanFilter_Init(&hcan2);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

	vBlue_Brick_Init();
	RGB_init();
    
vPortEnterCritical();
 
    xTaskCreate(PID_Task,
         "PID_task",
          100,
          NULL,
          2,
          &PID_Handler); 
	 xTaskCreate(Angle_Task,
         "Angle_task",
          100,
          NULL,
          2,
          &Angle_Handler); 
	xTaskCreate(Lanuch_Task,
         "Lanuch_task",
          150,
          NULL,
          2,
          &Lanuch_Handler); 
	 xTaskCreate(vN5055_task,
          "vN5055_task",
          128,
          NULL,
          2,
          &vN5055_Handle);
vPortExitCritical();
}

void RGB_init()
{
	HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_SET);
}


#include "DataProcess.H"

extern Remote_Handle_t Remote_Control;
extern UART_DataPack RemoteData;
int flag = 1;
int flag1 = 0;
int a = 0,b = 0;

Expect_Speed_Typedef VCOMCOMM_Data;  

RoboModule_Typedef MI,MII,MIII;

void Task_Creat()
{
 xTaskCreate
	(
	(TaskFunction_t)Motor_Task,
	"MotorTask",
	128,
	NULL,
	5,
	NULL
	);
	
	xTaskCreate
	(
	(TaskFunction_t)Dog_Task,
	"FeedDog",
	64,
	NULL,
	5,
	NULL
	);
}
/** 
  * LED0亮代表处于遥控器模式 
  * LED1亮代表处于紧急制动模式
  * LED2灭代表处于遥控器断连状态
  *
  **/

void Motor_Task(void const *argument)
{
	for(;;)
 {
	 
	 if(RemoteData.Key.Left_Key_Up)
	 {
		   vTaskDelay(150);
			switch(a)
		{
			case 0:flag = 1;a++;HAL_GPIO_WritePin(GPIOC, LED0_Pin,GPIO_PIN_RESET);break;
			case 1:flag = 0;a--;HAL_GPIO_WritePin(GPIOC, LED0_Pin,GPIO_PIN_SET);break;
		}
	 }
	 if(RemoteData.Key.Left_Key_Down)
	 {
		  vTaskDelay(150);
			switch(b)
		{
			case 0:flag1 = 1;b++;HAL_GPIO_WritePin(GPIOC, LED1_Pin,GPIO_PIN_RESET);break;
			case 1:flag1 = 0;b--;HAL_GPIO_WritePin(GPIOC, LED1_Pin,GPIO_PIN_SET);break;
		}
		 
	 }
	 if(flag)
	 {
	  MI.ExpVelocity = 1.5 * (Remote_Control.Ey * 0.866025 + Remote_Control.Ex * 0.5+ Remote_Control.Eangle * 0.75);
		MII.ExpVelocity = 1.5 * (- Remote_Control.Ey* 0.866025 + Remote_Control.Ex * 0.5+ Remote_Control.Eangle * 0.75);
		MIII.ExpVelocity = 1.5 * (0 - Remote_Control.Ex + Remote_Control.Eangle * 0.75);
		 
	 }
	
	 if(flag1 != 1)
	 {
//	  MI.ExpVelocity = 0;
//		MII.ExpVelocity = 0;
//		MIII.ExpVelocity = 0;
		CAN_RoboModule_DRV_Velocity_Mode(&hcan2,&MI);
	  CAN_RoboModule_DRV_Velocity_Mode(&hcan2,&MII);
	  CAN_RoboModule_DRV_Velocity_Mode(&hcan2,&MIII);
	 }
	  vTaskDelay(1);
	 
  }
}

void Dog_Task(void const *argument)
{
		for(;;)
	{
		WatchDog_Polling();
		vTaskDelay(1);
	}
}

void WatchDog_CallBack(WatchDogp handle)//遥控器断联自动刹车
{
	
	  MI.ExpVelocity = 0;
		MII.ExpVelocity = 0;
		MIII.ExpVelocity = 0;
	
	  HAL_GPIO_WritePin(GPIOC, LED2_Pin,GPIO_PIN_SET);
	
		CAN_RoboModule_DRV_Velocity_Mode(&hcan2,&MI);
	  CAN_RoboModule_DRV_Velocity_Mode(&hcan2,&MII);
	  CAN_RoboModule_DRV_Velocity_Mode(&hcan2,&MIII);
	  vTaskDelay(10);
	
}

void VCOMM_CallBack(uint8_t fun_code, uint16_t id, uint8_t *data, uint8_t len)       
{
	
	memcpy(&VCOMCOMM_Data, data, sizeof(Expect_Speed_Typedef));
	if(flag == 0){
	MI.ExpVelocity = 800 * (VCOMCOMM_Data.Expect_Speed_X * 0.866025 - VCOMCOMM_Data.Expect_Speed_Y * 0.5 - VCOMCOMM_Data.Expect_Speed_Yaw *0.25);
	MII.ExpVelocity = 800 * (- VCOMCOMM_Data.Expect_Speed_X * 0.866025 - VCOMCOMM_Data.Expect_Speed_Y * 0.5 - VCOMCOMM_Data.Expect_Speed_Yaw * 0.25);
	MIII.ExpVelocity = 800 * (0 + VCOMCOMM_Data.Expect_Speed_Y * 0.5 - VCOMCOMM_Data.Expect_Speed_Yaw * 0.25);
	}
	
}

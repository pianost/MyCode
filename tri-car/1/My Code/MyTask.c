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
  * LED0亮代表处于PC虚拟串口模式 
  * LED1亮代表处于紧急制动模式
  * LED3亮代表处于遥控器断连状态
  *
  **/
void Motor_Task(void const *argument)
{
	for(;;)
 {
		
	 if(RemoteData.Key.Left_Key_Up)
	 {
			switch(a)
		{
			case 0:while(RemoteData.Key.Left_Key_Up);flag = 1;a++;HAL_GPIO_WritePin(GPIOC, LED0_Pin,GPIO_PIN_RESET);break;
			case 1:while(RemoteData.Key.Left_Key_Up);flag = 0;a--;HAL_GPIO_WritePin(GPIOC, LED0_Pin,GPIO_PIN_SET);break;
		}
	 }
	 if(RemoteData.Key.Left_Key_Down)
	 {
			switch(b)
		{
			case 0:while(RemoteData.Key.Left_Key_Down);flag1 = 1;b++;HAL_GPIO_WritePin(GPIOC, LED1_Pin,GPIO_PIN_RESET);break;
			case 1:while(RemoteData.Key.Left_Key_Down);flag1 = 0;b--;HAL_GPIO_WritePin(GPIOC, LED1_Pin,GPIO_PIN_SET);break;
		}
		 
	 }
	 if(flag)
	 {
	  MI.ExpVelocity = Remote_Control.Ey * 0.866 + Remote_Control.Ex * 0.5+ Remote_Control.Eangle * 0.5;
		MII.ExpVelocity = - Remote_Control.Ey* 0.866 + Remote_Control.Ex * 0.5+ Remote_Control.Eangle * 0.5;
		MIII.ExpVelocity = 0 - Remote_Control.Ex + Remote_Control.Eangle * 0.5;
	 }
	 if(flag1)
	 {
	  MI.ExpVelocity = 0;
		MII.ExpVelocity = 0;
		MIII.ExpVelocity = 0;
	 }
		
		CAN_RoboModule_DRV_Velocity_Mode(&hcan2,&MI);
	  CAN_RoboModule_DRV_Velocity_Mode(&hcan2,&MII);
	  CAN_RoboModule_DRV_Velocity_Mode(&hcan2,&MIII);
		
	 
	  vTaskDelay(10);
	 
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
	
	MI.ExpVelocity = 700 *(VCOMCOMM_Data.Expect_Speed_Y * 0.866 + VCOMCOMM_Data.Expect_Speed_X * 0.5 + VCOMCOMM_Data.Expect_Speed_Yaw *0.25) ;
	MII.ExpVelocity = 700 *(- VCOMCOMM_Data.Expect_Speed_Y * 0.866 + VCOMCOMM_Data.Expect_Speed_X * 0.5 + VCOMCOMM_Data.Expect_Speed_Yaw * 0.25)  ;
	MIII.ExpVelocity = 700 *(0 - VCOMCOMM_Data.Expect_Speed_X * 0.5 + VCOMCOMM_Data.Expect_Speed_Yaw * 0.25) ;
	
	 if(MI.ExpVelocity > 17000)
	{
	 MI.ExpVelocity = 17000;
	}
		else if ( - MI.ExpVelocity > 17000)
	{
		MI.ExpVelocity = -17000;
	}
	 if(MII.ExpVelocity > 17000)
	{
	 MII.ExpVelocity = 17000;
	}
		else if ( - MII.ExpVelocity > 17000)
	{
		MII.ExpVelocity = -17000;
	}
	 if(MIII.ExpVelocity > 17000)
	{
	 MIII.ExpVelocity = 17000;
	}
		else if ( - MIII.ExpVelocity > 17000)
	{
		MIII.ExpVelocity = -17000;
	}
	
}

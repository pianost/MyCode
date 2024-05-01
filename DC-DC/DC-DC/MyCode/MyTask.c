#include "freertos.h"
#include "mytask.h"

kalman_filter_t filters[3];
ADC_PARAM_T Para;
KEY_T Key;
extern uint16_t ADC1_Valve[59];

void Task_Init()
{
	xTaskCreate
	((TaskFunction_t)CompareVoltage,
	"CompareVoltage",
	256,
	NULL,
	5,
	NULL
	);

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == ADC1)
	{
		HAL_ADC_Stop_DMA(hadc);
		for(uint8_t i = 0; i <14; i+=3)
		{
			Para.voltage_output = Kalman_Filter(&filters[0],ADC1_Valve[15 + i]); //前5组数据偏差较大 舍弃
		  Para.voltage_temp = Kalman_Filter(&filters[1],ADC1_Valve[16 + i]);
			Para.voltage_input = Kalman_Filter(&filters[1],ADC1_Valve[17 + i]);
		}
		HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADC1_Valve,sizeof ADC1_Valve);
	}
}

void CompareVoltage(void const * argument)
{
	int count = 0;
	Key.Voltage_Change = 3.00;
	while(1)
	{
	  Para.True_voltage_output = 3.3 * Para.voltage_output / 4095;
 	  Para.True_voltage_temp = Get_Kelvin_Temperature(Get_Rntc(Para.voltage_temp));
		Para.True_voltage_input = 3.3 * Para.voltage_input / 4095;
		Para.current_out = Para.True_voltage_output / 1;
		Para.current_in = Para.True_voltage_input / 1;
	  ShowFloat(90,1,Para.True_voltage_input);ShowFloat(90,2,Para.True_voltage_output);
		ShowFloat(90,3,Para.current_in);ShowFloat(90,4,Para.current_out);
		ShowFloat(82,5,Key.Voltage_Change);
		ShowFloat(55,7,Para.True_voltage_temp);
    SetResistorWiper(Key.Voltage_Change);
		GetKeyValve(&Key);
	  if(Para.True_voltage_temp > 50)
		{
			count++;
		 if(count == 50)
		 {
		  count =0;OLED_ShowString(40,6,"        ",8);HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3,GPIO_PIN_RESET);
		 }
		 else if(count == 25)
		 {
		  OLED_ShowString(40,6,"!!TEMP!!",8);HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3,GPIO_PIN_SET);
		 }
		}
		else
		{
		 count =0;OLED_ShowString(40,6,"        ",8);HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3,GPIO_PIN_RESET);
		}
	
	  OLED_flush(pdMS_TO_TICKS(10000));
		
	}
}

void ADC_filtersInit() 
{
  for (uint8_t i = 0; i < 3; ++i)
	{
    kalman_Init(&filters[i], 1, 10000);   
	}		
}

void screen_Init()
{
	OLED_ShowString(1,1,"Vol(V):(IN)",11);OLED_ShowString(48,2,"(OUT)",5);
	OLED_ShowString(1,3,"Cur(A):(IN)",11);OLED_ShowString(48,4,"(OUT)",5);
	OLED_ShowString(1,5,"Ex_Vol(V)",10);
	OLED_ShowString(1,6,"Alarm",5);
	OLED_ShowString(1,7,"Temp",4);
}

void GetKeyValve(KEY_T * param)
{
	param->Key_Value = __HAL_TIM_GetCounter(&htim2); 
	if(param->Key_Value > param->Last_Key_Value && param->Voltage_Change < 14.999)
	{
		param->Voltage_Change +=0.05;
	}
	else if(param->Key_Value < param->Last_Key_Value && param->Voltage_Change >3.0001)
	{
	 param->Voltage_Change -= 0.05;
	}
	param->Last_Key_Value = param->Key_Value;
}

void ShowFloat(uint8_t x,uint8_t y,float param)
{
	float a;
	a = (100 * (param - (int)param));
	OLED_ShowNum(x,y,param,2,1);OLED_ShowChar(x+12,y,'.',1);
	OLED_ShowNum_WithZero(x+18,y,a,2,1);
	
}

double Quick_ln(double a)
{
   int N = 15;
   int k,nk;
   float x,xx,y;
   x = (a-1)/(a+1);
   xx = x*x;
   nk = 2*N+1;
   y = 1.0/nk;
   for(k=N;k>0;k--)
   {
     nk = nk - 2;
     y = 1.0/nk+xx*y;
     
   }
   return 2.0*x*y;
}

double Get_Rntc(double ADC)
{
	return ADC * R1/(4095 - ADC);
	
}

double Get_Kelvin_Temperature(double Rntc)
{
	return 1/(1/T25 + Quick_ln(Rntc/R25)/B) - 273.15;
	
}

void SetResistorWiper(float Voltage)
{
  uint8_t DAC;
	float k,b;
	k = 1221 / 20480;
	b = 101343 / 80000 - Voltage_Offset;
  DAC = (Voltage - b) / k;
 
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
  HAL_SPI_Transmit(&hspi1,&DAC,sizeof DAC,10);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
}

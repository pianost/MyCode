#include "VL53-400.h"
uint8_t Data[8] = {0};


void VL53_400Read(uint8_t *data)
{
	data[0]= ID;data[1]= ReadData;
	HAL_UART_Transmit(&huart4,data,sizeof data,25);
	
}

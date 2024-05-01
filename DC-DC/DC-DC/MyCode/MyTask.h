#ifndef _MYTASK_H_
#define _MYTASK_H_

#include "gpio.h"
#include "task.h"
#include "main.h"
#include "adc.h"
#include "kalman.h"
#include "oled.h"
#include "tim.h"
#include "spi.h"

#define T25 298.15
#define R25 10000
#define B	3950
#define R1 10000
#define Voltage_Offset 0.0045

typedef struct{
float voltage_output;
float voltage_temp;
float voltage_input;
float current_in;
float current_out;
float True_voltage_output;
float True_voltage_temp;
float True_voltage_input;
}ADC_PARAM_T;

typedef struct{
int16_t Last_Key_Value;
int16_t Key_Value;
float Voltage_Change;
}KEY_T;

void SetResistorWiper(float Voltage);
void GetKeyValve(KEY_T * param);
void screen_Init(void); 
void Task_Init(void);
void CompareVoltage(void const * argument);
void ADC_filtersInit(void);
void ShowFloat(uint8_t x,uint8_t y,float param);
double Quick_ln(double a);
double Get_Kelvin_Temperature(double Rntc);
double Get_Rntc(double ADC);

#endif

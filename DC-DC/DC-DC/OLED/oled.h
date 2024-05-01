#ifndef __OLED_H__
#define __OLED_H__

#include "stdint.h"

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size2);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size);
void OLED_ShowString(uint8_t x, uint8_t y, char *chr, uint8_t Char_Size);
void OLED_ShowNum_WithZero(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size2);

uint8_t OLED_flush(uint32_t timeout);
int32_t oled_pow(uint8_t m, uint8_t n);

#endif



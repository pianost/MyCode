#include "oled.h"
#include "i2c.h"
#include "oledfont.h"
#include "string.h"

static uint8_t GRAM[8][128];
static uint8_t * pos;
////初始化命令
uint8_t CMD_Data[] = {
  0xAE,       //关闭显示
  0x00,       //设置显示时的起始列地址低四位。0
  0x10,       //设置显示时的起始列地址高四位。0
  0x40,        //设置显示RAM显示起始行寄存器
  0xB0,       //用于设置页地址，其低三位的值对应着GRAM的页地址。
  0x81,  0xff, //设置对比度255
  0xA1,       //列地址127被映射到SEG0
  0xA6,       //Set Normal/Inverse Display
  0xA8, 0x3F, //设置多路复用率
  0xC8,       //重新映射模式。扫描的COM COM0 (n - 1)
  0xD3, 0x00, //Set Display Offset, 0
  0xD5, 0x80, //Set Display Clock Divide Ratio/Oscillator Frequency 
  0xD9, 0xF1, //Set Pre-charge Period
  0xDA, 0x12, //Set COM Pins Hardware Configuration  
  0x8D, 0x14, //DCDC ON
  0x20, 0x00,  //设置寻址模式为水平模式
  0xb0,       //设置行列指针位置0,0
  0x00,
  0x10,
  0xAF,       //开启显示
};

//向缓存写数据
static inline void OLED_WR_DATA(uint8_t data) {
  *pos++ = data;
}

static inline void OLED_Set_Pos(uint8_t x, uint8_t y) {
  pos = GRAM[y] + x;
}

//初始化oled屏幕
void OLED_Init(void) {
  HAL_Delay(200);
  HAL_I2C_Mem_Write(&hi2c1, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, CMD_Data, sizeof(CMD_Data), 100);
}

uint8_t OLED_flush(uint32_t timeout) {
  uint32_t tick = HAL_GetTick();
  while(HAL_I2C_Mem_Write_DMA(&hi2c1, 0x78, 0x40, I2C_MEMADD_SIZE_8BIT, (uint8_t *) GRAM, sizeof(GRAM)) != HAL_OK)
    if(tick < HAL_GetTick() - timeout) return 0; 
  return 1;
}

//清屏
void OLED_Clear(void) {
  memset(GRAM, 0, sizeof(GRAM));
}

int32_t oled_pow(uint8_t m, uint8_t n) {
    uint32_t result = 1;
    while (n--)
        result *= m;
    return result;
}

//显示2个数字
//x,y :起点坐标
//len :数字的位数
//size:字体大小
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size2) {
    uint8_t t, temp;
    uint8_t enshow = 0;
    for (t = 0; t < len; t++) {
        temp = (num / oled_pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1)) {
            if (temp == 0) {
                OLED_ShowChar(x + (size2 == 16 ? 8 : 6) * t, y, ' ', size2);
                continue;
            } else
                enshow = 1;
        }
        OLED_ShowChar(x + (size2 == 16 ? 8 : 6) * t, y, temp + '0', size2);
    }
}

void OLED_ShowNum_WithZero(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size2) {
    uint8_t t, temp;
    uint8_t enshow = 0;
    for (t = 0; t < len; t++) {
        temp = (num / oled_pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1)) {
            if (temp == 0) {
              OLED_ShowChar(x + (size2 == 16 ? 8 : 6) * t, y, '0', size2);
                continue;
            } else
                enshow = 1;
        }
        OLED_ShowChar(x + (size2 == 16 ? 8 : 6) * t, y, temp + '0', size2);
    }
}

//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//mode:0,反白显示;1,正常显示
//size:选择字体 16/12
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size) {
    uint8_t c = 0, i = 0;
    c = chr - ' '; //得到偏移后的值
    if (x > 128 - 1) {
        x = 0;
        y = y + 2;
    }
    if (Char_Size == 16) {
        OLED_Set_Pos(x, y);
        for (i = 0; i < 8; i++)
            OLED_WR_DATA(F8X16[c * 16 + i]);
        OLED_Set_Pos(x, y + 1);
        for (i = 0; i < 8; i++)
            OLED_WR_DATA(F8X16[c * 16 + i + 8]);
    } else {
        OLED_Set_Pos(x, y);
        for (i = 0; i < 6; i++)
            OLED_WR_DATA(F6x8[c][i]);
    }
}

//显示一个字符号串
void OLED_ShowString(uint8_t x, uint8_t y, char *chr, uint8_t Char_Size) {
    uint8_t j = 0;
    while (chr[j] != '\0') {
      OLED_ShowChar(x, y, chr[j], Char_Size);
      x += 8;
      if (x > 120) {
        x = 0;
        y += 2;
      }
      j++;
    }
}

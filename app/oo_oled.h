/**
  ******************************************************************************
  * @file    stm32f10x_gpio.h
  * @author  RuiXiaoliang
  * @version V1.0.0
  * @date    17-May-2017
  * @brief   oled
  * 
  ******************************************************************************
  * @attention

  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OO_OLED_H
#define __OO_OLED_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
typedef void (*wr_byte_t)(uint8_t data, uint8_t cmd);
struct oo_oled
{
    uint16_t pixelx;
    uint16_t pixely;
    wr_byte_t wr_byte;
    struct oo_oled *oled;
};
//extern struct oo_oled oled_091,oled_242;
extern uint8_t POS_X_LAT;
extern uint8_t POS_Y_LAT;
extern uint8_t POS_X_LATV;
extern uint8_t POS_Y_LATV;
extern uint8_t POS_X_LONG;
extern uint8_t POS_Y_LONG;
extern uint8_t POS_X_LONGV;
extern uint8_t POS_Y_LONGV;
extern uint8_t POS_X_ATT;
extern uint8_t POS_Y_ATT;
extern uint8_t POS_X_ATTPV;
extern uint8_t POS_Y_ATTPV;
extern uint8_t POS_X_ATTYV;
extern uint8_t POS_Y_ATTYV;
extern uint8_t POS_X_ATTRV;
extern uint8_t POS_Y_ATTRV;
extern uint8_t POS_X_HIGH;
extern uint8_t POS_Y_HIGH;
extern uint8_t POS_X_HIGHV;
extern uint8_t POS_Y_HIGHV;
extern uint8_t POS_X_MODE;
extern uint8_t POS_Y_MODE;
extern uint8_t POS_X_MODEV;
extern uint8_t POS_Y_MODEV;
extern uint8_t POS_X_STAT;
extern uint8_t POS_Y_STAT;
extern uint8_t POS_X_STATV;
extern uint8_t POS_Y_STATV;
extern uint8_t POS_X_BAT;
extern uint8_t POS_Y_BAT;
extern uint8_t POS_X_BATV;
extern uint8_t POS_Y_BATV;
extern uint8_t POS_X_VOLT;
extern uint8_t POS_Y_VOLT;
extern uint8_t POS_X_VOLTV;
extern uint8_t POS_Y_VOLTV;
extern uint8_t POS_X_CURR;
extern uint8_t POS_Y_CURR;
extern uint8_t POS_X_CURRV;
extern uint8_t POS_Y_CURRV;
extern uint8_t POS_X_TEMP;
extern uint8_t POS_Y_TEMP;
extern uint8_t POS_X_TEMPV;
extern uint8_t POS_Y_TEMPV;
extern uint8_t POS_X_ALAM;
extern uint8_t POS_Y_ALAM;
extern uint8_t POS_X_ALAMV;
extern uint8_t POS_Y_ALAMV;

void oled_242_wr_byte(u8 dat,u8 cmd);
void oled_display_on(void);
void oled_display_off(void);	   							   		    
void oled_init(void);
void oled_clear(void);
void oled_draw_point(u8 x,u8 y,u8 t);
void oled_fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot);
void oled_show_char(u8 x,u8 y,u8 chr);
void oled_show_number(u8 x,u8 y,u32 num,u8 len,u8 size);
void oled_show_string(u8 x,u8 y, char *p);	 
void oled_set_pos(unsigned char x, unsigned char y);
void oled_show_chinese(u8 x,u8 y,u8 no,u8 mode);
void oled_draw_bmp(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);
void oled_init_display(void);
void oled_show_char_8x16(u8 x,u8 y,u8 chr);

#ifdef __cplusplus
}
#endif

#endif /* __OO_OLED_H */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

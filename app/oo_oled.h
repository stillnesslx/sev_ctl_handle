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
extern struct oo_oled oled_091,oled_242;

void oled_wr_byte(u8 dat,u8 cmd);	    
void oled_display_on(struct oo_oled *oled);
void oled_display_off(struct oo_oled *oled);	   							   		    
void oled_init(void);
void oled_clear(struct oo_oled *oled);
void oled_draw_point(u8 x,u8 y,u8 t,struct oo_oled *oled);
void oled_fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot,struct oo_oled *oled);
void oled_show_char(u8 x,u8 y,u8 chr,struct oo_oled *oled);
void oled_show_number(u8 x,u8 y,u32 num,u8 len,u8 size,struct oo_oled *oled);
void oled_show_string(u8 x,u8 y, u8 *p,struct oo_oled *oled);	 
void oled_set_pos(unsigned char x, unsigned char y,struct oo_oled *oled);
void oled_show_chinese(u8 x,u8 y,u8 no,u8 mode,struct oo_oled *oled);
void oled_draw_bmp(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[],struct oo_oled *oled);


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

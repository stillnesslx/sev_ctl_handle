/**
  ******************************************************************************
  * @file    stm32f10x_gpio.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the GPIO 
  *          firmware library.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_H
#define __BSP_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

#define DI_CODE_OFF 	0
#define DI_CODE_LAND 	1
#define DI_CODE_50M 	2
#define DI_CODE_RAISE 	3
#define DI_CODE_DROP	4
#define DI_CODE_OPEN	5
#define DI_CODE_CLOSE	6
#define DI_CODE_FREE	7
#define DI_CODE_100M	8
#define DI_CODE_150M	9
#define DI_CODE_COLLECT 10

#define DI_CHANNEL_NUM 11
#define DI_FILTER_VALUE 3
     
#define COM_DATA_LEN 32

struct di_data
{
    uint32_t di_new;
    //u32 di_old;
    uint32_t di_filtered;
    uint16_t di_timer[DI_CHANNEL_NUM];
    uint16_t di_filter_num[DI_CHANNEL_NUM];
};

struct com_receive_data
{
    char addr;
    char code;
    char data[COM_DATA_LEN];
    char crc;
};
struct com_send_data
{
    uint8_t addr;
    uint8_t code;
    struct
    {
        uint16_t in;
        uint16_t ana;
    }data;
    uint16_t sum;
};
extern struct di_data di_value;
extern void read_di(struct di_data *p);
extern void bsp_init(void);
void delay_ms(uint16_t t);

extern __IO uint16_t ADCConvertedValue;
extern struct di_data di_value;
extern struct com_send_data com_sdata;


#ifdef __cplusplus
}
#endif

#endif /* __BSP_H */
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

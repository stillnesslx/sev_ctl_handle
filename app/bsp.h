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
#include "fifo_buffer.h"

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
     
#define COM_DATA_LEN 28
#define COM_REV_FRM_LEN (COM_DATA_LEN+8+4)//40
#define COM_REV_BUF_LEN (COM_REV_FRM_LEN*2)


#define COM_DATA_SYNC1  0xAA
#define COM_DATA_SYNC2  0x44
#define COM_DATA_MSGID  0xC2
#define COM_DATA_SRCID  0x01
#define COM_DATA_SQN    0x01


#define SWAP16(s) ((((s) & 0xff) << 8) | (((s) >> 8) & 0xff))
#define SWAP32(l) (((l) >> 24) | (((l) & 0x00ff0000) >> 8) | (((l) & 0x0000ff00) << 8)  | ((l) << 24))

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
    uint8_t sync1;
    uint8_t sync2;
    uint8_t msg_id;
    uint8_t src_id;
    uint16_t sqn;
    uint16_t msg_len;
    struct
    {
        uint16_t in;
        uint16_t ana;
    }data;
    uint32_t crc;
};
struct display_data
{
    uint32_t latitude;
    uint32_t longitude;
    uint16_t high;
    uint16_t yaw;
    uint16_t pitch;
    uint16_t roll;
    uint16_t mode;
    uint16_t alarm;
    uint16_t voltage;
    uint16_t current;
    uint16_t tempt;
    uint16_t resv;
    //uint16_t status;
    //uint16_t battery;
};
extern struct di_data di_value;
extern struct display_data dsp_data;

extern void read_di(struct di_data *p);
extern void bsp_init(void);
void delay_ms(uint16_t t);

extern __IO uint16_t ADCConvertedValue;
extern struct di_data di_value;
extern struct com_send_data com_sdata;
extern t_fifo_buffer com_fifo;
extern uint8_t com_rev_buf[COM_REV_BUF_LEN];

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

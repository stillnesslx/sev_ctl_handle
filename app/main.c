/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
//#include "stm32_eval.h"
#include <stdio.h>
#include <math.h>
#include "bsp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "serial.h"
#include "comtest.h"
#include "oled.h"
#include "semphr.h"
#include "oo_oled.h"
#include "oledfont.h"  	
#include <string.h>
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Task priorities. */
#define LED_TASK_PRIORITY				( tskIDLE_PRIORITY + 6 )
#define RS422_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )
#define RIN_TASK_PRIORITY				( tskIDLE_PRIORITY + 5 )
#define OLED091_TASK_PRIORITY           ( tskIDLE_PRIORITY + 1 )
#define OLED242_TASK_PRIORITY           ( tskIDLE_PRIORITY + 4 )

/* Baud rate used by the comtest tasks. */
#define mainCOM_TEST_BAUD_RATE		( 115200 )
/* The LED used by the comtest tasks. See the comtest.c file for more
information. */
#define mainCOM_TEST_LED			( 3 )
/* Private macro -------------------------------------------------------------*/
#define DSP_MODE_EN 0
/* Private variables ---------------------------------------------------------*/
//char RxBuffer1[512];
//uint8_t RxCounter1=0;

u8 hx = 48,hy = 0,hlen = 3,hsize = 32;
u32 h = 60;
u8 test_en=0;
const uint8_t high[3] = {50,100,150};
SemaphoreHandle_t xSemaphore = NULL;

/* Private function prototypes -----------------------------------------------*/

void start_led_task(UBaseType_t prio);
void start_rin_task(UBaseType_t prio);
void start_rs422_task(UBaseType_t prio);
void start_oled091_task(UBaseType_t prio);
void start_oled242_task(UBaseType_t prio);

static void prvSetupHardware( void );
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     

  /* Initialize LEDs, Key Button, LCD and COM port(USART) available on
     STM3210X-EVAL board ******************************************************/

    prvSetupHardware();
  /* USARTx configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */


  /* Initialize the LCD */


  /* Display message on STM3210X-EVAL LCD *************************************/
  /* Clear the LCD */ 


  /* Retarget the C library printf function to the USARTx, can be USART1 or USART2
     depending on the EVAL board you are using ********************************/


  /* Turn on leds available on STM3210X-EVAL **********************************/


  /* Add your application code here*/
    
    vSemaphoreCreateBinary( xSemaphore );

    bsp_init();
    start_led_task(LED_TASK_PRIORITY);
    if( xSemaphore != NULL )
    {
        start_rin_task(RIN_TASK_PRIORITY);
        start_oled091_task(OLED091_TASK_PRIORITY);
    }
    start_oled242_task(OLED242_TASK_PRIORITY);
    //start_rs422_task(RS422_TASK_PRIORITY);
    vAltStartComTestTasks( RS422_TASK_PRIORITY, mainCOM_TEST_BAUD_RATE, mainCOM_TEST_LED);
    /* Start the scheduler. */
    vTaskStartScheduler();
    /* Infinite loop */
    while (1)
    {
    }
}
#define LED_STACK_SIZE	            ( ( unsigned short ) 56 )
#define RIN_STACK_SIZE	            ( ( unsigned short ) 56 )
#define RS422_STACK_SIZE	        ( ( unsigned short ) 128 )
#define RS422_RECEIVE_STACK_SIZE    ( ( unsigned short ) 128 )
#define OLED091_STACK_SIZE          ( ( unsigned short ) 256 )
#define OLED242_STACK_SIZE          ( ( unsigned short ) 256 )

//    uint32_t latitude;
//    uint32_t longitude;
//    uint16_t pitch;
//    uint16_t yaw;
//    uint16_t roll;
//    uint16_t high;
//    uint16_t mode;
//    uint16_t status;
//    uint16_t battery;
//    uint16_t voltage;
//    uint16_t current;
//    uint16_t tempt;
//    uint16_t alarm;
#define COM_REV_NUM_SYNC1       0
#define COM_REV_NUM_SYNC2       1
#define COM_REV_NUM_MSGID       2
#define COM_REV_NUM_SRCID       3
#define COM_REV_NUM_SQN         4
#define COM_REV_NUM_LEN         6
#define COM_REV_NUM_MSG         8


u16 oled242_delay = 1000;
extern uint8_t POS_X_HIGH,POS_Y_HIGH;
uint8_t oled_cmd = 0;
char chr_test = 82;
u8 stat = 0;
uint8_t com_rev_data_buf[COM_REV_FRM_LEN];
//0xaa 0x44 0xc2 0x01 0x01 0x26 0x46 0xa1 0xaa 0x40 0x85 0x37 0x64 0xc0 0x08 0xbc 0x07 0xc0 0x07 0x83 0x01 0xf4 0x00 0x00 0x00 0x00 0x00 0x00 0x0e 0xd8 0x00 0xdc 0x08 0xe8 0x00 0x01 0x00 0x00
//aa 44 c2 01 00 01 00 1c 46 a1 aa 40 85 37 64 c0 01 f4 07 83 08 bc 07 c0 00 01 00 01 0e d8 00 dc 05 78 00 00 0f d4 69 fa
//aa 44
//c2
//01
//00 01
//00 1c
//46 a1 aa 40
//85 37 64 c0
//01 f4
//07 83
//08 bc
//07 c0
//00 01
//00 01
//0e d8
//00 dc
//05 78
//0e 12 25 50
//
void memcpy_word(u16 *src,u16 *det,u8 len)
{
    while(len--)
    {
        *src++ = *det++;
    }
}
static void oled242_ch_task(void *pvParameters )
{
    //u8 stat = 0;
    uint8_t rd_cnt=0;
	( void ) pvParameters;
    //oled_init_display();
    //oled_show_number(POS_X_HIGHV,POS_Y_HIGHV,high[((di_value.di_filtered & 0x300) >> 8) -1],hlen,16);
    //olde242_ch_init_display();

	for(;;)
	{
        if(fifoBuf_getUsed(&com_fifo) > COM_DATA_LEN)
        {
            while((COM_DATA_SYNC1 != (com_rev_data_buf[0] = fifoBuf_getByte(&com_fifo))) || (COM_DATA_SYNC2 != (com_rev_data_buf[1] = fifoBuf_getByte(&com_fifo))))
            {
                 if(rd_cnt++ > COM_DATA_LEN)
                 {
                     break;
                 }
            }
            if(rd_cnt > COM_DATA_LEN)
            {
                rd_cnt = 0;
            }
            else
            {
                uint8_t i;
                uint32_t crc = 0,rcrc;
                fifoBuf_getData(&com_fifo, &com_rev_data_buf[2], COM_REV_FRM_LEN-2);
                for(i=0;i<COM_REV_FRM_LEN/4-1;i++)
                {
                    crc += *(uint32_t *)&com_rev_data_buf;
                }
                rcrc = SWAP32(*(uint32_t *)&com_rev_data_buf[COM_REV_FRM_LEN-4]);
                if((COM_DATA_MSGID == com_rev_data_buf[2])
                    && (COM_DATA_SRCID == com_rev_data_buf[3]) 
                    && (COM_DATA_LEN == ((uint16_t)com_rev_data_buf[6]<<8) | com_rev_data_buf[7])
                    && (crc == SWAP32(*(uint32_t *)&com_rev_data_buf[COM_REV_FRM_LEN-4])))
                {
                    //memcpy_word((u16 *)&dsp_data,(u16 *)&com_rev_data_buf[COM_REV_NUM_MSG],sizeof(struct display_data)/2);
                    dsp_data.latitude = SWAP32(*(uint32_t *)&com_rev_data_buf[COM_REV_NUM_MSG]);
                    dsp_data.longitude = SWAP32(*(uint32_t *)&com_rev_data_buf[COM_REV_NUM_MSG+4]);
                    dsp_data.high = SWAP16(*(uint16_t *)&com_rev_data_buf[COM_REV_NUM_MSG+8]);
                    dsp_data.yaw = SWAP16(*(uint16_t *)&com_rev_data_buf[COM_REV_NUM_MSG+10]);
                    dsp_data.pitch = SWAP16(*(uint16_t *)&com_rev_data_buf[COM_REV_NUM_MSG+12]);
                    dsp_data.roll = SWAP16(*(uint16_t *)&com_rev_data_buf[COM_REV_NUM_MSG+14]);
                    dsp_data.mode = SWAP16(*(uint16_t *)&com_rev_data_buf[COM_REV_NUM_MSG+16]);
                    dsp_data.alarm = SWAP16(*(uint16_t *)&com_rev_data_buf[COM_REV_NUM_MSG+18]);
                    dsp_data.voltage = SWAP16(*(uint16_t *)&com_rev_data_buf[COM_REV_NUM_MSG+20]);
                    dsp_data.current = SWAP16(*(uint16_t *)&com_rev_data_buf[COM_REV_NUM_MSG+22]);
                    dsp_data.tempt = SWAP16(*(uint16_t *)&com_rev_data_buf[COM_REV_NUM_MSG+24]);
                    //dsp_data.resv = SWAP16(*(uint16_t *)&com_rev_data_buf[COM_REV_NUM_MSG+26]);
                }
            }
        }

        //OLED_Clear();
        //OLED_ShowCHinese(hx,hy,0,0);//中
        //xSemaphoreTake( xSemaphore, portMAX_DELAY );
//        if(1 == oled_cmd)
//        {
//            oled_cmd = 0;
//            oled_clear();
//        }
//        else if(2 == oled_cmd)
//        {
//            oled_cmd = 0;
//            //oled_init_display();
//        }

        if(0 == stat)
        {
            oled_clear();
            //oled_show_char(POS_CH_MD_X+32,POS_CH_MD_Y,':',16,0,F8X16);

            if(0 == dsp_data.mode)
            {
                oled_show_chinese(POS_CH_MD_X+0,POS_CH_MD_Y,4,0);//调
                oled_show_chinese(POS_CH_MD_X+16,POS_CH_MD_Y,27,0);//试
                oled_show_chinese(POS_CH_MD_X+32,POS_CH_MD_Y,23,0);//模
                oled_show_chinese(POS_CH_MD_X+48,POS_CH_MD_Y,26,0);//式
                oled_show_string(POS_CH_MD_X+64,POS_CH_MD_Y,"  ",16,F8X16);//
            }
            else if(1 == dsp_data.mode)
            {
                oled_show_chinese(POS_CH_MD_X+0,POS_CH_MD_Y,28,0);//手
                oled_show_chinese(POS_CH_MD_X+16,POS_CH_MD_Y,5,0);//动
                oled_show_chinese(POS_CH_MD_X+32,POS_CH_MD_Y,23,0);//模
                oled_show_chinese(POS_CH_MD_X+48,POS_CH_MD_Y,26,0);//式
                oled_show_string(POS_CH_MD_X+64,POS_CH_MD_Y,"  ",16,F8X16);//
            }
            else if(2 == dsp_data.mode)
            {
                oled_show_chinese(POS_CH_MD_X+0,POS_CH_MD_Y,0,0);//半
                oled_show_chinese(POS_CH_MD_X+16,POS_CH_MD_Y,44,0);//自
                oled_show_chinese(POS_CH_MD_X+32,POS_CH_MD_Y,5,0);//动
                oled_show_chinese(POS_CH_MD_X+48,POS_CH_MD_Y,23,0);//模
                oled_show_chinese(POS_CH_MD_X+64,POS_CH_MD_Y,26,0);//式
            }
            else if(3 == dsp_data.mode)
            {
                oled_show_chinese(POS_CH_MD_X+0,POS_CH_MD_Y,44,0);//自
                oled_show_chinese(POS_CH_MD_X+16,POS_CH_MD_Y,5,0);//动
                oled_show_chinese(POS_CH_MD_X+32,POS_CH_MD_Y,23,0);//模
                oled_show_chinese(POS_CH_MD_X+48,POS_CH_MD_Y,26,0);//式
                oled_show_string(POS_CH_MD_X+64,POS_CH_MD_Y,"  ",16,F8X16);//
            }
            else
            {
                //dsp_data.mode = 0;
            }
            if(0 == dsp_data.alarm)
            {
                oled_show_chinese(POS_CH_AL_X+0,POS_CH_AL_Y,47,0);//正
                oled_show_chinese(POS_CH_AL_X+16,POS_CH_AL_Y,46,0);//常
            }
            else
            {
                oled_show_chinese(POS_CH_AL_X+0,POS_CH_AL_Y,45,0);//异
                oled_show_chinese(POS_CH_AL_X+16,POS_CH_AL_Y,46,0);//常
                oled_show_number(POS_CH_AL_X+32,POS_CH_AL_Y,dsp_data.alarm,16,F8X16);
            }
            oled_show_chinese(POS_CH_VT_X+0,POS_CH_VT_Y,3,0);//电
            oled_show_chinese(POS_CH_VT_X+16,POS_CH_VT_Y,36,0);//压
            //oled_show_char(POS_CH_VT_X+32,POS_CH_VT_Y,':',16,0,F8X16);
            oled_show_string(POS_CH_VT_X+32,POS_CH_VT_Y,":     V",16,F8X16);
            oled_show_float(POS_CH_VT_X+40,POS_CH_VT_Y,dsp_data.voltage/10.0f,16,F8X16,"%5.1f");

            oled_show_chinese(POS_CH_CR_X+0,POS_CH_CR_Y,3,0);//电
            oled_show_chinese(POS_CH_CR_X+16,POS_CH_CR_Y,21,0);//流
            //oled_show_char(POS_CH_CR_X+32,POS_CH_CR_Y,':',16,0,F8X16);
            oled_show_string(POS_CH_CR_X+32,POS_CH_CR_Y,":     A",16,F8X16);
            oled_show_float(POS_CH_CR_X+40,POS_CH_CR_Y,dsp_data.current/10.0f,16,F8X16,"%5.1f");

            oled_show_chinese(POS_CH_TP_X+0,POS_CH_TP_Y,33,0);//温
            oled_show_chinese(POS_CH_TP_X+16,POS_CH_TP_Y,6,0);//度
            //oled_show_char(POS_CH_TP_X+32,POS_CH_TP_Y,':',16,0,F8X16);
            oled_show_string(POS_CH_TP_X+32,POS_CH_TP_Y,":      \x7f\x43",16,F8X16);
            oled_show_float(POS_CH_TP_X+40,POS_CH_TP_Y,dsp_data.tempt/10.0f-100,16,F8X16,"%6.1f");
            
            //oled_show_number(POS_CH_ATT_X+72,POS_CH_ATT_Y,dsp_data.yaw,16,F6x8);
            //oled_show_number(POS_X_LATV,POS_Y_LATV,(dsp_data.latitude & 0xff) >> 8,2,16);
    //        oled_show_number(POS_X_LONGV,POS_Y_LONGV,(dsp_data.longitude & 0xff) >> 8,2,16);
    //        oled_show_number(POS_X_LATV+18,POS_Y_LATV,dsp_data.latitude & 0xff,2,16);
    //        oled_show_number(POS_X_LONGV+18,POS_Y_LONGV,dsp_data.longitude & 0xff,2,16);
    //        oled_show_number(POS_X_ATTPV,POS_Y_ATTPV,dsp_data.pitch,2,16);
    //        oled_show_number(POS_X_ATTYV,POS_Y_ATTYV,dsp_data.yaw,2,16);
    //        oled_show_number(POS_X_ATTRV,POS_Y_ATTRV,dsp_data.roll,2,16);
    //        oled_show_number(POS_X_HIGHV,POS_Y_HIGHV,high[((di_value.di_filtered & 0x300) >> 8) -1],hlen,16);
    //        
    //        oled_show_number(POS_X_MODEV,POS_Y_MODEV,dsp_data.mode,2,16);
    //        oled_show_number(POS_X_STATV,POS_Y_STATV,dsp_data.status,2,16);
    //        oled_show_number(POS_X_BATV,POS_Y_BATV,dsp_data.battery,2,16);
    //        oled_show_number(POS_X_VOLTV,POS_Y_VOLTV,dsp_data.voltage,3,16);
    //        oled_show_number(POS_X_CURRV,POS_Y_CURRV,dsp_data.current,3,16);
    //        oled_show_number(POS_X_TEMPV,POS_Y_TEMPV,dsp_data.tempt,2,16);
    //        oled_show_number(POS_X_ALAMV,POS_Y_ALAMV,dsp_data.alarm,2,16);
    //        oled_show_char(108,6,chr_test,12,0);
            if(test_en)
            oled_show_char(96,4,chr_test,16,0,F8X16);
            //stat = 1;
        }
        else if(1 == stat)
        {
            oled_clear();
            oled_show_chinese(POS_CH_HI_X+0,POS_CH_HI_Y,9,0);//高
            oled_show_chinese(POS_CH_HI_X+16,POS_CH_HI_Y,6,0);//度
            oled_show_string(POS_CH_HI_X+32,POS_CH_HI_Y,":     m",16,F8X16);
            //oled_show_char(POS_CH_HI_X+32,POS_CH_HI_Y,':',16,0,F8X16);
            oled_show_float(POS_CH_HI_X+40,POS_CH_HI_Y,high[((di_value.di_filtered & 0x300) >> 8) -1]+(float)(ADCConvertedValue-2048)/2048.0f*50.0f,16,F8X16,"%5.1f");

            oled_show_chinese(POS_CH_LAT_X+0,POS_CH_LAT_Y,18,0);//经
            oled_show_chinese(POS_CH_LAT_X+16,POS_CH_LAT_Y,31,0);//纬
            oled_show_chinese(POS_CH_LAT_X+32,POS_CH_LAT_Y,6,0);//度
            oled_show_char(POS_CH_LAT_X+48,POS_CH_LAT_Y,':',16,0,F8X16);
            oled_show_string(POS_CH_LAT_X,POS_CH_LAT_Y+2,"          \x7f",16,F8X16);
            oled_show_float(POS_CH_LAT_X,POS_CH_LAT_Y+2,fabs(dsp_data.latitude/10000000.0f-90.0f),16,F8X16,"%10.7f");
            oled_show_char(POS_CH_LAT_X+88,POS_CH_LAT_Y+2,(dsp_data.latitude/10000000.0f-90.0f)>0?'N':'S',16,0,F8X16);

            oled_show_string(POS_CH_LAT_X,POS_CH_LAT_Y+4,"           \x7f",16,F8X16);
            oled_show_float(POS_CH_LAT_X,POS_CH_LAT_Y+4,fabs(dsp_data.longitude/10000000.0f-180.0f),16,F8X16,"%11.7f");
            oled_show_char(POS_CH_LAT_X+96,POS_CH_LAT_Y+4,(dsp_data.longitude/10000000.0f-180.0f)>0?'E':'W',16,0,F8X16);
            //stat = 2;
        }
        else if(2 == stat)
        {
            oled_clear();
            //oled_show_chinese(POS_CH_ATT_X+0,POS_CH_ATT_Y,30,0);//姿
            //oled_show_chinese(POS_CH_ATT_X+16,POS_CH_ATT_Y,43,0);//态
            oled_show_chinese(POS_CH_ATT_X+0,POS_CH_ATT_Y,13,0);//航
            oled_show_chinese(POS_CH_ATT_X+16,POS_CH_ATT_Y,35,0);//向
            oled_show_chinese(POS_CH_ATT_X+32,POS_CH_ATT_Y,16,0);//角
            oled_show_char(POS_CH_ATT_X+48,POS_CH_ATT_Y,':',16,0,F8X16);
            oled_show_string(POS_CH_ATT_X+56,POS_CH_ATT_Y,"     \x7f",16,F8X16);
            oled_show_float(POS_CH_ATT_X+56,POS_CH_ATT_Y,dsp_data.yaw/10.0f-180,16,F8X16,"%5.1f");
            
            oled_show_chinese(POS_CH_ATT_X+0,POS_CH_ATT_Y+2,8,0);//俯
            oled_show_chinese(POS_CH_ATT_X+16,POS_CH_ATT_Y+2,37,0);//仰
            oled_show_chinese(POS_CH_ATT_X+32,POS_CH_ATT_Y+2,16,0);//角
            oled_show_char(POS_CH_ATT_X+48,POS_CH_ATT_Y+2,':',16,0,F8X16);
            oled_show_string(POS_CH_ATT_X+56,POS_CH_ATT_Y+2,"     \x7f",16,F8X16);
            oled_show_float(POS_CH_ATT_X+56,POS_CH_ATT_Y+2,dsp_data.pitch/10.0f-180,16,F8X16,"%5.1f");
            
            oled_show_chinese(POS_CH_ATT_X+0,POS_CH_ATT_Y+4,14,0);//横
            oled_show_chinese(POS_CH_ATT_X+16,POS_CH_ATT_Y+4,12,0);//滚
            oled_show_chinese(POS_CH_ATT_X+32,POS_CH_ATT_Y+4,16,0);//角
            oled_show_char(POS_CH_ATT_X+48,POS_CH_ATT_Y+4,':',16,0,F8X16);
            oled_show_string(POS_CH_ATT_X+56,POS_CH_ATT_Y+4,"     \x7f",16,F8X16);
            oled_show_float(POS_CH_ATT_X+56,POS_CH_ATT_Y+4,dsp_data.roll/10.0f-180,16,F8X16,"%5.1f");
            //stat = 0;
        }
        else
        {
            stat = 0;
        }
		vTaskDelay( oled242_delay / portTICK_PERIOD_MS ); /* Delay 1000 ms */
	}
}


static void oled242_task(void *pvParameters )
{
	( void ) pvParameters;
    oled_clear();
    oled_init_display();
    oled_show_number(POS_X_HIGHV,POS_Y_HIGHV,high[((di_value.di_filtered & 0x300) >> 8) -1],12,F6x8);
	for(;;)
	{
        //OLED_Clear();
        //OLED_ShowCHinese(hx,hy,0,0);//中
        //xSemaphoreTake( xSemaphore, portMAX_DELAY );
        if(1 == oled_cmd)
        {
            oled_cmd = 0;
            oled_clear();
        }
        else if(2 == oled_cmd)
        {
            oled_cmd = 0;
            oled_init_display();
        }
        oled_show_number(POS_X_LATV,POS_Y_LATV,(dsp_data.latitude & 0xff) >> 8,12,F6x8);
        oled_show_number(POS_X_LONGV,POS_Y_LONGV,(dsp_data.longitude & 0xff) >> 8,12,F6x8);
        oled_show_number(POS_X_LATV+18,POS_Y_LATV,dsp_data.latitude & 0xff,12,F6x8);
        oled_show_number(POS_X_LONGV+18,POS_Y_LONGV,dsp_data.longitude & 0xff,12,F6x8);
        oled_show_number(POS_X_ATTPV,POS_Y_ATTPV,dsp_data.pitch,12,F6x8);
        oled_show_number(POS_X_ATTYV,POS_Y_ATTYV,dsp_data.yaw,12,F6x8);
        oled_show_number(POS_X_ATTRV,POS_Y_ATTRV,dsp_data.roll,12,F6x8);
        oled_show_number(POS_X_HIGHV,POS_Y_HIGHV,high[((di_value.di_filtered & 0x300) >> 8) -1],12,F6x8);
        
        oled_show_number(POS_X_MODEV,POS_Y_MODEV,dsp_data.mode,12,F6x8);
        //oled_show_number(POS_X_STATV,POS_Y_STATV,dsp_data.status,12,F6x8);
        //oled_show_number(POS_X_BATV,POS_Y_BATV,dsp_data.battery,12,F6x8);
        oled_show_number(POS_X_VOLTV,POS_Y_VOLTV,dsp_data.voltage,12,F6x8);
        oled_show_number(POS_X_CURRV,POS_Y_CURRV,dsp_data.current,12,F6x8);
        oled_show_number(POS_X_TEMPV,POS_Y_TEMPV,dsp_data.tempt,12,F6x8);
        oled_show_number(POS_X_ALAMV,POS_Y_ALAMV,dsp_data.alarm,12,F6x8);
        oled_show_char(108,6,chr_test,12,0,F6x8);
        

		vTaskDelay( 100 / portTICK_PERIOD_MS ); /* Delay 100 ms */
	}
}
static void oled091_task(void *pvParameters )
{

    OLED_Clear();
    (void) pvParameters;
    //OLED_ShowNum(hx,hy,high[((di_value.di_filtered & 0x300) >> 8) -1],hlen,hsize);
    OLED_ShowNum(hx,hy,high[((di_value.di_filtered & 0x300) >> 8) -1],hlen,hsize);
    for(;;)
    {
        //OLED_Clear();
        //OLED_ShowCHinese(hx,hy,0,0);//中
        xSemaphoreTake( xSemaphore, portMAX_DELAY );
        OLED_ShowNum(hx,hy,high[((di_value.di_filtered & 0x300) >> 8) -1],hlen,hsize);
        //vTaskDelay( 100 / portTICK_PERIOD_MS ); /* Delay 100 ms */
    }
}
static void led_task(void *pvParameters )
{
	( void ) pvParameters;
	for(;;)
	{
		GPIO_WriteBit(GPIOA, GPIO_Pin_3, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_3)));
		vTaskDelay( 1000 / portTICK_PERIOD_MS ); /* Delay 1000 ms */
	}
}
static void rin_task(void *pvParameters )
{
	( void ) pvParameters;
	for(;;)
	{
		read_di(&di_value);
        if(di_value.di_filtered != di_value.di_new)
        {
            xSemaphoreGive( xSemaphore );
        }
		vTaskDelay( 100 / portTICK_PERIOD_MS ); /* Delay 10 ms */
	}
}
static void rs422_task(void *pvParameters )
{
    static char ch = 0;
	( void ) pvParameters;
	for(;;)
	{
		//rs422_send();
        USART_SendData(USART1, ch++);
		vTaskDelay( 200 / portTICK_PERIOD_MS ); /* Delay 10 ms */
	}
}
static void rs422_receive_task(void *pvParameters )
{
	( void ) pvParameters;
	for(;;)
	{
		//rs422_send();
        //USART_SendData(USART1, ch++);
        
        while(USART_GetFlagStatus(USART1,USART_FLAG_RXNE) == RESET)
        {
        }
        /* Store the received byte in the RxBuffer1 */
        //RxBuffer1[RxCounter1++%512] = USART_ReceiveData(USART1);
		vTaskDelay( 200 / portTICK_PERIOD_MS ); /* Delay 10 ms */
	}
}
//static void led_task1( void *pvParameters )
//{
//	( void ) pvParameters;
//	for(;;)
//	{
//		GPIO_WriteBit(GPIOA, GPIO_Pin_3, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_3)));
//		vTaskDelay( 1000 / portTICK_PERIOD_MS ); /* Delay 1 s */
//	}
//}

void start_led_task(UBaseType_t prio)
{
    xTaskCreate( led_task, "LED", LED_STACK_SIZE, NULL, prio, ( TaskHandle_t * ) NULL );
	//xTaskCreate( led_task1, "LED1", LED_STACK_SIZE, NULL, prio, ( TaskHandle_t * ) NULL );
}
void start_rin_task(UBaseType_t prio)
{
    xTaskCreate( rin_task, "RIN", RIN_STACK_SIZE, NULL, prio, ( TaskHandle_t * ) NULL );
}
void start_rs422_task(UBaseType_t prio)
{
    xTaskCreate( rs422_task, "RS422", RS422_STACK_SIZE, NULL, prio, ( TaskHandle_t * ) NULL );
}
void start_rs422_receive_task(UBaseType_t prio)
{
    xTaskCreate( rs422_receive_task, "RS422_RECEIVE", RS422_RECEIVE_STACK_SIZE, NULL, prio, ( TaskHandle_t * ) NULL );
}
void start_oled091_task(UBaseType_t prio)
{
    xTaskCreate( oled091_task, "OLED091", OLED091_STACK_SIZE, NULL, prio, ( TaskHandle_t * ) NULL );
	//xTaskCreate( led_task1, "LED1", LED_STACK_SIZE, NULL, prio, ( TaskHandle_t * ) NULL );
}
void start_oled242_task(UBaseType_t prio)
{
    if(1 == DSP_MODE_EN)
    {
        xTaskCreate( oled242_task, "OLED242", OLED242_STACK_SIZE, NULL, prio, ( TaskHandle_t * ) NULL );
    }
    else
    {
        xTaskCreate( oled242_ch_task, "OLED242", OLED242_STACK_SIZE, NULL, prio, ( TaskHandle_t * ) NULL );
        //xTaskCreate( oled242_ch_task2, "OLED242_2", OLED242_STACK_SIZE, NULL, prio, ( TaskHandle_t * ) NULL );
        //xTaskCreate( oled242_ch_task3, "OLED242_3", OLED242_STACK_SIZE, NULL, prio, ( TaskHandle_t * ) NULL );
    }
	//xTaskCreate( led_task1, "LED1", LED_STACK_SIZE, NULL, prio, ( TaskHandle_t * ) NULL );
}
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
static void prvSetupHardware( void )
{
	/* Start with the clocks in their expected state. */
	RCC_DeInit();

	/* Enable HSE (high speed external clock). */
	RCC_HSEConfig( RCC_HSE_ON );

	/* Wait till HSE is ready. */
	while( RCC_GetFlagStatus( RCC_FLAG_HSERDY ) == RESET )
	{
	}

	/* 2 wait states required on the flash. */
	*( ( unsigned long * ) 0x40022000 ) = 0x02;

	/* HCLK = SYSCLK */
	RCC_HCLKConfig( RCC_SYSCLK_Div1 );

	/* PCLK2 = HCLK */
	RCC_PCLK2Config( RCC_HCLK_Div1 );

	/* PCLK1 = HCLK/2 */
	RCC_PCLK1Config( RCC_HCLK_Div2 );

	/* PLLCLK = 8MHz * 9 = 72 MHz. */
	RCC_PLLConfig( RCC_PLLSource_HSE_Div1, RCC_PLLMul_9 );

	/* Enable PLL. */
	RCC_PLLCmd( ENABLE );

	/* Wait till PLL is ready. */
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
	{
	}

	/* Select PLL as system clock source. */
	RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK );

	/* Wait till PLL is used as system clock source. */
	while( RCC_GetSYSCLKSource() != 0x08 )
	{
	}

	/* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE and AFIO clocks */
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC, ENABLE );

	/* SPI2 Periph clock enable */
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2, ENABLE );


	/* Set the Vector Table base address at 0x08000000 */
	NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 );

	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

	/* Configure HCLK clock as SysTick clock source. */
	SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );

	//vParTestInitialise();
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

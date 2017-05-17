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
#include "bsp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "serial.h"
#include "comtest.h"
#include "oled.h"
#include "semphr.h"
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

/* Baud rate used by the comtest tasks. */
#define mainCOM_TEST_BAUD_RATE		( 115200 )
/* The LED used by the comtest tasks. See the comtest.c file for more
information. */
#define mainCOM_TEST_LED			( 3 )
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//char RxBuffer1[512];
//uint8_t RxCounter1=0;

u8 hx = 48,hy = 0,hlen = 3,hsize = 32;
u32 h = 60;
const uint8_t high[3] = {50,100,150};
SemaphoreHandle_t xSemaphore = NULL;
/* Private function prototypes -----------------------------------------------*/

void start_led_task(UBaseType_t prio);
void start_rin_task(UBaseType_t prio);
void start_rs422_task(UBaseType_t prio);
void start_oled091_task(UBaseType_t prio);

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
    //start_rs422_task(RS422_TASK_PRIORITY);
    vAltStartComTestTasks( RS422_TASK_PRIORITY, mainCOM_TEST_BAUD_RATE, mainCOM_TEST_LED );
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
static void oled091_task(void *pvParameters )
{
    //uint8_t t = '9';
        OLED_Clear();
        //LED_ON;
		//OLED_ShowCHinese(0,0,0,0);//中
		//OLED_ShowCHinese(18,0,1,0);//景
		//OLED_ShowCHinese(36,0,2,0);//园
		//OLED_ShowCHinese(54,0,3,0);//电
		//OLED_ShowCHinese(72,0,4,0);//子
		//OLED_ShowCHinese(90,0,5,0);//科
		//OLED_ShowCHinese(108,0,6,0);//技
		//OLED_ShowString(0,2,"0.91' OLED TEST");
		//OLED_ShowString(8,2,"ZHONGJINGYUAN");  
        //OLED_ShowString(20,4,"2014/05/01");  
		//OLED_ShowString(0,6,"ASCII:");  
		//OLED_ShowString(63,6,"CODE:");  
		//OLED_ShowChar(48,6,t);//显示ASCII字符	   
		//t++;
		//if(t>'~')t=' ';
		//OLED_ShowNum(54,0,h,3,16);//显示ASCII字符的码值
        //OLED_ShowNum(54,0,60,3,16);
    //OLED_ShowNum(hx,hy,h,hlen,hsize);
        //OLED_Clear();
	( void ) pvParameters;
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

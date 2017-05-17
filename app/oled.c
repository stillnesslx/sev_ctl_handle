//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//中景园电子
//店铺地址：http://shop73023976.taobao.com/?spm=2013.1.0.0.M4PqC2
//
//  文 件 名   : main.c
//  版 本 号   : v2.0
//  作    者   : HuangKai
//  生成日期   : 2014-0101
//  最近修改   : 
//  功能描述   : OLED 4接口演示例程(51系列)
//              说明: 
//              ----------------------------------------------------------------
//              GND    电源地
//              VCC  接5V或3.3v电源
//              D0   接PD6（SCL）
//              D1   接PD7（SDA）
//              RES  接PD4
//              DC   接PD5
//              CS   接PD3               
//              ----------------------------------------------------------------
// 修改历史   :
// 日    期   : 
// 作    者   : HuangKai
// 修改内容   : 创建文件
//版权所有，盗版必究。
//Copyright(C) 中景园电子2014/3/16
//All rights reserved
//******************************************************************************/

#include "oled.h"
#include "stdlib.h"
#include <stdio.h>
#include "oledfont.h"  	 
#include "stm32f10x.h"	
#include "stm32f10x_gpio.h"
#include "stm32f10x_spi.h"
#include "bmp.h"
//OLED的显存
//存放格式如下.
//[0]0 1 2 3 ... 127
//[1]0 1 2 3 ... 127
//[2]0 1 2 3 ... 127
//[3]0 1 2 3 ... 127
//[4]0 1 2 3 ... 127
//[5]0 1 2 3 ... 127
//[6]0 1 2 3 ... 127
//[7]0 1 2 3 ... 127
int parm_sel;
void uDelay(unsigned char l)
{
	while(l--);
}
void SPI2_Init()
{
    SPI_InitTypeDef  SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;	   

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2 ,ENABLE);	   //使能SPI2外设时钟

    /* 配置 SPI2 引脚: SCK, MISO and MOSI（PB13, PB14, PB15) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;          //复用功能（推挽）输出  SPI2
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* 片选  PB0 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;			  					 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		   //输出模式最大速度50MHz
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		   //通用推挽输出模式
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /*Command or data pin*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;			  					 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		   //输出模式最大速度50MHz
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		   //通用推挽输出模式
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /*Reset pin*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;			  					 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		   //输出模式最大速度50MHz
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		   //通用推挽输出模式
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* SPI2 配置 */ 
    SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;   //全双工  
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						   //主模式
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					   //8位
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;						   //时钟极性 空闲状态时，SCK保持低电平
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;						   //时钟相位 数据采样从第一个时钟边沿开始
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							   //软件产生NSS
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;  //波特率控制 SYSCLK/16
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;				   //数据高位在前
    SPI_InitStructure.SPI_CRCPolynomial = 7;							   //CRC多项式寄存器初始值为7 
    SPI_Init(SPI2, &SPI_InitStructure);

    /* 使能SPI2  */
    SPI_Cmd(SPI2, ENABLE);
}
void SPI2_NRF_SendByte(unsigned char byte)
{
    /* 循环检测发送缓冲区是否是空 */
    while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);

    /* 通过SPI2外设发出数据 */
    SPI_I2S_SendData(SPI2, byte);
}

//向SSD1106写入一个字节。
//dat:要写入的数据/命令
//cmd:数据/命令标志 0,表示命令;1,表示数据;
void OLED_WR_Byte(u8 dat,u8 cmd)
{
//	u8 i;
	if(cmd)
        OLED_DC_Set();
	else
        OLED_DC_Clr();
	
	OLED_CS_Clr();
    SPI2_NRF_SendByte(dat);
	uDelay(150);
	OLED_CS_Set();
	OLED_DC_Set();
}

void OLED_Set_Pos(unsigned char x, unsigned char y)
{
	OLED_WR_Byte(0xb0+y,OLED_CMD);
	OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	OLED_WR_Byte((x&0x0f)|0x01,OLED_CMD);
}
//开启OLED显示
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//关闭OLED显示
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   			 
//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!	  
//void OLED_Clear(void)  
//{  
//OLED_DrawBMP(33,0,128,8,BMP2);
//}

void OLED_Clear(void)  
{  
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
		OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
		OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址  
   
		for(n=0;n<128;n++)OLED_WR_Byte(0,OLED_DATA); 
	} //更新显示
}

//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//mode:0,反白显示;1,正常显示
//size:选择字体 16/12 
void OLED_ShowChar(u8 x,u8 y,u8 chr)
{
    unsigned char c=0,i=0;	
    c=chr-' ';//得到偏移后的值
    if(x>Max_Column-1){x=0;y=y+2;}

    OLED_Set_Pos(x,y+1);
    for(i=0;i<6;i++)
    {
        if(!parm_sel)
        OLED_WR_Byte(F6x8[c][i],OLED_DATA);
        else
        OLED_WR_Byte(~F6x8[c][i],OLED_DATA);
    }
}


//void OLED_ShowChar_816(u8 x,u8 y,u8 chr)
//{
//    unsigned char c=0,i=0;	
//    c=chr-' ';//得到偏移后的值
//    if(x>Max_Column-1){x=0;y=y+2;}
//    OLED_Set_Pos(x,y);	
//    for(i=0;i<8;i++)
//        OLED_WR_Byte(F8X16[c*16+i],OLED_DATA);
//    OLED_Set_Pos(x,y+1);
//    for(i=0;i<8;i++)
//        OLED_WR_Byte(F8X16[c*16+i+8],OLED_DATA);
//}
void OLED_ShowChar_816(u8 x,u8 y,u8 chr)
{
    unsigned char c=0,i=0;	
    //c=chr-' ';//得到偏移后的值
    c=chr-'0';//得到偏移后的值
    if(x>Max_Column-1){x=0;y=y+2;}
    OLED_Set_Pos(x,y);
    for(i=0;i<16;i++)
        OLED_WR_Byte(F16X24[c*48+i],OLED_DATA);
    OLED_Set_Pos(x,y+1);
    for(i=0;i<16;i++)
        OLED_WR_Byte(F16X24[c*48+i+16],OLED_DATA);
    OLED_Set_Pos(x,y+2);
    for(i=0;i<16;i++)
        OLED_WR_Byte(F16X24[c*48+i+32],OLED_DATA);
}
void OLED_ShowString_816(u8 x,u8 y,u8 *chr)
{
    unsigned char j=0;
    while (chr[j]!='\0')
    {
        OLED_ShowChar_816(x,y,chr[j]);
        x+=6;
        if(x>120){x=0;y+=2;}
        j++;
    }
}
//m^n函数
u32 oled_pow(u8 m,u8 n)
{
	u32 result=1;
	while(n--)result*=m;
	return result;
}				  
//显示2个数字
//x,y :起点坐标	 
//len :数字的位数
//size:字体大小
//mode:模式	0,填充模式;1,叠加模式
//num:数值(0~4294967295);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size)
{
	u8 t,temp;
	//u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
//		if(enshow==0&&t<(len-1))
//		{
//			if(temp==0)
//			{
//				OLED_ShowChar(x+(size/2)*t,y,' ');
//				continue;
//			}else enshow=1;
//		}
	 	//OLED_ShowChar(x+(size/2)*t,y,temp+'0'); 
        OLED_ShowChar_816(x+(size/2)*t,y,temp+'0');
        //OLED_ShowChar_816(x+(size/2)*t,y,temp+'0');
	}
} 
//显示一个字符号串
void OLED_ShowString(u8 x,u8 y,u8 *chr)
{
	unsigned char j=0;
	while (chr[j]!='\0')
	{		OLED_ShowChar(x,y,chr[j]);
			x+=6;
		if(x>120){x=0;y+=2;}
			j++;
	}
}

//显示汉字
void OLED_ShowCHinese(u8 x,u8 y,u8 no,u8 mode)
{      			    
	u8 t,adder=0;
	OLED_Set_Pos(x,y);	
    for(t=0;t<16;t++)
		{
        if(!mode)			
				OLED_WR_Byte(Hzk[2*no][t],OLED_DATA);
				else
				OLED_WR_Byte(~Hzk[2*no][t],OLED_DATA);
				adder+=1;
     }	
		OLED_Set_Pos(x,y+1);	
    for(t=0;t<16;t++)
			{	
				if(!mode)
				OLED_WR_Byte(Hzk[2*no+1][t],OLED_DATA);
				else
				OLED_WR_Byte(~Hzk[2*no+1][t],OLED_DATA);
				adder+=1;
      }					
}
/***********功能描述：显示显示BMP图片128×64起始点坐标(x,y),x的范围0～127，y为页的范围0～7*****************/
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[])
{ 	
 unsigned int j=0;
 unsigned char x,y;
  
  if(y1%8==0) y=y1/8;      
  else y=y1/8+1;
	for(y=y0;y<y1;y++)
	{
		OLED_Set_Pos(x0,y);
    for(x=x0;x<x1;x++)
	    {      
	    	OLED_WR_Byte(BMP[j++],OLED_DATA);	    	
	    }
	}
} 

//extern void Delay(__IO uint32_t nTime);
void Delay(__IO uint32_t nTime)
{
    while(nTime--)
    {
        uint32_t i = 72000;
        while(i--);
    }
}
//初始化SSD1306					    
void OLED_Init(void)
{
	parm_sel=0;
	SPI2_Init();

    OLED_RST_Set();
	Delay(100);
	OLED_RST_Clr();
	Delay(200);
	OLED_RST_Set(); 
					  
	OLED_WR_Byte(0xAE,OLED_CMD);//--turn off oled panel
	OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
	OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
	OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
	OLED_WR_Byte(0x81,OLED_CMD);//--set contrast control register
	OLED_WR_Byte(0xCF,OLED_CMD); // Set SEG Output Current Brightness
	OLED_WR_Byte(0xA1,OLED_CMD);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
	OLED_WR_Byte(0xC8,OLED_CMD);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
	OLED_WR_Byte(0xA6,OLED_CMD);//--set normal display
	OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
	OLED_WR_Byte(0x3f,OLED_CMD);//--1/64 duty
	OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	OLED_WR_Byte(0x00,OLED_CMD);//-not offset
	OLED_WR_Byte(0xd5,OLED_CMD);//--set display clock divide ratio/oscillator frequency
	OLED_WR_Byte(0x80,OLED_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
	OLED_WR_Byte(0xD9,OLED_CMD);//--set pre-charge period
	OLED_WR_Byte(0xF1,OLED_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	OLED_WR_Byte(0xDA,OLED_CMD);//--set com pins hardware configuration
	OLED_WR_Byte(0x12,OLED_CMD);
	OLED_WR_Byte(0xDB,OLED_CMD);//--set vcomh
	OLED_WR_Byte(0x40,OLED_CMD);//Set VCOM Deselect Level
	OLED_WR_Byte(0x20,OLED_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
	OLED_WR_Byte(0x02,OLED_CMD);//
	OLED_WR_Byte(0x8D,OLED_CMD);//--set Charge Pump enable/disable
	OLED_WR_Byte(0x14,OLED_CMD);//--set(0x10) disable
	OLED_WR_Byte(0xA4,OLED_CMD);// Disable Entire Display On (0xa4/0xa5)
	OLED_WR_Byte(0xA6,OLED_CMD);// Disable Inverse Display On (0xa6/a7) 
	OLED_WR_Byte(0xAF,OLED_CMD);//--turn on oled panel
	
	OLED_WR_Byte(0xAF,OLED_CMD); /*display ON*/ 
	OLED_Clear();
	OLED_Set_Pos(0,0); 	
}  

uint8_t x_buf[6];
uint8_t t;
char  ptr[20];
int  a,b,c,d,e,f;
void Menu(uint8_t index,float parameter,u8 page)
{
	
	
	for(t=0;t<6;t++)
	{
		x_buf[t]=0xff;
	}
	
	x_buf[index]=0;
  if(index<5)
  x_buf[index+1]=3;
	if(index<4)
	x_buf[index+2]=6;
	
	parm_sel=page;//设置参数页
    

	 OLED_DrawBMP(13,0,32+13,2,BMP1);
	
 if(x_buf[0]!=0xff)
 { 
	 if(x_buf[0]==0) a=1 ;else a=0;
	OLED_ShowCHinese(47,x_buf[0],22,a);//航向角 1
	OLED_ShowCHinese(63,x_buf[0],10,a);//航向角 1
  OLED_ShowCHinese(79,x_buf[0],11,a);
  OLED_ShowCHinese(95,x_buf[0],9,a);
	OLED_ShowCHinese(111,x_buf[0],22,a);//航向角 1
 }
	
 if(x_buf[1]!=0xff)
 {
	  if(x_buf[1]==0) b=1;else b=0;
	OLED_ShowCHinese(47,x_buf[1],22,b);//航向角 1
	OLED_ShowCHinese(63,x_buf[1],4,b);//俯仰角  2
	OLED_ShowCHinese(79,x_buf[1],5,b);
	OLED_ShowCHinese(95,x_buf[1],9,b);
	OLED_ShowCHinese(111,x_buf[1],22,b);//航向角 1
 }
 if(x_buf[2]!=0xff)
 {
	if(x_buf[2]==0) c=1;else c=0;
	OLED_ShowCHinese(47,x_buf[2],10,c);//航向角速度 3
	OLED_ShowCHinese(63,x_buf[2],11,c);
	OLED_ShowCHinese(79,x_buf[2],9,c);
	OLED_ShowCHinese(95,x_buf[2],2,c);
	OLED_ShowCHinese(111,x_buf[2],3,c);
 }
 
 if(x_buf[3]!=0xff)
 {
	if(x_buf[3]==0) d=1;else d=0;
	OLED_ShowCHinese(47,x_buf[3],4,d);//俯仰角速度4
	OLED_ShowCHinese(63,x_buf[3],5,d);
	OLED_ShowCHinese(79,x_buf[3],9,d);
	OLED_ShowCHinese(95,x_buf[3],2,d);
	OLED_ShowCHinese(111,x_buf[3],3,d);
 }
 
 if(x_buf[4]!=0xff)
 {
	if(x_buf[4]==0) e=1;else e=0;
	OLED_ShowCHinese(47,x_buf[4],10,e);	//航向像素差5
	OLED_ShowCHinese(63,x_buf[4],11,e);
	OLED_ShowCHinese(79,x_buf[4],17,e);
	OLED_ShowCHinese(95,x_buf[4],20,e);
	OLED_ShowCHinese(111,x_buf[4],21,e);
 }
 
 if(x_buf[5]!=0xff)
 {
	if(x_buf[5]==0) f=1;else f=0;
	OLED_ShowCHinese(47,x_buf[5],4,f);		//俯仰像素差6
	OLED_ShowCHinese(63,x_buf[5],5,f);
	OLED_ShowCHinese(79,x_buf[5],17,f);
	OLED_ShowCHinese(95,x_buf[5],20,f);
	OLED_ShowCHinese(111,x_buf[5],21,f);
 }
 
  if(index==4)
 {
	OLED_ShowCHinese(47,6,22,1);		
	OLED_ShowCHinese(63,6,22,1);
	OLED_ShowCHinese(79,6,22,1);
	OLED_ShowCHinese(95,6,22,1);
	OLED_ShowCHinese(111,6,22,1);
 }else if(index==5)
 {
	OLED_ShowCHinese(47,3,22,1);		
	OLED_ShowCHinese(63,3,22,1);
	OLED_ShowCHinese(79,3,22,1);
	OLED_ShowCHinese(95,3,22,1);
	OLED_ShowCHinese(111,3,22,1);
	 
	OLED_ShowCHinese(47,6,22,1);		
	OLED_ShowCHinese(63,6,22,1);
	OLED_ShowCHinese(79,6,22,1);
	OLED_ShowCHinese(95,6,22,1);
	OLED_ShowCHinese(111,6,22,1);
 }

 

	//////
  OLED_ShowString(0,3,"       ");
	sprintf(ptr,"%3.2f",parameter/100);
 
	OLED_ShowString(0,3,(u8 *)ptr);
 /////////////
 
 if(index==2||index==3)
 {
	OLED_ShowCHinese(0,6,23,0);
  OLED_ShowString_816(16,6,"/ S");
 }
 else if(index==0||index==1)
 {
	 OLED_ShowCHinese(0,6,23,1);
	 OLED_ShowString_816(16,6,"   ");
 }else
 {
	 OLED_ShowString_816(0,6,"      ");
	 OLED_ShowString(0,6,"PIXEL");
 }
 
 if(page==1)
 { OLED_ShowString(0,1,"       ");
	 OLED_ShowString(0,1," x 0.01");
 }
 else if(page==2)
 {
	 OLED_ShowString(0,1,"       ");
	 OLED_ShowString(0,1," x 0.1");
 }
  else if(page==3)
 {
	 OLED_ShowString(0,1,"       ");
	 OLED_ShowString(0,1," x 1");
 }
  else if(page==4)
 {
	 OLED_ShowString(0,1,"       ");
	 OLED_ShowString(0,1," x 10");
 }
  else if(page==5)
 {
	 OLED_ShowString(0,1,"       ");
	 OLED_ShowString(0,1," x 100 ");
 }
 else
	 OLED_ShowString(0,1,"       ");
}



























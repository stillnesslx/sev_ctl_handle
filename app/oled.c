//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//�о�԰����
//���̵�ַ��http://shop73023976.taobao.com/?spm=2013.1.0.0.M4PqC2
//
//  �� �� ��   : main.c
//  �� �� ��   : v2.0
//  ��    ��   : HuangKai
//  ��������   : 2014-0101
//  ����޸�   : 
//  ��������   : OLED 4�ӿ���ʾ����(51ϵ��)
//              ˵��: 
//              ----------------------------------------------------------------
//              GND    ��Դ��
//              VCC  ��5V��3.3v��Դ
//              D0   ��PD6��SCL��
//              D1   ��PD7��SDA��
//              RES  ��PD4
//              DC   ��PD5
//              CS   ��PD3               
//              ----------------------------------------------------------------
// �޸���ʷ   :
// ��    ��   : 
// ��    ��   : HuangKai
// �޸�����   : �����ļ�
//��Ȩ���У�����ؾ���
//Copyright(C) �о�԰����2014/3/16
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
//OLED���Դ�
//��Ÿ�ʽ����.
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

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2 ,ENABLE);	   //ʹ��SPI2����ʱ��

    /* ���� SPI2 ����: SCK, MISO and MOSI��PB13, PB14, PB15) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;          //���ù��ܣ����죩���  SPI2
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Ƭѡ  PB0 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;			  					 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		   //���ģʽ����ٶ�50MHz
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		   //ͨ���������ģʽ
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /*Command or data pin*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;			  					 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		   //���ģʽ����ٶ�50MHz
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		   //ͨ���������ģʽ
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /*Reset pin*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;			  					 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		   //���ģʽ����ٶ�50MHz
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		   //ͨ���������ģʽ
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* SPI2 ���� */ 
    SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;   //ȫ˫��  
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						   //��ģʽ
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					   //8λ
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;						   //ʱ�Ӽ��� ����״̬ʱ��SCK���ֵ͵�ƽ
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;						   //ʱ����λ ���ݲ����ӵ�һ��ʱ�ӱ��ؿ�ʼ
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							   //�������NSS
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;  //�����ʿ��� SYSCLK/16
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;				   //���ݸ�λ��ǰ
    SPI_InitStructure.SPI_CRCPolynomial = 7;							   //CRC����ʽ�Ĵ�����ʼֵΪ7 
    SPI_Init(SPI2, &SPI_InitStructure);

    /* ʹ��SPI2  */
    SPI_Cmd(SPI2, ENABLE);
}
void SPI2_NRF_SendByte(unsigned char byte)
{
    /* ѭ����ⷢ�ͻ������Ƿ��ǿ� */
    while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);

    /* ͨ��SPI2���跢������ */
    SPI_I2S_SendData(SPI2, byte);
}

//��SSD1106д��һ���ֽڡ�
//dat:Ҫд�������/����
//cmd:����/�����־ 0,��ʾ����;1,��ʾ����;
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
//����OLED��ʾ
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//�ر�OLED��ʾ
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   			 
//��������,������,������Ļ�Ǻ�ɫ��!��û����һ��!!!	  
//void OLED_Clear(void)  
//{  
//OLED_DrawBMP(33,0,128,8,BMP2);
//}

void OLED_Clear(void)  
{  
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //����ҳ��ַ��0~7��
		OLED_WR_Byte (0x00,OLED_CMD);      //������ʾλ�á��е͵�ַ
		OLED_WR_Byte (0x10,OLED_CMD);      //������ʾλ�á��иߵ�ַ  
   
		for(n=0;n<128;n++)OLED_WR_Byte(0,OLED_DATA); 
	} //������ʾ
}

//��ָ��λ����ʾһ���ַ�,���������ַ�
//x:0~127
//y:0~63
//mode:0,������ʾ;1,������ʾ
//size:ѡ������ 16/12 
void OLED_ShowChar(u8 x,u8 y,u8 chr)
{
    unsigned char c=0,i=0;	
    c=chr-' ';//�õ�ƫ�ƺ��ֵ
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
//    c=chr-' ';//�õ�ƫ�ƺ��ֵ
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
    //c=chr-' ';//�õ�ƫ�ƺ��ֵ
    c=chr-'0';//�õ�ƫ�ƺ��ֵ
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
//m^n����
u32 oled_pow(u8 m,u8 n)
{
	u32 result=1;
	while(n--)result*=m;
	return result;
}				  
//��ʾ2������
//x,y :�������	 
//len :���ֵ�λ��
//size:�����С
//mode:ģʽ	0,���ģʽ;1,����ģʽ
//num:��ֵ(0~4294967295);
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
//��ʾһ���ַ��Ŵ�
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

//��ʾ����
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
/***********������������ʾ��ʾBMPͼƬ128��64��ʼ������(x,y),x�ķ�Χ0��127��yΪҳ�ķ�Χ0��7*****************/
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
//��ʼ��SSD1306					    
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
	OLED_WR_Byte(0xA1,OLED_CMD);//--Set SEG/Column Mapping     0xa0���ҷ��� 0xa1����
	OLED_WR_Byte(0xC8,OLED_CMD);//Set COM/Row Scan Direction   0xc0���·��� 0xc8����
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
	
	parm_sel=page;//���ò���ҳ
    

	 OLED_DrawBMP(13,0,32+13,2,BMP1);
	
 if(x_buf[0]!=0xff)
 { 
	 if(x_buf[0]==0) a=1 ;else a=0;
	OLED_ShowCHinese(47,x_buf[0],22,a);//����� 1
	OLED_ShowCHinese(63,x_buf[0],10,a);//����� 1
  OLED_ShowCHinese(79,x_buf[0],11,a);
  OLED_ShowCHinese(95,x_buf[0],9,a);
	OLED_ShowCHinese(111,x_buf[0],22,a);//����� 1
 }
	
 if(x_buf[1]!=0xff)
 {
	  if(x_buf[1]==0) b=1;else b=0;
	OLED_ShowCHinese(47,x_buf[1],22,b);//����� 1
	OLED_ShowCHinese(63,x_buf[1],4,b);//������  2
	OLED_ShowCHinese(79,x_buf[1],5,b);
	OLED_ShowCHinese(95,x_buf[1],9,b);
	OLED_ShowCHinese(111,x_buf[1],22,b);//����� 1
 }
 if(x_buf[2]!=0xff)
 {
	if(x_buf[2]==0) c=1;else c=0;
	OLED_ShowCHinese(47,x_buf[2],10,c);//������ٶ� 3
	OLED_ShowCHinese(63,x_buf[2],11,c);
	OLED_ShowCHinese(79,x_buf[2],9,c);
	OLED_ShowCHinese(95,x_buf[2],2,c);
	OLED_ShowCHinese(111,x_buf[2],3,c);
 }
 
 if(x_buf[3]!=0xff)
 {
	if(x_buf[3]==0) d=1;else d=0;
	OLED_ShowCHinese(47,x_buf[3],4,d);//�������ٶ�4
	OLED_ShowCHinese(63,x_buf[3],5,d);
	OLED_ShowCHinese(79,x_buf[3],9,d);
	OLED_ShowCHinese(95,x_buf[3],2,d);
	OLED_ShowCHinese(111,x_buf[3],3,d);
 }
 
 if(x_buf[4]!=0xff)
 {
	if(x_buf[4]==0) e=1;else e=0;
	OLED_ShowCHinese(47,x_buf[4],10,e);	//�������ز�5
	OLED_ShowCHinese(63,x_buf[4],11,e);
	OLED_ShowCHinese(79,x_buf[4],17,e);
	OLED_ShowCHinese(95,x_buf[4],20,e);
	OLED_ShowCHinese(111,x_buf[4],21,e);
 }
 
 if(x_buf[5]!=0xff)
 {
	if(x_buf[5]==0) f=1;else f=0;
	OLED_ShowCHinese(47,x_buf[5],4,f);		//�������ز�6
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



























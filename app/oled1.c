//RuiXiaoliang
//20170515

#include "oled.h"
#include "oledfont.h"
#include "bsp.h"
uint8_t spi2_send_byte(uint8_t byte)
{
  /*!< Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);

  /*!< Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(SPI2, byte);

  /*!< Wait to receive a byte */
  //while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);

  /*!< Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI2);
}
void oled091_wr_byte(u8 dat,u8 cmd)
{	
	//u8 i;			  
	if(cmd)
	  OLED_DC_SET();
	else
	  OLED_DC_CLR();
	OLED091_CS_CLR();
    spi2_send_byte(dat);
//	for(i=0;i<8;i++)
//	{
//		OLED_SCLK_CLR();
//		if(dat&0x80)
//		   OLED_SDIN_SET();
//		else
//		   OLED_SDIN_CLR();
//		OLED_SCLK_SET();
//		dat<<=1;
//	}
    delay_ms(1);
	OLED091_CS_SET();
	OLED_DC_SET();
}

void OLED_Set_Pos(unsigned char x, unsigned char y) 
{ 
	oled091_wr_byte(0xb0+y,OLED_CMD);
	oled091_wr_byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	oled091_wr_byte((x&0x0f)|0x01,OLED_CMD); 
}   	  
//����OLED��ʾ    
void OLED_Display_On(void)
{
	oled091_wr_byte(0X8D,OLED_CMD);  //SET DCDC����
	oled091_wr_byte(0X14,OLED_CMD);  //DCDC ON
	oled091_wr_byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//�ر�OLED��ʾ     
void OLED_Display_Off(void)
{
	oled091_wr_byte(0X8D,OLED_CMD);  //SET DCDC����
	oled091_wr_byte(0X10,OLED_CMD);  //DCDC OFF
	oled091_wr_byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   			 
//��������,������,������Ļ�Ǻ�ɫ��!��û����һ��!!!	  
void OLED_Clear(void)  
{  
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		oled091_wr_byte (0xb0+i,OLED_CMD);    //����ҳ��ַ��0~7��
		oled091_wr_byte (0x00,OLED_CMD);      //������ʾλ�á��е͵�ַ
		oled091_wr_byte (0x10,OLED_CMD);      //������ʾλ�á��иߵ�ַ   
		for(n=0;n<128;n++)oled091_wr_byte(0,OLED_DATA); 
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
		if(SIZE ==16)
			{
			OLED_Set_Pos(x,y);	
			for(i=0;i<8;i++)
			oled091_wr_byte(F8X16[c*16+i],OLED_DATA);
			OLED_Set_Pos(x,y+1);
			for(i=0;i<8;i++)
			oled091_wr_byte(F8X16[c*16+i+8],OLED_DATA);
			}
			else {	
				OLED_Set_Pos(x,y+1);
				for(i=0;i<6;i++)
				oled091_wr_byte(F6x8[c][i],OLED_DATA);
				
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
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size/2)*t,y,' ');
				continue;
			}else enshow=1; 
		 	 
		}
	 	OLED_ShowChar(x+(size/2)*t,y,temp+'0'); 
	}
} 
//��ʾһ���ַ��Ŵ�
void OLED_ShowString(u8 x,u8 y,u8 *chr)
{
	unsigned char j=0;
	while (chr[j]!='\0')
	{		OLED_ShowChar(x,y,chr[j]);
			x+=8;
		if(x>120){x=0;y+=2;}
			j++;
	}
}
//��ʾ����
void OLED_ShowCHinese(u8 x,u8 y,u8 no)
{      			    
	u8 t,adder=0;
	OLED_Set_Pos(x,y);	
    for(t=0;t<16;t++)
		{
				oled091_wr_byte(Hzk[2*no][t],OLED_DATA);
				adder+=1;
     }	
		OLED_Set_Pos(x,y+1);	
    for(t=0;t<16;t++)
			{	
				oled091_wr_byte(Hzk[2*no+1][t],OLED_DATA);
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
	    	oled091_wr_byte(BMP[j++],OLED_DATA);	    	
	    }
	}
} 


//��ʼ��SSD1306					    
void OLED_Init(void)
{ 	
 
 	 
 	GPIO_InitTypeDef  GPIO_InitStructure;
 	
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //ʹ��A�˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_7;	 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//�ٶ�50MHz
 	GPIO_Init(GPIOA, &GPIO_InitStructure);	  //��ʼ��GPIOD3,6
 	GPIO_SetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_7|GPIO_Pin_4);	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //ʹ��A�˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_8;	 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//�ٶ�50MHz
 	GPIO_Init(GPIOB, &GPIO_InitStructure);	  //��ʼ��GPIOD3,6
 	GPIO_SetBits(GPIOB,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_8);	



 
  OLED_RST_SET();
	delay_ms(100);
	OLED_RST_CLR();
	delay_ms(200);
	OLED_RST_SET();
					  
oled091_wr_byte(0xAE,OLED_CMD);//�ر���ʾ
	
	oled091_wr_byte(0x40,OLED_CMD);//---set low column address
	oled091_wr_byte(0xB0,OLED_CMD);//---set high column address

	oled091_wr_byte(0xC8,OLED_CMD);//-not offset

	oled091_wr_byte(0x81,OLED_CMD);//���öԱȶ�
	oled091_wr_byte(0xff,OLED_CMD);

	oled091_wr_byte(0xa1,OLED_CMD);//���ض�������

	oled091_wr_byte(0xa6,OLED_CMD);//
	
	oled091_wr_byte(0xa8,OLED_CMD);//��������·��
	oled091_wr_byte(0x1f,OLED_CMD);
	
	oled091_wr_byte(0xd3,OLED_CMD);
	oled091_wr_byte(0x00,OLED_CMD);
	
	oled091_wr_byte(0xd5,OLED_CMD);
	oled091_wr_byte(0xf0,OLED_CMD);
	
	oled091_wr_byte(0xd9,OLED_CMD);
	oled091_wr_byte(0x22,OLED_CMD);
	
	oled091_wr_byte(0xda,OLED_CMD);
	oled091_wr_byte(0x02,OLED_CMD);
	
	oled091_wr_byte(0xdb,OLED_CMD);
	oled091_wr_byte(0x49,OLED_CMD);
	
	oled091_wr_byte(0x8d,OLED_CMD);
	oled091_wr_byte(0x14,OLED_CMD);
	
	oled091_wr_byte(0xaf,OLED_CMD); 	
}  
//end of file

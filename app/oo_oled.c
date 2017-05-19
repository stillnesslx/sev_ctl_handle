//RuiXiaoliang
//20170517
#include "oo_oled.h"
#include "oled.h"
#include "oledfont.h"  	 

#define PIXEL_X_MAX         128
#define PIXEL_Y_MAX         64

//#define POS_X_LAT           0
//#define POS_Y_LAT           0
//#define POS_X_LONG          0
//#define POS_Y_LONG          0
//#define POS_X_HIGH          0
//#define POS_Y_HIGH          0
//#define POS_X_ATT           0
//#define POS_Y_ATT           0
//#define POS_X_MODE          0
//#define POS_Y_MODE          0
//#define POS_X_STAT          0
//#define POS_Y_STAT          0
//#define POS_X_BAT           0
//#define POS_Y_BAT           0
//#define POS_X_VOLT          0
//#define POS_Y_VOLT          0
//#define POS_X_CURR          0
//#define POS_Y_CURR          0
//#define POS_X_TEMP          0
//#define POS_Y_TEMP          0
//#define POS_X_ALAM          0
//#define POS_Y_ALAM          0
uint8_t POS_X_LOGO=32;
uint8_t POS_Y_LOGO=0;
uint8_t POS_X_LAT=0;
uint8_t POS_Y_LAT=1;
uint8_t POS_X_LATV=30;
uint8_t POS_Y_LATV=1;
uint8_t POS_X_LONG=60;
uint8_t POS_Y_LONG=1;
uint8_t POS_X_LONGV=96;
uint8_t POS_Y_LONGV=1;
uint8_t POS_X_ATT=0;
uint8_t POS_Y_ATT=2;
uint8_t POS_X_ATTPV=30;
uint8_t POS_Y_ATTPV=2;
uint8_t POS_X_ATTYV=60;
uint8_t POS_Y_ATTYV=2;
uint8_t POS_X_ATTRV=90;
uint8_t POS_Y_ATTRV=2;
uint8_t POS_X_HIGH=0;
uint8_t POS_Y_HIGH=3;
uint8_t POS_X_HIGHV=30;
uint8_t POS_Y_HIGHV=3;
uint8_t POS_X_MODE=60;
uint8_t POS_Y_MODE=3;
uint8_t POS_X_MODEV=90;
uint8_t POS_Y_MODEV=3;
uint8_t POS_X_STAT=0;
uint8_t POS_Y_STAT=4;
uint8_t POS_X_STATV=30;
uint8_t POS_Y_STATV=4;
uint8_t POS_X_BAT=60;
uint8_t POS_Y_BAT=4;
uint8_t POS_X_BATV=84;
uint8_t POS_Y_BATV=4;
uint8_t POS_X_VOLT=0;
uint8_t POS_Y_VOLT=5;
uint8_t POS_X_VOLTV=30;
uint8_t POS_Y_VOLTV=5;
uint8_t POS_X_CURR=60;
uint8_t POS_Y_CURR=5;
uint8_t POS_X_CURRV=84;
uint8_t POS_Y_CURRV=5;
uint8_t POS_X_TEMP=0;
uint8_t POS_Y_TEMP=6;
uint8_t POS_X_TEMPV=24;
uint8_t POS_Y_TEMPV=6;
uint8_t POS_X_ALAM=60;
uint8_t POS_Y_ALAM=6;
uint8_t POS_X_ALAMV=84;
uint8_t POS_Y_ALAMV=6;


int parm_sel1;
void oled_242_wr_byte(u8 dat,u8 cmd);
void oled_091_wr_byte(u8 dat,u8 cmd);
//char chr_test = 0;
//struct oo_oled oled_091 = {128,32,oled_091_wr_byte,&oled_091} , oled_242 = {128,64,oled_242_wr_byte,&oled_242};
void oled_init_display(void)
{
    
    oled_show_string(POS_X_LOGO,POS_Y_LOGO,"droneyee");
    oled_show_string(POS_X_LAT,POS_Y_LAT,"lat:EXX\x6f");
    oled_show_string(POS_X_LONG,POS_Y_LONG,"long:NXX");
    oled_show_string(POS_X_HIGH,POS_Y_HIGH,"high:XXXm");
    //oled_show_char(84,2,chr_test);
    oled_show_string(POS_X_ATT,POS_Y_ATT,"att:PXX  YXX  RXX");
    oled_show_string(POS_X_MODE,POS_Y_MODE,"mode:A");
    oled_show_string(POS_X_STAT,POS_Y_STAT,"stat:O");
    oled_show_string(POS_X_BAT,POS_Y_BAT,"bat:XX%");
    oled_show_string(POS_X_VOLT,POS_Y_VOLT,"volt:XXXV");
    oled_show_string(POS_X_CURR,POS_Y_CURR,"cur:XXXA");
    oled_show_string(POS_X_TEMP,POS_Y_TEMP,"tmp:50");
    oled_show_string(POS_X_ALAM,POS_Y_ALAM,"alm:G");
}

void oled_091_wr_byte(u8 dat,u8 cmd)
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
void oled_242_wr_byte(u8 dat,u8 cmd)
{
//	u8 i;
	if(cmd)
        OLED_DC_Set();
	else
        OLED_DC_Clr();
	
	OLED242_CS_Clr();
    SPI2_NRF_SendByte(dat);
	uDelay(150);
	OLED242_CS_Set();
	OLED_DC_Set();
}

void oled_set_pos(unsigned char x, unsigned char y)
{
	oled_242_wr_byte(0xb0+y,OLED_CMD);
	oled_242_wr_byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	oled_242_wr_byte((x&0x0f)|0x01,OLED_CMD);
}
//����OLED��ʾ
void oled_display_on(void)
{
	oled_242_wr_byte(0X8D,OLED_CMD);  //SET DCDC����
	oled_242_wr_byte(0X14,OLED_CMD);  //DCDC ON
	oled_242_wr_byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//�ر�OLED��ʾ
void oled_display_off(void)
{
	oled_242_wr_byte(0X8D,OLED_CMD);  //SET DCDC����
	oled_242_wr_byte(0X10,OLED_CMD);  //DCDC OFF
	oled_242_wr_byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   			 
//��������,������,������Ļ�Ǻ�ɫ��!��û����һ��!!!	  
//void OLED_Clear(void)  
//{  
//OLED_DrawBMP(33,0,128,8,BMP2);
//}

void oled_clear(void)
{  
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		oled_242_wr_byte (0xb0+i,OLED_CMD);    //����ҳ��ַ��0~7��
		oled_242_wr_byte (0x00,OLED_CMD);      //������ʾλ�á��е͵�ַ
		oled_242_wr_byte (0x10,OLED_CMD);      //������ʾλ�á��иߵ�ַ  
   
		for(n=0;n<128;n++)oled_242_wr_byte(0,OLED_DATA); 
	} //������ʾ
}

//��ָ��λ����ʾһ���ַ�,���������ַ�
//x:0~127
//y:0~63
//mode:0,������ʾ;1,������ʾ
//size:ѡ������ 16/12 
void oled_show_char(u8 x,u8 y,u8 chr)
{
    unsigned char c=0,i=0;	
    c=chr-' ';//�õ�ƫ�ƺ��ֵ
    if(x>Max_Column-1){x=0;y=y+2;}

    oled_set_pos(x,y+1);
    for(i=0;i<6;i++)
    {
        if(!parm_sel1)
        oled_242_wr_byte(F6x8[c][i],OLED_DATA);
        else
        oled_242_wr_byte(~F6x8[c][i],OLED_DATA);
    }
}
//void oled_show_char(u8 x,u8 y,u8 chr)
//{
//    unsigned char c=0,i=0;	
//    c=chr;//�õ�ƫ�ƺ��ֵ
//    if(x>Max_Column-1){x=0;y=y+2;}

//    oled_set_pos(x,y+1);
//    for(i=0;i<6;i++)
//    {
//        if(!parm_sel1)
//        oled_242_wr_byte(ascii_6x8_sys[c][i],OLED_DATA);
//        else
//        oled_242_wr_byte(~ascii_6x8_sys[c][i],OLED_DATA);
//    }
//}
//void oled_show_char(u8 x,u8 y,u8 chr)
//{
//    unsigned char c=0,i=0;	
//    c=chr-' ';//�õ�ƫ�ƺ��ֵ
//    if(x>Max_Column-1){x=0;y=y+2;}

//    oled_set_pos(x,y+1);
//    for(i=0;i<16;i++)
//    {
//        if(!parm_sel1)
//        oled_242_wr_byte(ascii_8x16[c][i],OLED_DATA);
//        else
//        oled_242_wr_byte(~ascii_8x16[c][i],OLED_DATA);
//    }
//}

//void oled_show_char_8x16(u8 x,u8 y,u8 chr)
//{
//    unsigned char c=0,i=0;	
//    c=chr-' ';//�õ�ƫ�ƺ��ֵ
//    if(x>Max_Column-1){x=0;y=y+2;}
//    oled_set_pos(x,y);
//    for(i=0;i<8;i++)
//        oled_242_wr_byte(F8X16[c*16+i],OLED_DATA);
//    oled_set_pos(x,y+1);
//    for(i=0;i<8;i++)
//        oled_242_wr_byte(F8X16[c*16+i+8],OLED_DATA);
//}
void oled_show_char_8x16(u8 x,u8 y,u8 chr)
{
    unsigned char c=0,i=0;
    //c=chr-' ';//�õ�ƫ�ƺ��ֵ
    c=chr;//�õ�ƫ�ƺ��ֵ
    if(x>Max_Column-1){x=0;y=y+2;}
    oled_set_pos(x,y);
    for(i=0;i<8;i++)
        oled_242_wr_byte(ascii_8x16[c*16+i],OLED_DATA);
    oled_set_pos(x,y+1);
    for(i=0;i<8;i++)
        oled_242_wr_byte(ascii_8x16[c*16+i+8],OLED_DATA);
//    oled_set_pos(x,y+2);
//    for(i=0;i<16;i++)
//        oled_242_wr_byte(ascii_8x16[c*48+i+32],OLED_DATA);
}
void oled_show_string_8x16(u8 x,u8 y,u8 *chr,struct oo_oled *oled)
{
    unsigned char j=0;
    while (chr[j]!='\0')
    {
        oled_show_char_8x16(x,y,chr[j]);
        x+=6;
        if(x>120){x=0;y+=2;}
        j++;
    }
}
			  
//��ʾ2������
//x,y :�������	 
//len :���ֵ�λ��
//size:�����С
//mode:ģʽ	0,���ģʽ;1,����ģʽ
//num:��ֵ(0~4294967295);
void oled_show_number(u8 x,u8 y,u32 num,u8 len,u8 size)
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
        oled_show_char(x+(size/2)*t,y,temp+'0');
        //OLED_ShowChar_816(x+(size/2)*t,y,temp+'0');
	}
}
//��ʾһ���ַ��Ŵ�
void oled_show_string(u8 x,u8 y,char *chr)
{
	unsigned char j=0;
	while (chr[j]!='\0')
	{		oled_show_char(x,y,chr[j]);
			x+=6;
		if(x>120){x=0;y+=2;}
			j++;
	}
}

//��ʾ����
void oled_show_chinese(u8 x,u8 y,u8 no,u8 mode)
{      			    
	u8 t,adder=0;
	oled_set_pos(x,y);	
    for(t=0;t<16;t++)
		{
        if(!mode)			
				oled_242_wr_byte(Hzk[2*no][t],OLED_DATA);
				else
				oled_242_wr_byte(~Hzk[2*no][t],OLED_DATA);
				adder+=1;
     }	
		oled_set_pos(x,y+1);	
    for(t=0;t<16;t++)
			{
				if(!mode)
				oled_242_wr_byte(Hzk[2*no+1][t],OLED_DATA);
				else
				oled_242_wr_byte(~Hzk[2*no+1][t],OLED_DATA);
				adder+=1;
      }
}
/***********������������ʾ��ʾBMPͼƬ128��64��ʼ������(x,y),x�ķ�Χ0��127��yΪҳ�ķ�Χ0��7*****************/
void oled_draw_bmp(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[])
{
 unsigned int j=0;
 unsigned char x,y;
  
  if(y1%8==0) y=y1/8;      
  else y=y1/8+1;
	for(y=y0;y<y1;y++)
	{
		oled_set_pos(x0,y);
    for(x=x0;x<x1;x++)
	    {      
	    	oled_242_wr_byte(BMP[j++],OLED_DATA);	    	
	    }
	}
} 


//end of file

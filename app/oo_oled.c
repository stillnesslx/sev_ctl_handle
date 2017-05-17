//RuiXiaoliang
//20170517
#include "oo_oled.h"
#include "oled.h"
#include "oledfont.h"  	 

int parm_sel1;
void oled_242_wr_byte(u8 dat,u8 cmd);
void oled_091_wr_byte(u8 dat,u8 cmd);

struct oo_oled oled_091 = {128,32,oled_091_wr_byte,&oled_091} , oled_242 = {128,64,oled_242_wr_byte,&oled_242};

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

void oled_set_pos(unsigned char x, unsigned char y,struct oo_oled *oled)
{
	oled->wr_byte(0xb0+y,OLED_CMD);
	oled->wr_byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	oled->wr_byte((x&0x0f)|0x01,OLED_CMD);
}
//����OLED��ʾ
void oled_display_on(struct oo_oled *oled)
{
	oled->wr_byte(0X8D,OLED_CMD);  //SET DCDC����
	oled->wr_byte(0X14,OLED_CMD);  //DCDC ON
	oled->wr_byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//�ر�OLED��ʾ
void oled_display_off(struct oo_oled *oled)
{
	oled->wr_byte(0X8D,OLED_CMD);  //SET DCDC����
	oled->wr_byte(0X10,OLED_CMD);  //DCDC OFF
	oled->wr_byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   			 
//��������,������,������Ļ�Ǻ�ɫ��!��û����һ��!!!	  
//void OLED_Clear(void)  
//{  
//OLED_DrawBMP(33,0,128,8,BMP2);
//}

void oled_clear(struct oo_oled *oled)  
{  
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		oled->wr_byte (0xb0+i,OLED_CMD);    //����ҳ��ַ��0~7��
		oled->wr_byte (0x00,OLED_CMD);      //������ʾλ�á��е͵�ַ
		oled->wr_byte (0x10,OLED_CMD);      //������ʾλ�á��иߵ�ַ  
   
		for(n=0;n<128;n++)oled->wr_byte(0,OLED_DATA); 
	} //������ʾ
}

//��ָ��λ����ʾһ���ַ�,���������ַ�
//x:0~127
//y:0~63
//mode:0,������ʾ;1,������ʾ
//size:ѡ������ 16/12 
void oled_show_char(u8 x,u8 y,u8 chr,struct oo_oled *oled)
{
    unsigned char c=0,i=0;	
    c=chr-' ';//�õ�ƫ�ƺ��ֵ
    if(x>Max_Column-1){x=0;y=y+2;}

    oled_set_pos(x,y+1,oled);
    for(i=0;i<6;i++)
    {
        if(!parm_sel1)
        oled->wr_byte(F6x8[c][i],OLED_DATA);
        else
        oled->wr_byte(~F6x8[c][i],OLED_DATA);
    }
}


//void OLED_ShowChar_816(u8 x,u8 y,u8 chr)
//{
//    unsigned char c=0,i=0;	
//    c=chr-' ';//�õ�ƫ�ƺ��ֵ
//    if(x>Max_Column-1){x=0;y=y+2;}
//    oled_set_pos(x,y);	
//    for(i=0;i<8;i++)
//        oled->wr_byte(F8X16[c*16+i],OLED_DATA);
//    oled_set_pos(x,y+1);
//    for(i=0;i<8;i++)
//        oled->wr_byte(F8X16[c*16+i+8],OLED_DATA);
//}
void oled_show_char_816(u8 x,u8 y,u8 chr,struct oo_oled *oled)
{
    unsigned char c=0,i=0;	
    //c=chr-' ';//�õ�ƫ�ƺ��ֵ
    c=chr-'0';//�õ�ƫ�ƺ��ֵ
    if(x>Max_Column-1){x=0;y=y+2;}
    oled_set_pos(x,y,oled);
    for(i=0;i<16;i++)
        oled->wr_byte(F16X24[c*48+i],OLED_DATA);
    oled_set_pos(x,y+1,oled);
    for(i=0;i<16;i++)
        oled->wr_byte(F16X24[c*48+i+16],OLED_DATA);
    oled_set_pos(x,y+2,oled);
    for(i=0;i<16;i++)
        oled->wr_byte(F16X24[c*48+i+32],OLED_DATA);
}
void oled_show_string_816(u8 x,u8 y,u8 *chr,struct oo_oled *oled)
{
    unsigned char j=0;
    while (chr[j]!='\0')
    {
        oled_show_char_816(x,y,chr[j],oled);
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
void oled_show_number(u8 x,u8 y,u32 num,u8 len,u8 size,struct oo_oled *oled)
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
        oled_show_char_816(x+(size/2)*t,y,temp+'0',oled);
        //OLED_ShowChar_816(x+(size/2)*t,y,temp+'0');
	}
} 
//��ʾһ���ַ��Ŵ�
void oled_show_string(u8 x,u8 y,u8 *chr,struct oo_oled *oled)
{
	unsigned char j=0;
	while (chr[j]!='\0')
	{		oled_show_char(x,y,chr[j],oled);
			x+=6;
		if(x>120){x=0;y+=2;}
			j++;
	}
}

//��ʾ����
void oled_show_chinese(u8 x,u8 y,u8 no,u8 mode,struct oo_oled *oled)
{      			    
	u8 t,adder=0;
	oled_set_pos(x,y,oled);	
    for(t=0;t<16;t++)
		{
        if(!mode)			
				oled->wr_byte(Hzk[2*no][t],OLED_DATA);
				else
				oled->wr_byte(~Hzk[2*no][t],OLED_DATA);
				adder+=1;
     }	
		oled_set_pos(x,y+1,oled);	
    for(t=0;t<16;t++)
			{
				if(!mode)
				oled->wr_byte(Hzk[2*no+1][t],OLED_DATA);
				else
				oled->wr_byte(~Hzk[2*no+1][t],OLED_DATA);
				adder+=1;
      }
}
/***********������������ʾ��ʾBMPͼƬ128��64��ʼ������(x,y),x�ķ�Χ0��127��yΪҳ�ķ�Χ0��7*****************/
void oled_draw_bmp(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[],struct oo_oled *oled)
{
 unsigned int j=0;
 unsigned char x,y;
  
  if(y1%8==0) y=y1/8;      
  else y=y1/8+1;
	for(y=y0;y<y1;y++)
	{
		oled_set_pos(x0,y,oled);
    for(x=x0;x<x1;x++)
	    {      
	    	oled->wr_byte(BMP[j++],OLED_DATA);	    	
	    }
	}
} 


//end of file

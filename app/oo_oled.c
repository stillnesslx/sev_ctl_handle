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
//开启OLED显示
void oled_display_on(struct oo_oled *oled)
{
	oled->wr_byte(0X8D,OLED_CMD);  //SET DCDC命令
	oled->wr_byte(0X14,OLED_CMD);  //DCDC ON
	oled->wr_byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//关闭OLED显示
void oled_display_off(struct oo_oled *oled)
{
	oled->wr_byte(0X8D,OLED_CMD);  //SET DCDC命令
	oled->wr_byte(0X10,OLED_CMD);  //DCDC OFF
	oled->wr_byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   			 
//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!	  
//void OLED_Clear(void)  
//{  
//OLED_DrawBMP(33,0,128,8,BMP2);
//}

void oled_clear(struct oo_oled *oled)  
{  
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		oled->wr_byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
		oled->wr_byte (0x00,OLED_CMD);      //设置显示位置—列低地址
		oled->wr_byte (0x10,OLED_CMD);      //设置显示位置—列高地址  
   
		for(n=0;n<128;n++)oled->wr_byte(0,OLED_DATA); 
	} //更新显示
}

//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//mode:0,反白显示;1,正常显示
//size:选择字体 16/12 
void oled_show_char(u8 x,u8 y,u8 chr,struct oo_oled *oled)
{
    unsigned char c=0,i=0;	
    c=chr-' ';//得到偏移后的值
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
//    c=chr-' ';//得到偏移后的值
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
    //c=chr-' ';//得到偏移后的值
    c=chr-'0';//得到偏移后的值
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
			  
//显示2个数字
//x,y :起点坐标	 
//len :数字的位数
//size:字体大小
//mode:模式	0,填充模式;1,叠加模式
//num:数值(0~4294967295);
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
//显示一个字符号串
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

//显示汉字
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
/***********功能描述：显示显示BMP图片128×64起始点坐标(x,y),x的范围0～127，y为页的范围0～7*****************/
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

/*
* DispAdapter.c
*
* Created: 03.05.2022 13:11:21
*  Author: qfj
*/


#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "DispAdapter.h"
#include "config.h"


#include "LCD.h"
#include "display.h"
#include "display_driver.h"



void lcd_init(void){
	#ifdef ili9341
	LCD_Init();
		
	glcd_led_on();
	Orientation = Portrait;
	#endif
}


void lcd_Print(const char* Text, uint16_t X, uint16_t Y, unsigned char FontNr,
unsigned char XScale, unsigned char YScale, unsigned int ForeColor, unsigned int BackColor){
	
	#ifdef ili9341
	LCD_Print(Text,X,Y,FontNr,XScale,YScale,ForeColor,BackColor);
	#endif
}


void lcd_LOGO(uint16_t x, uint16_t y,uint16_t BackColor){
	#ifdef ili9341
	LCD_LOGO(x,y,BackColor);
	#endif
	
}


void lcd_Cls(unsigned int color){
	#ifdef ili9341
	LCD_Cls(color);
	#endif
}


void lcd_Plot(uint16_t x1, uint16_t y1, unsigned char line_type, unsigned int color){
	#ifdef ili9341
	LCD_Plot(x1,y1,line_type,color);
	#endif
}

void lcd_Box(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, unsigned int color){
	#ifdef ili9341
	LCD_Box(x1,y1,x2,y2,color);
	#endif
}

void lcd_Rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, unsigned char line_type, unsigned int color){
	#ifdef ili9341
	LCD_Rect(x1,y1,x2,y2,line_type,color);
	#endif
}

void lcd_Draw(uint16_t x1, uint16_t y1,uint16_t x2, uint16_t y2, unsigned char line_type, unsigned int color){
	#ifdef ili9341
	LCD_Draw(x1,y1,x2,y2,line_type,color);
	#endif
}

void lcd_hline(unsigned int x0, unsigned int y0, unsigned int length, unsigned int color){
	#ifdef ili9341
	LCD_hline(x0,y0,length,color);
	#endif
}

void lcd_vline(unsigned int x0, unsigned int y0, unsigned int length, unsigned int color){
	#ifdef ili9341
	LCD_vline(x0,y0,length,color);
	#endif
}

void lcd_Draw_Cross(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1){
	#ifdef ili9341
	LCD_Draw_Cross(x0,y0,x1,y1);
	#endif
}
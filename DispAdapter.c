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
#include "StringPixelCoordTable_ili9341.h"



MainscreenType MainScreen = {
	.Value.x = X_LEFT_EDGE,
	.Value.y = Y_VALUES_START,
	.Volume.x = X_LEFT_EDGE,
	.Volume.y = Y_VALUES_START + FONT2_H * 1,
	.Corr.x = X_LEFT_EDGE,
	.Corr.y = Y_VALUES_START + FONT2_H * 2,
	.Temp.x = X_LEFT_EDGE,
	.Temp.y = Y_VALUES_START + FONT2_H * 3,
	.Press.x = X_LEFT_EDGE,
	.Press.y = Y_VALUES_START + FONT2_H * 4,
};


void lcd_init(void){
	#ifdef ili9341
	LCD_Init();
	
	glcd_led_on();
	Orientation = Portrait;
	#endif
	
	#ifdef old_LCD
	init_LCD();
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

void Print_add_Line(char* Text,uint8_t first_line ){
	#ifdef ili9341
	InitScreen_AddLine_ili(Text,first_line);
	#endif
	#ifdef old_LCD
	LCD_InitScreen_AddLine(Text,first_line);
	#endif
}

void setInitScreen(uint16_t fore, uint16_t back, uint8_t nextLine, uint8_t FontNr, uint8_t XScale, uint8_t YScale){
	#ifdef ili9341
	setInitScreen_ili(fore,back,nextLine,FontNr,XScale,YScale);
	#endif // ili9341
}

void paint_info_line(char * line, _Bool update){
	#ifdef ili9341
	paint_info_line_ili(line,update);
	#endif
	
	#ifdef old_LCD
	LCD_paint_info_line(*line,update);
	#endif
}


void paint_value(char* text,uint8_t update,char* unit){
	if (update)
	{
		lcd_Print("               ",MainScreen.Value.x + DESCRUPTOR_LEN * FONT2_W,MainScreen.Value.y,2,1,1,FGC,BGC);
	}
	lcd_Print(text,MainScreen.Value.x + DESCRUPTOR_LEN * FONT2_W,MainScreen.Value.y,2,1,1,FGC,BGC);
	
	lcd_Print(unit,HALF_SPACE_WIDTH_FONT_2 + MainScreen.Value.x + (DESCRUPTOR_LEN+strlen(text)) * FONT2_W, MainScreen.Value.y,2,1,1,FGC,BGC);
}
void paint_volume(char* text,uint8_t update,char* unit){
	if (update)
	{
		lcd_Print("               ",MainScreen.Volume.x + DESCRUPTOR_LEN * FONT2_W,MainScreen.Volume.y,2,1,1,FGC,BGC);
	}
	lcd_Print(text,MainScreen.Volume.x + DESCRUPTOR_LEN * FONT2_W,MainScreen.Volume.y,2,1,1,FGC,BGC);
	
	lcd_Print(unit,HALF_SPACE_WIDTH_FONT_2 + MainScreen.Volume.x + (DESCRUPTOR_LEN+strlen(text)) * FONT2_W, MainScreen.Volume.y,2,1,1,FGC,BGC);
	
}

void paint_corr(char* text,uint8_t update,char* unit){
	if (update)
	{
		lcd_Print("               ",MainScreen.Corr.x + DESCRUPTOR_LEN * FONT2_W,MainScreen.Corr.y,2,1,1,FGC,BGC);
	}
	lcd_Print(text,MainScreen.Corr.x + DESCRUPTOR_LEN * FONT2_W,MainScreen.Corr.y,2,1,1,FGC,BGC);
	
	lcd_Print(unit,HALF_SPACE_WIDTH_FONT_2 + MainScreen.Corr.x + (DESCRUPTOR_LEN+strlen(text)) * FONT2_W, MainScreen.Corr.y,2,1,1,FGC,BGC);
}
void paint_temp(char* text,uint8_t update,char* unit){
	if (update)
	{
		lcd_Print("               ",MainScreen.Temp.x + DESCRUPTOR_LEN * FONT2_W,MainScreen.Temp.y,2,1,1,FGC,BGC);
	}
	lcd_Print(text,MainScreen.Temp.x + DESCRUPTOR_LEN * FONT2_W,MainScreen.Temp.y,2,1,1,FGC,BGC);
	
	lcd_Print(unit,HALF_SPACE_WIDTH_FONT_2 + MainScreen.Temp.x + (DESCRUPTOR_LEN+strlen(text)) * FONT2_W, MainScreen.Temp.y,2,1,1,FGC,BGC);
}

void paint_press(char* text,uint8_t update,char* unit){
	if (update)
	{
		lcd_Print("               ",MainScreen.Press.x + DESCRUPTOR_LEN * FONT2_W,MainScreen.Press.y,2,1,1,FGC,BGC);
	}
	lcd_Print(text,MainScreen.Press.x + DESCRUPTOR_LEN * FONT2_W,MainScreen.Press.y,2,1,1,FGC,BGC);

	lcd_Print(unit,HALF_SPACE_WIDTH_FONT_2 + MainScreen.Press.x + (DESCRUPTOR_LEN+strlen(text)) * FONT2_W, MainScreen.Press.y,2,1,1,FGC,BGC);
}

void paint_Main(void){
	lcd_Print("Val:",MainScreen.Value.x ,MainScreen.Value.y,2,1,1,FGC,BGC);
	lcd_Print("Vol:",MainScreen.Volume.x ,MainScreen.Volume.y,2,1,1,FGC,BGC);
	lcd_Print("Cor:",MainScreen.Corr.x ,MainScreen.Corr.y,2,1,1,FGC,BGC);
	lcd_Print("Tmp:",MainScreen.Temp.x ,MainScreen.Temp.y,2,1,1,FGC,BGC);
	lcd_Print("Prs:",MainScreen.Press.x ,MainScreen.Press.y,2,1,1,FGC,BGC);
} 



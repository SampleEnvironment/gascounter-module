/*
* DispAdapter.c
*
* Created: 03.05.2022 13:11:21
*  Author: qfj
*/


#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "DispAdapter.h"
#include "config.h"


#ifdef GCM_old_disp

#include "disp/gcm_old_lcd_driver.h"
#endif

#include "disp/display_lib.h"
#include "disp/ili9341_driver.h"


#include "StringPixelCoordTable_ili9341.h"

InitScreenType InitScreen_ili = {
	.ForeColor = ERR,
	.BackColor = BGC,
	.MaxNoOfLines = IAdd_Line_Max_Lines,
	.NextLine = 1,
	.LineFeed = IAdd_Line_LineFeed,
	.FontNr = 1,
	.XScale = 1,
	.YScale = 1
};



void lcd_init(void){
	#ifdef ili9341
	LCD_Init();
	
	glcd_led_on();
	Orientation = Portrait;
	#endif
	
	#ifdef GCM_old_disp
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

	#ifdef GCM_old_disp
		LCD_Clear();
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
	#ifdef GCM_old_disp
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
	
	#ifdef GCM_old_disp
	LCD_paint_info_line(line,update);
	#endif
}






void paint_string_row(char *text,ROW_NAME row,uint8_t update,char* unit,uint16_t color){
		if (update)
		{
			lcd_Print("               ",X_LEFT_EDGE + DESCRUPTOR_LEN * FONT2_W,Y_VALUES_START + FONT2_H * row,2,1,1,color,BGC);
		}
		
		strcat(text," ");
		
		lcd_Print(text,X_LEFT_EDGE + DESCRUPTOR_LEN * FONT2_W,Y_VALUES_START + FONT2_H * row,2,1,1,color,BGC);

		lcd_Print(unit,HALF_SPACE_WIDTH_FONT_2 + X_LEFT_EDGE + (DESCRUPTOR_LEN+strlen(text)-1) * FONT2_W, Y_VALUES_START + FONT2_H * row,2,1,1,color,BGC);
}

void paint_Main(void){
	lcd_Print("Inf:",X_LEFT_EDGE ,Y_VALUES_START + FONT2_H * INFO  ,2,1,1,FGC,BGC);
	lcd_Print("Val:",X_LEFT_EDGE ,Y_VALUES_START + FONT2_H * VALUE  ,2,1,1,FGC,BGC);
	lcd_Print("Vol:",X_LEFT_EDGE ,Y_VALUES_START + FONT2_H * VOLUME ,2,1,1,FGC,BGC);
	lcd_Print("Cor:",X_LEFT_EDGE ,Y_VALUES_START + FONT2_H * CORRVOL,2,1,1,FGC,BGC);
	lcd_Print("Tmp:",X_LEFT_EDGE ,Y_VALUES_START + FONT2_H * TEMP   ,2,1,1,FGC,BGC);
	lcd_Print("Prs:",X_LEFT_EDGE ,Y_VALUES_START + FONT2_H * PRESS  ,2,1,1,FGC,BGC);
	lcd_Print("Con:",X_LEFT_EDGE ,Y_VALUES_START + FONT2_H * CONN  ,2,1,1,FGC,BGC);
} 



void paint_Value(uint64_t val,ROW_NAME row,uint8_t precision, uint8_t min_width,char* unit){
	char numberbuffer[30];
	dtostrf(((double)val)/pow(10,precision),min_width,precision,numberbuffer);
	
	paint_string_row(numberbuffer,row,0,unit,white);
}

void paint_Error(char* text, ROW_NAME row){
	paint_string_row(text,row,1,"",red);
}

//=========================================================================
// Init screen with messages appearing one under the other
//=========================================================================

// Print line of InitScreen
// if FirstLine is 1 clear page and start with first line
void InitScreen_AddLine_ili(const char* Text, const char FirstLine)
{
	if (FirstLine == 1)
	{
		InitScreen_ili.NextLine = 1;
	}
	if (InitScreen_ili.NextLine == 1) LCD_Cls(InitScreen_ili.BackColor);  // Clear Screen, before first line is written

	if (FirstLine == 2)
	{
		--InitScreen_ili.NextLine;
		LCD_Print("                     ", X_IA_2, 30 + InitScreen_ili.NextLine * InitScreen_ili.LineFeed, InitScreen_ili.FontNr, InitScreen_ili.XScale, InitScreen_ili.YScale, InitScreen_ili.ForeColor, InitScreen_ili.BackColor);
	}

	
	LCD_Print(Text, X_IA_2,30 + InitScreen_ili.NextLine * InitScreen_ili.LineFeed, InitScreen_ili.FontNr, InitScreen_ili.XScale, InitScreen_ili.YScale, InitScreen_ili.ForeColor, InitScreen_ili.BackColor);
	++InitScreen_ili.NextLine;
	if (InitScreen_ili.NextLine > InitScreen_ili.MaxNoOfLines)
	{
		InitScreen_ili.NextLine = 1;
		_delay_ms(1000);		  // wait until next page is displayed
	}
	_delay_ms(500);

}

void setInitScreen_ili(uint16_t fore, uint16_t back, uint8_t nextLine, uint8_t FontNr, uint8_t XScale, uint8_t YScale){
	InitScreen_ili.ForeColor = fore;
	InitScreen_ili.BackColor = back;
	InitScreen_ili.NextLine = nextLine;
	InitScreen_ili.FontNr = FontNr;
	InitScreen_ili.XScale = XScale;
	InitScreen_ili.YScale = YScale;
}


void paint_info_line_ili(char *line, _Bool update)
{
	if (!update) LCD_Print("                      ",  X_PIL_2, Y_PIL_90, 1, 1, 1, FGC, BGC);  // clears line (not necessary if in update mode)
	LCD_Print(line,  X_PIL_2, Y_PIL_90, 1, 1, 1, ERR, BGC);
}
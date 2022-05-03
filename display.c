//
#define RELEASE_DISPLAY 1.03

// HISTORY -------------------------------------------------------------
// 1.00 - First release on March 2016
// 1.03 - See comments at the end of this file
// 1.03_custom - adapted HZB Klaus Kiefer 2018
// 1.03_withoffset - adapted HZB Klaus Kiefer 2019

// (c) 2007 Speed-IT-up (Display3000), Peter und Stefan Küsters
// es ist gestattet, diese Routinen in eigene Programme einzubauen,
// wenn die folgenden 6 eingrahmten Zeilen im Kopf Ihres Sourcecodes stehen.
//
// ------------------------------------------------------------------------------------------------------------
// Display-Software-Grundlagen wurden von Peter Küsters, www.display3000.com ermittelt
// Dieser Display-Code ist urheberrechtlich geschützt. Sie erhalten eine Source-Code-Lizenz,
// d.h. Sie dürfen den Code in eigenen Programmen verwenden, diese aber nur in kompilierter
// Form weitergeben. Die Weitergabe dieses Codes in lesbarer Form oder die Publizierung
// im Internet etc. ist nicht gestattet und stellen einen Verstoß gegen das Urheberrecht dar.
// Weitere Displays, Platinen, Spezialstecker und fertige Module: www.display3000.com
// ------------------------------------------------------------------------------------------------------------
//
// Dieses Beispiel hier läuft auf dem Board D071, D072, D073 von Display 3000












#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <avr/pgmspace.h>

#include <string.h>
#include <stdbool.h>


#include "display.h"
#include "StringPixelCoordTable_ili9341.h"
#include "display_driver.h"
#include "DispAdapter.h"





InitScreenType InitScreen_ili = {
	.ForeColor = ERR,
	.BackColor = BGC,
	.MaxNoOfLines = IAdd_Line_Max_Lines,
	.NextLine = 1,
	.LineFeed = IAdd_Line_LineFeed,
	.FontNr = 1,
	.XScale = 1,
.YScale = 1};








unsigned int WindowWidth(void)
{
	if (Orientation==Portrait || Orientation==Portrait180)
	return MAX_X;
	else
	return MAX_Y;
}

unsigned int WindowHeight(void)
{
	if (Orientation==Portrait || Orientation==Portrait180)
	return MAX_Y;
	else
	return MAX_X;
}

struct Logo
{
	const uint16_t w ;
	const uint16_t h ;
	const uint16_t data_len;
	const uint8_t bytes_per_line;
	const uint8_t h_Blue;
};







struct Logo HZB_logo = {.w = 160,.h = 56, .data_len = 1120,.bytes_per_line = 20,.h_Blue = 37};

extern const uint8_t HZB_LOGO_DISP_3000[] PROGMEM;


extern const uint8_t Font3[],Font4[],Font5[] PROGMEM;	// Shared font arrays stored in Program-Memory

TFontInfo FontInfo[3] =
{
	{ 8, 16,  8, 16}, // normal font
	{12, 24, 12, 24}, // large font
	{FONT3_W, FONT3_H, FONT3_W, FONT3_H}  // He-Level font (extra large)
};












void LCD_Print(const char* Text, uint16_t X, uint16_t Y, unsigned char FontNr,
unsigned char XScale, unsigned char YScale, unsigned int ForeColor, unsigned int BackColor){
	/*
	if (--FontNr > 1) FontNr = 1;
	const uint8_t* Font PROGMEM = (FontNr==0) ? Font3 : Font4;
	*/

	const uint8_t* Font ;//PROGMEM;
	
	switch (FontNr){
		case 1:  Font = Font3; break;
		case 2:  Font = Font4; break;
		case 3:  Font = Font5; break;
		default: Font = Font3; break;
	}
	--FontNr;
	const unsigned int Len = strlen(Text);
	const unsigned int CharWidth = FontInfo[FontNr].CharWidth;
	const unsigned int CharHeight = FontInfo[FontNr].CharHeight;
	const uint16_t BytePerChar = (CharHeight*CharWidth)/8; // number of Byte per line in Font Table
	const uint8_t BytesPerVertLine = CharHeight/8;//1 + ((CharHeight - 1) / 8); // Rounding Up Integer divisin only would be floor(Charheight/8)
	const uint8_t Scale = XScale;
	const uint8_t ScaleArea =  Scale * Scale;
	
	
	for (uint16_t LetterIndex = 0;LetterIndex<Len;LetterIndex++){
		unsigned char Ch = Text[LetterIndex];				// Ch is the position of character data in the font table
		if (Ch > 122)								// At the end of the font data some german special characters are defined
		{
			switch (Ch)								// special treatment eliminates the storage of unused data
			{
				case 228: Ch = 127; break;			// ä is at Pos. 127 etc.
				case 246: Ch = 128; break;			// ö
				case 252: Ch = 129; break;			// ü
				case 196: Ch = 130; break;			// Ä
				case 214: Ch = 131; break;			// Ö
				case 220: Ch = 132; break;			// Ü
				case 223: Ch = 133; break;			// ß
				default: Ch = '?'; break;				// not allowed: change to ?
			}
		}
		if (Ch != 32 && FontNr == 2 ){
			if (Ch < 64)
			{
				Ch -= 47;
			}
			else
			{
				Ch -= 54;
			}
		}
		else{
			Ch -= 32;
		}
		
		
		const uint16_t BytePos = Ch * BytePerChar; //determines Line in Font Table
		
		//uint8_t VertlineBMP[3]={0};

		for(uint8_t vertline = 0;vertline <CharWidth;vertline++){
			
			LCD_Window(X, Y, X+Scale-1, Y+(CharHeight*Scale));// Set character window
			for (uint8_t i =0;i<BytesPerVertLine;i++){
				uint8_t Byte = pgm_read_byte(&Font[BytePos+i+vertline*BytesPerVertLine]);
				
				
				for (int Charbit=0; Charbit<=7; ++Charbit)	// check each bit of this character line
				{
					const int bShowPixel = (Byte >> Charbit) & 0x01;	// check if pixel is on or off (select appropriate bit)
					const unsigned int Color = bShowPixel ?
					ForeColor :										// pixel = on: foreground color
					BackColor;										// pixel = off: background color
					//_delay_ms(100);
					for(uint8_t i = 0;i<ScaleArea;i++){
						LCD_SPI_Int(Color);								// draw pixel
					}
				}										// check next bit of this character
				
			}
			
			glcd_cs_high(); //Disable chipselect

			// next row of this character
			X+= Scale;
		}
		if (FontNr == 2)
		{
			X -= 10;
		}

	}


}






void LCD_LOGO(uint16_t x, uint16_t y,uint16_t BackColor){

	uint16_t ForeColor = HZB_Blue;



	LCD_Window(x, y, x+HZB_logo.w-1, y+HZB_logo.h);// Set character window

	uint16_t curr_Pixel = 0;
	
	for (uint16_t arr_index = 0; arr_index < HZB_logo.data_len; arr_index++)
	{


		uint8_t Byte  = pgm_read_byte(&HZB_LOGO_DISP_3000[arr_index]);

		


		

		for (int BMPbit=7; BMPbit >= 0; --BMPbit)	// check each bit of this  line
		{
			if ((curr_Pixel > HZB_logo.w / 2) || (arr_index/HZB_logo.bytes_per_line > HZB_logo.h_Blue))
			{
				ForeColor = HZB_Cyan;
			}else
			{
				ForeColor = HZB_Blue;
			}
			
			const uint8_t bShowPixel = (Byte >> BMPbit) & 0x01;	// check if pixel is on or off (select appropriate bit)
			const uint16_t Color = bShowPixel ?
			ForeColor :										// pixel = on: foreground color
			BackColor;										// pixel = off: background color
			
			LCD_SPI_Int(Color);								// draw pixel
			curr_Pixel++;

			
			if (curr_Pixel >= HZB_logo.w)
			{
				curr_Pixel = 0;
				break;
			}
			

		}

	}


	glcd_cs_high(); //Disable chipselect




}

//-------------------------------------------------------------------------------
//Clear display
//-------------------------------------------------------------------------------

void LCD_Cls(unsigned int color)
{
	for(unsigned int i = 0; i<=WindowHeight();i++){
		LCD_hline(0,i,WindowWidth()+1,color);
	}
	
	/*
	for(unsigned int i = 0; i<=WindowWidth();i++){
	LCD_vline(i,0,WindowHeight()+1,color);
	}
	*/
}

//-------------------------------------------------------------------------------
// Plot one pixel to the display
//-------------------------------------------------------------------------------
void LCD_Plot(uint16_t x1, uint16_t y1, unsigned char line_type, unsigned int color)
{

	if (line_type == THICK)
	{
		LCD_Window(x1, y1, x1+1, y1+1); 									// Define Window, exactly four pixels large
		LCD_SPI_Int(color);													// send four pixels (avoid loop overhead)
		LCD_SPI_Int(color);
		LCD_SPI_Int(color);
		LCD_SPI_Int(color);
	}
	else
	{
		LCD_Window(x1, y1, x1, y1); 										// Define Window, exactly one pixel large
		LCD_SPI_Int(color);													// send pixel
	}

	glcd_cs_high(); //Disable chipselect

}

//-------------------------------------------------------------------------------
// Draw a rectangular filled box
//-------------------------------------------------------------------------------
void LCD_Box(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, unsigned int color)
{

	LCD_Window(x1, y1, x2, y2);   										// Define Window
	for (unsigned int i=0; i<((x2-x1+1)*(y2-y1+1)); ++i)			// for every pixel...
	LCD_SPI_Int(color); 											// ...send color information

	glcd_cs_high(); //Disable chipselect


}

//-------------------------------------------------------------------------------
// Draw a rectangular (not filled)
//-------------------------------------------------------------------------------
void LCD_Rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, unsigned char line_type, unsigned int color)	//draw rectangle
{
	/*
	LCD_hline(x1, y1,  x2-x1, color);
	LCD_hline(x1, y2,  x2-x1, color);
	LCD_vline(x1, y1,  x2-x1, color);
	LCD_vline(x2, y1,  x2-x1, color);
	*/
	LCD_Draw(x1, y1, x2, y1, line_type, color);
	LCD_Draw(x2, y1, x2, y2, line_type, color);
	LCD_Draw(x1, y2, x2, y2, line_type, color);
	LCD_Draw(x1, y1, x1, y2, line_type, color);
}

//-------------------------------------------------------------------------------
// Draw a line
//-------------------------------------------------------------------------------
void LCD_Draw(uint16_t x1, uint16_t y1,uint16_t x2, uint16_t y2, unsigned char line_type, unsigned int color)//draw line - bresenham algorithm
{
	


	int x = x1;
	int y = y1;
	int d = 0;
	int hx = x2 - x1;    									// how many pixels on each axis
	int hy = y2 - y1;
	int xinc = 1;
	int yinc = 1;
	if (hx < 0)
	{
		xinc = -1;
		hx = -hx;
	}
	if (hy < 0)
	{
		yinc = -1;
		hy = -hy;
	}
	if (hy <= hx)
	{
		int c = 2 * hx;
		int m = 2 * hy;
		while (x != x2)
		{
			LCD_Plot(x, y, line_type, color);
			x += xinc;
			d += m;
			if (d > hx)
			{
				y += yinc;
				d -= c;
			}
		}
	}
	else
	{
		int c = 2 * hy;
		int m = 2 * hx;
		while (y != y2)
		{
			LCD_Plot(x, y, line_type, color);
			y += yinc;
			d += m;
			if (d > hy)
			{
				x += xinc;
				d -= c;
			}
		}
	}
	LCD_Plot(x, y, line_type, color); 												  // finally, the last pixel
}



void LCD_hline(unsigned int x0, unsigned int y0, unsigned int length, unsigned int color)
{
	
	
	LCD_Window(x0,y0,x0+length,y0);
	for(unsigned int i=0; i<length; i++){
		LCD_SPI_Int(color);
	}

	glcd_cs_high(); //Disable chipselect


}


void LCD_vline(unsigned int x0, unsigned int y0, unsigned int length, unsigned int color)
{
	

	
	LCD_Window(x0,y0,x0,y0+length);
	for(unsigned int i=0; i<length; i++){
		LCD_SPI_Int(color);
	}

	glcd_cs_high(); //Disable chipselect


}








void LCD_Draw_Cross(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1){
	LCD_Draw(x0,y0,x1,y1,0,red);
	LCD_Draw(x0,y1,x1,y0,0,red);
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



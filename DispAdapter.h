/*
 * DispAdapter.h
 *
 * Created: 03.05.2022 13:11:02
 *  Author: qfj
 */ 


#ifndef DISPADAPTER_H_
#define DISPADAPTER_H_

#include "disp/ili9341_driver.h"

// COLORS
#define bright_blue		0b1101111011011111	//Predefined colors will make programmers life easier
#define blue			0b0000000000011111
#define dark_blue		0b0000000000010011
#define bright_yellow	0b1111111111001100	//as the display uses 65.536 colors we have to define double-bytes for each color
#define yellow			0b1111111111100000	//Check the programmers manual to learn how to define your own color
#define orange			0b1111110011000110
#define bright_red		0b1111100011100011
#define red				0b1111100000000000
#define dark_red		0b1001100000000000
#define bright_green 	0b1001111111110011
#define green 			0b0000011111100000
#define dark_green 		0b0000001101100000
#define white 			0b1111111111111111
#define grey 			0b0011100011100111
#define black 			0b0000000000000000

///Layout Colors
#define FGC 0b0101101011011111				///< Foreground color
#define BGC black				///< Background color
#define ERR	white				///< Warning/Active color
#define D_BGC blue				///< Dialog Background color
#define D_FGC white				///< Dialog Foreground color

#define HZB_Blue        0b0000001011010011
#define HZB_Cyan        0b0000010011111100

typedef enum {
	VERSION,
	DATETIME,
	VALUE,
	VOLUME,
	CORRVOL,
	TEMP,
	PRESS,
	CONN,
	MULT1,
	MULT2
}ROW_NAME;

typedef struct {
	uint16_t ForeColor;
	uint16_t BackColor;
	uint8_t  MaxNoOfLines;
	uint8_t  NextLine;
	uint8_t  LineFeed;
	uint8_t  FontNr;
	uint8_t  XScale;
	uint8_t  YScale;
}InitScreenType;

extern EWindowOrientation Orientation;
extern char NetStat[3][6+1] ;



void lcd_init(void);
void lcd_Print(const char* Text, uint16_t X, uint16_t Y, unsigned char FontNr, unsigned char XScale, unsigned char YScale, unsigned int ForeColor, unsigned int BackColor);
void lcd_LOGO(uint16_t x, uint16_t y,uint16_t BackColor);
void lcd_Cls(unsigned int color);
void lcd_Plot(uint16_t x1, uint16_t y1, unsigned char line_type, unsigned int color);
void lcd_Box(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, unsigned int color);
void lcd_Rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, unsigned char line_type, unsigned int color);
void lcd_Draw(uint16_t x1, uint16_t y1,uint16_t x2, uint16_t y2, unsigned char line_type, unsigned int color);
void lcd_hline(unsigned int x0, unsigned int y0, unsigned int length, unsigned int color);
void lcd_vline(unsigned int x0, unsigned int y0, unsigned int length, unsigned int color);
void lcd_Draw_Cross(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1);
void Print_add_Line(char* Text,uint8_t first_line );
void setInitScreen(uint16_t fore, uint16_t back, uint8_t nextLine, uint8_t FontNr, uint8_t XScale, uint8_t YScale);
void paint_info_line(char * line, _Bool update);
void paint_string_row_col(char *text,ROW_NAME row,uint8_t col, uint16_t color);
void paint_string_row(char *text,ROW_NAME row,uint8_t update,char* unit,uint16_t color);
void paint_Value(uint64_t val,ROW_NAME row,uint8_t precision, uint8_t min_width,char* unit);
void paint_Error(char* text, ROW_NAME row);
void paint_Main(void);
void InitScreen_AddLine_ili(const char* Text, const char FirstLine);
void setInitScreen_ili(uint16_t fore, uint16_t back, uint8_t nextLine, uint8_t FontNr, uint8_t XScale, uint8_t YScale);
void paint_info_line_ili(char *line, _Bool update);
void paint_store_meas(uint8_t meas_in_Buffer, uint8_t max_Number );
void paint_send_stored_meas(uint8_t meas_in_Buffer, uint8_t max_Number, uint8_t update);
void displayTemPreVol(void);
void reset_display(uint8_t clear);
void I2C_Clear_view(uint8_t i2cState,uint8_t DS3231State, uint8_t BMPSate);
void paint_Date(void);

#endif /* DISPADAPTER_H_ */
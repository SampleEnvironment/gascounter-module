#include "display_driver.h"








/*
* ----------------------------------------------------------------------------
* "THE BEER-WARE LICENSE" (Revision 42):
* <propaliidealist@gmail.com> wrote this file. As long as you retain this notice you
* can do whatever you want with this stuff. If we meet some day, and you think
* this stuff is worth it, you can buy me a beer in return. Johnny Sorocil
* ----------------------------------------------------------------------------
*/


#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <stdlib.h>	// abs()
#include <string.h>	// strlen()



#include "disp/ili9341cmd.h"
#include "disp/spi.h" //SPI-Utilities for communicating with Display
//#include "display.h" 

EWindowOrientation Orientation = Landscape180;
EWindowOrientation ili_Orientation = Portrait180;



//-------------------------------------------------------------------------------
//ili9341Display Specific
//-------------------------------------------------------------------------------
void glcd_cs_low() {
	/*
	DDRD |= 0b100000;	// PD5 is output
	PORTD &=~ 0b100000;	// PD5 is 0
	*/
	RESET(LCD_SELECT);
}
void glcd_cs_high() {
	/*
	DDRD |= 0b100000;
	PORTD |=  0b100000;
	*/
	SET(LCD_SELECT);
}
void glcd_dc_low() {
	/*
	DDRD |= 0b1000000;	// PD6
	PORTD &=~ 0b1000000;
	*/
	RESET(LCD_DC);
}
void glcd_dc_high() {
	/*
	DDRD |= 0b1000000;
	PORTD |=  0b1000000;
	*/
	SET(LCD_DC);
}
void glcd_led_off() {
	/*
	DDRD |= 0b10000000;	// PD7
	PORTD &=~ 0b10000000;
	*/


	PORTD &= ~_BV(PIND6);
}
void glcd_led_on() {
	/*
	DDRD |= 0b10000000;
	PORTD |=  0b10000000;
	*/

	PORTD |= _BV(PIND6);

}
void glcd_rst_off() {
	/*
	DDRD |= 0b10000;	// PD4
	PORTD |=  0b10000;
	*/
	RESET(LCD_RESET);
}
void glcd_rst_on() {
	/*
	DDRD |= 0b10000;
	PORTD &=~ 0b10000;
	*/
	SET(LCD_RESET);
}

void glcd_sendCmd(unsigned char data)
{
	
	glcd_dc_low();
	glcd_cs_low();
	SPI_MasterTransmit(data);
	glcd_cs_high();
}


void glcd_sendData(unsigned char data)
{
	
	glcd_dc_high();
	glcd_cs_low();
	SPI_MasterTransmit(data);
	glcd_cs_high();
}

void glcd_setX(unsigned int x0,unsigned int x1)
{
	glcd_sendCmd(ILI9341_CMD_COLUMN_ADDRESS_SET);
	glcd_dc_high();
	glcd_cs_low();
	LCD_SPI_Int(x0);
	LCD_SPI_Int(x1);
	glcd_cs_high();

}

void glcd_setY(unsigned int y0,unsigned int y1)
{
	glcd_sendCmd(ILI9341_CMD_PAGE_ADDRESS_SET);
	glcd_dc_high();
	glcd_cs_low();
	LCD_SPI_Int(y0);
	LCD_SPI_Int(y1);
	glcd_cs_high();
}




#define ILI9341_PWCTR1     0xC0     ///< Power Control 1
#define ILI9341_PWCTR2     0xC1     ///< Power Control 2
#define ILI9341_VMCTR1     0xC5     ///< VCOM Control 1
#define ILI9341_VMCTR2     0xC7     ///< VCOM Control 2
#define ILI9341_MADCTL     0x36     ///< Memory Access Control
#define ILI9341_VSCRSADD   0x37     ///< Vertical Scrolling Start Address
#define ILI9341_RDPIXFMT   0x0C     ///< Read Display Pixel Format
#define ILI9341_FRMCTR1    0xB1     ///< Frame Rate Control (In Normal Mode/Full Colors)
#define ILI9341_DFUNCTR    0xB6     ///< Display Function Control
#define ILI9341_GAMMASET   0x26     ///< Gamma Set
#define ILI9341_PIXFMT     0x3A     ///< COLMOD: Pixel Format Set
#define ILI9341_GMCTRP1    0xE0     ///< Positive Gamma Correction
#define ILI9341_GMCTRN1    0xE1     ///< Negative Gamma Correction
#define ILI9341_SLPOUT     0x11     ///< Sleep Out
#define ILI9341_DISPON     0x29     ///< Display ON



//-------------------------------------------------------------------------------
//DRIVER INTERFACE TO DISPLAY.H
//-------------------------------------------------------------------------------


//-------------------------------------------------------------------------------
// Initialize the display
//-------------------------------------------------------------------------------
void LCD_Init(void){
	DDRB = 255; //all Ports to output
	SPI_MasterInit();

	glcd_cs_high();
	glcd_dc_high();

	glcd_rst_off();
	_delay_ms(10);
	
	
	glcd_rst_on();
	_delay_ms(120);
	
	//glcd_setOrientation(PORTRAIT);	// default
	
	glcd_sendCmd(ILI9341_CMD_POWER_ON_SEQ_CONTROL);
	glcd_sendData(ILI9341_CMD_IDLE_MODE_ON);
	glcd_sendData(ILI9341_CMD_MEMORY_WRITE);
	glcd_sendData(ILI9341_CMD_NOP);
	glcd_sendData(ILI9341_CMD_TEARING_EFFECT_LINE_OFF);
	glcd_sendData(0x02); 	// XXX

	glcd_sendCmd(ILI9341_CMD_POWER_CONTROL_B);
	glcd_sendData(ILI9341_CMD_NOP);
	glcd_sendData(ILI9341_CMD_POWER_CONTROL_2);
	glcd_sendData(ILI9341_CMD_PARTIAL_AREA);

	glcd_sendCmd(ILI9341_CMD_DRIVER_TIMING_CONTROL_A);
	glcd_sendData(0x85); 	// XXX
	glcd_sendData(ILI9341_CMD_NOP);
	glcd_sendData(0x78); 	// XXX

	glcd_sendCmd(ILI9341_CMD_DRIVER_TIMING_CONTROL_B);
	glcd_sendData(ILI9341_CMD_NOP);
	glcd_sendData(ILI9341_CMD_NOP);

	glcd_sendCmd(0xED);	// XXX
	glcd_sendData(0x64); 	// XXX
	glcd_sendData(0x03);	// XXX
	glcd_sendData(ILI9341_CMD_PARTIAL_MODE_ON);
	glcd_sendData(0X81); 	// XXX

	glcd_sendCmd(ILI9341_CMD_PUMP_RATIO_CONTROL);
	glcd_sendData(ILI9341_CMD_DISP_INVERSION_OFF);

	glcd_sendCmd(ILI9341_CMD_POWER_CONTROL_1);
	glcd_sendData(0x23);	//VRH[5:0] 	// XXX

	glcd_sendCmd(ILI9341_CMD_POWER_CONTROL_2);
	glcd_sendData(ILI9341_CMD_ENTER_SLEEP_MODE);

	glcd_sendCmd(ILI9341_CMD_VCOM_CONTROL_1);
	glcd_sendData(ILI9341_CMD_READ_MEMORY_CONTINUE);
	glcd_sendData(ILI9341_CMD_DISPLAY_OFF);

	glcd_sendCmd(ILI9341_CMD_VCOM_CONTROL_2);
	glcd_sendData(0x86);	//--	// XXX

	glcd_sendCmd(ILI9341_CMD_MEMORY_ACCESS_CONTROL);
	glcd_sendData(0x48);	//C8	//48 68gal.gal.gal.//28 E8 gal.gal.gal.	// XXX

	glcd_sendCmd(ILI9341_CMD_COLMOD_PIXEL_FORMAT_SET);
	glcd_sendData(ILI9341_CMD_WRITE_CONTENT_ADAPT_BRIGHTNESS);

	glcd_sendCmd(ILI9341_CMD_FRAME_RATE_CONTROL_NORMAL);
	glcd_sendData(ILI9341_CMD_NOP);
	glcd_sendData(0x18); 	// XXX

	glcd_sendCmd(ILI9341_CMD_DISPLAY_FUNCTION_CONTROL);
	glcd_sendData(0x08); 	// XXX
	glcd_sendData(0x82);	// XXX
	glcd_sendData(0x27);	// XXX

	glcd_sendCmd(ILI9341_CMD_ENABLE_3_GAMMA_CONTROL);
	glcd_sendData(ILI9341_CMD_NOP);

	glcd_sendCmd(0x26);	//Gamma curve selected 	// XXX
	glcd_sendData(ILI9341_CMD_SOFTWARE_RESET);

	glcd_sendCmd(ILI9341_CMD_POSITIVE_GAMMA_CORRECTION);
	glcd_sendData(0x0F); 	// XXX
	glcd_sendData(0x31);	// XXX
	glcd_sendData(ILI9341_CMD_PAGE_ADDRESS_SET);
	glcd_sendData(ILI9341_CMD_READ_DISP_PIXEL_FORMAT);
	glcd_sendData(ILI9341_CMD_READ_DISP_SIGNAL_MODE);
	glcd_sendData(0x08); 	// XXX
	glcd_sendData(0x4E); 	// XXX
	glcd_sendData(0xF1); 	// XXX
	glcd_sendData(ILI9341_CMD_VERT_SCROLL_START_ADDRESS);
	glcd_sendData(0x07); 	// XXX
	glcd_sendData(ILI9341_CMD_ENTER_SLEEP_MODE);
	glcd_sendData(0x03);	// XXX
	glcd_sendData(ILI9341_CMD_READ_DISP_SIGNAL_MODE);
	glcd_sendData(ILI9341_CMD_READ_DISP_STATUS);
	glcd_sendData(ILI9341_CMD_NOP);

	glcd_sendCmd(ILI9341_CMD_NEGATIVE_GAMMA_CORRECTION);
	glcd_sendData(ILI9341_CMD_NOP);
	glcd_sendData(ILI9341_CMD_READ_DISP_SIGNAL_MODE);
	glcd_sendData(0x14); 	// XXX
	glcd_sendData(0x03);	// XXX
	glcd_sendData(ILI9341_CMD_SLEEP_OUT);
	glcd_sendData(0x07); 	// XXX
	glcd_sendData(0x31); 	// XXX
	glcd_sendData(ILI9341_CMD_POWER_CONTROL_2);
	glcd_sendData(0x48); 	// XXX
	glcd_sendData(0x08); 	// XXX
	glcd_sendData(0x0F); 	// XXX
	glcd_sendData(ILI9341_CMD_READ_DISP_PIXEL_FORMAT);
	glcd_sendData(0x31); 	// XXX
	glcd_sendData(ILI9341_CMD_MEMORY_ACCESS_CONTROL);
	glcd_sendData(ILI9341_CMD_READ_DISP_SELF_DIAGNOSTIC);

	glcd_sendCmd(ILI9341_CMD_SLEEP_OUT);
	_delay_ms(120);

	glcd_sendCmd(ILI9341_CMD_DISPLAY_ON);
	glcd_sendCmd(ILI9341_CMD_MEMORY_WRITE);
	
	//LCD_Cls(black);
	//_delay_ms(150);
}

//-------------------------------------------------------------------------------
//Set Output window
//-------------------------------------------------------------------------------
void LCD_Window(int x1, int y1, int x2, int y2){
	char m  = 0;
	
	//New Orientation Data is only sent to Display if Orientation has changed since last call of LCD_Window()
	if (Orientation != ili_Orientation){
		switch (Orientation)
		{
			case Portrait:
			if (Orientation != ili_Orientation){
				ili_Orientation = Orientation;
				m  = (MADCTL_MX | MADCTL_BGR);
			}
			break;
			case Portrait180:
			if (Orientation != ili_Orientation){
				ili_Orientation = Orientation;
				m = (MADCTL_MY | MADCTL_BGR);
			}
			break;
			case Landscape:
			if (Orientation != ili_Orientation){
				ili_Orientation = Orientation;
				m  = (MADCTL_MV | MADCTL_BGR);
			}
			
			
			break;
			case Landscape180:
			if (Orientation != ili_Orientation){
				ili_Orientation = Orientation;
				m  = (MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
			}

			break;
		}
		
		// Setting ram line and collumn adress write order for the specified Orientation of the Display
		// so that Pixel(0,0) is always at the top-left corner 
		glcd_sendCmd(ILI9341_CMD_MEMORY_ACCESS_CONTROL);
		glcd_sendData(m);		
	}
	
	glcd_setX(x1, x2);
	glcd_setY(y1, y2);
	glcd_sendCmd(ILI9341_CMD_MEMORY_WRITE);
	glcd_dc_high();
	glcd_cs_low();
}


// Sends 16-bit word to Display , mostly used to send color Data
void LCD_SPI_Int(unsigned int Value)
{
	unsigned char data1 = Value>>8;
	unsigned char data2 = Value&0xff;
	//glcd_dc_high();
	//glcd_cs_low();
	SPI_MasterTransmit(data1);
	SPI_MasterTransmit(data2);
	//glcd_cs_high();
}



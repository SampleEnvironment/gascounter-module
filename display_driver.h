// common Functions needed in both drivers:
// LCD_Init() Display Initialisation
// LCD_Window() Sends Command bytes to Display about which Pixel or Pixels are to be Painted
// LCD_SPI_Int() Sends A 16-Bit encoded RGB Value to the Display

#ifndef DISPLAY_DRIVER_H
#define DISPLAY_DRIVER_H


#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>


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

#define HZB_Blue        0b0000001011010011
#define HZB_Cyan        0b0000010011111100


#define LCD_CLK 		PINB3//7		// Clock
#define LCD_RESET 		PINB6		// Reset Display
#define LCD_SELECT 		PINB7//2		// Cable Select
#define LCD_DC 			PINB5//4		// Data/Command
#define LCD_DATA 		PINB4//5		// Send Data



#define SET(bit) (PORTB |= _BV(bit))
#define RESET(bit) (PORTB &= ~_BV(bit))


typedef enum
{
	Portrait		= 0,	// Do not change these values!
	Portrait180		= 3,	// They are sent to the display
	Landscape		= 5,	// in order to set the graphics
	Landscape180	= 6		// mode of the display.
} EWindowOrientation;












#define MAX_X   239
#define MAX_Y   319

#define FONT_SPACE	6
#define FONT_X		8
#define FONT_Y		8


#define MADCTL_MY  0x80  ///< Bottom to top
#define MADCTL_MX  0x40  ///< Right to left
#define MADCTL_MV  0x20  ///< Reverse Mode
#define MADCTL_ML  0x10  ///< LCD refresh Bottom to top
#define MADCTL_RGB 0x00  ///< Red-Green-Blue pixel order
#define MADCTL_BGR 0x08  ///< Blue-Green-Red pixel order
#define MADCTL_MH  0x04  ///< LCD refresh right to left


extern EWindowOrientation ili_Orientation;

//#define PORTRAIT	0
//#define LANDSCAPE	1

void glcd_cs_low(void);
void glcd_cs_high(void); 
void glcd_dc_low(void);
void glcd_dc_high(void); 
void glcd_led_off(void);
void glcd_led_on(void);
void glcd_rst_off(void);
void glcd_rst_on(void);

//const unsigned char simpleFont[][8];
//unsigned char glcd_orientation;


//Display Specific
void glcd_sendCmd(unsigned char data);
void glcd_sendData(unsigned char data);
void glcd_setX(unsigned int x0,unsigned int x1);
void glcd_setY(unsigned int y0,unsigned int y1);
//void glcd_setXY(unsigned int x0, unsigned int y0);
//void LCD_sendComArgs(uint8_t commandByte, uint8_t *dataBytes, uint8_t numDataBytes);

//Common Functions
void LCD_Init(void);
void LCD_Window(int x1, int y1, int x2, int y2);
void LCD_SPI_Int(unsigned int Value);





#endif  // display_driver.h
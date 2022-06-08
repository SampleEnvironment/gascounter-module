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
#include "Gascounter_main.h"

#include "xbee.h"
#include "xbee_AT_comm.h"
#include "I2C_utilities.h"
#include "DS3231M.h"
#include "status.h"


#ifdef GCM_old_disp

#include "disp/gcm_old_lcd_driver.h"
#endif

#ifdef ili9341
#include "disp/display_lib.h"
#include "disp/ili9341_driver.h"
#include "StringPixelCoordTable_ili9341.h"
#endif



#ifdef ili9341


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
#endif


/************************************************************************/
/* Activity Indicator                                                   */
/************************************************************************/
uint8_t activity_indicator = 0; /**< @brief  Activity Inticator on the bottom Part of the Screen. It is incremented every #Measure_Interval for more information look in #displayTemPreVol()  */



uint8_t sensor_err = 0;

char strBuff[30];


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





void paint_string_row_col(char *text,ROW_NAME row,uint8_t col, uint16_t color){
	#ifdef ili9341
	lcd_Print(text,X_LEFT_EDGE + col * FONT2_W ,Y_VALUES_START + FONT2_H * row  ,2,1,1,color,BGC);
	#endif
}

void paint_string_row(char *text,ROW_NAME row,uint8_t update,char* unit,uint16_t color){
	#ifdef ili9341
	if (update)
	{
		lcd_Print("               ",X_LEFT_EDGE + DESCRUPTOR_LEN * FONT2_W,Y_VALUES_START + FONT2_H * row,2,1,1,color,BGC);
	}
	
	lcd_Print(text,X_LEFT_EDGE + DESCRUPTOR_LEN * FONT2_W,Y_VALUES_START + FONT2_H * row,2,1,1,color,BGC);
	
	lcd_Print(" ",X_LEFT_EDGE + DESCRUPTOR_LEN * FONT2_W + strlen(text) * FONT2_W,Y_VALUES_START + FONT2_H * row,2,1,1,color,BGC);

	lcd_Print(unit,HALF_SPACE_WIDTH_FONT_2 + X_LEFT_EDGE + (DESCRUPTOR_LEN+strlen(text)) * FONT2_W, Y_VALUES_START + FONT2_H * row,2,1,1,color,BGC);
	#endif
}

void paint_Main(void){
	#ifdef ili9341
	//lcd_Print("Inf:",X_LEFT_EDGE ,Y_VALUES_START + FONT2_H * DATETIME  ,2,1,1,FGC,BGC);

	sprintf(strBuff,"HZB-GCM v%i.%i",version.Branch_id,version.Fw_version);
	paint_string_row_col(strBuff,VERSION,0,FGC);
	
	if(Time.tm_year != 0){
		sprintf(strBuff,"%02i.%02i.%04i ", Time.tm_mday,Time.tm_mon,Time.tm_year);
		paint_string_row_col(strBuff,DATETIME,0,FGC);
	}
	
	lcd_Print("Value :",X_LEFT_EDGE ,Y_VALUES_START + FONT2_H * VALUE  ,2,1,1,FGC,BGC);
	lcd_Print("Volume:",X_LEFT_EDGE ,Y_VALUES_START + FONT2_H * VOLUME ,2,1,1,FGC,BGC);
	lcd_Print("CorVol:",X_LEFT_EDGE ,Y_VALUES_START + FONT2_H * CORRVOL,2,1,1,FGC,BGC);
	lcd_Print("Temp. :",X_LEFT_EDGE ,Y_VALUES_START + FONT2_H * TEMP   ,2,1,1,FGC,BGC);
	lcd_Print("Press.:",X_LEFT_EDGE ,Y_VALUES_START + FONT2_H * PRESS  ,2,1,1,FGC,BGC);
	lcd_Print("Coord.:",X_LEFT_EDGE ,Y_VALUES_START + FONT2_H * CONN  ,2,1,1,FGC,BGC);

	#endif
}



void paint_Value(uint64_t val,ROW_NAME row,uint8_t precision, uint8_t min_width,char* unit){
	#ifdef ili9341
	char numberbuffer[30];
	dtostrf(((double)val)/pow(10,precision),min_width,precision,numberbuffer);
	
	paint_string_row(numberbuffer,row,0,unit,white);
	#endif
}

void paint_Error(char* text, ROW_NAME row){
	#ifdef ili9341
	paint_string_row(text,row,1,"",red);
	#endif
}

//=========================================================================
// Init screen with messages appearing one under the other
//=========================================================================

// Print line of InitScreen
// if FirstLine is 1 clear page and start with first line
void InitScreen_AddLine_ili(const char* Text, const char FirstLine)
{
	#ifdef ili9341

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
	#endif // ili9341

}

void setInitScreen_ili(uint16_t fore, uint16_t back, uint8_t nextLine, uint8_t FontNr, uint8_t XScale, uint8_t YScale){
	#ifdef ili9341

	InitScreen_ili.ForeColor = fore;
	InitScreen_ili.BackColor = back;
	InitScreen_ili.NextLine = nextLine;
	InitScreen_ili.FontNr = FontNr;
	InitScreen_ili.XScale = XScale;
	InitScreen_ili.YScale = YScale;
	#endif
}


void paint_info_line_ili(char *line, _Bool update)
{
	#ifdef ili9341

	if (!update) LCD_Print("                      ",  X_PIL_2, Y_PIL_90, 1, 1, 1, FGC, BGC);  // clears line (not necessary if in update mode)
	LCD_Print(line,  X_PIL_2, Y_PIL_90, 1, 1, 1, ERR, BGC);
	#endif
}

void paint_store_meas(uint8_t meas_in_Buffer, uint8_t max_Number ){
	
	#ifdef ili9341
	sprintf(strBuff,"Buffer: %03d/%03d",meas_in_Buffer,max_Number);
	paint_string_row_col(strBuff,MULT1,0,orange);
	#endif
	
	#ifdef GCM_old_disp
	sprintf(strBuff,"Buff:%03d/%03d",meas_in_Buffer,max_Number);
	LCD_String(strBuff, 0,0);
	#endif
	
	

}

void paint_send_stored_meas(uint8_t meas_in_Buffer, uint8_t max_Number, uint8_t update){
	#ifdef ili9341
	if (!update)
	{
		//paint_string_row(xbee_get_coordID(),CONN,0,"", green);
		paint_string_row_col("Sending Datasets  ",CONN,0,green);
	}

	sprintf(strBuff,"Buffer: %03d/%03d",meas_in_Buffer,max_Number);
	paint_string_row_col(strBuff,MULT1,0,green);
	#endif
	
	#ifdef GCM_old_disp
	if (!update)
	{
		lcd_Cls(BGC);
		
		LCD_String("Sending Old",0,0);
		LCD_String("Datasets",0,1);
		LCD_String("Remaining:",0,2)
	}

	
	sprintf(strBuff,"Buff:%03d/%03d",meas_in_Buffer,max_Number);
	LCD_String(strBuff, 0,3);
	#endif
	
}


/**
* @brief Displays the current System-Time, #Temperature_value, #Pressure_value (if compensation is enabled), options#Value, options#Volume and options#CorrVolume on the LCD.
* Additionally the connection status and any error is displayed.
*
*
* @return void
*/
void displayTemPreVol(void){
	

	
	
	#ifdef ili9341
	

	
	if (!connected.BMP &&  connected.BMP_on_Startup)
	{
		paint_Error("BMP Sensor",TEMP);
		paint_Error("Error",PRESS);
		
		sensor_err = 1;
		
		
	}
	else
	{
		
		if ( CHECK_ERROR(TEMPPRESS_ERROR))
		{
			paint_Error("TEMP ERR",TEMP);
		}
		if (CHECK_ERROR(TEMPPRESS_ERROR))
		{
			paint_Error("PRESS ERR",PRESS);
			if (options.T_Compensation_enable){
				paint_Error("TEMP ERR",TEMP);
			}

		}
		
		// TEMPERATURE
		if (!(options.T_Compensation_enable && (CHECK_ERROR(TEMPPRESS_ERROR))))
		{
			if (sensor_err)
			{
				paint_string_row("",TEMP,1,"",white);
				sensor_err = 0;
			}
			
			paint_Value( options.Temperature_value - 2732,TEMP, 1, 4, "°C");
		}
		else{

			paint_Error("TEMP ERR",TEMP);
		}
		
		
		//PRESSURE
		if(!(options.p_Compensation_enable && (CHECK_ERROR(TEMPPRESS_ERROR))))
		{
			paint_Value(options.Pressure_value,PRESS, 1, 6, "mbar");
		}
		else{

			paint_Error("PRESS ERR",PRESS);
		}
		

	}



	
	//VOLUME


	paint_Value(options.Value / options.step_Volume,VALUE, position_volume_dot_point, 1,"m³");

	

	paint_Value(options.Volume / options.step_Volume,VOLUME, position_volume_dot_point, 1, "m³");
	

	paint_Value(options.CorrVolume / options.step_Volume, CORRVOL, position_volume_dot_point, 1,"m³");
	
	
	DS3231M_read_time();
	



	
	uint8_t indicator = activity_indicator % 4;
	char indStr[30] = " ";
	switch (indicator)
	{
		case 0:
		sprintf(indStr, "|");
		break;
		case 1:
		sprintf(indStr, "/");
		break;
		case 2:
		sprintf(indStr,  "-");
		break;
		case 3:
		sprintf(indStr, "\\");
		break;
	}
	

	


	sprintf(strBuff," %02i:%02i",Time.tm_hour, Time.tm_min);

	
	strcat(indStr,strBuff);
	if (connected.TWI && connected.DS3231M)
	{
		paint_string_row_col(indStr,DATETIME,11, FGC);
		
		}else{
		paint_string_row_col(indStr,DATETIME,11, orange);
	}

	
	

	
	xbee_get_DB();

	uint16_t x = X_LEFT_EDGE + 16 * FONT2_W+FONT2_W/2;
	uint16_t y = Y_VALUES_START + FONT2_H * VERSION;

	if (ex_mode == online)
	{
		paint_string_row(xbee_get_coordID(),CONN,0,"", green);
		LCD_conn_Stregth(0,xbee.RSSI,x, y, green);
		
	}
	else
	{
		if(xbee.netstat == NO_SERVER){
			paint_string_row(xbee_get_coordID(),CONN,0,"", orange);
			LCD_conn_Stregth(0,xbee.RSSI,x, y, orange);
			
		}
		if(xbee.netstat == NO_NETWORK){
			paint_string_row(xbee_get_coordID(),CONN,0,"", red);
			
			LCD_conn_Stregth(1,xbee.RSSI,x, y, dark_red);
		}
	}



	
	

	
	
	activity_indicator++;
	

	
	
	#endif
	
	
	
	
	

	#ifdef GCM_old_disp
	/*
	LCD_Clear_row_from_column(0,0);

	sprintf(print_temp,BYTE_TO_BINARY_PATTERN,BYTE_TO_BINARY(status.device));
	LCD_String(print_temp,0,0);
	
	sprintf(print_temp,"%i.%i.20%i",Time.tm_mday,Time.tm_mon,Time.tm_year);
	LCD_String(print_temp,0,0);
	
	*/
	
	
	if ( CHECK_ERROR(TEMPPRESS_ERROR))
	{
		/*
		lcd_Cls(BGC);
		LCD_String("TempComp is",0,0);
		LCD_String("enabled but",0,1);
		LCD_String("no conn to",0,2);
		LCD_String("TempPress",0,3);
		LCD_String("Sensor (BMP)",0,4);
		_delay_ms(2000);
		*/
		LCD_Clear_row_from_column(2, 3);
		LCD_String("TEMP ERR",3,3);
	}
	if (CHECK_ERROR(TEMPPRESS_ERROR))
	{
		/*
		lcd_Cls(BGC);
		LCD_String("PressComp is",0,0);
		LCD_String("enabled but",0,1);
		LCD_String("no conn to",0,2);
		LCD_String("TempPress",0,3);
		LCD_String("Sensor (BMP)",0,4);
		_delay_ms(2000);
		*/
		LCD_Clear_row_from_column(2, 4);
		LCD_String("PRESS ERR",3,4);
		if (options.T_Compensation_enable){
			LCD_String("TEMP ERR",3,3);
		}

	}
	

	
	// TEMPERATURE
	if (!(options.T_Compensation_enable && (CHECK_ERROR(TEMPPRESS_ERROR))))
	{
		LCD_Clear_row_from_column(2, 3);
		LCD_Value((int32_t) options.Temperature_value - 2732, 1, 2, 3, "°C");
	}
	else{
		LCD_Clear_row_from_column(2, 3);
		LCD_String("TEMP ERR",3,3);
	}
	
	
	//PRESSURE
	if(!(options.p_Compensation_enable && (CHECK_ERROR(TEMPPRESS_ERROR))))
	{
		LCD_Clear_row_from_column(2, 4);
		LCD_Value(options.Pressure_value, 1, 2, 4, "mbar");
	}
	else{
		LCD_Clear_row_from_column(2, 3);
		LCD_String("PRESS ERR",3,4);
	}
	

	if (!connected.BMP &&  connected.BMP_on_Startup)
	{
		LCD_Clear_row_from_column(2, 3);
		LCD_String("BMP Sensor",2,3);
		LCD_Clear_row_from_column(2, 4);
		LCD_String("Error",2,4);
		
		
	}



	
	//VOLUME
	#ifndef FUNCTION_TRACE
	//LCD_Clear_row_from_column(3, 0);
	//LCD_Value(options.Value / options.step_Volume, position_volume_dot_point, 2, 0, "m³");
	#endif
	LCD_String(xbee_get_coordID(),0,0);
	
	
	LCD_Clear_row_from_column(3, 1);
	LCD_Value(options.Volume / options.step_Volume, position_volume_dot_point, 2, 1, "m³");
	
	LCD_Clear_row_from_column(3, 2);
	LCD_Value(options.CorrVolume / options.step_Volume, position_volume_dot_point, 2, 2, "m³");
	
	DS3231M_read_time();
	
	
	if (connected.TWI && connected.DS3231M)
	{
		sprintf(strBuff,"%02i:%02i", Time.tm_hour, Time.tm_min);
	}
	else
	{
		sprintf(print_temp,"NoI2C");
	}
	


	
	
	uint8_t indicator = activity_indicator % 4;
	switch (indicator)
	{
		case 0:
		strcat(strBuff,"|");
		break;
		case 1:
		strcat(strBuff,"/");
		break;
		case 2:
		strcat(strBuff,"-");
		break;
		case 3:
		strcat(strBuff,"\\");
		break;
	}
	
	
	LCD_String(strBuff, 0, 5);
	
	activity_indicator++;
	
	if (ex_mode == online)
	{
		
		paint_info_line(NetStat[0],0);
	}
	else
	{
		paint_info_line(NetStat[xbee.netstat],0);
	}
	
	
	#ifndef FUNCTION_TRACE
	LCD_String("A:", 0, 0); // Value
	#endif
	
	LCD_String("V:", 0, 1); // Volume
	LCD_String("C:", 0, 2); // CorrVolume
	LCD_String("T:", 0, 3); // Temperature
	LCD_String("P:", 0, 4); // Pressure
	
	
	#endif
	
	
	
	
	
}

void I2C_Clear_view(uint8_t i2cState,uint8_t DS3231State, uint8_t BMPSate){
	lcd_Cls(BGC);
	char twiStr[11] ="";
	
	
	
	
	#ifdef ili9341
	sprintf(twiStr,"clearBUS:%d",i2cState);
	paint_string_row_col("I2C Bus Recovery",DATETIME,0,white);
	paint_string_row(twiStr,VALUE,1,"",FGC);
	
	if (!DS3231State)
	{
		paint_string_row("DS3231M OK",CORRVOL,1,"",FGC);
	}
	else
	{
		paint_string_row("DS3231M NO",CORRVOL,1,"",FGC);
	}
	
	if (!BMPSate)
	{
		paint_string_row("TEM/PRES OK",VOLUME,1,"",FGC);
	}
	else
	{
		paint_string_row("TEM/PRES NO",VOLUME,1,"",FGC);
	}
	
	
	#endif
	
	
	

	#ifdef  GCM_old_disp
	sprintf(twiStr,"clearBUS:%d",i2cState);
	LCD_String(twiStr,0,0);
	
	if (!DS3231State)
	{
		LCD_String("DS3231M OK",0,3);
	}
	else
	{
		LCD_String("DS3231M NO",0,3);
	}
	
	if (!BMPSate)
	{
		paint_string_row("TEM/PRES OK",VOLUME,1,"",FGC);
	}
	else
	{
		LCD_String("TEM/PRES NO",0,2);
	}
	
	#endif
	
	_delay_ms(2000);
	reset_display(1);
	
	
	
}




/**
* @brief Used to periodically reset contents of the display to the normal Layout.
*
*
* @return void
*/
void reset_display(uint8_t clear)
{




	
	#ifdef GCM_old_disp
	if (clear)
	{
		lcd_Cls(BGC);
		_delay_ms(100);
	}

	

	LCD_String("A:", 0, 0); // Value
	LCD_String("V:", 0, 1); // Volume
	LCD_String("C:", 0, 2); // CorrVolume
	LCD_String("T:", 0, 3); // Temperature
	LCD_String("P:", 0, 4); // Pressure

	
	LCD_Clear_row_from_column(3, 0);
	//LCD_Value(options.Value / options.step_Volume, position_volume_dot_point, 2, 0, "m³");
	LCD_String(xbee_get_coordID(),0,0);
	
	LCD_Clear_row_from_column(3, 1);
	LCD_Value(options.Volume / options.step_Volume, position_volume_dot_point, 2, 1, "m³");
	
	LCD_Clear_row_from_column(3, 2);
	LCD_Value(options.CorrVolume / options.step_Volume, position_volume_dot_point, 2, 2, "m³");
	#endif
	
	#ifdef ili9341
	
	if (clear)
	{
		lcd_Cls(BGC);
	}

	paint_Main();
	#endif


}

void paint_Date(void){
	sprintf(strBuff,"%02i.%02i.%04i ", Time.tm_mday,Time.tm_mon,Time.tm_year+2000);
	paint_string_row_col(strBuff,DATETIME,0,FGC);
	
}
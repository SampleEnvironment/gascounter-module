/**
@file 		Gascounter_main.c
@brief 		Gascounter Software
@author		Klaus Kiefer, Peter Wegmann
@version	1.5
@date		2020/03/12
@section	Main
*/
// Gascountzer_main.c - Copyright 2019, HZB, ILL/SANE & ISIS




#define RELEASE_MAIN 1.50





#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <time.h>


#include "Gascounter_main.h"
#include "config.h"
#include "DispAdapter.h"
#include "usart.h"
#include "xbee.h"
#include "xbee_utilities.h"
#include "xbee_AT_comm.h"
#include "status.h"
#include "DS3231M.h"
#include "BMP085.h"


#ifdef GCM_old_disp
#include "disp/gcm_old_lcd_driver.h"
#include "StringPixelCoordTabble_old.h"
#endif

#ifdef ili9341
#include "StringPixelCoordTable_ili9341.h"
#endif

#include "I2C_utilities.h"
#include "module_globals.h"
#include "adwandler.h"
#include "assert.h"
#include "timerUtil.h"







/************************************************************************/
/* STATUS                                                               */
/************************************************************************/


/**
* @brief Status related Information
*
*
* Contains Status-bytes for internal and external Status processing and reporting
*/
volatile statusType status_ms_bytes={.byte_90=0,  .byte_92=0,  .byte_95=0, .byte_96=0};


/************************************************************************/
/* USART                                                                */
/************************************************************************/
/**
* @brief USART communication
*
* Global Variables/Pointers needed for USART Communication
*/
volatile usartType USART= {.cmd_line=0 , .send_str_reader= 0 ,.sending_cmd = 0};


/************************************************************************/
/*  Deltas                                                              */
/************************************************************************/
/**
* @brief Dalta Values
*
* All Variables in the Gascounter representing a difference in Time, Pressure or Temperature since the last Measuerement/Send/Reset
*/
deltaType delta = {.Volume_since_last_send = 0,.Pressure_since_last_send = 0,.t_send = 0,.t_Display_reset = 0};


/************************************************************************/
/*  Time- and Pressurestamps                                            */
/************************************************************************/
/**
* @brief Time- and Pressurestamps
*
* Timestamps and Pressurevalues for last send/display-reset/ping - Event
*/
lastType last= {.Pressure_on_send = 0,.time_send = 0,.time_display_reset = 0, .time_Pressure_Temp_meas = 0,.time_valid_time_reading = 0};



/************************************************************************/
/* Options                                                              */
/************************************************************************/
/**
* @brief Active Options
*
* Active Options that are Set in the Gascounter
*/
optType options = {.offsetValue = DEF_offsetValue,
	.offsetVolume = DEF_offsetVolume,
	.offsetCorrVolume = DEF_offsetCorrVolume,
	.Value = DEF_Value,
	.Volume = DEF_Volume,
	.CorrVolume = DEF_CorrVolume,
	.t_transmission_min = DEF_t_transmission_min,
	.t_transmission_max = DEF_t_transmission_max,
	.delta_V = DEF_delta_V,
	.delta_p = DEF_delta_p,
	.step_Volume = DEF_step_Volume,
	.T_Compensation_enable = DEF_T_Compensation_enable,
	.Temperature_norm = DEF_Temperature_norm,
	.p_Compensation_enable = DEF_p_Compensation_enable,
	.Pressure_norm = DEF_Pressure_norm,
	.Ping_Intervall = DEF_Ping_Intervall,
	.Temperature_value = 0,
	.Pressure_value = 0
	
};


const char* optStrings[] = {
	"max. Interval (min)",
	"min. Interval (s)",
	"send at delta Volume",
	"send at delta Pressure",
	"Volume Step",
	"Temperature Norm",
	"Pressure Norm",
	"Ping Interval"
};




StartUpStatType StartStat = {
	.bound_err_counter = 0,
	.optStrings = optStrings,
	.optBounds =
	{
		{MIN_t_transmission_max,MAX_t_transmission_max,DEF_t_transmission_max},
		{MIN_t_transmission_min,MAX_t_transmission_min,DEF_t_transmission_min},
		{MIN_delta_V,MAX_delta_V,DEF_delta_V},
		{MIN_delta_p,MAX_delta_p,DEF_delta_p},
		{MIN_step_Volume,MAX_step_Volume,DEF_step_Volume},
		{MIN_Temperature_norm,MAX_Temperature_norm,DEF_Temperature_norm},
		{MIN_Pressure_norm,MAX_Pressure_norm,DEF_Pressure_norm},
		{MIN_Ping_Intervall,MAX_Ping_Intervall,DEF_Ping_Intervall}
	},
	.opt_len_received = 0
};

/************************************************************************/
/* Execution Mode                                                       */
/************************************************************************/
/**
* @brief Execution Mode
*
* Execution Mode, describes the general state of the gascounter.
* 1. #ex_mode_online: Gascounter is connected to the server, and no Errors have occured.(Nominal Operation)
* 2. #ex_mode_offline: Gascounter has lost the connection to the Server, but other than that no Errors have occurred. Saved Datea is sent on Reconnect
* 3. #ex_mode_error: Error State that can not be rcovered from. An admin has to resolve the issues
*/
enum PARENT_MODE ex_mode = online;



/************************************************************************/
/* Count Variables                                                      */
/************************************************************************/
/**
* @brief Absoulte Time since Startup
*
* Time elapsed since Gascounter was turned on in s (Overflow occurs every 136 Years)
*/
volatile uint32_t count_t_elapsed = 0;
/**
* @brief Volume Step Counter
*
* Counts the Volume step Ticks sent from the Gascounter. It gets reset to 0 every #Measure_Interval
*/
volatile uint16_t count_Volume_steps = 0;


/************************************************************************/
/* Time Periods                                                         */
/************************************************************************/

/**
* @brief Display Reset Period
*
* Time between resetting the content displayed in the LCD (in s)
*/
const uint16_t reset_display_Interval = 60 * 60;

/**
* @brief Measurement Period
*
* Time between Pressure-Temperature-Measurements (in s)
*/
const uint8_t Measure_Interval = 2;


/************************************************************************/
/* Temperature and Pressure Value                                       */
/************************************************************************/
/**
* @brief Temperature
*
* Temperature Value that is sent periodically to the server (in 0.1K)
*/

/**
* @brief Pressure
*
* Pressure Value that is sent periodically to the server (in 0.1mbar)
*/


// TODO maybe consolidate into struct within BMP.h file
int16_t BMP_Pressure_old = 0;
uint32_t BMP_Temperature_old = 0;

/************************************************************************/
/*   Old                                                                */
/************************************************************************/
/**
* @brief Old Volume Measurement
*
*
* Holds the old options#Value, options#Volume and options#CorrVolume Measurement in order to compare them with the new one  to check for overflows
*/
oldType old = {.Value = DEF_Value, .Volume = DEF_Volume, .CorrVolume=DEF_CorrVolume};


char print_temp[25] = "";/**< @brief   Char Array for displaying Strings*/

uint8_t position_volume_dot_point = 2;	/**< @brief  The position of the dot point of Volume, Value and CorrVolume. */

char NetStat[3][6+1] = {"Online", "NoNetw", "NoServ"}; /**< @brief  Holds Strings for Communicating the current Connectionstatus of the Gascounter-module to the user  */

uint8_t NetStatIndex = 0; /**< @brief  Current Element of #NetStat that is Printed on the Screen */

uint16_t Fw_version = FIRMWARE_VERSION ;
uint8_t Branch_id = BRANCH_ID;


/************************************************************************/
/* Storing Offline Data                                                 */
/************************************************************************/
/**
* @brief Measurement Buffer
*
*	Holds one Measurement, used while Network is down. the stored Data is then, when the Network is back up.
*
*/
typedef struct {
	uint8_t type;					/**< @brief Type of stored Measurement */
	uint8_t data_len;				/**< @brief Length of stored Data   */
	uint8_t data[MEASUREMENT_MESSAGE_LENGTH]; /**< @brief Stored Measurement Data */
} measBufferType;

measBufferType measBuffer[MEASBUFFER_LENGTH]; /**< @brief Array of Measurement Buffers. A maximum of #MEASBUFFER_LENGTH entrys is saved at any given time  */
uint8_t firstMeasBuff = 0;     /**< @brief  index of the first entry */
uint8_t nextfreeMeasBuff = 0;  /**< @brief index of the next free entry   */
uint8_t numberMeasBuff = 0;	/**< @brief number of stored measurements   */



uint8_t 	buffer[SINGLE_FRAME_LENGTH]; /**< @brief  Send and receive Buffer i.e. Option retreival or #ping_server() */
uint8_t 	sendbuffer[SINGLE_FRAME_LENGTH]; /**< @brief Send only Buffer used for sending data to the server for which no reply is expected i.e. Measurement data #MEAS_MSG*/



/************************************************************************/
/*  LAN-Gascounter IPv4 Address                                         */
/************************************************************************/
// Put Gascounter IP here:
#ifdef USE_LAN
volatile IP_v4Type GCM_IP = {
	.IP_oct_1 = 0,
	.IP_oct_2 = 0,
	.IP_oct_3 = 0,
	.IP_oct_4 = 0
};
//************************************************************************/

IP_v4Type EEMEM ee_GCM_IP;


_Bool CE_received = 0;
#endif // USE_LAN


typedef struct{
	_Bool Enable_eeprom_Fun_Trace;
	const uint8_t ringbuff_size;
	uint8_t curr_index;
}eeprom_Fun_Trace_struct;

eeprom_Fun_Trace_struct eeprom_Fun_Trace = {.ringbuff_size = FUNTRACE_ARRAY_SIZE,.curr_index = 7 ,.Enable_eeprom_Fun_Trace = false};

uint8_t EEMEM Fun_trace_array[FUNTRACE_ARRAY_SIZE+FUNTRACE_HEADER_LEN];  //eeprom Function Trace Array

uint16_t EEMEM funtrace_was_activated; //word in eeprom to indicate that funtrace was activated








void Funtrace_enter(uint8_t Function_ID){

	if (eeprom_Fun_Trace.Enable_eeprom_Fun_Trace)
	{
		eeprom_update_byte(&Fun_trace_array[eeprom_Fun_Trace.curr_index],Function_ID);
		
		eeprom_update_byte(&Fun_trace_array[6],eeprom_Fun_Trace.curr_index);
		
		eeprom_Fun_Trace.curr_index++;

		if (eeprom_Fun_Trace.curr_index == (eeprom_Fun_Trace.ringbuff_size + FUNTRACE_HEADER_LEN))
		{
			eeprom_Fun_Trace.curr_index = FUNTRACE_HEADER_LEN;
		}
		// Timestamp of last Message
		eeprom_update_byte(&Fun_trace_array[0],Time.tm_sec);
		eeprom_update_byte(&Fun_trace_array[1],Time.tm_min);
		eeprom_update_byte(&Fun_trace_array[2],Time.tm_hour);
		eeprom_update_byte(&Fun_trace_array[3],Time.tm_mday);
		eeprom_update_byte(&Fun_trace_array[4],Time.tm_mon);
		eeprom_update_byte(&Fun_trace_array[5],Time.tm_year);
		
	}



}







/**
* @brief packs Current Measurement Data:
* 1. time [0-2]
* 2. date [3-5]
* 3. options#Value [6-9]
* 4. options#Volume [10-13]
* 5. options#CorrVolume [14-17]
* 6. #Temperature_value [18-19]
* 7. #Pressure_value [20-21]
* 8. status#byte_91 [22]
*
* in this order into the #sendbuffer for subsequent transmission to the server
*
* @return void
*/
void Collect_Measurement_Data(void){
	FUNCTION_TRACE

	Funtrace_enter(1);
	
	uint64_t Value_holder;
	uint64_t Volume_holder;
	uint64_t CorrVolume_holder;
	
	
	Value_holder = options.Value / 1000;
	Volume_holder = options.Volume / 1000;
	CorrVolume_holder = options.CorrVolume / 1000;
	
	if (connected.DS3231M && connected.TWI)
	{
		DS3231M_read_time();
		
		sendbuffer[0] = Time.tm_sec;
		sendbuffer[1] = Time.tm_min;
		sendbuffer[2] = Time.tm_hour;
		sendbuffer[3] = Time.tm_mday;
		sendbuffer[4] = Time.tm_mon;
		sendbuffer[5] = Time.tm_year;
	}
	
	
	sendbuffer[6] = Value_holder >> 24;
	sendbuffer[7] = Value_holder >> 16;
	sendbuffer[8] = Value_holder >> 8;
	sendbuffer[9] = (uint8_t) Value_holder;
	
	sendbuffer[10] = Volume_holder >> 24;
	sendbuffer[11] = Volume_holder >> 16;
	sendbuffer[12] = Volume_holder >> 8;
	sendbuffer[13] = (uint8_t) Volume_holder;
	
	sendbuffer[14] = CorrVolume_holder >> 24;
	sendbuffer[15] = CorrVolume_holder >> 16;
	sendbuffer[16] = CorrVolume_holder >> 8;
	sendbuffer[17] = (uint8_t) CorrVolume_holder;
	
	sendbuffer[18] = options.Temperature_value >> 8;
	sendbuffer[19] = (uint8_t) options.Temperature_value;
	
	sendbuffer[20] = options.Pressure_value >> 8;
	sendbuffer[21] = (uint8_t) options.Pressure_value;
	
	uint8_t curr_Stat = 0;
	
	if(BIT_CHECK(status_reset_on_send,OPTIONS_ERROR)){BIT_SET(curr_Stat,status_bit_option_err_91);}
	
	if(BIT_CHECK(status_reset_on_send,TEMPPRESS_ERROR)){BIT_SET(curr_Stat,status_bit_Temp_Press_Err_91);}
	
	if (BIT_CHECK(status_reset_on_send,VOLUME_TOO_BIG_ERROR)){BIT_SET(curr_Stat,status_bit_volume_too_big_91);}
	
	if(BIT_CHECK(status_reset_on_send,TIMER_ERROR)){BIT_SET(curr_Stat,status_bit_DS3231M_err_91);}
	
	if(BIT_CHECK(status_reset_on_send,I2C_BUS_ERROR)){BIT_SET(curr_Stat,status_bit_I2C_err_91);}
	
	
	// network error is not reset on send/store
	if(BIT_CHECK(get_status(),NETWORK_ERROR)){BIT_SET(curr_Stat,status_bit_Network_err_91);}
	

	
	sendbuffer[22] = curr_Stat;
	
	memcpy(buffer,sendbuffer,MEASUREMENT_MESSAGE_LENGTH);
}




/**
* @brief Store undelivered measurements to the measurement ring buffer (#measBuffer). If the buffer is already full, the oldest measurement is overwritten.
*
* @param db_cmd_type Database Command
* @param buffer Pointer to measurement data
* @param length length of measurement data
*
* @return void
*/
void store_measurement(void)
{
	FUNCTION_TRACE
	Funtrace_enter(2);
	// write information to measurement buffer
	/*
	lcd_Cls(BGC);
	sprintf(print_temp,"%i.%i.%i",sendbuffer[3],sendbuffer[4],sendbuffer[5]);
	LCD_String(print_temp,0,1);
	sprintf(print_temp,"%i:%i:%i  ",sendbuffer[2],sendbuffer[1],sendbuffer[0]);
	LCD_String(print_temp,0,2);
	_delay_ms(2000);
	*/
	measBuffer[nextfreeMeasBuff].type  = MEAS_MSG;
	//	measBuffer[nextfreeMeasBuff].data  = buffer;
	
	//	for(uint8_t i = 0; i<length;i++) measBuffer[nextfreeMeasBuff].data[i] = buffer[i];
	memcpy(measBuffer[nextfreeMeasBuff].data,sendbuffer,MEASUREMENT_MESSAGE_LENGTH);
	measBuffer[nextfreeMeasBuff].data_len = MEASUREMENT_MESSAGE_LENGTH;
	
	// increase the number of stored measurements
	numberMeasBuff = numberMeasBuff + 1;
	if (numberMeasBuff > MEASBUFFER_LENGTH) numberMeasBuff = MEASBUFFER_LENGTH;
	
	// adapt the index for the next measurement to be stored
	nextfreeMeasBuff = nextfreeMeasBuff + 1;
	if (nextfreeMeasBuff > MEASBUFFER_LENGTH-1) nextfreeMeasBuff = 0;
	
	// if the number of stored measurements reaches the maximum number, the oldest measurements will be overwritten
	if (numberMeasBuff == MEASBUFFER_LENGTH) firstMeasBuff = nextfreeMeasBuff;
	

	paint_store_meas(numberMeasBuff,MEASBUFFER_LENGTH);




	
}





/**
* @brief Handles Initialization of the Device by executing the init functions of all the subsystems in sequence, and outputting the Status to the screen.
*
*
* @return void
*/
void init(void)
{
	
	lcd_init();

	lcd_Cls(white);

	lcd_LOGO(45,60,white);
	

	
	_delay_ms(500);
	version_INIT(FIRMWARE_VERSION,BRANCH_ID,FIRMWARE_VERSION);
	
	init_ports();
	

	#ifdef ili9341
	setInitScreen(black,white,6,1,1,1);
	Print_add_Line(STR_HZB_GASCOUNTER,0);
	#endif
	
	#ifdef GCM_old_disp
	Print_add_Line(STR_HZB_GASCOUNTER,1);
	#endif




	sprintf(print_temp,"v%i.%i",version.Branch_id,version.Fw_version);
	Print_add_Line(print_temp,0);
	#ifdef USE_LAN
	Print_add_Line(STR_LAN_VARIANT,0);
	#endif
	#ifdef USE_XBEE
	Print_add_Line(STR_XBEE_VARIANT,0);
	#endif
	

	
	
	#ifdef ili9341
	Print_add_Line(STR_INIT_START,0);
	#endif
	
	#ifdef GCM_old_disp
	_delay_ms(2000); // the delay is for the xbee to start when the system is plugged in
	Print_add_Line(STR_INIT_START,1);
	#endif


	Print_add_Line(STR_INIT_PORTS,2);
	
	Print_add_Line(STR_INIT_TIMER,2);
	init_timer();
	Print_add_Line(STR_INIT_INTERRUPTS,2);
	init_interrupts();
	Print_add_Line(STR_INIT_USART,2);
	usart_init(39);
	Print_add_Line(STR_INIT_I2C,2);
	i2c_init();


	
	
	//=========================================================================
	// Enable global interrupts
	//=========================================================================
	sei();

	//=========================================================================
	// Initialization I2C
	//=========================================================================




	// Timer
	Print_add_Line(STR_INIT_CLOCK,2);

	if (init_DS3231M(&paint_info_line) == 0) // trying to connect with DS3231M (time)
	{
		connected.DS3231M = 1;
		#ifdef ili9341
		Print_add_Line(STR_INIT_CLOCK_SUCC,2);
		#endif
		#ifdef GCM_old_disp
		Print_add_Line(STR_INIT_CLOCK_SUCC,0);
		#endif

	}
	else
	{
		connected.DS3231M = 0;
		SET_ERROR(TIMER_ERROR);
		#ifdef ili9341
		Print_add_Line(STR_INIT_CLOCK_ERR,2);
		#endif
		#ifdef GCM_old_disp
		Print_add_Line(STR_INIT_CLOCK_ERR,0);
		#endif
	}

	// Pressure and Temperature Sensor
	Print_add_Line(STR_INIT_PRESS,0);
	
	if (init_BMP(&paint_info_line) == 0) // trying to connect with BMP
	{
		connected.BMP = 1;
		connected.BMP_on_Startup = 1;
		#ifdef ili9341
		Print_add_Line(STR_INIT_PRESS_SUCC,2);
		#endif
		#ifdef GCM_old_disp
		Print_add_Line(STR_INIT_PRESS_SUCC,0);
		#endif
		CLEAR_ERROR(TEMPPRESS_ERROR);;
	}
	else
	{
		
		connected.BMP = 0;
		connected.BMP_on_Startup = 0;
		connected.TWI = 1;
		BMP_Temperature = 0;
		BMP_Pressure = 0;
		Print_add_Line(" ",0);

	}
	
	
	xbee_init(&paint_info_line,NULL,0);
	
	xbee_hardware_version();
	Print_add_Line(STR_INIT_DONE,0);
	

	

	
}


/**
* @brief Initializes the ports of the controller.
*
*
* @return void
*/
void init_ports(void)
{
	DDRC |= (1<<PINC4);
}



/**
* @brief Initializes Timer1 of the controller so it send interrupts with a frequency of 1 Hz.
*
*
* @return void
*/
void init_timer(void)
{
	//Timer1 f ~ 1Hz
	TCCR1B |= (1<<WGM12) | (1<<CS12) | (1<<CS10);
	OCR1A = 5999;
	TIMSK1 |= (1<<OCIE1A);
	
	/*
	//Timer0 f ~ 40Hz T ~ 25ms
	TCCR0A |= (1<<WGM01);
	TCCR0B |= (1<<CS00) | (1<<CS02); // N = 1024
	OCR0A = 2;
	TIMSK0 |= (1<<OCIE1A);
	*/
}



/**
* @brief Initializes the interrupts of the controller.
*
*
* @return void
*/
void init_interrupts(void)
{
	EICRA |= (1<<ISC21);
	EIMSK |= (1<<INT2);
}






/**
* @brief Resets #count_Volume_steps to zero and calculates the Temperature and Pressure corrected volume (according to which compensation options were set)
* that has passed through the counter (Volume_Since_Last_TP_Measurement) since the last call of #Temp_Press_CorrectedVolume. This volume is than added to the Absolute Values held in options#Value,
* options#Volume and options#CorrVolume (before adding they are checked for overflows). delta#Volume_since_last_send is also increased by Volume_Since_Last_TP_Measurement.
*
*
* @param dest_low  low  32-bit of coordinator address
* @param dest_high high 32-bit of coordinator address
*
* @return void
*/
void Temp_Press_CorrectedVolume(void)
{

	if(!CHECK_ERROR(TEMPPRESS_ERROR) && connected.TWI){ // only updated when BMP is connected
		
		options.Temperature_value = BMP_Temperature;
		options.Pressure_value =  BMP_Pressure/10;///100000; // conversion from Pa to mbar
		
	}
	uint32_t Ticks_since_last_TP_Meas;
	
	cli(); //disable interrupts to avoid counting errors of steps
	Ticks_since_last_TP_Meas = count_Volume_steps;
	count_Volume_steps = 0;              //reset Volume_steps
	sei();
	
	//Volume of Gas since the last Temp /Pressure Measurement was done
	uint32_t Volume_Since_Last_TP_Measurement = Ticks_since_last_TP_Meas* options.step_Volume; /**< @brief Volume that has passed through the counter since the last call of #Temp_Press_CorrectedVolume  */
	
	options.Value += Volume_Since_Last_TP_Measurement;
	
	if (options.Value < old.Value) // Checks for overflows.
	{
		
		SET_ERROR(VOLUME_TOO_BIG_ERROR);
		paint_info_line("Overfl",1);

	}
	
	old.Value = options.Value;
	
	options.Volume += Volume_Since_Last_TP_Measurement;
	
	if (options.Volume < old.Volume) // Checks for overflows.
	{
		SET_ERROR(VOLUME_TOO_BIG_ERROR);;
		paint_info_line("Overfl",1);

	}
	
	old.Volume = options.Volume;
	
	// Calculates Corrected Volume accourding to Options
	
	if (options.T_Compensation_enable && options.p_Compensation_enable)
	{
		options.CorrVolume = options.CorrVolume + (uint64_t)((((uint64_t) Volume_Since_Last_TP_Measurement*options.Pressure_value*options.Temperature_norm))/((options.Pressure_norm*options.Temperature_value)));
	}
	else if (options.p_Compensation_enable)
	{
		options.CorrVolume = options.CorrVolume + (uint64_t)((((uint64_t) Volume_Since_Last_TP_Measurement*options.Pressure_value)/options.Pressure_norm));
	}
	else if (options.T_Compensation_enable)
	{
		options.CorrVolume = options.CorrVolume + (uint64_t)(((uint64_t) Volume_Since_Last_TP_Measurement*options.Temperature_norm)/options.Temperature_value);
	}
	else
	{
		options.CorrVolume += Volume_Since_Last_TP_Measurement;
	}
	
	

	
	if (options.CorrVolume < old.CorrVolume) // Checks for overflows.
	{
		SET_ERROR(VOLUME_TOO_BIG_ERROR);;
		paint_info_line("Overfl",1);
	}
	
	old.CorrVolume = options.CorrVolume;
	
	// Delta Volume for detecting spikes in Volume/s or absolute Volume values since last sending of data
	delta.Volume_since_last_send += Volume_Since_Last_TP_Measurement;
	


}



// TODO: needs to be redone: maybe save last couple of measurements
void PT_Plausibility(void){
	Funtrace_enter(5);
	FUNCTION_TRACE;
	
	int8_t P_Bounds,T_Bounds;
	uint8_t i = 0;
	
	do{
		if (BMP_Pressure > MAX_NOMINAL_PRESSURE){P_Bounds = 1; }
		else if (BMP_Pressure < MIN_NOMINAL_PRESSURE){P_Bounds = -1;}
		else{P_Bounds = 0;}
		
		if (BMP_Temperature > MAX_NOMINAL_TEMPERATURE){T_Bounds= 1; }
		else if (BMP_Temperature < MIN_NOMINAL_TEMPERATURE){T_Bounds = -1;}
		else{T_Bounds = 0;}
		
		

		if (!T_Bounds && !P_Bounds)
		{// P and T Values are probably correct nothing to do here
			CLEAR_ERROR(I2C_BUS_ERROR);;
			CLEAR_ERROR(TEMPPRESS_ERROR);;
			

			return;
		}
		else
		{
			
			SET_ERROR(TEMPPRESS_ERROR);;
			
			if(BMP_Temp_and_Pressure())
			{
				SET_ERROR(I2C_BUS_ERROR);

				return;
			}
			_delay_ms(100);
		}
		i++;
	} while (i < 3);
	
	
	int32_t P_Rate = abs( (int32_t)BMP_Pressure - (int32_t)BMP_Pressure_old);
	int16_t T_Rate = abs(BMP_Temperature - BMP_Temperature_old);
	
	
	// rates must not be exceeded
	if(T_Rate > MAX_TEMPERATURE_DELTA|| P_Rate > MAX_PRESSURE_DELTA){
		return;
	}
	
	
	// Measurements are still out of bounds after three measurements--> cannot be explained by simple bit flip in I2C-bus
	
	
	// Data shows that high Pressure peaks are followed by Temperature Drops (they set in about ~10min later), these are the only occasions,
	// when out of bounds Values are accepted
	
	// sufficient padding is provided if only one of the values is out of bound and while the other one might still be within acceptable range
	
	
	// BMP_Temperature has to be less than (MIN_NOMINAL_TEMPERATURE  + 16°C), while the pressure wave goes through
	//--> Pressure wave detection
	if ((BMP_Pressure > MAX_NOMINAL_PRESSURE) &&
	(BMP_Temperature < 2891) )
	{
		CLEAR_ERROR(I2C_BUS_ERROR);;
		CLEAR_ERROR(TEMPPRESS_ERROR);;
		paint_info_line("P wave",0);
		_delay_ms(500);
		return;
	}
	
	
	// BMP_Pressure has to be greater than (1000mbar ), while the Temperature wave goes through
	if ((BMP_Temperature < MIN_NOMINAL_TEMPERATURE) &&
	(BMP_Pressure > 100000))
	{
		CLEAR_ERROR(I2C_BUS_ERROR);;
		CLEAR_ERROR(TEMPPRESS_ERROR);;
		paint_info_line("T wave",0);
		_delay_ms(500);
		return;
	}
	
	
	
}






/**
* @brief Handles the login process with the server. First a login request is sent and then the server sends back the options for the gascounter. Which are firstwritten to @p buffer and then copied into the #frameBuffer.
*
* @param db_cmd_type alsways set to #LOGIN_MSG
* @param buffer Pointer to the buffer that contains login data (answer from the server will be written in the same buffer)
* @param dest_high high 32-bit of coordinator address
* @param dest_low low  32-bit of coordinator address
*
* @return uint8_t 0xFF if bad options were received otherwise the return value is the index in #frameBuffer where the answer from the server is stored
*/
uint8_t xbee_send_login_msg(uint8_t db_cmd_type, uint8_t *buffer)
{

	
	#ifdef ALLOW_LOGIN
	uint8_t reply_Id = 0;
	
	#ifdef ili9341
	Print_add_Line("Request Opts",0);
	#endif
	#ifdef GCM_old_disp
	Print_add_Line("Request Opts",1);
	#endif

	
	//status_bit_registration_90


	buffer[0] = status_ms_bytes.byte_90;
	
	
	// Try to send login message "number_trials" times
	#ifdef USE_LAN
	uint8_t number_trials = 2;
	#endif
	#ifdef USE_XBEE
	uint8_t number_trials = 1;
	#endif
	
	while(number_trials)
	{
		#ifdef USE_LAN
		uint8_t sim_xb_rep = 0xFF;
		sim_xb_rep  = xbee_hasReply(CE_MSG_TYPE,EQUAL);
		if (sim_xb_rep != 0xFF){
			
			
			sendbuffer[0] = 1;
			
			xbee_pseudo_send_AT_response( 'C', 'E', 0, sendbuffer, 1);
			

			
		}
		
		buffer_removeData(sim_xb_rep);
		#endif
		
		
		reply_Id = xbee_send_request(db_cmd_type,buffer,1);
		//reply_Id = xbee_send_request_only(db_cmd_type, buffer, 1);
		
		//#ifdef ALLOW_DEBUG
		//sprintf(print_temp,"%i",reply_Id);
		//LCD_InitScreen_AddLine(print_temp, 0);
		//_delay_ms(10000);
		//#endif
		

		
		
		if(reply_Id != 0xFF)
		{
			#ifdef ili9341
			Print_add_Line("Request Opts ...data rec.",2);
			#endif
			#ifdef GCM_old_disp
			Print_add_Line("...data rec.",0);
			#endif
			

			_delay_ms(500);
			//sprintf(print_temp,"%i",frameBuffer[reply_Id].data_len);
			//LCD_InitScreen_AddLine(print_temp,0);
			
			StartStat.opt_len_received =  frameBuffer[reply_Id].data_len ;
			
			if(frameBuffer[reply_Id].data_len == NUMBER_LOGIN_BYTES) {
				return reply_Id;	//good options
			}
			else {
				
				SET_ERROR(OPTIONS_ERROR);
				SET_ERROR(OPTIONS_LEN_ERROR);
				
				return 0xFF;  //bad options
			}
			
			}else{
			SET_ERROR(INIT_OFFLINE_ERROR);
		}
		
		
		if(!(--number_trials))
		{
			//stop trying and go in error mode; no functionality available from here on
			Print_add_Line("Login failed",1);
			Print_add_Line("Offline mod.",0);
			_delay_ms(2000);
			return 0xFF;

		}

		
	}
	return 0xFF;
	#else
	return 1;	//bad options /main will set default
	#endif
}



/**
* @brief Decodes incoming Messages from the Server and act accordingly
*
* @param reply_id index of the message in #frameBuffer which will be decoded
*
*
* @return void
*/
void execute_server_CMDS(uint8_t reply_id){
	FUNCTION_TRACE
	Funtrace_enter(7);
	switch (frameBuffer[reply_id].type)
	{
		//=================================================================
		case SET_OPTIONS_CMD:// set received Options
		Set_Options((uint8_t*)frameBuffer[reply_id].data,SET_OPTIONS_CMD);
		break;
		
		//=================================================================
		case TRIGGER_MEAS_CMD: // Send Measurement Data immediately
		Collect_Measurement_Data();
		xbee_send_message(TRIGGER_MEAS_CMD,sendbuffer,MEASUREMENT_MESSAGE_LENGTH);
		break;
		
		//=================================================================
		case GET_OPTIONS_CMD : ;// send current options to Server
		
		
		uint32_t offsetValue_holder = options.offsetValue/1000; //convert into 1L unit
		uint32_t offsetVolume_holder = options.offsetVolume/1000; //convert into 1L unit
		uint32_t offsetCorrVolume_holder = options.offsetCorrVolume/1000; //convert into 1L unit
		uint16_t delta_V_holder = options.delta_V/1000; //convert into 1L unit
		uint16_t step_Volume_holder = options.step_Volume/1000; //convert into 1L unit
		uint8_t length = 0;
		
		sendbuffer[length++] = offsetValue_holder >> 24;
		sendbuffer[length++] = offsetValue_holder >> 16;
		sendbuffer[length++] = offsetValue_holder >> 8;
		sendbuffer[length++] = (uint8_t) offsetValue_holder;
		
		sendbuffer[length++] = offsetVolume_holder >> 24;
		sendbuffer[length++] = offsetVolume_holder >> 16;
		sendbuffer[length++] = offsetVolume_holder >> 8;
		sendbuffer[length++] = (uint8_t) offsetVolume_holder;
		
		sendbuffer[length++] = offsetCorrVolume_holder >> 24;
		sendbuffer[length++] = offsetCorrVolume_holder >> 16;
		sendbuffer[length++] = offsetCorrVolume_holder >> 8;
		sendbuffer[length++] = (uint8_t) offsetCorrVolume_holder;
		
		sendbuffer[length++] = options.t_transmission_min >> 8;
		sendbuffer[length++] = (uint8_t) options.t_transmission_min;
		
		sendbuffer[length++] = options.t_transmission_max >> 8;
		sendbuffer[length++] = (uint8_t) options.t_transmission_max;
		
		sendbuffer[length++] = delta_V_holder >> 8;
		sendbuffer[length++] = (uint8_t) delta_V_holder;
		
		sendbuffer[length++] = options.delta_p >> 8;
		sendbuffer[length++] = (uint8_t) options.delta_p;
		
		sendbuffer[length++] = step_Volume_holder >> 8;
		sendbuffer[length++] = (uint8_t) step_Volume_holder;
		
		//sendbuffer[length++] = 0;//TODO remove
		//sendbuffer[length++] = 0;
		
		//sendbuffer[length++] = 0;//TODO Remove
		//sendbuffer[length++] = 0;
		
		sendbuffer[length++] = options.T_Compensation_enable;
		
		sendbuffer[length++] = options.Temperature_norm >> 8;
		sendbuffer[length++] = (uint8_t) options.Temperature_norm;
		
		sendbuffer[length++] = options.p_Compensation_enable;
		
		sendbuffer[length++] = options.Pressure_norm >> 8;
		sendbuffer[length++] = (uint8_t) options.Pressure_norm;
		
		sendbuffer[length++] = options.Ping_Intervall;

		
		sendbuffer[length++] = status_ms_bytes.byte_92; // status_byte_92 is always zero
		
		xbee_send_message(GET_OPTIONS_CMD,sendbuffer,length);
		break;
		
		case DEPRECATED_ILM_BROADCAST: // ILM broadcast Message received do nothing
		
		break;
		
		case SET_FUNTRACE_CMD:
		if (frameBuffer[reply_id].data[0] == 0)
		{
			eeprom_Fun_Trace.Enable_eeprom_Fun_Trace = false;
			
			#ifdef ili9341
			paint_info_line("FunTrace: OFF",1);
			#endif
			#ifdef GCM_old_disp
			LCD_String("FunTrace:OFF",0,5);
			#endif
			
			_delay_ms(2000);
		}else
		{
			eeprom_Fun_Trace.Enable_eeprom_Fun_Trace = true;
			#ifdef ili9341
			paint_info_line("FunTrace: ON",1);
			#endif
			#ifdef GCM_old_disp
			LCD_String("FunTrace:ON",0,5);
			#endif
			_delay_ms(2000);
			
			eeprom_write_word(&funtrace_was_activated,FUNTRACE_PASS);
		}
		break;

		

		case GET_FUNTRACE_CMD: ;

		uint8_t funtrace_raw_buff[256];
		
		uint16_t fun_pass = eeprom_read_word(&funtrace_was_activated);
		
		if (fun_pass != FUNTRACE_PASS)
		{
			for (uint8_t i = 0; i < FUNTRACE_ARRAY_SIZE + FUNTRACE_HEADER_LEN-1 ; i++)
			{
				sendbuffer[i] = 0;
			}
			
			xbee_send_message(GET_FUNTRACE_CMD,sendbuffer,FUNTRACE_ARRAY_SIZE+6);
			break;
		}
		
		
		
		
		
		eeprom_read_block(&funtrace_raw_buff,&Fun_trace_array,FUNTRACE_ARRAY_SIZE+FUNTRACE_HEADER_LEN);
		memcpy(sendbuffer,funtrace_raw_buff,6);

		uint8_t last_fun_index = funtrace_raw_buff[6];
		
		for(uint8_t i = 6; i < FUNTRACE_ARRAY_SIZE+FUNTRACE_HEADER_LEN-1; i++){
			
			
			sendbuffer[i] = funtrace_raw_buff[last_fun_index];

			
			if (last_fun_index == FUNTRACE_HEADER_LEN)
			{
				last_fun_index = FUNTRACE_ARRAY_SIZE + FUNTRACE_HEADER_LEN-1;
			}
			else
			{
				last_fun_index--;
			}
			
		}
		

		xbee_send_message(GET_FUNTRACE_CMD,sendbuffer,FUNTRACE_ARRAY_SIZE+6);

		
		
		break;
		
		case SET_PING_INTERVALL_CMD:
		;
		uint8_t Val_outof_Bounds = 0;
		
		uint16_t buff_ping_Intervall  =          frameBuffer[reply_id].data[0]  ;
		
		CHECK_BOUNDS(buff_ping_Intervall,MIN_Ping_Intervall,MAX_Ping_Intervall,DEF_Ping_Intervall,Val_outof_Bounds);
		if (!Val_outof_Bounds)
		{
			options.Ping_Intervall = buff_ping_Intervall;
		}
		sendbuffer[0] = 0;
		xbee_send_message(SET_PING_INTERVALL_CMD,sendbuffer,1);
		break;



		#ifdef USE_LAN
		case CE_MSG_TYPE:
		
		if (ex_mode == offline)
		{
			CE_received = 1;

		}

		sendbuffer[0] = 1;
		
		xbee_pseudo_send_AT_response( 'C', 'E', 0, sendbuffer, 1);
		
		break;
		
		case NI_MSG_TYPE:;
		
		uint8_t NI_len = frameBuffer[reply_id].data_len;
		memcpy(xbee.CoordIdentifier,(char *)frameBuffer[reply_id].data,NI_len);
		xbee.CoordIdentifier[NI_len] =  '\0';
		
		xbee_pseudo_send_AT_response( 'N', 'I', 0, sendbuffer, 0);
		
		default:;
		uint8_t AT_Code = frameBuffer[reply_id].type;
		
		
		// handle unknown AT command
		if (AT_Code >= 200)
		{
			
			
			xbee_pseudo_send_AT_response( 
				AT_Lut[(uint8_t)(AT_Code-AT_START)][0], // translate AT_code back to At ASCII chars
				AT_Lut[(uint8_t)(AT_Code-AT_START)][1],
				1, // Status == 1 --> atcommand not known
				sendbuffer,
				0); // empty payload
		}
		
		break;
		
		#endif
		
	}
	//remove Frame from Buffer
	
	
	buffer_removeData(reply_id);
}


// Pings Server and resets Time
/**
* @brief Pings the server in order to check if the connection is still live. If no Pong was received after #COM_TIMEOUT_TIME the Network error Bit is set in status#device.
*
* @param dest_high high 32-bit of coordinator address
* @param dest_low  low  32-bit of coordinator address
*
* @return uint8_t 1 if Ping successful and 0 if no Pong was received
*/
uint8_t ping_server(void)
{
	
	#ifdef USE_LAN
	CLEAR_ERROR(NETWORK_ERROR);
	
	#endif
	buffer[0]= status_ms_bytes.byte_95; // Ping Status Byte is always 0
	uint8_t reply_id = xbee_send_request(PING_MSG,buffer,1);
	if( 0xFF == reply_id)
	{
		paint_info_line("NoPong",0);
		_delay_ms(500);
		SET_ERROR(NETWORK_ERROR);
		ex_mode = offline;
		t_start(RECONNECT,options.Ping_Intervall*60);
		paint_Date();
		return 0;
	}
	else
	{
		// Ping Successful --> time is set to the received time
		if (connected.DS3231M)
		{
			struct tm newtime;
			
			newtime.tm_sec  = frameBuffer[reply_id].data[0];
			newtime.tm_min  = frameBuffer[reply_id].data[1];
			newtime.tm_hour = frameBuffer[reply_id].data[2];
			newtime.tm_mday = frameBuffer[reply_id].data[3];
			newtime.tm_mon  = frameBuffer[reply_id].data[4];
			newtime.tm_year = frameBuffer[reply_id].data[5];
			
			
			DS3231M_set_time(&newtime);
		}
		
	}
	
	paint_Date();
	return 1;
}




uint32_t checkBounds(uint32_t var,BOUNDS_TYPE BoundVar,uint8_t * flag){
	uint32_t min = StartStat.optBounds[BoundVar][0];
	uint32_t max = StartStat.optBounds[BoundVar][1];
	uint32_t def = StartStat.optBounds[BoundVar][2];
	if((var < min) || (var > max || isnan(var))){
		StartStat.errorVal[StartStat.bound_err_counter] = var;
		var = def;
		*flag = 1;
		StartStat.boundsErrors[StartStat.bound_err_counter] = BoundVar;
		StartStat.bound_err_counter++;
	}
	return var;
	
}






/**
* @brief Receives new options in #buffer and validates them. The validity of the options is reported back to the Server via a #OPTIONS_SET_ACK Message containing the status#byte_93.
*
* @param optBuffer Byte-Array containing the raw options received from the server
* @param dest_high high 32-bit of coordinator address
* @param dest_low  low  32-bit of coordinator address
*
* @return void
*/
void Set_Options(uint8_t *optBuffer,uint8_t answer_code){
	FUNCTION_TRACE
	
	uint8_t outofBundsFlag =0;
	
	Funtrace_enter(10);
	_delay_ms(200);

	paint_info_line(" Op96 ",0);
	_delay_ms(3000);
	
	struct tm newtime;
	if (connected.DS3231M)
	{
		
		
		newtime.tm_sec  = optBuffer[0];
		newtime.tm_min  = optBuffer[1];
		newtime.tm_hour = optBuffer[2];
		newtime.tm_mday = optBuffer[3];
		newtime.tm_mon  = optBuffer[4];
		newtime.tm_year = optBuffer[5];
		
		
		
		DS3231M_set_time(&newtime);
		
		_delay_ms(50);
		
		DS3231M_read_time();
		
	}
	#ifdef DEBUG_DS3231M
	
	
	sprintf(print_temp,"sec:%i",optBuffer[0]);
	Print_add_Line(print_temp,1);
	sprintf(print_temp,"min:%i",optBuffer[1]);
	Print_add_Line(print_temp,0);
	sprintf(print_temp,"hou:%i",optBuffer[2]);
	Print_add_Line(print_temp,0);
	sprintf(print_temp,"day:%i",optBuffer[3]);
	Print_add_Line(print_temp,0);
	sprintf(print_temp,"mon:%i",optBuffer[4]);
	Print_add_Line(print_temp,0);
	sprintf(print_temp,"yea:%i",optBuffer[5]);
	Print_add_Line(print_temp,0);
	_delay_ms(2000);
	
	#endif
	
	
	
	optType optholder = {
		.offsetValue =			 (((uint64_t) optBuffer[6] << 24) | ((uint64_t) optBuffer[7] << 16) | ((uint64_t) optBuffer[8] << 8) | optBuffer[9])* 1000 ,
		.offsetVolume =          (((uint64_t) optBuffer[10] << 24) | ((uint64_t) optBuffer[11] << 16) | ((uint64_t) optBuffer[12] << 8) | optBuffer[13])* 1000 ,
		.offsetCorrVolume =	     (((uint64_t) optBuffer[14] << 24) | ((uint64_t) optBuffer[15] << 16) | ((uint64_t) optBuffer[16] << 8) | optBuffer[17])* 1000 ,
		.Value =  0 ,
		.Volume = 0 ,
		.CorrVolume = 0 ,
		.t_transmission_min =    ((uint16_t) optBuffer[18] << 8) | optBuffer[19] ,
		.t_transmission_max =    ((uint16_t) optBuffer[20] << 8) | optBuffer[21] ,
		.delta_V =				 (((uint32_t) optBuffer[22] << 8) | optBuffer[23])* 1000 ,
		.delta_p =				 ((uint16_t) optBuffer[24] << 8) | optBuffer[25],
		.step_Volume =			 (((uint32_t) optBuffer[26] << 8) | optBuffer[27])* 1000 ,
		.T_Compensation_enable =  optBuffer[28],
		.Temperature_norm =      ((uint16_t) optBuffer[29] << 8) | optBuffer[30] ,
		.p_Compensation_enable =  optBuffer[31] ,
		.Pressure_norm =         ((uint16_t) optBuffer[32] << 8) | optBuffer[33],
		.Ping_Intervall = optBuffer[34]
	};
	

	
	status_ms_bytes.byte_96 = optBuffer[35];
	
	

	
	#ifdef TP_comp_debug

	lcd_Cls(BGC);
	if (optholder.T_Compensation_enable)
	{
		Print_add_Line("Tcomp ena",1);
	}
	else
	{
		Print_add_Line("Tcomp dis",1);
	}
	if (optholder.p_Compensation_enable)
	{
		Print_add_Line("Pcomp ena",0);
	}
	else
	{
		Print_add_Line("Pcomp dis",0);
	}
	
	
	optholder.T_Compensation_enable = 1;
	optholder.p_Compensation_enable = 1;
	
	#endif
	
	#ifdef force_tp_comp
	optholder.T_Compensation_enable = 1;
	optholder.p_Compensation_enable = 1;
	#endif
	

	
	
	
	// check Option Validity
	if( (0 == optholder.t_transmission_max)||
	(0 == optholder.delta_V) ||
	(0 == optholder.delta_p) ||
	(0 == optholder.step_Volume) ||
	(0 == optholder.Temperature_norm)||
	(0 == optholder.Pressure_norm))
	{
		BIT_SET(sendbuffer[0],status_bit_success_setting_options_93);  // not successfully accepted
		SET_ERROR(OPTIONS_ERROR);
		xbee_send_message(answer_code,sendbuffer,1);

		return ;
	}
	
	//checking bounds

	optholder.t_transmission_max = (uint16_t)checkBounds(optholder.t_transmission_max,TRANS_MAX,&outofBundsFlag);
	optholder.t_transmission_min = (uint16_t)checkBounds(optholder.t_transmission_min,TRANS_MIN,&outofBundsFlag);
	optholder.delta_V = (uint32_t)checkBounds(optholder.delta_V,DELTA_V,&outofBundsFlag);
	optholder.delta_p= (uint16_t)checkBounds(optholder.delta_p,DELTA_P,&outofBundsFlag);
	optholder.step_Volume= (uint32_t)checkBounds(optholder.step_Volume,STEP_VOL,&outofBundsFlag);
	optholder.Temperature_norm= (uint16_t)checkBounds(optholder.Temperature_norm,TEMP_NORM,&outofBundsFlag);
	optholder.Pressure_norm= (uint16_t)checkBounds(optholder.Pressure_norm,PRESS_NORM,&outofBundsFlag);
	optholder.Ping_Intervall= (uint8_t)checkBounds(optholder.Ping_Intervall,PING_INT,&outofBundsFlag);


	if (outofBundsFlag)
	{
		
		BIT_SET(sendbuffer[0],status_bit_success_setting_options_93);  // not successfully accepted
		SET_ERROR(OPTIONS_ERROR);
		SET_ERROR(OPTIONS_OUT_OF_BOUNDS_ERROR);
		xbee_send_message(answer_code,sendbuffer,1);

		return ;
	}


	if (optholder.delta_V < optholder.step_Volume)
	{
		optholder.delta_V = optholder.step_Volume;
	}
	
	if (BIT_CHECK(status_ms_bytes.byte_96,status_bit_set_offsets_96))
	{
		options.offsetValue =  optholder.offsetValue ;
		options.offsetVolume = optholder.offsetVolume ;
		options.offsetCorrVolume = optholder.offsetCorrVolume;
		
		options.Value = options.offsetValue;
		options.Volume = options.offsetVolume;
		options.CorrVolume = options.offsetCorrVolume;
	}
	options.t_transmission_min = optholder.t_transmission_min;
	options.t_transmission_max = optholder.t_transmission_max;
	options.delta_V =  optholder.delta_V;
	options.delta_p = optholder.delta_p;
	options.step_Volume = optholder.step_Volume;
	options.T_Compensation_enable = optholder.T_Compensation_enable;
	options.Temperature_norm = optholder.Temperature_norm;
	options.p_Compensation_enable = optholder.p_Compensation_enable;
	options.Pressure_norm = optholder.Pressure_norm;
	options.Ping_Intervall = optholder.Ping_Intervall;
	
	old.Value = options.Value;
	old.Volume = options.Volume;
	old.CorrVolume = options.CorrVolume;
	
	
	if (options.step_Volume >= 1000000)
	{
		position_volume_dot_point = 0;
	}
	else if (options.step_Volume >= 100000)
	{
		position_volume_dot_point = 1;
	}
	else if (options.step_Volume >= 10000)
	{
		position_volume_dot_point = 2;
	}
	else
	{
		position_volume_dot_point = 3;
	}
	



	

	CLEAR_ERROR(OPTIONS_ERROR);;
	CLEAR_ERROR(VOLUME_TOO_BIG_ERROR);;// no overflow in Value, Volume and CorrVolume
	
	// Options successfully set
	sendbuffer[0] = 0;
	CLEAR_ERROR(OPTIONS_ERROR);;
	xbee_send_message(answer_code,sendbuffer,1);
	paint_info_line(" Op93 ",0);
	_delay_ms(2000);

	if((options.T_Compensation_enable || options.p_Compensation_enable)&& (!connected.BMP)){
		SET_ERROR(TEMPPRESS_ERROR);
		}else{
		CLEAR_ERROR(TEMPPRESS_ERROR);;
	}

	return ;
}



long readVcc() {
	// Read 1.1V reference against AVcc
	// set the reference to Vcc and the measurement to the internal 1.1V reference
	#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
	ADMUX = _BV(MUX5) | _BV(MUX0) ;
	#else
	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	#endif
	
	_delay_ms(20); // Wait for Vref to settle
	ADCSRA |= _BV(ADSC); // Start conversion
	while ( ADCSRA & (1<<ADSC) ){} // measuring
	
	uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
	uint8_t high = ADCH; // unlocks both
	
	long result = ((high<<8) | low);
	
	//result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
	return result; // Vcc in millivolts
}


//=========================================================================
// MAIN PROGRAM
//=========================================================================
int main(void)
{
	// Display light control pin
	DDRD |= (1<<DDD6);			// Set Pin B0 as output
	
	
	init();
	


	
	
	//=========================================================================
	// Conversion/temp variables
	//=========================================================================
	uint8_t 	buffer[SINGLE_FRAME_LENGTH];

	/*
	adc_init(0x0e);
	lcd_Cls(BGC);
	while (1)
	{
	double Vcc = readChannel( (_BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1)),20) ;
	sprintf(print_temp,"%f V",Vcc);
	LCD_String(print_temp,0,0);
	_delay_ms(100);
	lcd_Cls(BGC);
	_delay_ms(50);
	}
	*/
	//=========================================================================
	// Display connection is in progress
	//=========================================================================
	#ifdef GCM_old_disp
	Print_add_Line("Network con.",1);
	Print_add_Line("in progress",0);
	
	#endif
	
	#ifdef ili9341
	Print_add_Line("Network connection",0);
	Print_add_Line("in progress",0);
	#endif

	
	//=========================================================================
	// Try to establish connection to the network
	//=========================================================================
	#ifdef USE_LAN
	Print_add_Line("Pls. activate",0);
	Print_add_Line("Coordinator",0);

	_delay_ms(3000);
	
	
	//reset timer!
	uint32_t time_first_try = count_t_elapsed;
	uint8_t delta_t;
	
	uint8_t sim_xb_rep = 0xFF;
	
	_Bool CoordActive = false;
	

	// main part
	while(1)
	{
		delta_t = count_t_elapsed - time_first_try;
		if(delta_t > 60)
		{
			Print_add_Line("...failed!",0);
			Print_add_Line("offline mode",0);
			SET_ERROR(INIT_OFFLINE_ERROR);
			break;			//stop trying on timeout return bad reply
		}
		
		sim_xb_rep  = xbee_hasReply(CE_MSG_TYPE,EQUAL);
		if (sim_xb_rep != 0xFF ){
			CoordActive = 1;
			
			sendbuffer[0] = 1;
			
			xbee_pseudo_send_AT_response( 'C', 'E', 0, sendbuffer, 1);
			break;			//stop trying Coordinator was activated

			
		}
		
	}
	
	sim_xb_rep = 0xFF;

	if (CoordActive)
	{
		
		
		// Coordinator is active and IP should be set correctly --> Send login data
		uint8_t reply_id = xbee_send_login_msg(LOGIN_MSG, buffer);
		
		if (reply_id!= 0xFF ){ // GOOD OPTIONS RECEIVED
			Set_Options((uint8_t *)frameBuffer[reply_id].data,OPTIONS_SET_ACK);
		}
		else // DEFECTIVE OPTIONS RECEIVED OR NO NETWORK
		{
			SET_ERROR(OPTIONS_ERROR);
			SET_ERROR(INIT_OFFLINE_ERROR);
		}
		
	}
	#endif

	#ifdef USE_XBEE
	// set default sc mask
	xbee_Set_Scan_Channels(xbee.ScanChannels);
	xbee_WR();
	
	//sprintf(print_temp,"SC:%#04x",xbee_Scan_Channels());
	//Print_add_Line(print_temp,2);

	if(xbee_reset_connection(0))
	{
		if(xbee_get_server_adrr())
		{
			
			/*
			//=========================================================================
			// Ping Coordinator to check if connection is stable reconnect if needed
			for(int i = 0;i< 5 ;i++){
			ping_server(dest_high,dest_low);
			_delay_ms(200);
			}
			*/
			_delay_ms(2000);
			if(!CHECK_ERROR(NETWORK_ERROR))
			{
				#ifdef ili9341
				Print_add_Line("..success!",2);
				#endif
				#ifdef GCM_old_disp
				Print_add_Line("...success!",0);
				#endif
				
				_delay_ms(2000);
				//=========================================================================
				// Device Login
				//=========================================================================

				uint8_t reply_id = xbee_send_login_msg(LOGIN_MSG, buffer);
				
				if (reply_id!= 0xFF ){ // GOOD OPTIONS RECEIVED
					Set_Options((uint8_t*)frameBuffer[reply_id].data,OPTIONS_SET_ACK);
				}

				
			}
			else
			{ // No stable Connection was reached
				Print_add_Line("...failed!",0);
				Print_add_Line("offline mode",0);
				SET_ERROR(INIT_OFFLINE_ERROR);
			}


		}
	}
	
	else
	{
		Print_add_Line("...failed!",0);
		Print_add_Line("offline mode",0);
		xbee.netstat = NO_NETWORK;
		SET_ERROR(INIT_OFFLINE_ERROR);

	} // end of if(ex_errorCode != ex_errorCode_Offline)
	#endif

	//=========================================================================
	//Initial setup
	//=========================================================================
	
	// faulty options, BMP, DS3231M, or no connection on Startup all lead to Error State
	//
	if ((CHECK_ERROR(INIT_OFFLINE_ERROR)) || (CHECK_ERROR(TEMPPRESS_ERROR)) || (CHECK_ERROR(TIMER_ERROR) )|| (CHECK_ERROR(OPTIONS_ERROR)))
	{
		while (1)
		{
			char errString[300];
			//==============================================
			//Initially Offline
			if(CHECK_ERROR(INIT_OFFLINE_ERROR)){
				
				
				sprintf(errString,STR_INIT_OFFLINE_MESSAGE,(xbee.netstat == NO_NETWORK)?STR_NO_COORDINATOR_FOUND:STR_NO_SERVER_FOUND);

				
				ErrMessage(STR_INIT_OFFLINE,errString,blue,white);
				
				
			}
			//==============================================
			//Faulty Options Received
			if(CHECK_ERROR(OPTIONS_ERROR)){
				if (CHECK_ERROR(OPTIONS_LEN_ERROR))
				{
					sprintf(errString,STR_OPTIONS_LENGTH_MESSAGE,NUMBER_LOGIN_BYTES,StartStat.opt_len_received);
					
					ErrMessage(STR_OPTIONS_LENGTH,errString,blue,white);
				}
				
				
				if(CHECK_ERROR(OPTIONS_OUT_OF_BOUNDS_ERROR)){
					sprintf(errString,STR_OPTION_RANGE_ERR_MESSAGE);
					
					char boundsstr[100];
					for(uint8_t i = 0;i < StartStat.bound_err_counter; i++ ){
						uint8_t errVal = StartStat.boundsErrors[i];
		
						
						sprintf(boundsstr,STR_RANGE_ERROR,
						StartStat.optStrings[errVal],
						StartStat.errorVal[errVal],
						StartStat.optBounds[errVal][0],
						StartStat.optBounds[errVal][1]);
						strcat(errString,boundsstr);
					}
					
					ErrMessage(STR_OPTION_RANGE_ERR,errString,blue,white);
				}
				
			}
			//==============================================
			//BMP or DS3231M error
			if (CHECK_ERROR(TEMPPRESS_ERROR) )
			{
				
				ErrMessage(STR_BMP_SENSOR_ERROR,STR_BMP_SENSOR_ERROR_MESSAGE,blue,white);
			}
			if (CHECK_ERROR(TIMER_ERROR))
			{
				ErrMessage(STR_RTC_ERROR,STR_RTC_ERROR_MESSAGE,blue,white);

			}
		}
	}else  // One measure send cycle on startup

	{
		xbee_coordIdentifier(); // get name of connected Coordinator
		reset_display(1);
		paint_Date();
		//MEASUREMENT BLOCK
		if (connected.BMP){ // measurement is only done if T OR P compensation is enabled
			if(BMP_Temp_and_Pressure()){
				SET_ERROR(TEMPPRESS_ERROR);;

			}
		}
		Temp_Press_CorrectedVolume();
		displayTemPreVol();
		
		//SENDING BLOCK
		Collect_Measurement_Data();

		if (CHECK_ERROR(NETWORK_ERROR))
		{
			//sendbuffer[MEASUREMENT_MESSAGE_LENGTH-1]= 0; //delete Status byte
			store_measurement();

		}
		else
		{
			// send Measurement Data to Server
			if( 0xFF ==xbee_send_request(MEAS_MSG,sendbuffer,MEASUREMENT_MESSAGE_LENGTH))
			{
				paint_info_line("No Ack",0);
				_delay_ms(500);
				SET_ERROR(NETWORK_ERROR);
				ex_mode = offline;
				t_start(RECONNECT,options.Ping_Intervall*60);

				
				
			}
		}
		

		
		
		
		// Reset for delta_V calculation
		delta.Volume_since_last_send = 0;
		
		// Reset for delta_P calculation
		last.Pressure_on_send = options.Pressure_value;
		
		// Reset for time since last sent i.e. delta_t_send
		last.time_send = count_t_elapsed;
		
		



	}

	//=========================================================================
	// Main loop
	//=========================================================================
	

	while(1)
	{
		


		//===============================================================================================
		// RESET DISPLAY
		// Resets Display after t_reset_display time has elapsed
		//=======================================================
		#ifdef USE_DISPLAY
		delta.t_Display_reset = count_t_elapsed - last.time_display_reset;
		
		if (delta.t_Display_reset >= reset_display_Interval)  // After this time (in minutes) the display is reset (to avoid display problems after a possible loss of power)
		{
			last.time_display_reset = count_t_elapsed;
			reset_display(1);
			


		}
		#endif
		
		
		// Measurement of Temp and Pressure every Measure_Intervall (5s)
		
		if((count_t_elapsed % Measure_Interval == 0) && ((count_t_elapsed - last.time_Pressure_Temp_meas) > 1 )) //every Measure_Intervall (5s)
		{
			last.time_Pressure_Temp_meas = count_t_elapsed;
			
			if (!connected.BMP)
			{
				if (!BMP_Connected()){
					connected.BMP = 1;
					connected.TWI = 1;
					init_BMP(&paint_info_line);
				}
			}
			
			
			
			if (connected.BMP){ // measurement is only done if BMP Sensor is connected
				
				BMP_Pressure_old = BMP_Pressure;
				BMP_Temperature_old = BMP_Temperature;
				
				if(BMP_Temp_and_Pressure())
				{
					SET_ERROR(I2C_BUS_ERROR);;
					SET_ERROR(TEMPPRESS_ERROR);;
					
					
					// Quick Bus recovery to recover from short disturbances, the Server was notified regardless
					paint_info_line("clrI2C",1);
					uint8_t i2cState = I2C_ClearBus();
					
					
					connected.TWI = 1;
					
					if(!i2cState){
						if (!BMP_Temp_and_Pressure())
						{
							CLEAR_ERROR(TEMPPRESS_ERROR);;
							CLEAR_ERROR(I2C_BUS_ERROR);;
							connected.BMP =1 ;
							}else{
							connected.BMP = 0;
						}
					}
					
				}
				else
				{
					CLEAR_ERROR(TEMPPRESS_ERROR);;
					
					CLEAR_ERROR(I2C_BUS_ERROR);;

				}
				
				// Plausibility check of pressure and Temperature Measurement.
				// this checks the Measured values for Plausibility, and does another Measurement if they are out of the specified range.
				// ---> it is very unlikely that a Bitflip occurs twice
				if (connected.TWI )
				{
					PT_Plausibility();
				}
				
				
			}
			
			Temp_Press_CorrectedVolume();
			displayTemPreVol();
		}
		
		
		delta.t_send = count_t_elapsed - last.time_send;
		
		
		//Avoid Overflow when subtracting
		if(options.Pressure_value > last.Pressure_on_send)
		{
			delta.Pressure_since_last_send = options.Pressure_value - last.Pressure_on_send;
		}
		else
		{
			delta.Pressure_since_last_send = last.Pressure_on_send -options.Pressure_value;
		}
		
		
		//============================================================================================================================================
		// checks if either:
		// 1. the maximum time(t_transmission_max) has passed since last send  OR
		// 2. delta_V is reached and at least t_transmission_min time has passed since last send
		// 3. delta_P is reached and at least t_transmission_min time has passed since last send
		
		
		if((delta.t_send >= options.t_transmission_max * 60)|| //
		(delta.t_send >= options.t_transmission_min && delta.Volume_since_last_send > options.delta_V)||
		(delta.t_send >= options.t_transmission_min && delta.Pressure_since_last_send > options.delta_p)||
		(delta.t_send >= options.t_transmission_min && status_reset_on_send) )
		{

			

			Collect_Measurement_Data();
			
			// reset cached error bits;
			status_reset_on_send = 0;
			if (CHECK_ERROR(NETWORK_ERROR))
			{
				store_measurement();
				
			}
			else
			{
				// send Measurement Data to Server
				if( 0xFF ==xbee_send_request(MEAS_MSG,sendbuffer,MEASUREMENT_MESSAGE_LENGTH))
				{
					paint_info_line("No Ack",0);
					_delay_ms(500);
					SET_ERROR(NETWORK_ERROR);
					ex_mode = offline;
					t_start(RECONNECT,options.Ping_Intervall*60);

				}
			}
			
			// Reset for delta_V calculation
			delta.Volume_since_last_send = 0;
			
			// Reset for delta_P calculation
			last.Pressure_on_send = options.Pressure_value;
			
			// Reset for time since last sent i.e. delta_t_send
			last.time_send = count_t_elapsed;
			

		}
		
		
		//=========================================================================================
		//		Checks on all I2C DEVICES are done every 5min
		//=========================================================================================
		
		if (t_check(I2C_CHECK))
		{
			t_start(I2C_CHECK,20);
			
			if(!connected.TWI || !connected.DS3231M || (!connected.BMP && connected.BMP_on_Startup) || CHECK_ERROR(I2C_BUS_ERROR))
			{
				
				uint8_t i2cState = I2C_ClearBus();
				uint8_t DS3231Mstate = 0;
				uint8_t BMPState = 0;
				
				connected.TWI = 1;
				connected.DS3231M = 1;
				connected.BMP = 1;
				
				_delay_ms(50);
				if(!i2cState)
				{
					
					DS3231M_read_time();
					
					DS3231Mstate = CHECK_ERROR(TIMER_ERROR);
					
					if(!DS3231Mstate)
					{
						CLEAR_ERROR(I2C_BUS_ERROR);
						
					}

					
					BMPState = BMP_Temp_and_Pressure();

					if (!BMPState)
					{
						CLEAR_ERROR(TEMPPRESS_ERROR);;
						CLEAR_ERROR(I2C_BUS_ERROR);;
						connected.BMP =1 ;
					}
					else
					{
						connected.BMP = 0;
					}
					
					
				}
				
				if (connected.DS3231M || connected.BMP)
				{
					connected.TWI = 1;
				}
				
				I2C_Clear_view(i2cState,DS3231Mstate,BMPState);
				
			}
			
			
			
		}











		switch(ex_mode)
		{
			//==================================================================================================================
			//                 ONLINE-MODE
			//==================================================================================================================
			case online:
			//====================================================================================================================================


			
			
			
			
			
			//==============================================================================================
			//   PING
			//==========================================================
			// since pressure/temp is measured every 5s and ping is done every 60+2 seconds to ensure they dont get triggerd at the same Second
			if (t_check(PING))
			{
				t_start(PING,options.Ping_Intervall * 60);
				
				
				if(!ping_server()){

					break; // Ping unsuccessful --> offline Mode

				}
				
				
			}
			
			//==============================================================================================
			//   EXECUTE SERVER COMMANDS
			//==========================================================
			
			uint8_t reply_id;
			reply_id  = xbee_hasReply(LAST_NON_CMD_MSG,GREATER_THAN);
			if (reply_id != 0xFF){
				execute_server_CMDS(reply_id);
			}
			
			
			
			break;
			
			
			//==================================================================================================================
			//                 OFFLINE-MODE
			//==================================================================================================================
			case offline: ;
			
			//====================================================================================================================================

			#ifdef USE_LAN

			uint8_t reply_id_off  = xbee_hasReply(LAST_NON_CMD_MSG,GREATER_THAN);
			if (reply_id_off != 0xFF){
				execute_server_CMDS(reply_id_off);
			}

			#endif
			
			//=============================================================================================================================================
			//     RECONNECT
			//========================================================
			// try to Reconnect after every 60s (Reconnect_after_time)
			//========================================================
			if (t_check(RECONNECT)){
				
				t_start(RECONNECT,options.Ping_Intervall*60);
				
				#ifdef USE_XBEE
				
				if (!xbee_reconnect(0))
				{
					ex_mode = online;
					//Associated
					CLEAR_ERROR(NETWORK_ERROR);
					CLEAR_ERROR(NO_REPLY_ERROR);
					if(!ping_server()){
						paint_info_line("NoServ",0);
						NetStatIndex = 2;
						
					}
					else{
						ex_mode = online;
						CLEAR_ERROR(NETWORK_ERROR);;
						CLEAR_ERROR(NO_REPLY_ERROR);;
						
					}
					
				}
				else
				{

					paint_info_line("NoNetw",0);
					NetStatIndex = 1;
					

				}
				
				if(ex_mode == online){
					
					#endif
					#ifdef USE_LAN
					if(ping_server() || CE_received){
						ex_mode = online;
						CLEAR_ERROR(NETWORK_ERROR);;
						CE_received = 0;
						#endif
						
						
						//===============================================
						// send old measurements
						// transmit old measurements that could not be sent (if available) to database server
						_delay_ms(2000);
						if (numberMeasBuff > 0)
						{
							paint_send_stored_meas(numberMeasBuff,MEASBUFFER_LENGTH, 0);
							
							while (!CHECK_ERROR(NETWORK_ERROR) && ((numberMeasBuff) > 0))
							{

								paint_send_stored_meas(numberMeasBuff,MEASBUFFER_LENGTH, 1);
								
								memcpy(sendbuffer,measBuffer[firstMeasBuff].data,MEASUREMENT_MESSAGE_LENGTH);
								if( xbee_send_request(MEAS_MSG,sendbuffer, MEASUREMENT_MESSAGE_LENGTH) == 0xFF  )
								{//=========================
									paint_info_line("No Ack",0);
									_delay_ms(500);
									SET_ERROR(NETWORK_ERROR);
									ex_mode = offline;
									t_start(RECONNECT,options.Ping_Intervall*60);
									reset_display(1);
									break;
									
								}
								else
								{
									
									_delay_ms(1000);
									--numberMeasBuff;
									if (firstMeasBuff < (MEASBUFFER_LENGTH-1))
									{
										firstMeasBuff++;
									}
									else
									{
										firstMeasBuff = 0;
									}
									
								}
							}//while (!CHECK_ERROR(NETWORK_ERROR) && ((numberMeasBuff) > 0))
							reset_display(1);
						}//if (numberMeasBuff > 0)
					}// reconnect successful
				}// try reconnect every Reconnect_after_time
				
				break;


				default:
				break;
				
			}//switch(ex_mode)

		}//while(1)

	}//main()





	//=========================================================================
	// Interrupts
	//=========================================================================

	ISR(TIMER1_COMPA_vect)
	{
		count_t_elapsed++;
		system_tick();
	}

	ISR(INT2_vect)
	{
		count_Volume_steps++;
		
	}


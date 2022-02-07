/**
*	@file 		Gascounter_Main.h
*	@brief 		Gascounter Macros and Typedefs
*	@author		Klaus Kiefer, Peter Wegmann
*	@version	1.5
*	@date		2020/03/25
*
*
*/



// Gascounter_Main.h - Copyright 2016-2019, HZB, ILL/SANE & ISIS
// reused for gas counter 3/2019 KK


#ifndef GC_MAIN_H
#define GC_MAIN_H


#define FIRMWARE_VERSION  208 /**< @brief  Software Version  */

#define BRANCH_ID 2


//=========================================================================
// LAN OR XBEE
// =========================================================================
//#define USE_LAN /**< @brief LAN Hardware Variant*/
//#define USE_XBEE /**< @brief Xbee Hardware Variant*/

#ifdef USE_LAN

#define WRITE_IP_TO_EEPROM true
#define OCT1 192
#define OCT2 168
#define OCT3 178
#define OCT4 89
#endif

//=========================================================================


#define MEASBUFFER_LENGTH      250      /**< @brief Maximum number of concurrently saved Measurements*/
#define MEASUREMENT_MESSAGE_LENGTH 23  /**< @brief Number of Bytes that makes up one Measurement Message*/
#define NUMBER_LOGIN_BYTES	    36     /**< @brief Number of Bytes in login Message from Server   */

#define COM_TIMEOUT_TIME 		10	/**< @brief Communication Timeout-period in Seconds  */





#define USE_DISPLAY
#define ALLOW_COM					// Allow the possibility to receive answer from XBee module
#define ALLOW_DEBUG
#define ALLOW_LOGIN
#define ENABLE_ASSERTIONS
//#define ENABLE_FUNC_TRACE
//#define DEBUG_OPTIONS
//#define DEBUG_DS3231M
//#define TP_comp_debug
//#define force_tp_comp


#define DEF_offsetValue						0		// in 1 mL
//MIN
//MAX
#define DEF_offsetVolume					0		// in 1 mL
//MIN
//MAX
#define DEF_offsetCorrVolume				0		// in 1 mL
//MIN
//MAX
#define DEF_Value							0		// in 1 mL
//MIN
//MAX
#define DEF_Volume							0		// in 1 mL
//MIN
//MAX
#define DEF_CorrVolume						0		// in 1 mL
//MIN
//MAX


#define DEF_t_transmission_min				10		// in seconds
#define MIN_t_transmission_min				1		// in seconds
#define MAX_t_transmission_min				36000		// in seconds

#define DEF_t_transmission_max				120		// in minutes
#define MIN_t_transmission_max				1		// in minutes
#define MAX_t_transmission_max				1000	// in minutes

#define DEF_delta_V					      100000	// in 1 mL
#define MIN_delta_V						   	1000	// in 1 mLf
#define MAX_delta_V					    65000000	// in 1 mL

#define DEF_delta_p							100		// in 0.1 mbar
#define MIN_delta_p						      1  	// in 0.1 mbar
#define MAX_delta_p						   5000		// in 0.1 mbar

#define DEF_step_Volume						10000	// in 1 mL // normally 10L
#define MIN_step_Volume						 1000 	// in 1 mL // normally 10L
#define MAX_step_Volume				    65000000	// in 1 mL // normally 10L



#define DEF_T_Compensation_enable			0		// 1...ON	0...Off
#define DEF_p_Compensation_enable			0		// 1...ON	0...Off

#define DEF_Temperature_norm				2882	// in 0.1K //15.0 C as default reference
#define MIN_Temperature_norm				2500	// in 0.1K //15.0 C as default reference
#define MAX_Temperature_norm				3200	// in 0.1K //15.0 C as default reference

#define DEF_Pressure_norm					10133	// in 0.1 mBar
#define MIN_Pressure_norm					 8000	// in 0.1 mBar
#define MAX_Pressure_norm					13000	// in 0.1 mBar

#define DEF_Ping_Intervall					   10   // in Minutes
#define MIN_Ping_Intervall						1   // in Minutes
#define MAX_Ping_Intervall					  255	// in Minutes


#define MAX_NOMINAL_PRESSURE			117000 // 1170.00 mbar
#define MIN_NOMINAL_PRESSURE             95000 // 950.00 mbar
#define MAX_NOMINAL_TEMPERATURE			  3280  // 55°C
#define MIN_NOMINAL_TEMPERATURE			  2631  //-10°C

#define MAX_PRESSURE_DELTA			   12000 //  120mbar/5s
#define MAX_TEMPERATURE_DELTA			 130 //  13°C/5s

#define FUNTRACE_ARRAY_SIZE			10
#define FUNTRACE_HEADER_LEN			7
#define FUNTRACE_PASS				43690


#define CHECK_BOUNDS(VAR,MIN,MAX,DEF,FLAG) if((VAR < MIN) || (VAR > MAX || isnan(VAR))){VAR = DEF; FLAG = 1;};



/**
* @brief Time Pressure or Temperature deltas
*
* All Variables in Gascounter representing a difference in Time, Pressure or Temperature since the last Measuerement/Send/Reset
*/
typedef struct {
	uint32_t Volume_since_last_send;       /**< @brief Volume   since last send/save: resets to 0 every time the data is sent or saved in 1 mL */
	uint16_t Pressure_since_last_send;     /**< @brief Pressure change since last send/save: resets to 0 every time the data is sent or saved (in 0.1 mbar) */
	uint32_t t_send;					   /**< @brief Time since last sending of Data */
	uint32_t t_Display_reset;			   /**< @brief Time since last Display Reset */
} deltaType;


/**
* @brief Pressure- & Timestamps of previous Events
*
* Absolute Times/Pressure of the last time an action was executetd i.e: Pinging the Server
*/
typedef struct {
	uint32_t Pressure_on_send;         /**< @brief Pressure the last time Data was sent/stored to the Server/memory (in 0.1 mbar)*/
	uint32_t time_send;				   /**< @brief Absolute time of last sending of Data*/
	uint32_t time_display_reset;       /**< @brief Absolute time of last Display Reset*/
	uint32_t time_ping;				   /**< @brief Absolute time of last Ping*/
	uint32_t time_Pressure_Temp_meas;  /**< @brief Absolute time of last Temp and Pressure Measurement */
	uint32_t time_TWI_action;          /**< @brief Absolute time of last TWI read/write/stop */
	uint32_t time_I2C_check;		   /**< @brief Absolute time of last I2C Peripheral checkup */
	uint32_t time_valid_time_reading;   /**< @brief Absolute time of last Valid Time reading  */
}lastType;





/**
* @brief Options of the Gascounter
*
* Struct containing fields for all options of a Gascounter
*/
typedef struct{
	uint64_t offsetValue;			/**< @brief Offset for uncorrected Volume from Database (in 1ml)*/
	uint64_t offsetVolume;			/**< @brief Offset for uncorrected Volume from Database (in 1ml)*/
	uint64_t offsetCorrVolume;		/**< @brief Temperature and Pressure offset of corrected Volume of Gascounter. Should correspond to the Value on the analog Gascounter when receiving Options on startup (in 1ml) */
	uint64_t Value;				    /**< @brief Uncorrected Volume, periodically sent to Server (in 1ml)*/
	uint64_t Volume;				/**< @brief Uncorrected Volume, periodically sent to Server (in 1ml)*/
	uint64_t CorrVolume;			/**< @brief Corrected Volume, periodically sent to Server (in 1ml)*/
	uint16_t t_transmission_min;	/**< @brief Minimal time between Measurement-Data Transmissions (in seconds) */
	uint16_t t_transmission_max;	/**< @brief Maximum time between Measurement-Data Transmissions (in minutes) */
	uint32_t delta_V;				/**< @brief Delta Volume since last Measurement-Data Transmission (in 1ml) */
	uint16_t delta_p;				/**< @brief Delta Pressure since last Measurement-Data Transmission (in 0.1mbar) */
	uint32_t step_Volume;			/**< @brief Volume-Step that corresponds to one Tick from the Gascounter. Specific for the kind of Gascounter that is used. (in 1ml) */
	//int16_t offset_pressure;		/**< @brief Offset Pressure for Altitude compensation (in 0.1mbar) */  //TODO Remove
	//uint16_t span_pressure;			/**< @brief Span Pressure ?? in 10^-4 */
	uint8_t T_Compensation_enable;  /**< @brief Temperature-Compensation Boolean 1 = ON; 0 = OFF */
	uint16_t Temperature_norm;      /**< @brief Normal Temperature usually 15C (in 0.1K) */
	uint8_t p_Compensation_enable;  /**< @brief Pressure-Compensation Boolean 1 = ON; 0 = OFF */
	uint16_t Pressure_norm;			/**< @brief Normal Pressure usually 1033.3mbar (in 0.1mbar) */
	uint8_t Ping_Intervall;		/**< @brief Time between Pings to the Server (in minutes) */
}optType;



/**
* @brief USART communication
*
* Global Variables/Pointers needed for USART Communication
*/
typedef struct{
	uint8_t cmd_line;			/**< @brief USART Receive Block (Data is received bytewise)*/
	uint8_t *send_str_reader;	/**< @brief Pointer to the next byte to send via USART*/
	uint8_t sending_cmd;		/**< @brief Number of bytes to send via USART */
}usartType;



/**
* @brief Old Volume Measurement
*
* Holds the old #Value, #Volume and #CorrVolume Measurement in order to compare them with the new one  to check for overflows
*/
typedef struct{
	uint64_t Value ;		/**< @brief  Holds the old options#Value to compare them with the new to check for overflows.  */
	uint64_t Volume;		/**< @brief  Holds the old options#Volume to compare them with the new to check for overflows.  */
	uint64_t CorrVolume ;	/**< @brief  Holds the old options#CorrVolume to compare them with the new to check for overflows   */
}oldType;








#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c%i%i%i%i"
#define BYTE_TO_BINARY(byte)  \
(byte & 0x80 ? '1' : '0'), \
(byte & 0x40 ? '1' : '0'), \
(byte & 0x20 ? '1' : '0'), \
(byte & 0x10 ? '1' : '0'), \
(byte & 0x08 ? '1' : '0'), \
(byte & 0x04 ? '1' : '0'), \
(byte & 0x02 ? '1' : '0'), \
(byte & 0x01 ? '1' : '0'), \
connected.BMP, \
connected.DS3231M, \
connected.TWI, \
connected.BMP_on_Startup



extern volatile uint16_t count_Volume_steps;
extern volatile uint32_t count_t_elapsed;
extern volatile usartType USART;



enum PARENT_MODE
{
	offline,
	online
};



//==============================================================
// Device status
//==============================================================






/**
* @brief Status related information
*
*
* Contains Status-bytes for internal and external Status processing and reporting
*/
typedef struct
{
	uint8_t byte_90; /**< @brief  sent on registration with Server (#LOGIN_MSG)*/
	//uint8_t byte_91; /**< @brief  sent on Measurement Data Transmission  (#MEAS_MSG). communicates Device Status to server.*/
	uint8_t byte_92; /**< @brief  sent in response to send options Command (#GET_OPTIONS_CMD). Is currently always set to zero.   */
	//uint8_t byte_93; /**< @brief  sent in response to receiving options (#OPTIONS_SET_ACK). The 0th Bit set to one indicates an error with setting Options */
	//uint8_t byte_94;    always equals byte_91
	uint8_t byte_95; /**< @brief sent with every Ping (#PING_MSG). Currently always set to zero. */
	uint8_t byte_96; /**< @brief  received with Options from the Server. Indicates wether Volume-offsets should be kept or updated */
}statusType;


extern volatile statusType status_ms_bytes;
extern volatile uint8_t Xbee_Associated;
extern enum PARENT_MODE ex_mode;
extern uint16_t status_reset_on_send;


// I²C Codes

#define DEVICE_NOT_CONNECTED 2
#define I2C_ERROR 1
#define I2C_SUCCESS 0



//==============================================================
// Status bits
//==============================================================

// #90 registration
#define status_bit_registration_90				0

// #91 send data
#define status_bit_I2C_err_91					1 /**< @brief Bit that indicates a problem with the I2C-Bus --> BMP085 and DS3231M are not usable */
#define status_bit_option_err_91				2 /**< @brief Bit that indicates a problem setting options */
#define status_bit_Temp_Press_Err_91			3 /**< @brief Bit that indicates a problem with the BMP085 Sensor */
#define status_bit_volume_too_big_91			4 /**< @brief Bit that indicates that an overflow has occourred in Value (#optType), Volume (#optType) or CorrVolume (#optType) */
#define status_bit_DS3231M_err_91				5 /**< @brief Bit that indicates a problem with the DS3231M Chip */




// #93 send response options set
#define status_bit_success_setting_options_93	0 /**< @brief Bit that indicates if options were set successfully  */

// #96 set options
#define status_bit_set_offsets_96				0


// nw error		status.device|=(1<<0);ex_mode = ex_mode_offline;
//no reply			status.device|=(1<<1);
//#define SET_ERROR(OPTIONS_ERROR); 				status.device|=(1<<2);status.device_reset_on_Send |= status.device;
//#define SET_ERROR(TEMPPRESS_ERROR); 			status.device|=(1<<3);status.device_reset_on_Send |= status.device;
//#define SET_ERROR(TIMER_ERROR);					status.device|=(1<<4);status.device_reset_on_Send |= status.device;
//#define SET_ERROR(INIT_OFFLINE_ERROR);          status.device|=(1<<5);
//#define SET_ERROR(I2C_BUS_ERROR);				status.device|=(1<<6);status.device_reset_on_Send |= status.device;
//#define SET_ERROR(VOLUME_TOO_BIG_ERROR);		status.device|=(1<<7);status.device_reset_on_Send |= status.device;

//#define CLEAR_ERROR(NETWORK_ERROR); 			status.device&=~(1<<0);ex_mode = ex_mode_online;
//#define CLEAR_ERROR(NO_REPLY_ERROR); 			status.device&=~(1<<1);
//#define CLEAR_ERROR(OPTIONS_ERROR); 			status.device&=~(1<<2);
//#define CLEAR_ERROR(TEMPPRESS_ERROR); 			status.device&=~(1<<3);
//#define CLEAR_ERROR(TIMER_ERROR);			    status.device&=~(1<<4);
//#define CLEAR_ERROR(INIT_OFFLINE_ERROR);        status.device&=~(1<<5);
//#define CLEAR_ERROR(I2C_BUS_ERROR);             status.device&=~(1<<6);
//#define CLEAR_ERROR(VOLUME_TOO_BIG_ERROR);		status.device&=~(1<<7);

//#define CHECK_ERROR(NETWORK_ERROR) 			(status.device & (1<<0))
//#define CHECK_ERROR(NO_REPLY_ERROR) 			(status.device & (1<<1))
//#define CHECK_ERROR(OPTIONS_ERROR) 			(status.device & (1<<2))
//#define CHECK_ERROR(TEMPPRESS_ERROR) 			(status.device & (1<<3))
//#define CHECK_ERROR(TIMER_ERROR)				(status.device & (1<<4))
//#define CHECK_ERROR(INIT_OFFLINE_ERROR)        (status.device & (1<<5))
//#define CHECK_ERROR(I2C_BUS_ERROR)				(status.device & (1<<6))
//#define CHECK_ERROR(VOLUME_TOO_BIG_ERROR)      (status.device & (1<<7))


/* a=target variable, b=bit number to act upon 0-n */
#define BIT_SET(a,b) ((a) |= (1<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1<<(b)))
//#define BIT_FLIP(a,b) ((a) ^= (1<<(b)))
#define BIT_CHECK(a,b) (!!((a) & (1<<(b))))



#define SINGLE_FRAME_LENGTH 	256		// Full length of one frame  ATTENTION if changed then change DATA_LENGTH in xbee_utilities.h as well



// Function Declaration
void store_measurement(void);

void init(void);
void init_ports(void);
void init_timer(void);
void init_interrupts(void);

void displayTemPreVol(void);
void Temp_Press_CorrectedVolume(void);
void PT_Plausibility(void);
void reset_display(void);
void Funtrace_enter(uint8_t Function_ID);
uint8_t xbee_send_login_msg(uint8_t db_cmd_type, uint8_t *buffer);
void execute_server_CMDS(uint8_t reply_id);
uint8_t ping_server(void);
void Set_Options(uint8_t *optBuffer,uint8_t answer_code);
uint8_t analyze_Connection(void);

#endif  // Gascounter_main.h

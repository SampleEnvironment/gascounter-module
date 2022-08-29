/*
 * StringPixelCoordTabble_old.h
 *
 * Created: 24.06.2022 19:35:49
 *  Author: qfj
 */ 


#ifndef STRINGPIXELCOORDTABBLE_OLD_H_
#define STRINGPIXELCOORDTABBLE_OLD_H_



// INIT STRINGS
#define STR_HZB_GASCOUNTER "HZB Gascount"
#define STR_LAN_VARIANT "LAN-VARIANT"
#define STR_XBEE_VARIANT "XBEE-VARIANT"
#define STR_INIT_START "Init start"
#define STR_INIT_PORTS "Init ports"
#define STR_INIT_TIMER "Init timer"
#define STR_INIT_INTERRUPTS "Init interr."
#define STR_INIT_USART "Init usart"
#define STR_INIT_I2C "Init I2C"
#define STR_INIT_CLOCK "Init Clock"
#define STR_INIT_CLOCK_SUCC "...success"
#define STR_INIT_CLOCK_ERR "...error"
#define STR_INIT_PRESS "Init Press"
#define STR_INIT_PRESS_SUCC "...success"
#define STR_INIT_PRESS_ERR "...error"
#define STR_INIT_DONE "Init done"

// ERROR STRINGS
#define STR_INIT_OFFLINE "Netw. Err"
#define STR_INIT_OFFLINE_MESSAGE "Network connection error\n%s"
#define STR_NO_COORDINATOR_FOUND "No Coord."
#define STR_NO_SERVER_FOUND "No Server"

#define STR_OPTIONS_LENGTH "Opt. len Err"
#define STR_OPTIONS_LENGTH_MESSAGE "expected:%i\nreceived:%i"

#define STR_OPTION_RANGE_ERR "Bounds Err."
#define STR_OPTION_RANGE_ERR_MESSAGE " "
#define STR_RANGE_ERROR "%s\nr:%lu\ne:[%lu..%lu]\n"

#define STR_BMP_SENSOR_ERROR "BMP Error"
#define STR_BMP_SENSOR_ERROR_MESSAGE "Temp. Press.Sensor not connected"

#define STR_RTC_ERROR "RTC ERROR"
#define STR_RTC_ERROR_MESSAGE "Realtime clock not initialized"


#endif /* STRINGPIXELCOORDTABBLE_OLD_H_ */
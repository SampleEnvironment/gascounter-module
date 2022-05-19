/*
 * timerUtil.h
 *
 * Created: 18.05.2022 09:57:37
 *  Author: qfj
 */ 


#ifndef TIMERUTIL_H_
#define TIMERUTIL_H_


typedef struct  
{
	uint32_t start;
	uint32_t end;
}TimerType;


typedef struct  
{
	TimerType TArr[20];
	volatile uint32_t * t_elapsed;	
}TimerArrType;



typedef enum {
	RECONNECT,
	PING,
	I2C_CHECK
	
}TIMER_NAME;

void t_start(TIMER_NAME t_name,uint32_t duration_s);
uint8_t t_check(TIMER_NAME t_name);
#endif /* TIMERUTIL_H_ */
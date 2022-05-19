/*
 * timerUtil.c
 *
 * Created: 18.05.2022 09:57:51
 *  Author: qfj
 */ 

#include <stdint.h>

#include "Gascounter_main.h"
#include "timerUtil.h"

TimerArrType timers = {.t_elapsed = &count_t_elapsed,.TArr[RECONNECT].end = 0,.TArr[RECONNECT].start = 0};


void t_start(TIMER_NAME t_name,uint32_t duration_s){
	timers.TArr[t_name].start = *timers.t_elapsed;
	timers.TArr[t_name].end = *timers.t_elapsed +duration_s; 
} 

uint8_t t_check(TIMER_NAME t_name){
	if (*timers.t_elapsed >= timers.TArr[t_name].end){
	return 1;
	}
	return 0;
}
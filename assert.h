#ifndef ASSERT_H_
#define ASSERT_H_

#ifndef F_CPU
#define F_CPU 6144000
#endif

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "Gascounter_main.h"
#include "LCD.h"


#ifdef ENABLE_ASSERTIONS

#define AFASSERT(X) \
if (!(X)) \
{ \
	char assertstr[20];\
	LCD_Clear_row_from_column_to_column(0,11,0);\
	LCD_Clear_row_from_column_to_column(0,11,1);\
	LCD_Clear_row_from_column_to_column(0,11,2);\
	LCD_String("assert failed",0,0);\
	sprintf(assertstr,"%s",__FILE__);assertstr[16] = '\0';\
	LCD_String(&assertstr[5],0,1);\
	sprintf(assertstr,"line:%d",__LINE__);assertstr[11] = '\0';\
	LCD_String(assertstr,0,2);\
	while(1); \
}
#endif


#ifndef ENABLE_ASSERTIONS
	#define AFASSERT(X)
#endif



#ifdef ENABLE_FUNC_TRACE
#define FUNCTION_TRACE \
char fun_trace[60];\
LCD_Clear_row_from_column_to_column(0,11,0);\
sprintf(fun_trace,"%s",__FUNCTION__);\
fun_trace[11]='\0';\
LCD_String(fun_trace,0,0);\
_delay_ms(10);
#endif

 #ifndef ENABLE_FUNC_TRACE
	#define FUNCTION_TRACE
 #endif


#endif /* ASSERT_H_ */
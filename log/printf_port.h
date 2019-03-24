#ifndef _PRINTF_PORT_H_
#define _PRINTF_PORT_H_
#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
//
#define LOG_LEVEL_NONE   0
#define LOG_LEVEL_NORMAL 1
#define LOG_LEVEL_DEBUG  2
//	
#ifndef LOG_LEVEL_SET
#define LOG_LEVEL_SET LOG_LEVEL_NORMAL  // one of LOG_LEVEL_NONE / LOG_LEVEL_NORMAL / LOG_LEVEL_DEBUG
#endif

#if LOG_LEVEL_SET>LOG_LEVEL_NONE
//
#define LOG(level, fmt_str, ...) 	do                                           \
									{                                            \
										if(LOG_LEVEL_SET>=level)                 \
	                             		{printf_P(PSTR(fmt_str), ##__VA_ARGS__);}\
	                             	}while(0)

//
#define PRINT(fmt_str, ...)         do                                           \
									{                                            \
										if(LOG_LEVEL_SET>=LOG_LEVEL_NORMAL)      \
	                             		{printf_P(PSTR(fmt_str), ##__VA_ARGS__);}\
	                             	}while(0)


extern char log_uart_getchar(void);
extern void log_init(void);

#else /*LOG_LEVEL_SET>LOG_LEVEL_NONE*/

#define LOG(level, fmt_str, ...) do{}while(0)
#define PRINT(fmt_str, ...)      do{}while(0)

#endif/*LOG_LEVEL_SET>LOG_LEVEL_NONE*/

#ifdef __cplusplus
}
#endif
#endif

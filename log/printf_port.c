

#include "printf_port.h"
#define UART_BAUD_RATE  115200
#if LOG_LEVEL_SET>LOG_LEVEL_NONE
//
static int log_uart_putchar(char c, FILE *stream);
static FILE mystdout = FDEV_SETUP_STREAM(log_uart_putchar, NULL,_FDEV_SETUP_WRITE);
//
static void log_init_uart(void)
{
	#define BAUD UART_BAUD_RATE
	#define BAUD_TOL 3
   	#include <util/setbaud.h>
	//初始化。工作模式，帧结构等（UCSRnC）
	UCSR0C = (0<<UMSEL01)|(0<<UMSEL00)|0x06; //异步，8位数据，无奇偶校验，一个停止位 10000110
	//波特率的设置。（UBRRnL ,UBRRnH）
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE; 
	#if USE_2X
		UCSR0A |= (1<<U2X0);
	#else
		UCSR0A &= ~(1<<U2X0);
	#endif
	//选择终端号，编写中断服务程序。
	//中断的相关设置
	UCSR0B = (uint8_t)(1<<TXEN0)|(1<<RXEN0);//仅使能发送和接收
}
//串口发送byte,轮询 
static int log_uart_putchar(char c, FILE *stream)
{
	while(!(UCSR0A&(1<<UDRE0)));
	UDR0 = c;
	return 0;
}
//
static void log_init_printf(void)
{
	stdout = &mystdout;
}

char log_uart_getchar(void)
{
	while(!(UCSR0A&(1<<RXC0))); //等待接收到新数据
 	return UDR0;
}
//
void log_init(void)
{
	log_init_uart();
	log_init_printf();
}

#endif
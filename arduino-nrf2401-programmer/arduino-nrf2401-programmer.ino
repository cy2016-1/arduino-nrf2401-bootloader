#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "stk500v2_cmd.h"
#include "nrf24_device.h"
//此例程将arduino UNO 或 Nano作为无线编程器，将IDE传下来的烧写数据
//转发给目标板的bootloader,编程器端的接线如下：
//////////////////////////////////////////////////////////////
//          +--INT0[PD2]-[IN ] <====> IRQ----+             //
//          |--MISO[PB4]-[IN ] <====> MISO---|             //
// Arduino--|--MOSI[PB3]-[OUT] <====> MOSI---|---nRF24l01+ //
//          |--SCK [PB5]-[OUT] <====> SCK----|             //
//          |--SS  [PB2]-[OUT] <====> CSN----|             //
//          +--PB0 [PB1]-[OUT] <====> CE-----+             //
//////////////////////////////////////////////////////////////
//同时，如果想启用跳频传输，请将宏定义 NRF_RANDOM_SEED_ADC_PIN
//对应的ADC管脚悬空
/////////////////////////////////////////////////////////////////
//0-disable serial log output 
//1-serial log enable and only normal level
//2-serial log enable and print all
#define LOG_LEVEL_SET 1 
//enable/disable nrf24l01p-freq-hopping
#define NRF_ENABLE_FREQ_HOPPING    1     // 0 - disable / >0 - enable
#if NRF_ENABLE_FREQ_HOPPING
	//freq-hopping need some random value,so we get random seed by 
	//reading noise signal on a ADC pin, NRF_RANDOM_SEED_ADC_PIN defines
	//the target pin number (range: PIN0~PIN5)
	//Note: the target ADC pin MUST connect nothing!
	#define NRF_RANDOM_SEED_ADC_PIN    PIN0
	//air-data-rate after freq-hopping
	#define NRF_FREQ_HOPPING_AIR_DATA_RATE_SET  0     // 0-2Mbps / 1-1Mbps / 2-250Kbps
	//lowest freq (or left freq boundary) when generating RF channel
	//we will get a random RF channel between NRF_FREQ_HOPPING_LOWEST_CHANNEL and 2524 MHz
    #define NRF_FREQ_HOPPING_LOWEST_CHANNEL    2490  //MHz
#endif
//Maximum number of TX retransmits, range: 0~15
#define NRF_RETRANSMIT_MAX        5
//enable/disable nrf24l01p packet-loss statistics
#define NRF_ENABLE_PACKLOSS_CNT   1    // 0 - disable / >0 - enable

#define NRF_DEF_ADR_BYTE           0xA5   
#define NRF_DEF_AIR_DATA_RATE_SET  2     // 0-2Mbps / 1-1Mbps / 2-250Kbps
#define NRF_DEF_RF_CHANNEL         2522  // 2400~2525 MHz


//
#define LOG_LEVEL_NONE   0
#define LOG_LEVEL_NORMAL 1
#define LOG_LEVEL_DEBUG  2
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

#if LOG_LEVEL_SET>=LOG_LEVEL_DEBUG
extern void log_hex_dump(char*, int);
#define DUMP(src, sz) log_hex_dump(src, sz)
#else
#define DUMP(src, sz) do{/*do nothing*/}while(0)
#endif

extern char log_uart_getchar(void);
extern void log_init(void);
#define UART_BAUD_RATE  115200
//
static int log_uart_putchar(char c, FILE *stream);
static FILE mystdout;// = FDEV_SETUP_STREAM(log_uart_putchar, NULL,_FDEV_SETUP_WRITE);
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
	mystdout.put   = log_uart_putchar;
	mystdout.get   = NULL;
	mystdout.flags = _FDEV_SETUP_WRITE;
	mystdout.udata = 0;
	log_init_uart();
	log_init_printf();
}

#if LOG_LEVEL_SET>=LOG_LEVEL_DEBUG
void log_dump_send_byte(unsigned char ch)
{
	log_uart_putchar((char)ch, (FILE*)0);
}
void log_hex_dump(char* src, int sz)
{
	/*
	0000: 11 22 33 44 55 66 77 88    99 00 aa bb cc dd ee ff 
	0010: xx xx xx xx
	*/
	if(sz>0)
	{
		while(sz)
		{
			unsigned char ch;
			unsigned char line_cnt = 0;
			LOG(LOG_LEVEL_DEBUG, "%04X: ", (int)src);
			while(line_cnt<16)
			{
				ch = ((*src)&0xF0)>>4;
				if(ch<0x0A)
					ch += '0';//0~9
				else
					ch += ('A'-0x0A);
				log_dump_send_byte(ch);
				ch = (*src)&0x0F;
				if(ch<0x0A)
					ch += '0';//0~9
				else
					ch += ('A'-0x0A);
				log_dump_send_byte(ch);
				log_dump_send_byte(' ');
				src++;
				sz--;
				line_cnt++;
				if(0==sz)
				{
					log_dump_send_byte('\r');
					log_dump_send_byte('\n');
					return;
				}
				if(8==line_cnt)
				{
					log_dump_send_byte(' ');
					log_dump_send_byte(' ');
					log_dump_send_byte(' ');
				}
			}
			log_dump_send_byte('\r');
			log_dump_send_byte('\n');	
		}
	}
}
#endif

//init SPI
void nrf_port_spi_init(void)
{
//init PORT 
//If PORTxn is written to '1' when the pin is configured as an input pin, 
//the pull-up resistor is activated. 
//bits:  7    6     5     4     3    2    1    0       
//DDRB:  0    0     1     0     1    1    1    0  : 0x2E
//PORTB: 0    0     0     1     0    1    0    0  : 0x14
	DDRB  = 0x2E;
	PORTB = 0x14;
	DDRD  = 0x00;
	PORTD = 0x04;
//init SPI
/* Enable SPI, Master, set clock rate fck/2 */
	SPCR = ((1<<SPE)|(1<<MSTR));
	SPSR = (1<<SPI2X);
	//init IRQ interrupt
	;;;
}
//time delay
void nrf_port_delay_us(unsigned char dly_us)
{
	//avr-gcc下不使用标准库提供的计时函数 _delay_us()
	//_delay_us()存在浮点运算，会导致代码尺寸变大
	//而且nrf24l01对计时精度要求不高,参考 _delay_us()的实现代码，此处使用简单的for循环实现
	for(;dly_us;dly_us--)
	{
		_delay_loop_1((unsigned char)((F_CPU)/3e6));
	}
}
//time delay
void nrf_port_delay_ms(unsigned char dly_ms)
{
	//参考 _delay_ms()的实现代码，此处使用简单的for循环实现
	for(;dly_ms;dly_ms--)
	{
		_delay_loop_2(((F_CPU)/4e3));
	}
}
//set CE pin high
void nrf_port_CE_high(void)
{//pin PB1
	PORTB |= (unsigned char)(1<<PIN1);
}
//set CE pin low
void nrf_port_CE_low(void)
{//pin PB1
	PORTB &= (unsigned char)(~(1<<PIN1));
}
//set CSN pin high
void nrf_port_CSN_high(void)
{//pin PB2
	PORTB |= (unsigned char)(1<<PIN2);
}
//set CSN pin low
void nrf_port_CSN_low(void)
{//pin PB2
	PORTB &= (unsigned char)(~(1<<PIN2));
}
//get status of IRQ pin
//return： 0   - IRQ low (active)
//         ~=0 - IRQ high (not active) 
char nrf_port_IRQ_level(void)
{//pin PD2
	return PIND&0x04;
}
//SPI write-read byte
//in  - byte to write
//return the byte read back
char nrf_port_spi_rw(char in)
{
	SPDR = in;                  //Start transmission
	while(!(SPSR&(1<<SPIF))); //Wait for transmission complete
	return SPDR;
}

//low level api for cmd+data writing
//return current value of STATUS reg
static char nrf_cmd_write(char cmd, char* buf, char wrlen)
{
	char status = 0;
	NRF_CSN_LOW();   //Every new command must be started by a high to low transition on CSN
	NRF_DELAY_uS(1); //wait for Data Valid on  MISO 
	status = NRF_SPI_RW(cmd);
	while(wrlen>0)
	{//loop for sending bytes
		NRF_SPI_RW(*buf);
		buf++;
		wrlen--;
	}
	NRF_CSN_HIGH();  //set CSN before return  
	NRF_DELAY_uS(1); //keep CSN high for 1us, CSN Inactive time must no less than 50nS
	return status;
}
//low level api for cmd+data reading
//return current value of STATUS reg
static char nrf_cmd_read(char cmd, char* buf, char rdlen)
{
	char status = 0;
	NRF_CSN_LOW();
	NRF_DELAY_uS(1);
	status = NRF_SPI_RW(cmd);
	while(rdlen>0)
	{//loop for reading bytes
		*buf = NRF_SPI_RW(0xFF);
		buf++;
		rdlen--;
	}
	NRF_CSN_HIGH(); 
	NRF_DELAY_uS(1);
	return status;
}
//low level api for cmd writing
//return current value of STATUS reg
static char nrf_write_cmd_only(char cmd)
{
	return nrf_cmd_write(cmd, (char*)0, 0);
}
/********************************
Descrip:  Read multi-bytes from a register
Arg    :  
          regadr - 5 bit Register Map Address
          buf    - buffer for data saving
          rdlen  - number of bytes to read
Return :  current value of STATUS reg
*********************************/
char nrf_reg_read_buf(char regadr, char* buf, char rdlen)
{
	return nrf_cmd_read(NRF_CMD_R_REG(regadr), buf, rdlen);
}
/********************************
Descrip:  Read byte value from a register
Arg    :  regadr - 5 bit Register Map Address
Return :  the byte read back
*********************************/
char nrf_reg_read_byte(char regadr)
{
	char ret = 0;
	nrf_cmd_read(NRF_CMD_R_REG(regadr), &ret, 1);
	return ret;
}
/********************************
Descrip:  write multi-bytes to a register
Arg    :  
          regadr - 5 bit Register Map Address
          buf    - buffer for data saving
          wrlen  - number of bytes to write
Return :  current value of STATUS reg
*********************************/
char nrf_reg_write_buf(char regadr, char* buf, char wrlen)
{
	return nrf_cmd_write(NRF_CMD_W_REG(regadr), buf, wrlen);
}
/********************************
Descrip:  write byte to a register
Arg    :  
          regadr - 5 bit Register Map Address
          val    - byte to write
Return :  current value of STATUS reg
*********************************/
char nrf_reg_write_byte(char regadr, char val)
{
	return nrf_cmd_write(NRF_CMD_W_REG(regadr), &val, 1);
}
/********************************
Descrip:  flush RX FIFO
Arg    :  none
Return :  current value of STATUS reg
*********************************/     
char nrf_rx_flush(void)
{
	return nrf_write_cmd_only(NRF_CMD_FLUSH_RX);
}
/********************************
Descrip:  read RX-payload width in the RX FIFO.
Arg    :  none
Return :  payload width, used for nrf_rx_read_payload() 
*********************************/ 
char nrf_rx_payload_wid(void)
{
	char wid = 0;
	nrf_cmd_read(NRF_CMD_R_RX_PL_WID, &wid, 1);
	return wid;
}
/********************************
Descrip:  read RX-payload: 1 – 32 bytes. 
Arg    :  buf    - buffer for data saving
          rdlen  - number of bytes to read
Return :  current value of STATUS reg
*********************************/
char nrf_rx_read_payload(char* buf, char rdlen)
{
	return nrf_cmd_read(NRF_CMD_R_RX_PAYLOAD, buf, rdlen);
}
/********************************
Descrip:  flush TX FIFO
Arg    :  none
Return :  current value of STATUS reg
*********************************/
char nrf_tx_flush(void)
{
	return nrf_write_cmd_only(NRF_CMD_FLUSH_TX);
}
/********************************
Descrip:  reuse last transmitted payload.
Arg    :  none
Return :  current value of STATUS reg
*********************************/
char nrf_tx_reuse_payload(void)
{
	return nrf_write_cmd_only(NRF_CMD_REUSE_TX_PL);
}
/********************************
Descrip:  Write TX-payload, 1 – 32 bytes. 
Arg    :  buf    - buffer for data saving
          wrlen  - number of bytes to write
Return :  current value of STATUS reg
*********************************/
char nrf_tx_write_payload(char* buf, char wrlen)
{
	return nrf_cmd_write(NRF_CMD_W_TX_PAYLOAD, buf, wrlen);
}
/********************************
Descrip:  read the STATUS register
Arg    :  none
Return :  current value of STATUS reg
*********************************/
char nrf_status(void)
{
	return nrf_write_cmd_only(NRF_CMD_NOP);
} 
/********************************
Descrip:  write payload to be transmitted together with ACK packet on pipe.
Arg    :  pipe   - valid in the range: NRF_PIPE0 ~ NRF_PIPE5
          buf    - buffer for data saving
          wrlen  - number of bytes to write
Return :  current value of STATUS reg
*********************************/
char nrf_ack_write_payload(char pipe, char* buf, char wrlen)
{
	return nrf_cmd_write(NRF_CMD_W_ACK_PAYLOAD(pipe), buf, wrlen);
}
//
void nrf_power_up(void)
{
	char cfg = nrf_reg_read_byte(NRF_REG_CONFIG);
	if(!(cfg&(1<<NRF_BIT_PWR_UP)))
	{//set power up
		nrf_reg_write_byte(NRF_REG_CONFIG, cfg|(1<<NRF_BIT_PWR_UP));
	}
	NRF_DELAY_mS(2); //start up 1.5ms
}

void nrf_power_down(void)
{
	char cfg = nrf_reg_read_byte(NRF_REG_CONFIG);
	if(cfg&(1<<NRF_BIT_PWR_UP))   
	{//set power down
		cfg &= ~(char)(1<<NRF_BIT_PWR_UP); //clear bit
		nrf_reg_write_byte(NRF_REG_CONFIG, cfg);
	}
	NRF_DELAY_uS(1);
}
//mode - 0-PTX / 1-PRX
void nrf_set_primary_mode(char mode)
{
	if(mode<2)
	{
		char cfg = nrf_reg_read_byte(NRF_REG_CONFIG);
		//set bit,Enable CRC
		cfg |= (1<<NRF_BIT_EN_CRC);  
		//clr bit,CRC encoding scheme: 1 byte    
		cfg &= ~(char)((1<<NRF_BIT_CRCO)|(1<<NRF_BIT_PRIM_RX)); 
		//set mode
		cfg |= mode;
		nrf_reg_write_byte(NRF_REG_CONFIG, cfg);
	}
}
//set pipe address width, common for all data pipes.
//must be set before nrf_set_pipe_addr()/nrf_set_tx_addr()
//wid - must be one of the three values: 
//      NRF_VAL_AW_3Bytes
//      NRF_VAL_AW_4Bytes
//      NRF_VAL_AW_5Bytes
void nrf_set_addr_wid(char wid)
{
	nrf_reg_write_byte(NRF_REG_SETUP_AW, wid);
}
//set receive address for data pipe.
//pipe - pipe number, range: NRF_PIPE0~NRF_PIPE5
//addr - address buffer, MSBytes first for easy-reading.
//       e.g. { 0xA4, 0xA3, 0xA2, 0xA1, 0xA0 } (5bytes)
//len  - number of address bytes to write
void nrf_set_pipe_addr(char pipe, char* addr, char len)
{
	if((pipe>=NRF_PIPE0)&&(pipe<=NRF_PIPE5)&&(len>0)&&(len<=5))
	{//valid
		char* p;
		char adrtmp[5];
		if(pipe>=NRF_PIPE2)
		{//only use last byte
			addr += (len-1);
			len   = 1;
		}
		//LSBytes first 
		p = adrtmp;
		addr += (len-1);
		for(char i=0; i<len; i++)	
		{
			*p = *addr;
			p++;
			addr--;
		}
		nrf_reg_write_buf(NRF_REG_RX_ADDR_P0+pipe, adrtmp, len);
	}
}
//set transmit address, used for PTX only
//addr - address buffer, MSBytes first for easy-reading.
//       e.g. { 0xA4, 0xA3, 0xA2, 0xA1, 0xA0 } (5bytes)
//len  - number of address bytes to write
void nrf_set_tx_addr(char* addr, char len)
{
	if((len>0)&&(len<=5))
	{
		char adrtmp[5];
		char* p = adrtmp;
		//LSBytes first 
		addr += (len-1);
		for(char i=0; i<len; i++)	
		{
			*p = *addr;
			p++;
			addr--;
		}
		nrf_reg_write_buf(NRF_REG_TX_ADDR, adrtmp, len);
	}
}
//enable a data pipe
//pipe - pipe number, range: NRF_PIPE0~NRF_PIPE5
void nrf_pipe_enable(char pipe)
{
	if((pipe>=NRF_PIPE0)&&(pipe<=NRF_PIPE5))
	{
		char val;
		//enable [Dynamic Payload Length] and [Payload with ACK] (common for for all data pipes)
		nrf_reg_write_byte(NRF_REG_FEATURE, (1<<NRF_BIT_EN_DPL)|(1<<NRF_BIT_EN_ACK_PAY));
		//enable RX address of pipe
		val = nrf_reg_read_byte(NRF_REG_EN_RXADDR);
		val |= (1<<pipe);
		nrf_reg_write_byte(NRF_REG_EN_RXADDR, val);
		//enable auto-ACK for pipe
		val = nrf_reg_read_byte(NRF_REG_EN_AA);
		val |= (1<<pipe);
		nrf_reg_write_byte(NRF_REG_EN_AA, val);
		//enable Dynamic Payload Length for pipe
		val = nrf_reg_read_byte(NRF_REG_DYNPD);
		val |= (1<<pipe);
		nrf_reg_write_byte(NRF_REG_DYNPD, val);
	}
		
}
//get status of TX-FIFO and RX-FIFO
//return fifo status byte, used by 
//     NRF_FIFO_STATUS_TXREUSE()
//     NRF_FIFO_STATUS_TXFULL()
//     NRF_FIFO_STATUS_TXEMPTY()
//     NRF_FIFO_STATUS_RXFULL()
//     NRF_FIFO_STATUS_RXEMPTY()
char nrf_fifo_status(void)
{
	return nrf_reg_read_byte(NRF_REG_FIFO_STATUS);
}
//clear IRQ sources
//src_bits - range: 0x00~0x07                                                          
//           bit 7 ~ 3     2      1      0                               
//               NA  NA  RX_DR  TX_DS  MAX_RT                            
//                         0      1      0   : 0x02 - clr TX_DS only      
//                         1      0      1   : 0x05 - clr RX_DR and MAX_RT
//                         1      1      1   : 0x07 - clr all             
void nrf_clear_irq_src(char src_bits)
{
	src_bits &= 0x07;
	nrf_reg_write_byte(NRF_REG_STATUS, src_bits<<4);
}
//freq - 2400~2525 MHz
void nrf_set_rf_channel(uint16_t freq)
{
	nrf_reg_write_byte(NRF_REG_RF_CH, (char)(freq-2400));
}
//rate - one of NRF_VAL_RF_DR_250kbps
//              NRF_VAL_RF_DR_1Mbps
//              NRF_VAL_RF_DR_2Mbps 
void nrf_set_rf_data_rate(char rate)
{
	//default RF power : 0dBm
	nrf_reg_write_byte(NRF_REG_RF_SETUP, (char)(rate|NRF_VAL_RF_PWR_0dBm));
}
//delay - Auto Retransmit Delay, 
//        range: [NRF_VAL_ARD_250uS~NRF_VAL_ARD_4000uS]
//        delay = NRF_VAL_ARD_MAKE(delay_us)
//retry - Auto Retransmit Count, 
//        range: [NRF_VAL_ARC_none, NRF_VAL_ARC_1time~NRF_VAL_ARC_15time]
//        retry = NRF_VAL_ARC_MAKE(try_time)
void nrf_set_auto_retrans(char delay, char retry)
{
	nrf_reg_write_byte(NRF_REG_SETUP_RETR, (delay&0xF0)|retry);
}

void flash_copy(uint16_t flsh_adr, uint8_t* ram_adr, uint16_t sz)
{
	while(sz)
	{
		*ram_adr = pgm_read_byte(flsh_adr);
		flsh_adr++;
		ram_adr++;
		sz--;
	}
}

void ram_copy(uint8_t* dest, uint8_t* src, uint8_t sz)
{
	while(sz)
	{
		*dest = *src;
		dest++;
		src++;
		sz--;
	}
}

void uart_init(void)
{
	#define BAUD 115200
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
	UCSR0B = (uint8_t)(1<<TXEN0)|(1<<RXEN0);//仅使能发送和接收
}

void uart_putbyte(uint8_t c)
{
	while(!(UCSR0A&(1<<UDRE0)));
	UDR0 = c;
}

uint8_t uart_getbyte(void)
{
	while(!(UCSR0A&(1<<RXC0))); //等待接收到新数据
	return UDR0;
}

#if NRF_ENABLE_PACKLOSS_CNT
unsigned int pack_sum      = 0;
unsigned int pack_loss_cnt = 0;
#define packloss_get()    (pack_loss_cnt)
#define packloss_set(cnt) do{ pack_loss_cnt=(cnt); }while(0)
#define packloss_add(cnt) do{ pack_loss_cnt+=(cnt); }while(0)
#define packsum_get()     (pack_sum)
#define packsum_set(cnt)  do{ pack_sum=(cnt); }while(0)
#define packsum_add(cnt)  do{ pack_sum+=(cnt); }while(0)
#endif

#if NRF_DEF_AIR_DATA_RATE_SET==0
	#define NRF_AIR_DATA_RATE NRF_VAL_RF_DR_2Mbps
#elif NRF_DEF_AIR_DATA_RATE_SET==1
	#define NRF_AIR_DATA_RATE NRF_VAL_RF_DR_1Mbps
#elif NRF_DEF_AIR_DATA_RATE_SET==2
	#define NRF_AIR_DATA_RATE NRF_VAL_RF_DR_250kbps
#endif
#if NRF_AIR_DATA_RATE==NRF_VAL_RF_DR_2Mbps
	#define NRF_AUTO_RETRANS_DELAY NRF_VAL_ARD_500uS
#elif NRF_AIR_DATA_RATE==NRF_VAL_RF_DR_1Mbps
	#define NRF_AUTO_RETRANS_DELAY NRF_VAL_ARD_750uS
#elif NRF_AIR_DATA_RATE==NRF_VAL_RF_DR_250kbps
	#define NRF_AUTO_RETRANS_DELAY NRF_VAL_ARD_1750uS
#endif
void nrf_set_default_addr(void)
{
	char adr[5];
	static const char adr_rom[5] PROGMEM = 
	{//MSByte first
		NRF_DEF_ADR_BYTE, NRF_DEF_ADR_BYTE, NRF_DEF_ADR_BYTE, NRF_DEF_ADR_BYTE, NRF_DEF_ADR_BYTE
	}; 
	flash_copy((uint16_t)adr_rom, (uint8_t*)adr, 5);
	nrf_set_addr_wid(NRF_VAL_AW_5Bytes);
	nrf_set_pipe_addr(NRF_PIPE0, adr, 5);
	nrf_set_tx_addr(adr, 5);
}

void nrf_init(void)
{
	NRF_SPI_INIT();
	NRF_CE_LOW();
	NRF_CSN_HIGH();
	NRF_DELAY_mS(120); //wait for power on reset
	//config as PTX
	nrf_set_primary_mode(0); 
	nrf_set_rf_channel(NRF_DEF_RF_CHANNEL);
	nrf_set_rf_data_rate(NRF_AIR_DATA_RATE);
	nrf_set_auto_retrans(NRF_AUTO_RETRANS_DELAY, NRF_RETRANSMIT_MAX);
	nrf_set_default_addr();
	nrf_pipe_enable(NRF_PIPE0);
	nrf_tx_flush();
	nrf_rx_flush();
	nrf_clear_irq_src(0x07);
	nrf_power_up();
}
void nrf_clear(void)
{
	nrf_tx_flush();
	nrf_rx_flush();
	nrf_clear_irq_src(0x07);
}
uint8_t nrf_wait_for_irq(void)
{	//wait for IRQ pin active
	while(0!=NRF_IRQ_PIN());
#if NRF_ENABLE_PACKLOSS_CNT
	uint8_t cnt = NRF_OBSERVE_ARC(nrf_reg_read_byte(NRF_REG_OBSERVE_TX));
	packloss_add(cnt);
	if(cnt>0)
		LOG(LOG_LEVEL_NORMAL, "obsrv: %d , %d\r\n", cnt, packloss_get());
	if(cnt<NRF_RETRANSMIT_MAX)
	{
		cnt++; //last re-transmit success
	}
	packsum_add(cnt);
#endif
	return 0;
}
void nrf_sending_start(void)
{
	NRF_CE_HIGH();
	NRF_DELAY_uS(12);
	NRF_CE_LOW();
}

uint8_t stk500v2_cal_checksum(uint8_t* frame_buffer, uint16_t frame_len_no_cksum)
{
	uint8_t cksum = 0x00;

	uint8_t* pbuf = frame_buffer;
	uint16_t sz = frame_len_no_cksum - 5;
	frame_buffer[2] = sz>>8;  //SIZE1
	frame_buffer[3] = sz&0xFF;//SIZE0

	while(frame_len_no_cksum)
	{
		cksum ^= *pbuf;
		pbuf++;
		frame_len_no_cksum--;
	}
	return cksum;
}
//
//frame state machine
#define SM_START     0
#define SM_SEQ       1
#define SM_SIZE1     2
#define SM_SIZE0     3
#define SM_TOKEN     4
#define SM_DATA      5
#define SM_CHECKSUM  6
#define sm_reset()          do{ sm=SM_START; frame_len_no_cksum=0; frame_datacnt=0; frame_ptr=frame_buffer; }while(0)
#define buf_append_byte(ch) do{ *frame_ptr++ = ch; }while(0)
#define buf_get_byte(idx)   (frame_buffer[idx])
uint16_t stk500v2_get_frame(uint8_t* frame_buffer, uint16_t bufsz)
{
	uint8_t  sm;
	uint8_t* frame_ptr;
	uint16_t frame_len_no_cksum;
	uint16_t frame_datacnt;
	sm_reset();
	while(1)
	{
		uint8_t ch = uart_getbyte();
		buf_append_byte(ch);
		switch(sm)
		{
			case SM_START:
				{
					if(MESSAGE_START==ch)
						sm = SM_SEQ;
					else
						sm_reset();
				}
				break;
			case SM_SEQ:
				{
					sm = SM_SIZE1;
				}
				break;
			case SM_SIZE1:
				{
					sm = SM_SIZE0;
				}
				break;
			case SM_SIZE0:
				{
					frame_datacnt      = buf_get_byte(2)*256+ch;
					frame_len_no_cksum = frame_datacnt+5;
					sm = SM_TOKEN;
				}
				break;
			case SM_TOKEN:
				{
					if(TOKEN==ch)
						sm = SM_DATA;
					else
						sm_reset();
				}
				break;
			case SM_DATA:
				{
					frame_datacnt--;
					if(0==frame_datacnt)
						sm = SM_CHECKSUM;
				}
				break;
			case SM_CHECKSUM:
				{
					return frame_len_no_cksum+1;
				}
				break;
			default:
				break;
		}
	}
}

void stk500v2_send_ack(uint8_t* frame_buffer, uint16_t frame_len_no_cksum)
{
	uint16_t frame_len = frame_len_no_cksum+1;
	frame_buffer[frame_len_no_cksum] = stk500v2_cal_checksum(frame_buffer, frame_len_no_cksum);
	for(uint16_t i=0; i<frame_len; i++)
	{
		uart_putbyte(frame_buffer[i]);
	}
}
//return sum of bytes in RX FIFO
uint8_t nrf_read_rx_fifo(uint8_t* pbuf)
{
	uint8_t sum = 0;
	while(!NRF_FIFO_STATUS_RXEMPTY(nrf_fifo_status()))
	{
		uint8_t wid = nrf_rx_payload_wid();
		nrf_rx_read_payload((char*)pbuf, (char)wid);
		pbuf += wid;
		sum += wid;
	}
	return sum;
}
//
//ret - ack frame length from nrf-bootloader  OR
//      0 -- sending timeout, nrf-bootloader not response
//      1 -- receive timeout, 
uint16_t stk500v2_cmd_execution(uint8_t* frame_buffer, uint16_t frame_len)
{
	#define SEND_RETRY_MAX 100
	#define RECV_RETRY_MAX 200
	uint16_t retry;
	uint8_t* frame_ptr = frame_buffer;
	//send frame data to nrf-bootloader
	nrf_clear();
	while(frame_len)
	{
		char cpsz = 32;
		uint8_t status;
		if(frame_len<32)
		{
			cpsz = (char)frame_len;
		}
		nrf_tx_write_payload((char*)frame_ptr, cpsz);
		frame_ptr += cpsz;
		frame_len -= cpsz;
		//
		retry = 0;
		status = 0x1E; //make a fake MAX-RT IRQ to enter loop
		while(NRF_STATUS_MAXRT(status)&&(retry<=SEND_RETRY_MAX))
		{
			if(retry>0)
			{
				LOG(LOG_LEVEL_NORMAL, "S-RT\r\n");
			}
			nrf_clear_irq_src(0x01); //clr MAX-RT IRQ
			nrf_sending_start();
			nrf_wait_for_irq();
			status = nrf_status();
			retry++;
		}
		if(retry>SEND_RETRY_MAX)
		{//failed for all re-trans
			LOG(LOG_LEVEL_NORMAL, "RT-MAX:%d\r\n", retry);
			return 0;
		}
		nrf_clear_irq_src(0x07);
	}
	//receive ACK frame from nrf-bootloader
	uint8_t  query = ~(uint8_t)MESSAGE_START;
	retry          = 0;
	frame_len      = 0;
	frame_ptr      = frame_buffer;
	while(retry<RECV_RETRY_MAX)
	{
		nrf_tx_write_payload((char*)&query, 1);
		nrf_sending_start(); //as PTX, must send one byte to get ACK payload from PRX
		nrf_wait_for_irq();
		if(NRF_STATUS_MAXRT(nrf_status()))
		{//no response, maybe ACK frame is not ready
			LOG(LOG_LEVEL_NORMAL, "RT %02X\r\n", nrf_fifo_status());
			retry++;
		}
		else
		{//query-byte send over
			uint8_t rxlen = nrf_read_rx_fifo(frame_ptr);
			frame_len += rxlen;
			frame_ptr += rxlen;
			if(rxlen)
			{//IRQ = TX_DS + RX_DR -- has ACK payload
				LOG(LOG_LEVEL_DEBUG, "MORE\r\n");
				retry = 0;//query more payload
			}
			else if(frame_len>0)
			{//TX_DS only ack frame buffer not empty
				LOG(LOG_LEVEL_DEBUG, "OVER\r\n");
				nrf_tx_flush();
				nrf_clear_irq_src(0x07);
				break;//end of ACK frame
			}
			else
			{//TX_DS only and ack frame buffer is empty
				//maybe ACK frame is not ready
				LOG(LOG_LEVEL_DEBUG, "GOON\r\n");
				retry++;//goon to query first piece of ACK frame
			}
		}
		nrf_tx_flush();
		nrf_clear_irq_src(0x07);
	}
	if(retry==RECV_RETRY_MAX)
	{//error
		LOG(LOG_LEVEL_NORMAL, "FAILED\r\n");
		nrf_clear();
		return 1;
	}
	return frame_len;
}

#if NRF_ENABLE_FREQ_HOPPING

uint8_t random_byte(void)
{
	uint8_t ret;
	_delay_ms(2);
	ADCSRA |= (1<<ADSC);       //start ADC Conversion : single conversion mode
	while(!(ADCSRA&(1<<ADIF)));//wait conversion complete
	ret = ADC&0xFF;            //read ADC value
	ADCSRA |= (1<<ADIF);       //clear ADC Interrupt Flag
	return ret;
}
void random_deinit(void)
{
	ADMUX  = 0x00;
	ADCSRA = 0x00;
}
//return: 0-illegal seed / >0-valid seed
uint16_t random_seed_generate(void)
{
	//Voltage Reference Selection:
	ADMUX  = ((0<<REFS1)|(1<<REFS0));   // AVCC, 5V on arduino UNO/Nano
	//Analog Channel Selection
	ADMUX |= (NRF_RANDOM_SEED_ADC_PIN&0x07); //select ADC pin
	//ADC Prescaler select: 
	//AVR ADC must be clocked at the frequency between 50 and 200kHz. 
	//So we need to set proper prescaller bits so that scaled system clock would fit in this range.
	//arduino at 16MHz, 128 scaling factor, 16000000/128=125kHz
	ADCSRA = ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));
	//Enable ADC but disable ADC Interrupt
	ADCSRA |= ((1<<ADEN)|(0<<ADIE));
	//create seed
	uint8_t r0 = random_byte();
	uint8_t r1 = random_byte();
	uint8_t r2 = random_byte();
	if((r0==r1)&&(r1==r2))
	{
		return 0; //bad seed
	}
	else
	{
		return (r0<<8)|r1;
	}
}

struct _rf_hopping_cfg_
{
	uint8_t delay_ms;      //
	uint8_t rf_channel;
	uint8_t air_data_rate;
	uint8_t pipe_addr[5];  //five bytes, LSByte first   
} hopping_cfg;
// false - freq-hopping NOT finished
// true  - freq-hopping HAS BEEN finished
uint8_t hop_status_check(void)
{
	return (hopping_cfg.rf_channel<0x80);
}

void hop_config_clr(void)
{
	hopping_cfg.rf_channel = 0xFF;
}

void hop_config_reload(void)
{//create random cfg
	uint8_t rf;
	uint8_t a0,a1,a2,a3,a4;
	uint16_t seed = random_seed_generate();
	if(seed>0)
	{
		srand(seed);
		rf  = (uint8_t)(rand()%(2525-NRF_FREQ_HOPPING_LOWEST_CHANNEL));
		a0  = (uint8_t)(rand()%255);
		a1  = (uint8_t)(rand()%255);
		a2  = (uint8_t)(rand()%255);
		a3  = (uint8_t)(rand()%255);
		a4  = (uint8_t)(rand()%255);
		if((NRF_DEF_RF_CHANNEL-NRF_FREQ_HOPPING_LOWEST_CHANNEL)==rf)
		{
			rf -= 2; //skip the default RF channel
		}
		if((0x00==a0)&&(0x00==a1)&&(0x00==a2)&&(0x00==a3)&&(0x00==a4))
		{
			a0=a1=a2=a3=a4=(uint8_t)(~NRF_DEF_ADR_BYTE);
		}
	}
	else
	{
		rf = (NRF_DEF_RF_CHANNEL-NRF_FREQ_HOPPING_LOWEST_CHANNEL)-2;
		a0=a1=a2=a3=a4 = (uint8_t)(~NRF_DEF_ADR_BYTE);
	}
	random_deinit();
	//(LOG_LEVEL_NORMAL, "\r\nHOP-CFG: %02d-%02X %02X %02X %02X %02X\r\n", rf, a4, a3, a2, a1, a0);
	//
	hopping_cfg.rf_channel    = rf+(NRF_FREQ_HOPPING_LOWEST_CHANNEL-2400); //NRF_FREQ_HOPPING_LOWEST_CHANNEL~2524MHz
	hopping_cfg.pipe_addr[0]  = a4; //A4
	hopping_cfg.pipe_addr[1]  = a3; //A3
	hopping_cfg.pipe_addr[2]  = a2; //A2
	hopping_cfg.pipe_addr[3]  = a1; //A1
	hopping_cfg.pipe_addr[4]  = a0; //A0
	//
	hopping_cfg.delay_ms      = 5;

	#if NRF_FREQ_HOPPING_AIR_DATA_RATE_SET==0
		hopping_cfg.air_data_rate = NRF_VAL_RF_DR_2Mbps;
	#elif NRF_FREQ_HOPPING_AIR_DATA_RATE_SET==1
		hopping_cfg.air_data_rate = NRF_VAL_RF_DR_1Mbps;
	#elif NRF_FREQ_HOPPING_AIR_DATA_RATE_SET==2
		hopping_cfg.air_data_rate = NRF_VAL_RF_DR_250kbps;
	#endif
}
//1B 04 00 11 0E F0 05 00 02 C0 D0 02 C1 D1 06 C2 D0 D1 D2 D3 D4 00 CK 
//                   |     |____|  |______|  |_________________|  +---> end of config sequence
//                   |       |        |              |                     
//                   |       |        |              +------> nrf2401 config item2,LSByte-first for multi-bytes-data
//                   |       |        +------> nrf2401 config item1
//                   |       +-----> nrf2401 config item0
//                   +----> the bootloader should delay N ms before using this config sequence     
const uint8_t hop_seq_def[] PROGMEM = 
{ 
	MESSAGE_START, 0x00, 0x00, 0x11, TOKEN, CMD_NRF2401_FREQ_HOPPING, // data length: 17 bytes 
	0x00, // delay ms
	0x00, // reserve
	//set RF channel
	0x02, NRF_CMD_W_REG(NRF_REG_RF_CH), 0x00,
	//set air-data-rate
	0x02, NRF_CMD_W_REG(NRF_REG_RF_SETUP), NRF_VAL_RF_PWR_0dBm,
	//set pipe0 address, 5 bytes, LSBytes first
	0x06, NRF_CMD_W_REG(NRF_REG_RX_ADDR_P0), 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 
	//END of config flow
	0x00
};
uint8_t hop_create_command(uint8_t* buf, uint8_t sz)
{
	flash_copy((uint16_t)hop_seq_def, buf, (uint16_t)sizeof(hop_seq_def));
	//byte[6] is delay ms
	buf[6]   = hopping_cfg.delay_ms;
	//byte[10] is data byte of RF channel 
	buf[10]  = hopping_cfg.rf_channel;
	//byte[13] is data byte of RF air-data-rate
	buf[13] |= hopping_cfg.air_data_rate;
	//byte[16]~byte[20] is pipe address 
	//LSBytes first
	buf[16]  = hopping_cfg.pipe_addr[4]; //A0
	buf[17]  = hopping_cfg.pipe_addr[3]; //A1
	buf[18]  = hopping_cfg.pipe_addr[2]; //A2
	buf[19]  = hopping_cfg.pipe_addr[1]; //A3
	buf[20]  = hopping_cfg.pipe_addr[0]; //A4
	//byte[22] is check-sum
	buf[22]  = stk500v2_cal_checksum(buf, 22);
	return 23; //frame length
}

uint8_t hopping_buffer[50];
void nrf_config_reset(void)
{//back to default set
	LOG(LOG_LEVEL_DEBUG, "RF-RESET\r\n");
	NRF_CE_LOW(); //go Standby-I
	nrf_set_rf_channel(NRF_DEF_RF_CHANNEL);
	nrf_set_rf_data_rate(NRF_AIR_DATA_RATE);
	nrf_set_auto_retrans(NRF_AUTO_RETRANS_DELAY, NRF_RETRANSMIT_MAX);
	nrf_set_default_addr();
}
void nrf_config_hopping(void)
{
	LOG(LOG_LEVEL_DEBUG, "RF-HOP\r\n");
	NRF_CE_LOW(); //go Standby-I
	//RF channel
	nrf_set_rf_channel(2400+hopping_cfg.rf_channel);
	//air-data-rate
	nrf_set_rf_data_rate(hopping_cfg.air_data_rate);
	//auto-retransmit
	if(hopping_cfg.air_data_rate==NRF_VAL_RF_DR_2Mbps)
		nrf_set_auto_retrans(NRF_VAL_ARD_500uS, NRF_RETRANSMIT_MAX);
	else if(hopping_cfg.air_data_rate==NRF_VAL_RF_DR_1Mbps)
		nrf_set_auto_retrans(NRF_VAL_ARD_750uS, NRF_RETRANSMIT_MAX);
	else if(hopping_cfg.air_data_rate==NRF_VAL_RF_DR_250kbps)
		nrf_set_auto_retrans(NRF_VAL_ARD_1750uS, NRF_RETRANSMIT_MAX);
	//addr-width
	nrf_set_addr_wid(NRF_VAL_AW_5Bytes);  
	//RX pipe0 addr: MSbytes first
	nrf_set_pipe_addr(NRF_PIPE0, (char*)(hopping_cfg.pipe_addr), sizeof(hopping_cfg.pipe_addr));
	//TX addr: MSbytes first
	nrf_set_tx_addr((char*)(hopping_cfg.pipe_addr), sizeof(hopping_cfg.pipe_addr));
	//delay as same as bootloader
	NRF_DELAY_mS(hopping_cfg.delay_ms+2);
}
#endif

uint8_t frame_buffer[300];
void setup() {
	// put your setup code here, to run once:
	cli();
	log_init(); 
	uart_init();
	nrf_init();
#if NRF_ENABLE_PACKLOSS_CNT
	packloss_set(0);
	packsum_set(0);
#endif
#if NRF_ENABLE_FREQ_HOPPING
	hop_config_clr();
#endif
}

void loop() {
	// put your main code here, to run repeatedly:
	uint16_t frame_len = stk500v2_get_frame(frame_buffer, sizeof(frame_buffer));
	if(frame_buffer[frame_len-1]==stk500v2_cal_checksum(frame_buffer, frame_len-1))
	{//frame ok, goon
	#if NRF_ENABLE_FREQ_HOPPING
		if((CMD_SIGN_ON==frame_buffer[5])&&(!hop_status_check()))
		{
			uint16_t hop_cmd_len;
			hop_config_reload(); //create random config data
			hop_cmd_len = hop_create_command(hopping_buffer, (uint8_t)sizeof(hopping_buffer));
			hop_cmd_len = stk500v2_cmd_execution(hopping_buffer, hop_cmd_len);
			if(hop_cmd_len>1)
			{//bootloader has received the command
				nrf_config_hopping();
			}
		}
	#endif
		frame_len = stk500v2_cmd_execution(frame_buffer, frame_len);
		if(0==frame_len)
		{//error in [sending command frame]
			frame_buffer[6] = STATUS_CMD_TOUT;
			frame_len = 8;
		}
		else if(1==frame_len)
		{//error in [receiving ACK frame]
			frame_buffer[6] = STATUS_CMD_FAILED;
			frame_len = 8;
		}
	}
	else
	{//checksum error
		frame_buffer[6] = STATUS_CKSUM_ERROR;
		frame_len = 8;
	}
	stk500v2_send_ack(frame_buffer, frame_len-1);
	if(CMD_LEAVE_PROGMODE_ISP==frame_buffer[5])
	{
	#if NRF_ENABLE_FREQ_HOPPING
		hop_config_clr();
		nrf_config_reset();
	#endif
	#if NRF_ENABLE_PACKLOSS_CNT
		LOG(LOG_LEVEL_NORMAL, "package sum=%d , loss=%d\r\n", packsum_get(), packloss_get());
		packloss_set(0);
		packsum_set(0);
	#endif
	}
}

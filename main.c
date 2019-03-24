#include <inttypes.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "stk500v2_cmd.h"
#include "nrf24_register.h"
#include "printf_port.h"

#define SIGRD 5    //to solve "error: 'SIGRD' undeclared" problem in avr-gcc
#include <avr/boot.h>

//
#define NRF_DEF_ADR_BYTE      0xA5
#define NRF_AIR_DATA_RATE_SET 2 // 0-2Mbps / 1-1Mbps / 2-250Kbps
#define NRF_RF_CHANNEL        2522  // 2400~2525 MHz
#define WDTCSR_SET_OFF       (0)
#define WDTCSR_SET_1S        ((1<<WDE)|(1<<WDP2)|(1<<WDP1)) 
#define TMR1_INTERVAL_MS     (2000)
//
void app_code_start(void) __attribute__ ((naked));
#define BOOT_MODE_NORMAL   (0x00)
#define BOOT_MODE_CRITICAL (0x08)
//
uint8_t  frame_buffer[300];
uint16_t isp_address;
uint8_t  nrf2401_status; 
uint8_t  boot_mode;
#define  boot_mode_set(mode) do{boot_mode=mode;}while(0)
#define  boot_mode_get()     (boot_mode)
//
#define buf_get_byte(idx)     (frame_buffer[idx])
#define buf_set_byte(idx, ch) frame_buffer[idx] = (ch)
#define isp_addr_set(new_adr) do{isp_address=(new_adr);}while(0)
#define isp_addr_get()        (isp_address) 
#define isp_addr_add(step)    do{isp_address+=(step);}while(0)
#define nrf_dev_online()      do{nrf2401_status=0xAC;}while(0) 
#define nrf_dev_status()      (nrf2401_status==0xAC)        
//
#define TMR1_OVERFLOW()   (TIFR1&(1<<TOV1))
#define timer_check()     (TMR1_OVERFLOW()) //ret: 0-not timeout / >0-has timeout
#define wdt_set(set_val)  do{ WDTCSR=(1<<WDCE)|(1<<WDE); WDTCSR=set_val; }while(0)

void tmr1_set(void)
{
	TCCR1A = (uint8_t)(0x00); //normal mode
	TCCR1B = (uint8_t)(0x05); //prescaler = clk_io/1024
	TCNT1  = (uint16_t)(0xFFFF-((F_CPU/1000)*(TMR1_INTERVAL_MS))/1024);
}

void timer_start(void)
{
	if(BOOT_MODE_NORMAL==boot_mode_get())
		tmr1_set();
	else
		wdt_set(WDTCSR_SET_1S);
}
void timer_reset(void)
{
	if(BOOT_MODE_NORMAL==boot_mode_get())
		tmr1_set();
	else
		wdt_reset();
}
void timer_off(void)
{
	TCCR1B = 0x00;
	wdt_set(WDTCSR_SET_OFF); 
}
//
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
uint8_t  cal_checksum(uint16_t frame_len_no_cksum)
{
	uint8_t cksum = 0x00;
	//re-calculate msg body size here for reducing binary code size
	uint16_t sz = frame_len_no_cksum - 5;
	buf_set_byte(2, sz>>8  ); //SIZE1
	buf_set_byte(3, sz&0xFF); //SIZE0
	//
	uint8_t* pbuf = frame_buffer;
	while(frame_len_no_cksum)
	{
		cksum ^= *pbuf;
		pbuf++;
		frame_len_no_cksum--;
	}
	return cksum;
}
////////////////////////////////////////////////////////////////////
void stk500v2_uart_init(void)
{
	#define BAUD 115200
	#define BAUD_TOL 3
   	#include <util/setbaud.h>
	UCSR0C = (0<<UMSEL01)|(0<<UMSEL00)|0x06; //8N1 10000110
	UBRR0H = UBRRH_VALUE;                    //set baudrate
	UBRR0L = UBRRL_VALUE; 
	#if USE_2X
		UCSR0A |= (1<<U2X0);
	#else
		UCSR0A &= ~(1<<U2X0);
	#endif
	UCSR0B = (uint8_t)(1<<TXEN0)|(1<<RXEN0);//enable TX/RX
}
void stk500v2_uart_putbyte(uint8_t c)
{
	while(!(UCSR0A&(1<<UDRE0)));
	UDR0 = c;
}

//////////////////////////////////////////////////////////////
//          +--INT0[PD2]-[IN ] <====> IRQ----+             //
//          |--MISO[PB4]-[IN ] <====> MISO---|             //
// Arduino--|--MOSI[PB3]-[OUT] <====> MOSI---|---nRF24l01+ //
//          |--SCK [PB5]-[OUT] <====> SCK----|             //
//          |--SS  [PB2]-[OUT] <====> CSN----|             //
//          +--PB0 [PB1]-[OUT] <====> CE-----+             //
//////////////////////////////////////////////////////////////
#define NRF_CE_LOW()   do{ PORTB &= (uint8_t)(~(1<<PIN1));}while(0)
#define NRF_CE_HIGH()  do{ PORTB |= (uint8_t)(1<<PIN1);   }while(0)
#define NRF_CSN_LOW()  do{ PORTB &= (uint8_t)(~(1<<PIN2));}while(0)
#define NRF_CSN_HIGH() do{ PORTB |= (uint8_t)(1<<PIN2);   }while(0)

#define nrf_rx_flush()           nrf_write_cmd_only(NRF_CMD_FLUSH_RX)
#define nrf_tx_flush()           nrf_write_cmd_only(NRF_CMD_FLUSH_TX)
#define nrf_clear_irq(flag)      nrf_reg_byte_read_write(NRF_CMD_W_REG(NRF_REG_STATUS), flag)
#define nrf_set_rf_channel(freq) nrf_reg_byte_read_write(NRF_CMD_W_REG(NRF_REG_RF_CH), (freq-2400))//freq - 2400~2525 MHz
#define nrf_status()             nrf_write_cmd_only(NRF_CMD_NOP)
#define nrf_fifo_status()        nrf_reg_byte_read_write(NRF_CMD_R_REG(NRF_REG_FIFO_STATUS), 0)
#define nrf_rx_payload_wid()     nrf_reg_byte_read_write(NRF_CMD_R_RX_PL_WID, 0)
//get status of IRQ pin
//return： 0   - IRQ low (active)
//         ~=0 - IRQ high (not active) 
#define NRF_IRQ_PIN() (PIND&0x04)
uint8_t NRF_SPI_RW(uint8_t in)
{
	SPDR = in;                  //Start transmission
	while(!(SPSR&(1<<SPIF))); //Wait for transmission complete
	return SPDR;
}
void NRF_DELAY_1uS(void)
{
	_delay_loop_1((uint8_t)((F_CPU)/3e6));
}
void NRF_DELAY_mS(uint8_t dly_ms)
{
	for(;dly_ms;dly_ms--)
	{
		_delay_loop_2(((F_CPU)/4e3));
	}
}

uint8_t nrf_buffer[40];

void nrf_cmd_read_write(uint8_t len)
{
	uint8_t* pbuf = nrf_buffer;
	NRF_CSN_LOW();   //Every new command must be started by a high to low transition on CSN
	NRF_DELAY_1uS(); //wait for Data Valid on  MISO 
	while(len>0)
	{//loop for exchanging bytes
		*pbuf = NRF_SPI_RW(*pbuf);
		pbuf++;
		len--;
	}
	NRF_CSN_HIGH();  //set CSN before return  
	NRF_DELAY_1uS(); //keep CSN high for 1us, CSN Inactive time must no less than 50nS
}
uint8_t nrf_write_cmd_only(uint8_t cmd)
{
	nrf_buffer[0] = cmd;
	nrf_cmd_read_write(1);
	return nrf_buffer[0];
}
uint8_t nrf_reg_byte_read_write(uint8_t regcmd, uint8_t val)
{
	nrf_buffer[0] = regcmd;
	nrf_buffer[1] = val;
	nrf_cmd_read_write(2);
	return nrf_buffer[1];
}
void nrf_write_ack_payload(uint8_t* buf, uint8_t len)
{
	uint8_t* p = nrf_buffer;
	*p++ = NRF_CMD_W_ACK_PAYLOAD(NRF_PIPE0);
	for(uint8_t i=0; i<len; i++)
	{
		*p++ = buf[i];
	}
	nrf_cmd_read_write(len+1);
}
uint8_t nrf_rx_read_payload(void)
{
	uint8_t sz = 0;
	//read payload wid
	nrf_buffer[0] = NRF_CMD_R_RX_PL_WID;
	nrf_cmd_read_write(2);
	sz = nrf_buffer[1];
	//read payload
	nrf_buffer[0] = NRF_CMD_R_RX_PAYLOAD;
	nrf_cmd_read_write(sz+1);
	//now rx payload is nrf_buffer[1]~nrf_buffer[sz]
	return sz;
}

#if NRF_AIR_DATA_RATE_SET==0
	#define NRF_AIR_DATA_RATE NRF_VAL_RF_DR_2Mbps
#elif NRF_AIR_DATA_RATE_SET==1
	#define NRF_AIR_DATA_RATE NRF_VAL_RF_DR_1Mbps
#elif NRF_AIR_DATA_RATE_SET==2
	#define NRF_AIR_DATA_RATE NRF_VAL_RF_DR_250kbps
#endif
static const uint8_t nrf_init_seq_table[] PROGMEM = 
{
	//nrf_rx_flush()
	0x01, NRF_CMD_FLUSH_RX,   
	//nrf_tx_flush()          
	0x01, NRF_CMD_FLUSH_TX, 
	//nrf_clear_irq(0x70);//clear all IRQ            
	0x02, NRF_CMD_W_REG(NRF_REG_STATUS), 0x70,    
	//nrf_set_rf_channel
	0x02, NRF_CMD_W_REG(NRF_REG_RF_CH), (NRF_RF_CHANNEL-2400),
	//nrf_set_rf_data_rate(NRF_VAL_RF_DR_2Mbps);
	0x02, NRF_CMD_W_REG(NRF_REG_RF_SETUP), (NRF_AIR_DATA_RATE|NRF_VAL_RF_PWR_0dBm),
	// nrf_set_addr_wid(NRF_VAL_AW_5Bytes);
	0x02, NRF_CMD_W_REG(NRF_REG_SETUP_AW), NRF_VAL_AW_5Bytes,
	//set adr for pipe0, LSBytes first
	0x06, NRF_CMD_W_REG(NRF_REG_RX_ADDR_P0), NRF_DEF_ADR_BYTE, NRF_DEF_ADR_BYTE, NRF_DEF_ADR_BYTE, NRF_DEF_ADR_BYTE, NRF_DEF_ADR_BYTE,
	// nrf_set_feature((1<<NRF_BIT_EN_DPL)|(1<<NRF_BIT_EN_ACK_PAY));//enable [Dynamic Payload Length] 
	// and [Payload with ACK] (common for for all data pipes)
	0x02, NRF_CMD_W_REG(NRF_REG_FEATURE), (1<<NRF_BIT_EN_DPL)|(1<<NRF_BIT_EN_ACK_PAY),
	// nrf_enable_pipe((1<<NRF_PIPE0)); enable RX address of pipe0
	0x02, NRF_CMD_W_REG(NRF_REG_EN_RXADDR), (1<<NRF_PIPE0),
	// nrf_enable_pipe_auto_ack((1<<NRF_PIPE0)); enable auto-ACK for pipe0
	0x02, NRF_CMD_W_REG(NRF_REG_EN_AA), (1<<NRF_PIPE0),
	// nrf_enable_pipe_dpl((1<<NRF_PIPE0)); enable Dynamic Payload Length for pipe0
	0x02, NRF_CMD_W_REG(NRF_REG_DYNPD), (1<<NRF_PIPE0),
	//power up as PRX
	0X02, NRF_CMD_W_REG(NRF_REG_CONFIG), (1<<NRF_BIT_EN_CRC)|(1<<NRF_BIT_PRIM_RX)|(1<<NRF_BIT_PWR_UP),
	0x02, NRF_CMD_R_REG(NRF_REG_RX_ADDR_P0), 0x00,
	/////////////////////////////////////////////////////////
	//END of config flow
	0x00
};
static const uint8_t nrf_deinit_seq_table[] PROGMEM = 
{
	//nrf_rx_flush()
	0x01, NRF_CMD_FLUSH_RX,   
	//nrf_tx_flush()          
	0x01, NRF_CMD_FLUSH_TX, 
	//nrf_clear_irq(0x70);//clear all IRQ            
	0x02, NRF_CMD_W_REG(NRF_REG_STATUS), 0x70,    
	//power down
	0X02, NRF_CMD_W_REG(NRF_REG_CONFIG), 0x70,
	/////////////////////////////////////////////////////////
	//END of config flow
	0x00
};

void nrf_config_sequence(uint8_t* seq)
{
	while(1)
	{
		uint8_t len = *seq;
		if(0==len)
			break;
		seq++;
		ram_copy(nrf_buffer, seq, len);
		nrf_cmd_read_write(len);
		seq += len;
	}
}
void nrf_config_flash_sequence(uint16_t seq_flsh_adr, uint8_t seq_len)
{
	flash_copy((uint16_t)seq_flsh_adr, frame_buffer, seq_len);
	nrf_config_sequence(frame_buffer);
}

void nrf_wait_for_irq(void)
{	//wait for IRQ pin active
	while(0!=NRF_IRQ_PIN())
	{//check timeout
		if(timer_check())
		{
			app_code_start();
		}
	}
}

uint8_t stk500v2_getbyte(void)
{
	uint8_t ch;
	if(nrf_dev_status())
	{//data port is nrf24l01p
		static uint8_t  rx_buf[32];
		static uint8_t  rx_hidx;     //index the first data byte
		static uint8_t  rx_size;     //current size of data bytes
		while(0==rx_size)
		{//rx buffer empty
			nrf_wait_for_irq();
			if(NRF_STATUS_RXDR(nrf_status()))
			{//RX_DR IRQ 
				rx_size = nrf_rx_read_payload();
				//now rx payload is nrf_buffer[1]~nrf_buffer[rx_size]
				ram_copy(rx_buf, &nrf_buffer[1], rx_size);
				rx_hidx = 0;
			}
			nrf_tx_flush();
			nrf_clear_irq(0x70);
		}
		ch = rx_buf[rx_hidx]; //get the first data byte
		rx_hidx++;
		rx_size--;
	}
	else
	{//data port is uart
		while(!(UCSR0A&(1<<RXC0)))
		{
			if(timer_check())
			{
				app_code_start();
			}
		}
 		ch = UDR0;
	}

	return ch;
}

void nrf2401_init(uint8_t wait_pwron_ms)
{
	/////////////////////////////////////////////////
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
	/////////////////////////////////////////////////
	NRF_CE_LOW();
	NRF_CSN_HIGH();
	NRF_DELAY_mS(wait_pwron_ms); //wait for power on reset
	nrf_config_flash_sequence((uint16_t)nrf_init_seq_table, sizeof(nrf_init_seq_table)/sizeof(uint8_t));
	NRF_DELAY_mS(2); //start up 1.5ms
	//check if nrf2401 is working
	if(NRF_DEF_ADR_BYTE==nrf_buffer[1])
	{
		nrf_dev_online();
		NRF_CE_HIGH();   //enter RX mode
	}
	else
	{//disable SPI
		SPCR = 0x00; 
	}
}
////////////////////////////////////////////////////////////////////
void stk500v2_ack_wait_irq()
{
	nrf_wait_for_irq();
	nrf_rx_flush();
	nrf_clear_irq(0x70);
}
void stk500v2_send_ack(uint16_t frame_len)
{
	if(nrf_dev_status())
	{
		uint8_t* frm_ptr = frame_buffer;
		stk500v2_ack_wait_irq();
		while(1)
		{
			uint8_t cpysz  = 32;
			while(NRF_STATUS_TXFULL(nrf_status())||(0==frame_len))
			{
				stk500v2_ack_wait_irq();
				if(NRF_FIFO_STATUS_TXEMPTY(nrf_fifo_status()))
					return;
			}
			if(frame_len<32)
				cpysz = frame_len;
			nrf_write_ack_payload(frm_ptr, cpysz);
			frm_ptr   += cpysz;
			frame_len -= cpysz;
		}
	}
	else
	{//use uart as data port
		for(uint16_t i=0; i<frame_len; i++)
		{
			stk500v2_uart_putbyte(buf_get_byte(i));
		}
	}
}

#define status_byte_set(s) do{ frame_status = s; }while(0)
#define status_byte_get()  (frame_status)
void sm_cmd_action(void)
{
	uint8_t  delay_after_ack    = 0;
	uint8_t  frame_status       = STATUS_CMD_OK;
	uint8_t  cmd                = buf_get_byte(5);
	uint16_t frame_len_no_cksum = 7; //7 is the length of OK-ACK-WITHOUT-CHECKSUM 
	switch(cmd)
	{//byte[5] is command word
		case CMD_SIGN_ON           :
			{	//PC  : 1B 01 00 01 0E 01 14
				//STK : 1B 01 00 0B 0E 01 00 08 53 54 4B 35 30 30 5F 32 02  #STK500_2
				static const uint8_t stk_bootloader[] PROGMEM = {0x08, 'S', 'T', 'K', '5', '0', '0', '_', '2'};
				flash_copy((uint16_t)stk_bootloader, &frame_buffer[7], 9);
				frame_len_no_cksum = 16;
			}
			break;
		case CMD_GET_PARAMETER     :
			{	//PC  : 1B 0A 00 02 0E 03 97 89 
				//STK : 1B 0A 00 03 0E 03 OK XX CC
				buf_set_byte(7, 0x00);
				frame_len_no_cksum = 8;
			}
			break;
		case CMD_LOAD_ADDRESS      :
			{
				//PC  : 1B 0E 00 05 0E 06 00 00 00 00 18 
				//address in frame is based on word ,not on byte
				//adr_word = 0x01 ==> adr_byte = adr_word*2 = (0x01<<1)
				isp_addr_set((buf_get_byte(8)*256+buf_get_byte(9))<<1);
			}
			break;
		case CMD_ENTER_PROGMODE_ISP:
			{//enter critical boot mode
				timer_off();  
				boot_mode_set(BOOT_MODE_CRITICAL);
				timer_start();
			}
			break;
		case CMD_LEAVE_PROGMODE_ISP:
			{
				delay_after_ack = 5; //delay 5 ms
			}
			break;
		case CMD_PROGRAM_FLASH_ISP :
			{	//PC  : 1B 6F 00 8A 0E 13 00 80 C1 06 40 4C 20 FF FF DB0 DB1 DB2 ... DB127 19
				uint16_t page_addr = isp_addr_get();
				if((0==(page_addr%SPM_PAGESIZE))&&(0==buf_get_byte(6))&&(SPM_PAGESIZE==buf_get_byte(7)))
				{
					uint8_t* pbuf = &frame_buffer[15];
					boot_page_erase (page_addr);
			        boot_spm_busy_wait ();      // Wait until the memory is erased.
			        for(uint8_t i=0; i<SPM_PAGESIZE; i+=2)
			        {
			        	uint8_t lb = *pbuf++; //low-byte
						uint8_t hb = *pbuf++; //high-byte
						boot_page_fill(page_addr+i, (hb<<8)|lb);
			        }
					boot_page_write(page_addr);     // Store buffer in flash page.
			        boot_spm_busy_wait();                // Wait until the memory is written.
			        // Reenable RWW-section again. We need this if we want to jump back
			        // to the application after bootloading.
			        boot_rww_enable ();
					isp_addr_add(SPM_PAGESIZE);
				}
				else
				{
					status_byte_set(STATUS_CMD_TOUT);
				}
			}
			break;
		case CMD_READ_FLASH_ISP    :
			{	//PC  : 1B 75 00 04 0E 14 01 00 20 51 
				//STK : 1B 75 01 03 0E 14 00 XX XX (n bytes) XX 00 CC
				uint16_t rd_len = buf_get_byte(6)*256+buf_get_byte(7);
				flash_copy(isp_addr_get(), &frame_buffer[7], rd_len);
				isp_addr_add(rd_len);
				buf_set_byte(rd_len+7, 0x00);
				frame_len_no_cksum = rd_len+8;
			}
			break;
		case CMD_PROGRAM_EEPROM_ISP:
			{	//PC  : 1B 1F 00 0E 0E 15 00 04 C1 14 C1 C2 A0 FF FF AA BB CC DD 63 
				uint8_t sz1 = buf_get_byte(6);
				uint8_t sz0 = buf_get_byte(7);
				if((sz1==0)&&(sz0<=128))
				{
					sz0 += 15;
					for(uint8_t i=15; i<sz0; i++)
					{
						while(EECR&(1<<EEPE));     //Wait for completion of previous write
						EEAR  = isp_addr_get()>>1; //Set up address register 
						EEDR  = buf_get_byte(i);
						EECR |= (1<<EEMPE);        //Write logical one to EEMPE
						EECR |= (1<<EEPE);         //Start eeprom write by setting EEPE
						isp_addr_add(2);
					}
				}
			}
			break;
		case CMD_READ_EEPROM_ISP   :
			{	//PC  : 1B 13 00 04 0E 16 00 04 A0 B0
				//STK : 1B 13 00 XX 0E 16 00 XX XX XX XX 00 CC
				uint8_t sz1 = buf_get_byte(6);
				uint8_t sz0 = buf_get_byte(7);
				frame_len_no_cksum = 0;
				if((sz1==0)&&(sz0<=128))
				{
					uint8_t rd_idx_end = sz0+7;
					//7~N
					for(uint8_t i=7; i<rd_idx_end; i++)
					{//for EEPROM command, addr in CMD_LOAD_ADDRESS is based on byte not word 
						while(EECR&(1<<EEPE)); //Wait for completion of previous write
						EEAR  = isp_addr_get()>>1; //Set up address register 
						EECR |= (1<<EERE);         //Start eeprom read by writing EERE
						buf_set_byte(i, EEDR);     //read the byte
						isp_addr_add(2);
					}
					buf_set_byte(rd_idx_end, 0x00);
					frame_len_no_cksum = sz0+8;
				}
			}
			break;
		case CMD_READ_FUSE_ISP     :
		case CMD_READ_LOCK_ISP     :
			{	//PC  : 1B 07 00 06 0E 18 04 5X 0X 00 00 58 // CMD_READ_FUSE_ISP
				//PC  : 1B 0A 00 06 0E 1A 04 58 00 00 00 5F // CMD_READ_LOCK_ISP
				//STK : 1B 07 00 04 0E 18 00 XX 00 CC
				uint8_t fuse = buf_get_byte(7)+buf_get_byte(8);
				if(CMD_READ_LOCK_ISP==cmd)
				{//LOCK bits
					fuse = GET_LOCK_BITS;
				}
				else if(fuse==(0x50+0x00))
				{//Fuse bits
					fuse = GET_LOW_FUSE_BITS;
				}
				else if(fuse==(0x58+0x08))
				{//Fuse High bits
					fuse = GET_HIGH_FUSE_BITS;
				}
				else
				{//Extended Fuse bits
					fuse = GET_EXTENDED_FUSE_BITS;
				}
				buf_set_byte(7, boot_lock_fuse_bits_get(fuse)); //fuse byte
				buf_set_byte(8, STATUS_CMD_OK); //OK
				frame_len_no_cksum = 9;
			}
			break;
		case CMD_READ_SIGNATURE_ISP:
			{	//PC : 1B 04 00 06 0E 1B 04 30 00 00 00 38
				//STK: 1B 04 00 04 0E 1B 00 XX 00 CC
				//signature for ATmega328P is 1E 95 0F
				buf_set_byte(7, boot_signature_byte_get(buf_get_byte(9)<<1));
				buf_set_byte(8, STATUS_CMD_OK); //OK
				frame_len_no_cksum = 9;
			}
			break;
	#ifndef DISABLE_CMD_SPI_MULTI
		case CMD_SPI_MULTI         :
			{	//PC  : 1B 04 00 08 0E 1D 04 04 00 30 00 00 00 34 
				//STK : 1B 04 00 XX 0E 1D 00 XX XX XX XX 00 CC
				uint8_t isp_cmd = buf_get_byte(9);
				uint8_t ret_byte = 0x00;
				if(0x30==isp_cmd)
				{//read signature byte
					ret_byte = boot_signature_byte_get(buf_get_byte(11)<<1);
				}
				buf_set_byte(10, ret_byte);
				buf_set_byte(11, STATUS_CMD_OK); //OK
				frame_len_no_cksum = 12;
			}
			break;
	#endif
		case CMD_NRF2401_FREQ_HOPPING:
			{
				//PC  : 1B 04 00 11 0E F0 05 00 02 C0 D0 02 C1 D1 06 C2 D0 D1 D2 D3 D4 00 CK 
				//                         |     |____|  |______|  |_________________|  +---> end of config sequence
				//                         |       |        |              |                     
				//                         |       |        |              +------> nrf2401 config item2,LSByte-first for multi-bytes-data
				//                         |       |        +------> nrf2401 config item1
				//                         |       +-----> nrf2401 config item0
				//                         +----> the bootloader should delay N ms before using this config sequence      
				//STK : 1B 04 00 02 0E F0 00 CC  // just ACK OK
				//byte[6] -- delay ms
				delay_after_ack = buf_get_byte(6); //delay ms
				//byte[7] -- reserve
				//byte[8]~byte[frm_len-2] is nrf config sequence
			}
			break;
		case CMD_SET_PARAMETER     : //do nothing, just send OK-ACK
		case CMD_CHIP_ERASE_ISP    :
		case CMD_PROGRAM_LOCK_ISP  :
		case CMD_PROGRAM_FUSE_ISP  :
		default:
			break;
	}
	if(frame_len_no_cksum>0)
	{	// 1B SQ LL LL 0E CW OK .......
		//OK-ACK frame for any CMD has STATUS_CMD_OK on byte[6]
		buf_set_byte(6, status_byte_get());
		buf_set_byte(frame_len_no_cksum, cal_checksum(frame_len_no_cksum));
		stk500v2_send_ack(frame_len_no_cksum+1);
		NRF_DELAY_mS(delay_after_ack);
		if(CMD_LEAVE_PROGMODE_ISP==cmd)
		{
			app_code_start();
		}
		else if((CMD_NRF2401_FREQ_HOPPING==cmd)&&nrf_dev_status())
		{
			NRF_CE_LOW(); //exit RX mode and enter Standby-I
			nrf_config_sequence(&frame_buffer[8]);
			NRF_CE_HIGH();//re-enter RX mode
		}
	}
}

void app_code_start(void)
{
	UCSR0B = 0x00;  //disable uart
	if(nrf_dev_status())
	{//disable nrf2401
		NRF_CE_LOW();
		nrf_config_flash_sequence((uint16_t)nrf_deinit_seq_table, sizeof(nrf_deinit_seq_table)/sizeof(uint8_t));
		SPCR = 0x00; 
	}
	timer_off();
	((void(*)(void))0)();
}
//since this bootloader is not linked against the avr-gcc crt1 functions,
//to reduce the code size, we need to provide our own initialization
void __jump_main__(void) __attribute__ ((naked)) __attribute__ ((section (".init9")));
void __jump_main__(void)
{
	cli();                                   //disable interrupt
	asm volatile ( "clr __zero_reg__" );	 //clear r1 contains zero
	SP = (uint16_t)(RAMEND);                 //init SPH SPL
	asm volatile ( "jmp main"); 
}
//frame state machine
#define SM_START     0
#define SM_SEQ       1
#define SM_SIZE1     2
#define SM_SIZE0     3
#define SM_TOKEN     4
#define SM_DATA      5
#define SM_CHECKSUM  6

#define sm_reset()          do{ sm = SM_START; frame_len_no_cksum = 0; }while(0)
#define buf_reset()         do{ frame_ptr = frame_buffer;              }while(0)
#define buf_append_byte(ch) do{ *(frame_ptr++) = ch;                   }while(0)

int main(void) __attribute__ ((naked));
int main(void)
{
	uint16_t frame_datacnt = 0;
	uint8_t  sm;             //frame state machine
	uint16_t frame_len_no_cksum;
	uint8_t* frame_ptr;
	sm = MCUSR&((1<<WDRF)|(1<<EXTRF)); //check reset source
	MCUSR = 0;
	boot_mode_set(sm&(1<<WDRF));
	if(!sm)
		sm = 103; //not wdt-reset or extern-reset, must have more than 100ms delay in nrf2401_init()
	else
		sm = 0;   //wdt-reset or extern-reset, no need delay
	//
	nrf2401_init(sm);       //init data port
	stk500v2_uart_init();
	timer_start();
	sm_reset();             //init frame state machine
	buf_reset();            //clear frame buffer
	while(1)
	{
		uint8_t ch = stk500v2_getbyte();
		timer_reset();
		buf_append_byte(ch);
		switch(sm)
		{
			case SM_START:
				{
					if(MESSAGE_START==ch)
						sm = SM_SEQ;
					else
						buf_reset();
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
					frame_datacnt = buf_get_byte(2)*256+ch;
					frame_len_no_cksum = frame_datacnt+5;
					sm = SM_TOKEN;
				}
				break;
			case SM_TOKEN:
				{
					sm = SM_DATA;
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
					if(cal_checksum(frame_len_no_cksum)==ch)
					{
						sm_cmd_action();
					}
					sm_reset();
					buf_reset();
				}
				break;
			default:
				break;
		}
	}
	return 0;
}

#ifndef __NRF24_REGISTER_H__
#define __NRF24_REGISTER_H__

#ifdef __cplusplus
extern "C"{
#endif
////////////////////////////////////////////
////   define regs/bits for nrf24l01+   ////
////////////////////////////////////////////
//Configuration Register
#define NRF_REG_CONFIG 0x00 
//Mask interrupt caused by RX_DR
//1: Interrupt not reflected on the IRQ pin
//0: Reflect RX_DR as active low interrupt on the IRQ pin
#define NRF_BIT_MASK_RX_DR  6
//Mask interrupt caused by TX_DS
//1: Interrupt not reflected on the IRQ pin
//0: Reflect TX_DS as active low interrupt on the IRQ pin
#define NRF_BIT_MASK_TX_DS  5 	
//Mask interrupt caused by MAX_RT
//1: Interrupt not reflected on the IRQ pin
//0: Reflect MAX_RT as active low interrupt on the IRQ pin
#define NRF_BIT_MASK_MAX_RT 4
//Enable CRC. Forced high if one of the bits in the EN_AA is high
#define NRF_BIT_EN_CRC      3
//CRC encoding scheme: '0' - 1 byte / '1' – 2 bytes 
#define NRF_BIT_CRCO        2
//1: POWER UP, 0:POWER DOWN
#define NRF_BIT_PWR_UP      1
//RX/TX control  1: PRX, 0: PTX 
#define NRF_BIT_PRIM_RX     0
////////////////////////////////////////////
//Enable ‘Auto Acknowledgment’ Function
#define NRF_REG_EN_AA  0x01
//Enable auto acknowledgement data pipe 5
#define NRF_BIT_ENAA_P5     5
//Enable auto acknowledgement data pipe 4
#define NRF_BIT_ENAA_P4     4
//Enable auto acknowledgement data pipe 3
#define NRF_BIT_ENAA_P3     3
//Enable auto acknowledgement data pipe 2
#define NRF_BIT_ENAA_P2     2
//Enable auto acknowledgement data pipe 1
#define NRF_BIT_ENAA_P1     1
//Enable auto acknowledgement data pipe 0
#define NRF_BIT_ENAA_P0     0
////////////////////////////////////////////
//Enabled RX Addresses
#define NRF_REG_EN_RXADDR  0x02
//Enable data pipe 5
#define NRF_BIT_ERX_P5      5
//Enable data pipe 4
#define NRF_BIT_ERX_P4      4
//Enable data pipe 3
#define NRF_BIT_ERX_P3      3
//Enable data pipe 2
#define NRF_BIT_ERX_P2      2
//Enable data pipe 1
#define NRF_BIT_ERX_P1      1
//Enable data pipe 0
#define NRF_BIT_ERX_P0      0
////////////////////////////////////////////
//Setup of Address Widths (common for all data pipes)
#define NRF_REG_SETUP_AW   0x03
////////////////////////////////////////////
//Setup of Automatic Retransmission
#define NRF_REG_SETUP_RETR 0x04
////////////////////////////////////////////
//RF Channel
//The RF channel frequency determines the center of the channel used by the nRF24L01. The channel 
//occupies a bandwidth of 1MHz at 1Mbps and 2MHz at 2Mbps. nRF24L01 can operate on frequencies from 
//2.400GHz to 2.525GHz. The resolution of the RF channel frequency setting is 1MHz. 
//At 2Mbps the channel occupies a bandwidth wider than the resolution of the RF channel frequency setting. 
//To ensure non-overlapping channels in 2Mbps mode, the channel spacing must be 2MHz or more. At 
//1Mbps the channel bandwidth is the same as the resolution of the RF frequency setting.
//The RF channel frequency is set by the RF_CH register according to the following formula:
//F0 = 2400 + RF_CH [MHz]
#define NRF_REG_RF_CH      0x05
////////////////////////////////////////////
//RF Setup Register
#define NRF_REG_RF_SETUP   0x06
////////////////////////////////////////////
//Status Register 
//In parallel to the SPI command word applied on the MOSI pin, 
//the STATUS register is shifted serially out on the MISO pin.
#define NRF_REG_STATUS     0x07
//Data Ready RX FIFO interrupt. Asserted when new data arrives RX FIFO.
//Write 1 to clear bit
#define NRF_BIT_RX_DR       6
//Data Sent TX FIFO interrupt. Asserted when packet transmitted on TX. 
//If AUTO_ACK is activated, this bit is set high only when ACK is received.
//Write 1 to clear bit
#define NRF_BIT_TX_DS       5
//Maximum number of TX retransmits interrupt
//Write 1 to clear bit. 
//If MAX_RT is asserted it must be cleared to enable further communication. 
#define NRF_BIT_MAX_RT      4
// Data pipe number for the payload available for reading from RX_FIFO
//000-101: Data Pipe Number
//110: Not Used
//111: RX FIFO Empty
#define NRF_BIT_RX_P_NO_2   3
#define NRF_BIT_RX_P_NO_1   2
#define NRF_BIT_RX_P_NO_0   1
//TX FIFO full flag. (bit read only)
//1: TX FIFO full. 
//0: Available locations in TX FIFO
#define NRF_BIT_TX_FULL     0
////////////////////////////////////////////
//Transmit observe register
#define NRF_REG_OBSERVE_TX  0x08
////////////////////////////////////////////
#define NRF_REG_RPD         0x09
////////////////////////////////////////////
//Receive address data pipe 0. 
//5 Bytes maximum length. (LSByte is written first. Write the number of bytes defined by SETUP_AW)
//default value: 0xE7E7E7E7E7
#define NRF_REG_RX_ADDR_P0  0x0A
////////////////////////////////////////////
//Receive address data pipe 1. 
//5 Bytes maximum length. (LSByte is written first. Write the number of bytes defined by SETUP_AW)
//default value: 0xC2C2C2C2C2
#define NRF_REG_RX_ADDR_P1  0x0B
////////////////////////////////////////////
//Receive address data pipe 2. 
//Only LSB. MSBytes are equal to RX_ADDR_P1 39:8
//default value: 0xC3
#define NRF_REG_RX_ADDR_P2  0x0C
////////////////////////////////////////////
//Receive address data pipe 3. 
//Only LSB. MSBytes are equal to RX_ADDR_P1 39:8
//default value: 0xC4
#define NRF_REG_RX_ADDR_P3  0x0D
////////////////////////////////////////////
//Receive address data pipe 4. 
//Only LSB. MSBytes are equal to RX_ADDR_P1 39:8
//default value: 0xC5
#define NRF_REG_RX_ADDR_P4  0x0E
////////////////////////////////////////////
//Receive address data pipe 5. 
//Only LSB. MSBytes are equal to RX_ADDR_P1 39:8
//default value: 0xC6
#define NRF_REG_RX_ADDR_P5  0x0F
////////////////////////////////////////////
//Transmit address. Used for a PTX operation only. (LSByte is written first)
//Set RX_ADDR_P0 equal to this address to handle automatic acknowledge 
//if this is a PTX operation with Enhanced ShockBurst™ enabled.
//default value: 0xE7E7E7E7E7
#define NRF_REG_TX_ADDR     0x10
////////////////////////////////////////////
//Number of bytes in RX payload in data pipe 0 (1 to 32 bytes).
//0 Pipe not used
//1 = 1 byte
//…
//32 = 32 bytes
#define NRF_REG_RX_PW_P0    0x11
////////////////////////////////////////////
//Number of bytes in RX payload in data pipe 1 (1 to 32 bytes)
#define NRF_REG_RX_PW_P1    0x12
////////////////////////////////////////////
//Number of bytes in RX payload in data pipe 2 (1 to 32 bytes)
#define NRF_REG_RX_PW_P2    0x13
////////////////////////////////////////////
//Number of bytes in RX payload in data pipe 3 (1 to 32 bytes)
#define NRF_REG_RX_PW_P3    0x14
////////////////////////////////////////////
//Number of bytes in RX payload in data pipe 4 (1 to 32 bytes)
#define NRF_REG_RX_PW_P4    0x15
////////////////////////////////////////////
//Number of bytes in RX payload in data pipe 5 (1 to 32 bytes)
#define NRF_REG_RX_PW_P5    0x16
////////////////////////////////////////////
//FIFO Status Register
#define NRF_REG_FIFO_STATUS 0x17
//Used for a PTX operation (bit read only)
//Pulse the rfce high for at least 10µs to Reuse last transmitted payload. 
//TX payload reuse is active until W_TX_PAYLOAD or FLUSH TX is executed.
//TX_REUSE is set by the SPI command REUSE_TX_PL, 
//and is reset by the SPI commands W_TX_PAYLOAD or FLUSH TX
#define NRF_BIT_FIFO_TX_REUSE  6
//TX FIFO full flag. (bit read only) 
//1: TX FIFO full. 0: Available locations in TX FIFO.
#define NRF_BIT_FIFO_TX_FULL   5
//TX FIFO empty flag.
//1: TX FIFO empty. 0: Data in TX FIFO.
#define NRF_BIT_FIFO_TX_EMPTY  4
//RX FIFO full flag.
//1: RX FIFO full. 0: Available locations in RX FIFO
#define NRF_BIT_FIFO_RX_FULL   1
//RX FIFO empty flag.
//1: RX FIFO empty. 0: Data in RX FIFO.
#define NRF_BIT_FIFO_RX_EMPTY  0
////////////////////////////////////////////
//Enable dynamic payload length
#define NRF_REG_DYNPD     0x1C
//Enable dynamic payload length data pipe 5. (Requires EN_DPL and ENAA_P5)
#define NRF_BIT_DPL_P5      5
//Enable dynamic payload length data pipe 4. (Requires EN_DPL and ENAA_P4)
#define NRF_BIT_DPL_P4      4
//Enable dynamic payload length data pipe 3. (Requires EN_DPL and ENAA_P3)
#define NRF_BIT_DPL_P3      3
//Enable dynamic payload length data pipe 2. (Requires EN_DPL and ENAA_P2)
#define NRF_BIT_DPL_P2      2
//Enable dynamic payload length data pipe 1. (Requires EN_DPL and ENAA_P1)
#define NRF_BIT_DPL_P1      1
//Enable dynamic payload length data pipe 0. (Requires EN_DPL and ENAA_P0)
#define NRF_BIT_DPL_P0      0
////////////////////////////////////////////
//Feature Register
#define NRF_REG_FEATURE     0x1D	
//Enables Dynamic Payload Length
#define NRF_BIT_EN_DPL      2
//Enables Payload with ACK
#define NRF_BIT_EN_ACK_PAY  1
//Enables the W_TX_PAYLOAD_NOACK command
#define NRF_BIT_EN_DYN_ACK  0
////////////////////////////////////////////
////   define reg values for nrf24l01+  ////
////////////////////////////////////////////
#define NRF_VAL_AW_3Bytes     0x01 //RX/TX Address field width: 3 bytes 
#define NRF_VAL_AW_4Bytes     0x02 //RX/TX Address field width: 4 bytes 
#define NRF_VAL_AW_5Bytes     0x03 //RX/TX Address field width: 5 bytes 

#define NRF_VAL_ARD_250uS     0x00 //Auto Retransmit Delay: Wait 250µS
#define NRF_VAL_ARD_500uS     0x10 
#define NRF_VAL_ARD_750uS     0x20 
#define NRF_VAL_ARD_1000uS    0x30 
#define NRF_VAL_ARD_1250uS    0x40 
#define NRF_VAL_ARD_1500uS    0x50 
#define NRF_VAL_ARD_1750uS    0x60 
#define NRF_VAL_ARD_2000uS    0x70 
#define NRF_VAL_ARD_2250uS    0x80 
#define NRF_VAL_ARD_2500uS    0x90 
#define NRF_VAL_ARD_2750uS    0xA0 
#define NRF_VAL_ARD_3000uS    0xB0 
#define NRF_VAL_ARD_3250uS    0xC0 
#define NRF_VAL_ARD_3500uS    0xD0 
#define NRF_VAL_ARD_3750uS    0xE0 
#define NRF_VAL_ARD_4000uS    0xF0 
//delay_us>=250
#define NRF_VAL_ARD_MAKE(delay_us) (((delay_us)/250-1)<<4)

#define NRF_VAL_ARC_none      0x00 //Auto Retransmit Count: Re-Transmit disabled
#define NRF_VAL_ARC_1time     0x01 //Up to 1 Re-Transmit on fail of AA
#define NRF_VAL_ARC_2time     0x02 //Up to 2 Re-Transmit on fail of AA
#define NRF_VAL_ARC_3time     0x03
#define NRF_VAL_ARC_4time     0x04
#define NRF_VAL_ARC_5time     0x05
#define NRF_VAL_ARC_6time     0x06
#define NRF_VAL_ARC_7time     0x07
#define NRF_VAL_ARC_8time     0x08
#define NRF_VAL_ARC_9time     0x09
#define NRF_VAL_ARC_10time    0x0A
#define NRF_VAL_ARC_11time    0x0B
#define NRF_VAL_ARC_12time    0x0C
#define NRF_VAL_ARC_13time    0x0D
#define NRF_VAL_ARC_14time    0x0E
#define NRF_VAL_ARC_15time    0x0F
#define NRF_VAL_ARC_MAKE(try_time) ((try_time)&0x0F)

#define NRF_VAL_RF_CONT_WAVE_ON 0x80 //Enables continuous carrier transmit when high
//RF_SETUP bit:  7    6     5     4     3    2    1    0
//                       dr-low       dr-hi              
//               0    0     1     0     0    0    0    0  : 250k : 0x20
//               0    0     0     0     0    0    0    0  : 1M   : 0x00 
//               0    0     0     0     1    0    0    0  : 2M   : 0x08 
#define NRF_VAL_RF_DR_250kbps   0x20 //RF air-data-rate : 250kbps
#define NRF_VAL_RF_DR_1Mbps     0x00 //RF air-data-rate : 1Mbps
#define NRF_VAL_RF_DR_2Mbps     0x08 //RF air-data-rate : 2Mbps
//RF_SETUP bit:  7    6     5     4     3    2    1    0
//                                          PWR1 PWR0
//                                           0    0    0  : -18dBm : 0x00
//                                           0    1    0  : -12dBm : 0x02
//                                           1    0    0  : -6dBm  : 0x04
//                                           1    1    0  : 0dBm   : 0x06
#define NRF_VAL_RF_PWR_N18dBm   0x00 //RF output power in TX mode : -18dBm
#define NRF_VAL_RF_PWR_N12dBm   0x02 //RF output power in TX mode : -12dBm
#define NRF_VAL_RF_PWR_N6dBm    0x04 //RF output power in TX mode : -6dBm
#define NRF_VAL_RF_PWR_0dBm     0x06 //RF output power in TX mode : 0dBm
//pipe number
#define NRF_PIPE0  0 //pipe number for pipe0
#define NRF_PIPE1  1 //pipe number for pipe1
#define NRF_PIPE2  2 //pipe number for pipe2
#define NRF_PIPE3  3 //pipe number for pipe3
#define NRF_PIPE4  4 //pipe number for pipe4
#define NRF_PIPE5  5 //pipe number for pipe5
//access to status value
#define NRF_STATUS_HIT_BIT(status,bit) (((status)&(1<<bit))>0)
#define NRF_STATUS_RXDR(status)    NRF_STATUS_HIT_BIT(status,NRF_BIT_RX_DR)
#define NRF_STATUS_TXDS(status)    NRF_STATUS_HIT_BIT(status,NRF_BIT_TX_DS)
#define NRF_STATUS_MAXRT(status)   NRF_STATUS_HIT_BIT(status,NRF_BIT_MAX_RT)
#define NRF_STATUS_TXFULL(status)  NRF_STATUS_HIT_BIT(status,NRF_BIT_TX_FULL)
#define NRF_STATUS_PIPE(status)    (((status)&0x0E)>>1)
//access to transmit-observe value
//Count retransmitted packets.
#define NRF_OBSERVE_ARC(observe_val)  ((observe_val)&0x0F)
//Count lost packets. The counter is overflow protected to 15, and discontinues at max until reset.
#define NRF_OBSERVE_PLOS(observe_val) (((observe_val)&0xF0)>>4)
//access to FIFO status value
#define NRF_FIFO_STATUS_TXREUSE(fifo_status) NRF_STATUS_HIT_BIT(fifo_status,NRF_BIT_FIFO_TX_REUSE)
#define NRF_FIFO_STATUS_TXFULL(fifo_status)  NRF_STATUS_HIT_BIT(fifo_status,NRF_BIT_FIFO_TX_FULL)
#define NRF_FIFO_STATUS_TXEMPTY(fifo_status) NRF_STATUS_HIT_BIT(fifo_status,NRF_BIT_FIFO_TX_EMPTY)
#define NRF_FIFO_STATUS_RXFULL(fifo_status)  NRF_STATUS_HIT_BIT(fifo_status,NRF_BIT_FIFO_RX_FULL)
#define NRF_FIFO_STATUS_RXEMPTY(fifo_status) NRF_STATUS_HIT_BIT(fifo_status,NRF_BIT_FIFO_RX_EMPTY)

////////////////////////////////////////////
//// command byte define for nrf24l01+  ////
////////////////////////////////////////////
//Read command and status registers. 
//reg_adr = 5 bit Register Map Address
#define NRF_CMD_R_REG(reg_adr)  ((reg_adr)&0x1F)
//Write command and status registers. 
//reg_adr = 5 bit Register Map Address
//Executable in power down or standby modes only
#define NRF_CMD_W_REG(reg_adr) (((reg_adr)&0x1F)+0x20)
//Read RX-payload: 1 – 32 bytes. 
//A read operation always starts at byte 0. Payload is deleted from FIFO after it is read. 
//Used in RX mode. 
#define NRF_CMD_R_RX_PAYLOAD   (0x61)
//Write TX-payload: 1 – 32 bytes. 
//A write operation always starts at byte 0 used in TX payload
#define NRF_CMD_W_TX_PAYLOAD   (0xA0)
//Flush TX FIFO, used in TX mode
#define NRF_CMD_FLUSH_TX       (0xE1)
//Flush RX FIFO, used in RX mode
//Should not be executed during transmission of acknowledge, 
//that is, acknowledge package will not be completed
#define NRF_CMD_FLUSH_RX       (0xE2)
//Used for a PTX device, Reuse last transmitted payload. 
//TX payload reuse is active until W_TX_PAYLOAD or FLUSH TX is executed. 
//TX payload reuse must not be activated or deactivated during package transmission
#define NRF_CMD_REUSE_TX_PL    (0xE3)
//Read RX-payload width for the top R_RX_PAYLOAD in the RX FIFO
#define NRF_CMD_R_RX_PL_WID    (0x60)
//Used in RX mode.
//Write Payload to be transmitted together with ACK packet on pipe. 
//(pipe valid in the range: NRF_PIPE0 ~ NRF_PIPE5 ). 
//Maximum three ACK packet payloads can be pending. 
//Payloads with same pipe are handled using FIFO principle. 
//Write payload: 1– 32 bytes. A write operation always starts at byte 0
#define NRF_CMD_W_ACK_PAYLOAD(pipe) (((pipe)&0x07)+0xA8)
//No Operation. Might be used to read the STATUS register
#define NRF_CMD_NOP             (0xFF)

#ifdef __cplusplus
}
#endif

#endif 
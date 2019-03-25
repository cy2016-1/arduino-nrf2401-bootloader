
#ifndef __NRF24_DEVICE_H__
#define __NRF24_DEVICE_H__

#ifdef __cplusplus
extern "C"{
#endif
#include <inttypes.h>
#include "nrf24_port.h"
#include "nrf24_register.h"
//
#define NRF_SPI_INIT()   nrf_port_spi_init()	
#define NRF_SPI_RW(w)    nrf_port_spi_rw(w)
#define NRF_CSN_HIGH()   nrf_port_CSN_high()
#define NRF_CSN_LOW()    nrf_port_CSN_low()
#define NRF_CE_HIGH()    nrf_port_CE_high()
#define NRF_CE_LOW()     nrf_port_CE_low()
#define NRF_DELAY_uS(us) nrf_port_delay_us(us)
#define NRF_DELAY_mS(ms) nrf_port_delay_ms(ms)
#define NRF_IRQ_PIN()    nrf_port_IRQ_level()
//
extern char nrf_reg_read_buf(char regadr, char* buf, char rdlen);
extern char nrf_reg_read_byte(char regadr);
extern char nrf_reg_write_buf(char regadr, char* buf, char wrlen);
extern char nrf_reg_write_byte(char regadr, char val);

extern char nrf_rx_flush(void);
extern char nrf_rx_payload_wid(void);
extern char nrf_rx_read_payload(char* buf, char rdlen);

extern char nrf_tx_flush(void);
extern char nrf_tx_reuse_payload(void);
extern char nrf_tx_write_payload(char* buf, char wrlen);

extern char nrf_ack_write_payload(char pipe, char* buf, char wrlen);

extern char nrf_status(void);

extern void nrf_power_up(void);
extern void nrf_power_down(void);
extern void nrf_set_primary_mode(char mode);
extern void nrf_set_addr_wid(char wid);
extern void nrf_set_pipe_addr(char pipe, char* addr, char len);
extern void nrf_set_tx_addr(char* addr, char len);
extern void nrf_pipe_enable(char pipe);
extern char nrf_fifo_status(void);
extern void nrf_clear_irq_src(char src_bits);
extern void nrf_set_rf_channel(uint16_t freq);
extern void nrf_set_rf_data_rate(char rate);
extern void nrf_set_auto_retrans(char delay, char retry);


#ifdef __cplusplus
}
#endif

#endif 
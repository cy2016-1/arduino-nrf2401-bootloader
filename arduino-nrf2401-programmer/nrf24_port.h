
#ifndef __NRF24_PORT_H__
#define __NRF24_PORT_H__

#ifdef __cplusplus
extern "C"{
#endif /*__cplusplus*/

extern void nrf_port_delay_us(unsigned char dly_us);
extern void nrf_port_delay_ms(unsigned char dly_ms);
extern void nrf_port_CE_high(void);
extern void nrf_port_CE_low(void);
extern void nrf_port_CSN_high(void);
extern void nrf_port_CSN_low(void);
extern char nrf_port_spi_rw(char in);
extern char nrf_port_IRQ_level(void);
extern void nrf_port_spi_init(void);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /*__NRF24_PORT_H__*/
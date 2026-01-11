#ifndef I2C_H
#define I2C_H
#include <stdint.h>
#include "app_util_platform.h"
#include "app_error.h"

extern volatile bool m_xfer_done;
extern uint8_t m_sample;
extern volatile ret_code_t nrf_result;
void twi_init (void);
ret_code_t write_reg(uint8_t reg, uint8_t val);
ret_code_t read_reg(uint8_t reg, uint8_t *val , uint8_t size);

#endif
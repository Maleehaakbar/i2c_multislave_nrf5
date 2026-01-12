#ifndef I2C_H
#define I2C_H
#include <stdint.h>
#include "app_util_platform.h"
#include "app_error.h"

#define TEST_QMC58833L 0
#define TEST_MPU6050 1

#define QMC5883L_I2C_ADDR_DEF  0x0dU
#define MPU6050_SLAVE_ADDR 0x68U

#if TEST_QMC58833L
#define SLAVE_ADDRESS  QMC5883L_I2C_ADDR_DEF
#else
#define SLAVE_ADDRESS  MPU6050_SLAVE_ADDR
#endif

extern volatile bool m_xfer_done;
extern uint8_t m_sample;
extern volatile ret_code_t nrf_result;
void twi_init (void);
ret_code_t write_reg(uint8_t reg, uint8_t val);
ret_code_t read_reg(uint8_t reg, uint8_t *val , uint8_t size);

#endif
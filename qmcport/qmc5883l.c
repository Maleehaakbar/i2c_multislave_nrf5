/*
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file qmc5883l.c
 *
 * ESP-IDF Driver for 3-axis magnetic sensor QMC5883L
 *
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include "qmc5883l.h"
#include "nrf_drv_twi.h"
#include "i2c.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define I2C_FREQ_HZ 100000 // 100kHz

#define REG_XOUT_L 0x00
#define REG_XOUT_H 0x01
#define REG_YOUT_L 0x02
#define REG_YOUT_H 0x03
#define REG_ZOUT_L 0x04
#define REG_ZOUT_H 0x05
#define REG_STATE  0x06
#define REG_TOUT_L 0x07
#define REG_TOUT_H 0x08
#define REG_CTRL1  0x09
#define REG_CTRL2  0x0a
#define REG_FBR    0x0b
#define REG_ID     0x0d

#define MASK_MODE  0xfe
#define MASK_ODR   0xf3


qmc5883l_range_t range;


#define CHECK_ARG(VAL) do { if (!(VAL)) return NRF_ERROR_INVALID_PARAM; } while (0)


///////////////////////////////////////////////////////////////////////////////

ret_code_t qmc5883l_reset()    //dont use APP_ERROR_CHCECK in sensor driver, use it in application layer
{
    ret_code_t ret;
    ret = write_reg(REG_CTRL2, 0x80);
    range = QMC5883L_RNG_2;

    return ret;
}

ret_code_t qmc5883l_get_chip_id(uint8_t *id)
{
    return read_reg(REG_ID, id,1);   //sensor have 8-bit registers, so size = 1
}

ret_code_t qmc5883l_set_mode(qmc5883l_mode_t mode)
{
    CHECK_ARG(mode <= QMC5883L_MODE_CONTINUOUS);
    ret_code_t ret;
    uint8_t v;
    
    ret = read_reg(REG_CTRL1, &v,1);
    if(ret!= NRF_SUCCESS)
    {
        return ret;
    }
   
    ret = write_reg(REG_CTRL1, (v & 0xfe) | mode);

    return ret;
}

ret_code_t qmc5883l_get_mode(qmc5883l_mode_t *mode)
{
    CHECK_ARG(mode);
    ret_code_t ret;
    uint8_t v;
    ret = read_reg(REG_CTRL1, &v,1);
    *mode = v & 1;

    return ret;
}

ret_code_t qmc5883l_set_config(qmc5883l_odr_t odr, qmc5883l_osr_t osr, qmc5883l_range_t rng)
{
    CHECK_ARG(odr <= QMC5883L_DR_200 && osr <= QMC5883L_OSR_512 && rng <= QMC5883L_RNG_8);

    uint8_t v;
    ret_code_t ret;
    read_reg(REG_CTRL1, &v,1); //read
    range = rng;
    ret = write_reg(REG_FBR, 1); // Define set/reset period
     if(ret!= NRF_SUCCESS)
    {
        return ret;
    }

    /*configure all parameters except mode ,mode bits 0-1*/  
    ret = write_reg(REG_CTRL1,(v & 0x03) | ((odr & 3) << 2) | ((rng & 1) << 4) | ((osr & 3) << 6)); 

    return ret;
}

ret_code_t qmc5883l_get_config(qmc5883l_odr_t *odr, qmc5883l_osr_t *osr, qmc5883l_range_t *rng)
{
    CHECK_ARG(odr && osr && rng);
    ret_code_t ret;
    uint8_t v;
    ret = read_reg(REG_CTRL1, &v, 1);
    *odr = (v >> 2) & 3;
    *osr = (v >> 6) & 3;
    *rng = (v >> 4) & 1;

    return ret;
}

ret_code_t qmc5883l_set_int(bool enable)
{
    return write_reg(REG_CTRL2, enable ? 1 : 0);
}

ret_code_t qmc5883l_get_int(bool *enable)
{
    CHECK_ARG(enable);
    ret_code_t ret;
    uint8_t v;
    ret = read_reg(REG_CTRL2, &v, 1);
    *enable = v & 1;

    return ret;
}

ret_code_t qmc5883l_data_ready(bool *ready)
{
    CHECK_ARG(ready);
    ret_code_t ret;
    uint8_t v;
    read_reg(REG_STATE, &v,1);
    *ready = v & 1;

    return ret;
}

ret_code_t qmc5883l_get_raw_data(qmc5883l_raw_data_t *raw)
{
    CHECK_ARG(raw);
    ret_code_t ret;
    ret = read_reg(REG_XOUT_L, (uint8_t*)raw, 6);
    if (ret != NRF_SUCCESS)
       NRF_LOG_INFO("Could not read data register, err = %d", ret);
    return ret;
}

ret_code_t qmc5883l_raw_to_mg(qmc5883l_raw_data_t *raw, qmc5883l_data_t *data)
{
    CHECK_ARG(raw && data);
    ret_code_t ret;
   
    /* divide by int16 max value to compute full scale factor*/
    float f = (range == QMC5883L_RNG_2 ? 2000.0 : 8000.0) / 32768;  

    /* raw data*(sensitivity/range */
    data->x = raw->x * f; 
    data->y = raw->y * f;
    data->z = raw->z * f;

    return NRF_SUCCESS;
}

ret_code_t qmc5883l_get_data( qmc5883l_data_t *data)
{
    qmc5883l_raw_data_t raw;
    ret_code_t ret;
    ret = qmc5883l_get_raw_data(&raw);
    return qmc5883l_raw_to_mg(&raw, data);
}

ret_code_t qmc5883l_get_raw_temp(int16_t *temp)
{
    CHECK_ARG(temp);
    
    ret_code_t ret = read_reg(REG_TOUT_L, (uint8_t*)temp, 2);
    if (ret != NRF_SUCCESS)
         NRF_LOG_INFO("Could not read TOUT register, err = %d", ret);
    return ret;
}

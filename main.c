/**
 * Copyright (c) 2015 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup tw_sensor_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

#define CHIPID_REGISTER    0x0dU

/**
 * Default I2C address
 */
#define QMC5883L_I2C_ADDR_DEF  (0x0dU)


/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/*to check failure or success in interupt handler*/
static volatile ret_code_t nrf_result = NRF_SUCCESS;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from sensor. */
static uint8_t m_sample;

/*function to write to sensor*/
void write_to_sensor(uint8_t reg)
{   m_xfer_done = false;
    ret_code_t err_code;
    err_code = nrf_drv_twi_tx(&m_twi,QMC5883L_I2C_ADDR_DEF,&reg, sizeof(reg), true);  //no stop generate,repeated start
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
}
/**
 * @brief Function for handling data from sensor.
 *
 * @param[in] temp          Temperature in Celsius degrees read from sensor.
 */
__STATIC_INLINE void data_handler(uint8_t temp)
{
    NRF_LOG_INFO("bytes received %x .", temp);
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{   
   
    switch (p_event->type)
    {  
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                data_handler(m_sample);
            }
            nrf_result = NRF_SUCCESS;
            m_xfer_done = true;
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
          {
            nrf_result = NRF_ERROR_NOT_FOUND;
            m_xfer_done = true;
          }
          break;
          case NRF_DRV_TWI_EVT_DATA_NACK: {
            
            nrf_result = NRF_ERROR_INTERNAL;
            m_xfer_done = true;
          }
          break;

        default:
            break;
    }
}

/**
 * @brief i2c initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);

}

/**
 * @brief Function for reading data from sensor.
 */
static void read_sensor_data()
{   
    write_to_sensor(CHIPID_REGISTER);
    m_xfer_done = false;
    
    /* Read 1 byte from the specified address - skip 3 bits dedicated for fractional part of temperature. */
    ret_code_t err_code = nrf_drv_twi_rx(&m_twi, QMC5883L_I2C_ADDR_DEF, &m_sample, sizeof(m_sample));
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\r\nTWI sensor example started.");
    NRF_LOG_FLUSH();
    twi_init();  //i2c init
    NRF_LOG_INFO("\r\n i2c init.");
    NRF_LOG_FLUSH();      //flush logs before going to sleep
   // write_to_sensor(CHIPID_REGISTER);

    while (true)
    {
        nrf_delay_ms(500);
        read_sensor_data();
        while (m_xfer_done == false){
          __WFE();
        }
        if (nrf_result!= NRF_SUCCESS)
        {
          NRF_LOG_INFO("\r\n failed to recv data.");
          NRF_LOG_FLUSH();
          return 1;
        }
        
        NRF_LOG_FLUSH();
    }
}

/** @} */


//debugging, add other events in event handler, set done flag properly, log properly , (running i2c scanner give 0xd address)
// data received , gives 0, 3 ,FF, should return just 0xff
//as i made connection on breadboard , it stops working(may be due to absence of external pull ups)..direct connection without breadboard works

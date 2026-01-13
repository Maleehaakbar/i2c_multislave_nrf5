#include <stdio.h>
#include "boards.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "i2c.h"

#define TWI_INSTANCE_ID     0

/* Indicates if operation on TWI has ended. */
volatile bool m_xfer_done = false;

/*to check failure or success in interupt handler*/
 volatile ret_code_t nrf_result = NRF_SUCCESS;

/* TWI instance. */
 const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from sensor. */
 uint8_t m_sample;

 uint8_t current_address;


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
               // data_handler(m_sample);
                nrf_result = NRF_SUCCESS;
            }
           
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

ret_code_t write_reg(uint8_t reg, uint8_t val)
{   
    ret_code_t err_code;
    uint8_t value[2] ={reg, val};
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi,current_address ,value, sizeof(value), false); 
    while (m_xfer_done == false);
    return err_code;
    
}

ret_code_t read_reg(uint8_t reg, uint8_t *val , uint8_t size)
{   
    ret_code_t err_code;
    /*set the flag t false , as interrupt handler make it true when transfer completes.*/
    m_xfer_done = false;

    /*init write the register address from where data will be read with repeated start*/
    err_code = nrf_drv_twi_tx(&m_twi,current_address ,&reg, sizeof(reg), true); 
    APP_ERROR_CHECK(err_code);

    /*wait till the write transaction completes*/
    while (m_xfer_done == false);
  
    /*again set flag to false for receive data*/
    m_xfer_done = false;

    /*init read*/
    err_code = nrf_drv_twi_rx(&m_twi, current_address , val, size);
    APP_ERROR_CHECK(err_code);

    /*wait until transfer is done*/
    while (m_xfer_done == false);
    return err_code;
}

void i2c_address(uint8_t addr)
{ 
   current_address = addr;
}

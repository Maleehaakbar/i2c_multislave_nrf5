
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_drv_twi.h"
#include "mpu_6050.h"
#include "i2c.h"


/*
  A Function to verify the product id
  (its a basic test to check if we are communicating with the right slave, every type of I2C Device has 
  a special WHO_AM_I register which holds a specific value, we can read it from the MPU6050 or any device
  to confirm we are communicating with the right device)
*/ 
ret_code_t mpu6050_verify_product_id(uint8_t* who_am_i)
{



    // Note: All the register addresses including WHO_AM_I are declared in 
    // MPU6050.h file, you can check these addresses and values from the
    // datasheet of your slave device.
    return read_reg(ADDRESS_WHO_AM_I, who_am_i, 1);
    
}


/*
  Function to initialize the mpu6050
*/ 
void mpu6050_init(void)
{   

  // Set the registers with the required values, see the datasheet to get a good idea of these values
  write_reg(MPU_PWR_MGMT1_REG , 0x00); 
  write_reg(MPU_SAMPLE_RATE_REG , 0x07); 
  write_reg(MPU_CFG_REG , 0x06); 						
  write_reg(MPU_INT_EN_REG, 0x00); 
  write_reg(MPU_GYRO_CFG_REG , 0x18); 
  write_reg(MPU_ACCEL_CFG_REG,0x00);   		
}



/*
  Read the Gyro values from the MPU6050's internal Registers
*/ 
ret_code_t MPU6050_ReadGyro(int16_t *pGYRO_X , int16_t *pGYRO_Y , int16_t *pGYRO_Z )
{
  uint8_t buf[6]; 
  ret_code_t ret;
	
  ret = read_reg(MPU6050_GYRO_OUT,  buf, 6);
    *pGYRO_X = (buf[0] << 8) | buf[1];
    if(*pGYRO_X & 0x8000) *pGYRO_X-=65536;
		
    *pGYRO_Y= (buf[2] << 8) | buf[3];
    if(*pGYRO_Y & 0x8000) *pGYRO_Y-=65536;
	
    *pGYRO_Z = (buf[4] << 8) | buf[5];
    if(*pGYRO_Z & 0x8000) *pGYRO_Z-=65536;

  return ret;
}	




/*
  A Function to read accelerometer's values from the internal registers of MPU6050
*/ 
ret_code_t MPU6050_ReadAcc( int16_t *pACC_X , int16_t *pACC_Y , int16_t *pACC_Z )
{
  uint8_t buf[6];	
  ret_code_t ret;
  
  ret = read_reg(MPU6050_ACC_OUT, buf, 6);
  
    read_reg(MPU6050_ACC_OUT, buf, 6);
    
    *pACC_X = (buf[0] << 8) | buf[1];
    if(*pACC_X & 0x8000) *pACC_X-=65536;

    *pACC_Y= (buf[2] << 8) | buf[3];
    if(*pACC_Y & 0x8000) *pACC_Y-=65536;

    *pACC_Z = (buf[4] << 8) | buf[5];
    if(*pACC_Z & 0x8000) *pACC_Z-=65536;
  
  return ret;
}






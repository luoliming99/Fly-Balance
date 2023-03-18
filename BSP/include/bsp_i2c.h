#ifndef __BSP_I2C_H
#define __BSP_I2C_H

#include "ch32f20x.h"

#define I2C_CLK_CMD         RCC_APB1PeriphClockCmd
#define I2C_CLK             RCC_APB1Periph_I2C1

#define I2C_GPIO_CLK_CMD    RCC_APB2PeriphClockCmd
#define I2C_GPIO_CLK        RCC_APB2Periph_GPIOB

#define I2C_x               I2C1    
#define I2C_SCL_PORT        GPIOB   
#define I2C_SCL_PIN         GPIO_Pin_8
#define I2C_SDA_PORT        GPIOB 
#define I2C_SDA_PIN         GPIO_Pin_9
#define I2C_SPEED           400000

#define I2C_7BIT_ADDR       0xD0    /* 7位地址已经左移1位后的值 */

void i2c_init(void);
int i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint32_t nbytes, uint8_t *p_data);
int i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint32_t nbytes, uint8_t *p_data);

#endif

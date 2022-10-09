#ifndef __BSP_SPI_H
#define __BSP_SPI_H

#include "ch32f20x.h"

#define SPI_CLK_CMD         RCC_APB1PeriphClockCmd
#define SPI_CLK             RCC_APB1Periph_SPI2

#define SPI_GPIO_CLK_CMD    RCC_APB2PeriphClockCmd
#define SPI_GPIO_CLK        RCC_APB2Periph_GPIOB

#define SPI_x               SPI2    
#define SPI_CSN_PORT        GPIOB   
#define SPI_CSN_PIN         GPIO_Pin_12
#define SPI_SCK_PORT        GPIOB   
#define SPI_SCK_PIN         GPIO_Pin_13
#define SPI_MISO_PORT       GPIOB   
#define SPI_MISO_PIN        GPIO_Pin_14
#define SPI_MOSI_PORT       GPIOB   
#define SPI_MOSI_PIN        GPIO_Pin_15

#define SPI_CSN_HIGH        GPIO_SetBits(SPI_CSN_PORT, SPI_CSN_PIN)
#define SPI_CSN_LOW         GPIO_ResetBits(SPI_CSN_PORT, SPI_CSN_PIN)

#define SPI_TIMEOUT_CNT     ((uint32_t)0x1000)


void spi_init(void);
int spi_swap_byte(uint8_t *byte_out, uint8_t byte_in);

int spi_write_reg(uint8_t reg, uint8_t value);
int spi_read_reg(uint8_t reg, uint8_t *value);
int spi_write(uint8_t reg, uint32_t n_bytes, uint8_t *p_data);
int spi_read(uint8_t reg, uint32_t n_bytes, uint8_t *p_data);

#endif


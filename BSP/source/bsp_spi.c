#include "bsp_spi.h"

/**
 * \brief 引脚连接
 * CSN  - PB12 
 * SCK  - PB13
 * MISO - PB14
 * MOSI - PB15
 */
static void __spi_gpio_config(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0}; 

	SPI_GPIO_CLK_CMD(SPI_GPIO_CLK, ENABLE);     /* 使能GPIO时钟 */
	
    /* GPIO配置 */
    GPIO_InitStructure.GPIO_Pin = SPI_CSN_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(SPI_CSN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = SPI_SCK_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(SPI_SCK_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = SPI_MISO_PIN;
    GPIO_Init(SPI_MISO_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = SPI_MOSI_PIN;
    GPIO_Init(SPI_MOSI_PORT, &GPIO_InitStructure);
    
    SPI_CSN_HIGH;
}

static void __spi_mode_config(void)
{
    SPI_InitTypeDef  SPI_InitStructure = {0};
    
    SPI_CLK_CMD(SPI_CLK, ENABLE);   /* 使能SPI时钟 */

    /* 
     * SPI模式配置
     */
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  /* 全双工 */
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                       /* 主机模式 */
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                   /* 8位数据帧格式 */
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;                          /* 时钟极性：空闲状态时，SCK保持为低电平 */
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;                        /* 时钟相位：数据采样从第一个时钟边沿开始 */
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                           /* 启用软件从设备管理，CSN引脚上的电平由SSI位的值决定 */
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;  /* 波特率 = Fpclk1 / 16，比特率 = 波特率 / 2 */
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                  /* 先发送MSB */
    SPI_InitStructure.SPI_CRCPolynomial = 7;                            /* 指定CRC计算时用到的多项式 */
    
    SPI_Init(SPI_x , &SPI_InitStructure);

    SPI_Cmd(SPI_x, ENABLE);
}

/******************************************************************************/
void spi_init(void)
{
    __spi_gpio_config();
    __spi_mode_config();
}

/******************************************************************************/
int spi_swap_byte(uint8_t *byte_out, uint8_t byte_in)
{
    uint32_t timeout_cnt = 0;
    
    /* 等待发送缓冲区为空，TXE事件 */
    timeout_cnt = 0;
    while(SPI_I2S_GetFlagStatus(SPI_x, SPI_I2S_FLAG_TXE) == RESET)
    {
        if (timeout_cnt++ > SPI_TIMEOUT_CNT)
        {
            return -1;
        }
    }
    
    /* 把要写入的数据写入发送缓冲区 */
    SPI_I2S_SendData(SPI_x, byte_in);
    
    /* 等待接收缓冲区非空，RXNE事件 */
    timeout_cnt = 0;
    while(SPI_I2S_GetFlagStatus(SPI_x, SPI_I2S_FLAG_RXNE) == RESET)
    {
        if (timeout_cnt++ > SPI_TIMEOUT_CNT)
        {
            return -1;
        }
    }
    
    /* 获取接收缓冲区数据 */
    *byte_out = (uint8_t)SPI_I2S_ReceiveData(SPI_x);
    return 0;
}

/******************************************************************************/
int spi_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t dumy;
    
    SPI_CSN_LOW;
    if (0 != spi_swap_byte(&dumy, reg))     /* 写寄存器地址 */
    {
        return -1;
    }
    if (0 != spi_swap_byte(&dumy, value))   /* 写寄存器值 */
    {
        return -1;
    }
    SPI_CSN_HIGH;
    return 0;
}

/******************************************************************************/
int spi_read_reg(uint8_t reg, uint8_t *value)
{
    uint8_t dumy;
    
    SPI_CSN_LOW;
    if (0 != spi_swap_byte(&dumy, reg))     /* 写寄存器地址 */
    {
        return -1;
    }
    if (0 != spi_swap_byte(value, 0xFF))    /* 读寄存器值 */
    {
        return -1;
    }
    SPI_CSN_HIGH;
    return 0;
}

/******************************************************************************/
int spi_write(uint8_t reg, uint32_t n_bytes, uint8_t *p_data)
{
    uint8_t dumy;
    
    SPI_CSN_LOW;
    if (0 != spi_swap_byte(&dumy, reg))         /* 写寄存器地址 */
    {
        return -1;
    }
    while (n_bytes)
    {
        if (0 != spi_swap_byte(&dumy, *p_data)) /* 写寄存器值 */
        {
            return -1;
        }
        p_data++;
        n_bytes--;
    }  
    SPI_CSN_HIGH;
    return 0;
}

/******************************************************************************/
int spi_read(uint8_t reg, uint32_t n_bytes, uint8_t *p_data)
{
    uint8_t dumy;
    
    SPI_CSN_LOW;
    if (0 != spi_swap_byte(&dumy, reg))         /* 写寄存器地址 */
    {
        return -1;
    }
    while (n_bytes)
    {
        if (0 != spi_swap_byte(p_data, 0xFF))   /* 读寄存器值 */
        {
            return -1;
        }
        
        p_data++;
        n_bytes--;
    }  
    SPI_CSN_HIGH;
    return 0;
}

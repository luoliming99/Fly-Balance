#include "nrf24l01.h"
#include "bsp_spi.h"
#include "bsp_uart.h"

const uint8_t tx_addr[TX_ADR_WIDTH] = {0xAA,0xBB,0xEE,0xEE,0xFF}; /* 发送地址 */
const uint8_t rx_addr[RX_ADR_WIDTH] = {0xAA,0xBB,0xEE,0xEE,0xFF}; /* 接收地址 */

static void __nrf24l01_gpio_config(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure; 

	/* 使能相关时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
    /* GPIO配置 */
    GPIO_InitStructure.GPIO_Pin = NRF24L01_CE_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(NRF24L01_CE_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = NRF24L01_IRQ_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(NRF24L01_IRQ_PORT, &GPIO_InitStructure);
}

/******************************************************************************/
int nrf24l01_init(void)
{
    uint8_t tmp[TX_ADR_WIDTH] = {0};
    
    __nrf24l01_gpio_config();
    spi_init();
    
    /* 检测NRF24L01是否通信正常 */  
    if (0 != spi_write(NRF_WRITE_REG+TX_ADDR, TX_ADR_WIDTH, (uint8_t *)tx_addr))
    {
        return -1;
    }
    if (0 != spi_read(TX_ADDR, TX_ADR_WIDTH, tmp))
    {
        return -1;
    }
    for (uint8_t i = 0; i < TX_ADR_WIDTH; i++)
    {
        if (tmp[i] != tx_addr[i])
        {
            return -2;
        }
    }

    return 0;
}

/******************************************************************************/
void nrf24l01_tx_mode(void)
{
    NRF24L01_CE_LOW;

  	spi_write(NRF_WRITE_REG+TX_ADDR, TX_ADR_WIDTH, (uint8_t *)tx_addr);     /* 设置TX节点地址 */
    spi_write(NRF_WRITE_REG+RX_ADDR_P0, TX_ADR_WIDTH, (uint8_t *)tx_addr);  /* 设置通道0接收地址 */
    spi_write_reg(NRF_WRITE_REG+EN_AA, 0x01);                               /* 使能通道0的自动应答 */
    spi_write_reg(NRF_WRITE_REG+EN_RXADDR, 0x01);                           /* 使能通道0的接收地址 */
    spi_write_reg(NRF_WRITE_REG+SETUP_RETR, 0x1A);                          /* 设置自动重发间隔时间：500us+86us，最大自动重发次数：10次 */
    spi_write_reg(NRF_WRITE_REG+RF_CH, 50);                                 /* 设置RF通道工作频率 */
    spi_write_reg(NRF_WRITE_REG+RF_SETUP, 0x0F);                            /* 设置TX发射参数：0db增益，2Mbps，低噪声增益开启 */
    spi_write_reg(NRF_WRITE_REG+CONFIG, 0x0E);                              /* 配置基本工作模式的参数：所有中断开启，CRC使能，16位CRC校验，上电，发射模式 */
    
    spi_write_reg(NRF_WRITE_REG+FEATURE, 0x06);                             /* Enables Dynamic Payload Length, Enables Payload with ACK */
    spi_write_reg(NRF_WRITE_REG+DYNPD, 0x01);                               /* Enable dynamic payload length data pipe 0 */
    
	NRF24L01_CE_HIGH;
}

/******************************************************************************/
void nrf24l01_rx_mode(void)
{
    NRF24L01_CE_LOW;
    
    spi_write(NRF_WRITE_REG+RX_ADDR_P0, TX_ADR_WIDTH, (uint8_t *)tx_addr);  /* 设置通道0接收地址 */
    spi_write_reg(NRF_WRITE_REG+EN_AA, 0x01);                               /* 使能通道0的自动应答 */
    spi_write_reg(NRF_WRITE_REG+EN_RXADDR, 0x01);                           /* 使能通道0的接收地址 */
    spi_write_reg(NRF_WRITE_REG+RF_CH, 50);                                 /* 设置RF通道工作频率 */
    spi_write_reg(NRF_WRITE_REG+RX_PW_P0, RX_PLOAD_WIDTH);                  /* 设置接收数据通道0的有效数据宽度 */
    spi_write_reg(NRF_WRITE_REG+RF_SETUP, 0x0F);                            /* 设置TX发射参数：0db增益，2Mbps，低噪声增益开启 */
    spi_write_reg(NRF_WRITE_REG+CONFIG, 0x0F);                              /* 配置基本工作模式的参数：所有中断开启，CRC使能，16位CRC校验，上电，接收模式 */
    
    spi_write_reg(NRF_WRITE_REG+FEATURE, 0x06);                             /* Enables Dynamic Payload Length, Enables Payload with ACK */
    spi_write_reg(NRF_WRITE_REG+DYNPD, 0x01);                               /* Enable dynamic payload length data pipe 0 */
    
  	NRF24L01_CE_HIGH;
}

/******************************************************************************/
int nrf24l01_tx_packet(uint8_t *p_buf, uint8_t len)
{
    uint8_t status = 0;
    
    if (len < PLOAD_WIDTH_MIN || len > PLOAD_WIDTH_MAX)
    {
        return -1;
    }
    
    NRF24L01_CE_LOW;
    spi_write(WR_TX_PLOAD, len, p_buf);
    NRF24L01_CE_HIGH;
    while (NRF24L01_IRQ_GET != 0);                      /* 等待数据发送完成 */
    spi_read_reg(STATUS, &status);
    if (status & MAX_TX)                                /* 达到最大重发次数 */
    {
        spi_write_reg(FLUSH_TX, 0xFF);                  /* 清除Tx FIFO */
        spi_write_reg(NRF_WRITE_REG+STATUS, MAX_TX);    /* 清除中断标志 */
    }
    if (status & TX_OK)
    {
        spi_write_reg(NRF_WRITE_REG+STATUS, TX_OK);     /* 清除中断标志 */
        return 0;
    }
    return status;  /* 其它错误 */
}

/******************************************************************************/
int nrf24l01_rx_packet(uint8_t *p_buf)
{
    uint8_t status = 0;
    uint8_t rx_len = 0;
    
    spi_read_reg(STATUS, &status);
    if (status & RX_OK)
    {
        spi_read(R_RX_PL_WID, 1, &rx_len);
        if (rx_len < PLOAD_WIDTH_MIN || rx_len > PLOAD_WIDTH_MAX)
        {
            return -1;
        }
        spi_read(RD_RX_PLOAD, rx_len, p_buf);
        spi_write_reg(FLUSH_RX, 0xFF);                  /* 清除Rx FIFO */
        spi_write_reg(NRF_WRITE_REG+STATUS, RX_OK);     /* 清除中断标志 */
        return 0;
    }
    return status;  /* 其它错误 */
}

/******************************************************************************/
int nrf24l01_rx_packet_ack_with_payload(uint8_t *rx_buf, uint8_t *tx_buf, uint8_t tx_len)
{
    uint8_t status = 0;
    uint8_t rx_len = 0;
    
    spi_read_reg(STATUS, &status);
    if (status & RX_OK)
    {
        spi_read(R_RX_PL_WID, 1, &rx_len);
        if (rx_len < PLOAD_WIDTH_MIN || rx_len > PLOAD_WIDTH_MAX)
        {
            return -1;
        }
        spi_read(RD_RX_PLOAD, rx_len, rx_buf);
        spi_write_reg(NRF_WRITE_REG+STATUS, RX_OK);     /* 清除中断标志 */
        /* 装载payload随ACK发送给PTX端 */
        if (tx_buf != NULL && tx_len >= PLOAD_WIDTH_MIN && tx_len <= PLOAD_WIDTH_MAX)
        {
            spi_write(W_ACK_PAYLOAD(0), tx_len, tx_buf);
        }
        return 0;
    }
    return status;  /* 其它错误 */
}

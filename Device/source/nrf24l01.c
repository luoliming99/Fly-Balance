#include "nrf24l01.h"
#include "bsp_spi.h"
#include "bsp_uart.h"

const uint8_t tx_addr[TX_ADR_WIDTH] = {0xAA,0xBB,0xCC,0x00,0x01}; /* ���͵�ַ */
const uint8_t rx_addr[RX_ADR_WIDTH] = {0xAA,0xBB,0xCC,0x00,0x01}; /* ���յ�ַ */

static void __nrf24l01_gpio_config(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure; 

	/* ʹ�����ʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
    /* GPIO���� */
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
    uint8_t i;
    
    __nrf24l01_gpio_config();
    spi_init();
    
    /* ���NRF24L01�Ƿ�ͨ������ */  
    if (0 != spi_write(NRF_WRITE_REG+TX_ADDR, TX_ADR_WIDTH, (uint8_t *)tx_addr))
    {
        return -1;
    }
    if (0 != spi_read(TX_ADDR, TX_ADR_WIDTH, tmp))
    {
        return -1;
    }
    for (i = 0; i < TX_ADR_WIDTH; i++)
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
  	spi_write(NRF_WRITE_REG+TX_ADDR, TX_ADR_WIDTH, (uint8_t *)tx_addr);     /* ����TX�ڵ��ַ */
    spi_write(NRF_WRITE_REG+RX_ADDR_P0, TX_ADR_WIDTH, (uint8_t *)tx_addr);  /* ����ͨ��0���յ�ַ */
    spi_write_reg(NRF_WRITE_REG+EN_AA, 0x01);                               /* ʹ��ͨ��0���Զ�Ӧ�� */
    spi_write_reg(NRF_WRITE_REG+EN_RXADDR, 0x01);                           /* ʹ��ͨ��0�Ľ��յ�ַ */
    spi_write_reg(NRF_WRITE_REG+SETUP_RETR, 0x1A);                          /* �����Զ��ط����ʱ�䣺500us+86us������Զ��ط�������10�� */
    spi_write_reg(NRF_WRITE_REG+RF_CH, 0);                                 /* ����RFͨ������Ƶ�� */
    spi_write_reg(NRF_WRITE_REG+RF_SETUP, 0x0F);                            /* ����TX���������0db���棬2Mbps�����������濪�� */
    spi_write_reg(NRF_WRITE_REG+CONFIG, 0x0E);                              /* ���û�������ģʽ�Ĳ����������жϿ�����CRCʹ�ܣ�16λCRCУ�飬�ϵ磬����ģʽ */
	NRF24L01_CE_HIGH;
}

/******************************************************************************/
void nrf24l01_rx_mode(void)
{
    NRF24L01_CE_LOW;
    spi_write(NRF_WRITE_REG+RX_ADDR_P0, TX_ADR_WIDTH, (uint8_t *)tx_addr);  /* ����ͨ��0���յ�ַ */
    spi_write_reg(NRF_WRITE_REG+EN_AA, 0x01);                               /* ʹ��ͨ��0���Զ�Ӧ�� */
    spi_write_reg(NRF_WRITE_REG+EN_RXADDR, 0x01);                           /* ʹ��ͨ��0�Ľ��յ�ַ */
    spi_write_reg(NRF_WRITE_REG+RF_CH, 0);                                 /* ����RFͨ������Ƶ�� */
    spi_write_reg(NRF_WRITE_REG+RX_PW_P0, RX_PLOAD_WIDTH);                  /* ���ý�������ͨ��0����Ч���ݿ�� */
    spi_write_reg(NRF_WRITE_REG+RF_SETUP, 0x0F);                            /* ����RX���������0db���棬2Mbps�����������濪�� */
    spi_write_reg(NRF_WRITE_REG+CONFIG, 0x0F);                              /* ���û�������ģʽ�Ĳ����������жϿ�����CRCʹ�ܣ�16λCRCУ�飬�ϵ磬����ģʽ */
  	NRF24L01_CE_HIGH;
}

/******************************************************************************/
int nrf24l01_tx_packet(uint8_t *p_buf)
{
    uint8_t status = 0;
    
    NRF24L01_CE_LOW;
    spi_write(WR_TX_PLOAD, TX_PLOAD_WIDTH, p_buf);
    NRF24L01_CE_HIGH;
    while (NRF24L01_IRQ_GET != 0);                  /* �ȴ����ݷ������ */
    spi_read_reg(STATUS, &status);
    spi_write_reg(NRF_WRITE_REG+STATUS, status);    /* ���״̬�Ĵ������жϱ�־ */
    if (status & MAX_TX)                            /* �ﵽ����ط����� */
    {
        spi_write_reg(FLUSH_TX, 0xFF);              /* ���Tx FIFO */
        return MAX_TX;
    }
    else if (status & TX_OK)
    {
        return 0;
    }
    return status;  /* �������� */
}

/******************************************************************************/
int nrf24l01_rx_packet(uint8_t *p_buf)
{
    uint8_t status = 0;
    
    spi_read_reg(STATUS, &status);
    spi_write_reg(NRF_WRITE_REG+STATUS, status);    /* ���״̬�Ĵ������жϱ�־ */
    if (status & RX_OK)
    {
        spi_read(RD_RX_PLOAD, RX_PLOAD_WIDTH, p_buf);
        spi_write_reg(FLUSH_RX, 0xFF);              /* ���Rx FIFO */
        return 0;
    }
    return status;  /* �������� */
}

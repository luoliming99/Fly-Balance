#include "bsp_uart.h"

/* Support Printf Function Definition */
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;

/* 重定向c库函数printf到串口，重定向后可使用printf函数 */
int fputc(int ch, FILE *f)
{
    /* 发送一个字节数据到串口 */
    USART_SendData(DEBUG_USART, (uint8_t) ch);

    /* 等待发送完毕 */
    while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);		

    return (ch);
}

/* 重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数 */
int fgetc(FILE *f)
{
    /* 等待串口输入数据 */
    while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_RXNE) == RESET);

    return (int)USART_ReceiveData(DEBUG_USART);
}

static void __nvic_config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 配置USART为中断源 */
    NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
    /* 抢断优先级*/
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    /* 子优先级 */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    /* 使能中断 */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* 初始化配置NVIC */
    NVIC_Init(&NVIC_InitStructure);
}

/******************************************************************************/
void uart_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	USART_InitTypeDef USART_InitStructure = {0};

	/* 打开串口GPIO的时钟 */
	DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);
	
	/* 打开串口外设的时钟 */
	DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);

	/* 将USART Tx的GPIO配置为推挽复用模式 */
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    /* 将USART Rx的GPIO配置为浮空输入模式 */
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
	
	/* 配置串口的工作参数 */
	USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(DEBUG_USART, &USART_InitStructure);
	
	/* 串口中断优先级配置 */
	__nvic_config();
	
	/* 使能串口接收中断 */
	USART_ITConfig(DEBUG_USART, USART_IT_RXNE, ENABLE);	
	
	/* 使能串口 */
	USART_Cmd(DEBUG_USART, ENABLE);		
}

/******************************************************************************/
void uart_send_one_byte(USART_TypeDef *pUSARTx, uint8_t ch)
{
	/* 发送一个字节数据到USART */
	USART_SendData(pUSARTx, ch);
		
	/* 等待发送数据寄存器为空 */
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

/******************************************************************************/
void uart_send_string(USART_TypeDef *pUSARTx, char *str)
{
	uint32_t i = 0;
    
    while(*(str + i) != '\0') 
    {
        uart_send_one_byte(pUSARTx, *(str + i));
        i++;
    }
}

/******************************************************************************/
void uart_send_data_buf(USART_TypeDef *pUSARTx, uint8_t *pBuf, uint32_t len)
{
    uint32_t i = 0;
    
	for (i = 0; i < len; i++)
    {
        uart_send_one_byte(pUSARTx, *(pBuf + i));
    }
}



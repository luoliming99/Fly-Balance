#include "bsp_i2c.h"

static void __i2c_gpio_config(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    
    I2C_GPIO_CLK_CMD(I2C_GPIO_CLK, ENABLE);     /* 使能GPIO时钟 */

    GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(I2C_SCL_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(I2C_SDA_PORT, &GPIO_InitStructure);
}

static void __i2c_mode_config(void)
{
    I2C_InitTypeDef  I2C_InitStructure = {0};
    
    I2C_CLK_CMD(I2C_CLK, ENABLE);   /* 使能I2C时钟 */

    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = I2C_7BIT_ADDR; 
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable ;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
    
    I2C_Init(I2C_x, &I2C_InitStructure);
    
    I2C_Cmd(I2C_x, ENABLE);
}

/******************************************************************************/
void i2c_init(void)
{
    __i2c_gpio_config();
    __i2c_mode_config(); 
}

/******************************************************************************/
int i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint32_t nbytes, uint8_t *p_data)
{
    while(I2C_GetFlagStatus(I2C_x, I2C_FLAG_BUSY)); /* 等待I2C总线忙完 */
    I2C_GenerateSTART(I2C_x, ENABLE);               /* START */
    while(!I2C_CheckEvent(I2C_x, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2C_x, slave_addr << 1, I2C_Direction_Transmitter); /* SLAVE_ADDR + W */
    while(!I2C_CheckEvent(I2C_x, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    I2C_Cmd(I2C_x, ENABLE); /* Clear EV6 by setting again the PE bit */
    I2C_SendData(I2C_x, reg_addr);  /* send REG_ADDR */
    while(!I2C_CheckEvent(I2C_x, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    
    I2C_GenerateSTART(I2C_x, ENABLE);   /* RE-START */
    while(!I2C_CheckEvent(I2C_x, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2C_x, slave_addr << 1, I2C_Direction_Receiver);    /* SLAVE_ADDR + R */
    while(!I2C_CheckEvent(I2C_x, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    
    while (nbytes)
    {
        if (nbytes == 1)
        {
            I2C_AcknowledgeConfig(I2C_x, DISABLE);   /* Disable Acknowledgement */
            I2C_GenerateSTOP(I2C_x, ENABLE);         /* STOP */
        }
        
        while(!I2C_CheckEvent(I2C_x, I2C_EVENT_MASTER_BYTE_RECEIVED));
        
        *p_data = I2C_ReceiveData(I2C_x);
        p_data++;
        nbytes--;
    }
    
    /* Enable Acknowledgement to be ready for another reception */
    I2C_AcknowledgeConfig(I2C_x, ENABLE);
    
    return 0;
}

/******************************************************************************/
int i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint32_t nbytes, uint8_t *p_data)
{
    while(I2C_GetFlagStatus(I2C_x, I2C_FLAG_BUSY));  /* 等待I2C总线忙完 */
    I2C_GenerateSTART(I2C_x, ENABLE);                /* START */
    while(!I2C_CheckEvent(I2C_x, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2C_x, slave_addr << 1, I2C_Direction_Transmitter);   /* SLAVE_ADDR + W */
    while(!I2C_CheckEvent(I2C_x, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    I2C_SendData(I2C_x, reg_addr);   /* send REG_ADDR */
    while(!I2C_CheckEvent(I2C_x, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    
    while (nbytes)
    {
        I2C_SendData(I2C_x, *p_data);
        
        while(!I2C_CheckEvent(I2C_x, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
        
        p_data++;
        nbytes--;
    }
    
    I2C_GenerateSTOP(I2C_x, ENABLE);                 /* STOP */
    
    return 0;
}

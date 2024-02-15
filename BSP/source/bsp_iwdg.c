#include "bsp_iwdg.h"


extern uint8_t g_comm_wdg_reg;

/*********************************************************************
 * @fn      iwdg_feed_init
 *
 * @brief   Initializes IWDG.
 *
 * @param   IWDG_Prescaler: specifies the IWDG Prescaler value.
 *            IWDG_Prescaler_4: IWDG prescaler set to 4.
 *            IWDG_Prescaler_8: IWDG prescaler set to 8.
 *            IWDG_Prescaler_16: IWDG prescaler set to 16.
 *            IWDG_Prescaler_32: IWDG prescaler set to 32.
 *            IWDG_Prescaler_64: IWDG prescaler set to 64.
 *            IWDG_Prescaler_128: IWDG prescaler set to 128.
 *            IWDG_Prescaler_256: IWDG prescaler set to 256.
 *          Reload: specifies the IWDG Reload value.
 *            This parameter must be a number between 0 and 0x0FFF.
 *
 * @return  none
 */
void iwdg_feed_init(uint16_t prer, uint16_t rlr)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(prer);
    IWDG_SetReload(rlr);
    IWDG_ReloadCounter();
    IWDG_Enable();
}

/******************************************************************************/
void comm_wdg_enable(void)
{
    g_comm_wdg_reg |= 0x80;
}

/******************************************************************************/
void comm_wdg_disable(void)
{
    g_comm_wdg_reg = 0;
}

/******************************************************************************/
void comm_wdg_feed(void)
{
    g_comm_wdg_reg &= 0xC0;
}

/******************************************************************************/
uint8_t is_comm_wdg_interrupted(void)
{
    return (g_comm_wdg_reg & 0x40);
}

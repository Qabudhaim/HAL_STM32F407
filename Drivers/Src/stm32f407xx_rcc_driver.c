/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: Nov 22, 2021
 *      Author: obscure
 */

#include "stm32f407xx_rcc_driver.h"

uint16_t ahb_presc[] = { 2, 4, 8, 16, 64, 128, 256, 512 };
uint8_t apb_presc[] = { 2, 4, 8, 16 };

/**
 *
 * @return System clock
 */
uint32_t rcc_get_sysclk (void)
{
    uint32_t SysClk;

    if ((RCC->CFGR & (0x3 << RCC_CFGR_SW)) == RCC_SYSCLK_HSI)
    {
        SysClk = 16000000;
    }
    else if ((RCC->CFGR & (0x3 << RCC_CFGR_SW)) == RCC_SYSCLK_HSE)
    {
        SysClk = 8000000;
    }
    else if ((RCC->CFGR & (0x3 << RCC_CFGR_SW)) == RCC_SYSCLK_HSI)
    {
        SysClk = rcc_get_pll();
    }

    return SysClk;
}

/**
 *
 * @param busAddress AHB, APB1, APB2
 * @return pre-scaler for the chosen bus
 */
uint16_t rcc_get_bus_prescaler (uint8_t busAddress)
{
    uint16_t preScaler;
    uint8_t tmp;

    if (busAddress == RCC_SYSBUS_AHB)
    {
        //AHB pre-scaler
        tmp = RCC->CFGR & (0xF << RCC_CFGR_HPRE);
        if (tmp < 8)
        {
            preScaler = 1;
        }
        else
        {
            preScaler = ahb_presc[tmp - 8];
        }

    }
    else if (busAddress == RCC_SYSBUS_APB1)
    {
        //APB1 pre-scaler
        tmp = RCC->CFGR & (0x7 << RCC_CFGR_PPRE1);
        if (tmp < 4)
        {
            preScaler = 1;
        }
        else
        {
            preScaler = apb_presc[tmp - 4];
        }
    }
    else if (busAddress == RCC_SYSBUS_APB2)
    {
        //APB1 pre-scaler
        tmp = RCC->CFGR & (0x7 << RCC_CFGR_PPRE2);
        if (tmp < 4)
        {
            preScaler = 1;
        }
        else
        {
            preScaler = apb_presc[tmp - 4];
        }
    }

    return preScaler;
}

/**
 *
 * @param peripheral_bus APB1, APB2
 * @return the peripheral clock
 */
uint32_t rcc_get_pclk (uint8_t peripheral_bus)
{
    uint32_t SysClk = rcc_get_sysclk();
    uint8_t AHBPreScaler = rcc_get_bus_prescaler(RCC_SYSBUS_AHB);

    if (peripheral_bus == RCC_SYSBUS_APB1)
    {
        uint8_t APB1PreScaler = rcc_get_bus_prescaler(RCC_SYSBUS_APB1);
        return (SysClk / AHBPreScaler / APB1PreScaler);
    }
    else if (peripheral_bus == RCC_SYSBUS_APB2)
    {
        uint8_t APB2PreScaler = rcc_get_bus_prescaler(RCC_SYSBUS_APB2);
        return (SysClk / AHBPreScaler / APB2PreScaler);
    }
    else if (peripheral_bus == RCC_SYSBUS_AHB)
    {
        return (SysClk / AHBPreScaler);
    }
    else
    {
        return -1;  //! Error
    }
}

/**
 *
 * @return PLL clock
 */
uint32_t rcc_get_pll (void){
    return 0;
}

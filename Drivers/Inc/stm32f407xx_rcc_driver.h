/*
 * stm32f407xx_rcc_driver.h
 *
 *  Created on: Nov 22, 2021
 *      Author: obscure
 */

#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_

#include "stm32f407xx.h"

#define RCC_SYSCLK_HSI              0
#define RCC_SYSCLK_HSE              1
#define RCC_SYSCLK_PLL              2

#define RCC_SYSBUS_AHB              0
#define RCC_SYSBUS_APB1             1
#define RCC_SYSBUS_APB2             2

/****************************************************************************
 *                           RCC registers bits                             *
 ****************************************************************************/
/**
 * RCC CFGR bits
 */
#define RCC_CFGR_SW                 0
#define RCC_CFGR_SWS                2
#define RCC_CFGR_HPRE               4
#define RCC_CFGR_PPRE1              10
#define RCC_CFGR_PPRE2              13
#define RCC_CFGR_RTCPRE             16
#define RCC_CFGR_MCO1               21
#define RCC_CFGR_I2SSCR             23
#define RCC_CFGR_MCO1PRE            24
#define RCC_CFGR_MCO2PRE            27
#define RCC_CFGR_MCO2               30

uint32_t rcc_get_sysclk (void);
uint16_t rcc_get_bus_prescaler (uint8_t busAddress);
uint32_t rcc_get_pclk (uint8_t peripheral_bus);
uint32_t rcc_get_pll (void);

#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */

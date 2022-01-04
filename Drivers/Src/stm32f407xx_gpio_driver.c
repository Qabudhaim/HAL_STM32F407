/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Nov 14, 2021
 *      Author: obscure
 */

#include "stm32f407xx_gpio_driver.h"

/**
 *
 * @param pGPIOx GPIO Port address
 * @param ENorDI Enabling/Disabling the port's clock
 */
void gpio_pclk_control (gpio_reg_t * pGPIOx, uint8_t ENorDI)
{
    if (ENorDI == ENABLE)
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_EN();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLK_EN();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_EN();
        }
        else if (pGPIOx == GPIOI)
        {
            GPIOI_PCLK_EN();
        }
    }
    else
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DI();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DI();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_DI();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_DI();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_DI();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_DI();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLK_DI();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_DI();
        }
        else if (pGPIOx == GPIOI)
        {
            GPIOI_PCLK_DI();
        }
    }
}

/**
 *
 * @param pGPIOHandle Handle structure that contains port address
 *                    in addition to user specific configuration
 */
void gpio_init (gpio_handle_t * pGPIOHandle)
{
    uint32_t tmp = 0;
    // 1. Enable peripheral clock
    gpio_pclk_control(pGPIOHandle->pGPIOx, ENABLE);

    // 2. Configure port mode
    if (pGPIOHandle->GPIOConf.pinMode <= GPIO_MODE_ANALOG)
    {
        tmp = (pGPIOHandle->GPIOConf.pinMode
                << (pGPIOHandle->GPIOConf.pinNumber * 2));
        pGPIOHandle->pGPIOx->MODER &= ~(0x3
                << (pGPIOHandle->GPIOConf.pinNumber * 2));
        pGPIOHandle->pGPIOx->MODER |= tmp;
    }
    else
    {
        // 1. Configure rising edge
        if (pGPIOHandle->GPIOConf.pinMode == GPIO_MODE_IT_RE)
        {
            EXTI->RTSR |= (1 << pGPIOHandle->GPIOConf.pinNumber);
            EXTI->FTSR &= ~(1 << pGPIOHandle->GPIOConf.pinNumber);
        }

        // 2. Configure falling edge
        else if (pGPIOHandle->GPIOConf.pinMode == GPIO_MODE_IT_FE)
        {
            EXTI->RTSR &= ~(1 << pGPIOHandle->GPIOConf.pinNumber);
            EXTI->FTSR |= (1 << pGPIOHandle->GPIOConf.pinNumber);
        }

        // 3. Configure rising & falling edge
        else if (pGPIOHandle->GPIOConf.pinMode == GPIO_MODE_IT_RFE)
        {
            EXTI->RTSR |= (1 << pGPIOHandle->GPIOConf.pinNumber);
            EXTI->FTSR |= (1 << pGPIOHandle->GPIOConf.pinNumber);
        }

        // 2. Configure the GPIO port selection in SYSCFG_EXTICR
        uint8_t tmp1 = pGPIOHandle->GPIOConf.pinNumber / 4;
        uint8_t tmp2 = pGPIOHandle->GPIOConf.pinNumber % 4;
        SYSCFG_PCLK_EN();
        SYSCFG->EXTICR[tmp1] = (GPIO_PORT_TO_CODE(pGPIOHandle->pGPIOx)
                << (tmp2 * 4));

        // 3. Enable the exti interrupt delivery using IMR
        EXTI->IMR |= (1 << pGPIOHandle->GPIOConf.pinNumber);
    }

    // 3. Configure output type
    tmp = 0;
    tmp = (pGPIOHandle->GPIOConf.pinOPType << pGPIOHandle->GPIOConf.pinNumber);
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIOConf.pinNumber);
    pGPIOHandle->pGPIOx->OTYPER |= tmp;

    // 4. Configure output speed
    tmp = 0;
    tmp = (pGPIOHandle->GPIOConf.pinSpeed
            << (pGPIOHandle->GPIOConf.pinNumber * 2));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3
            << (pGPIOHandle->GPIOConf.pinNumber * 2));
    pGPIOHandle->pGPIOx->OSPEEDR |= tmp;

    // 5. Configure Pull-up/Pull-down resistors
    tmp = 0;
    tmp = (pGPIOHandle->GPIOConf.pinPUPD
            << (pGPIOHandle->GPIOConf.pinNumber * 2));
    pGPIOHandle->pGPIOx->PUPDR &=
            ~(0x3 << (pGPIOHandle->GPIOConf.pinNumber * 2));
    pGPIOHandle->pGPIOx->PUPDR |= tmp;

    // 6. Configure Alternate functionality
    if (pGPIOHandle->GPIOConf.pinMode == GPIO_MODE_ALTFUN)
    {
        const uint8_t afr_div = 8;  //! Each register is divided into 8 partitions
        const uint8_t afr_size = 4;  //! Each pin has 4 bits
        uint8_t tmp1 = pGPIOHandle->GPIOConf.pinNumber / afr_div;
        uint8_t tmp2 = pGPIOHandle->GPIOConf.pinNumber % afr_div;
        pGPIOHandle->pGPIOx->AFR[tmp1] &= ~(0xF << (tmp2 * afr_size));
        pGPIOHandle->pGPIOx->AFR[tmp1] |= (pGPIOHandle->GPIOConf.pinAltFun
                << (tmp2 * afr_size));
    }
}

/**
 *
 * @param pGPIOx GPIO Port address
 */
void gpio_deinit (gpio_reg_t * pGPIOx)
{
    if (pGPIOx == GPIOA)
    {
        GPIOA_PCLK_DI();
    }
    else if (pGPIOx == GPIOB)
    {
        GPIOB_PCLK_DI();
    }
    else if (pGPIOx == GPIOC)
    {
        GPIOC_PCLK_DI();
    }
    else if (pGPIOx == GPIOD)
    {
        GPIOD_PCLK_DI();
    }
    else if (pGPIOx == GPIOE)
    {
        GPIOE_PCLK_DI();
    }
    else if (pGPIOx == GPIOF)
    {
        GPIOF_PCLK_DI();
    }
    else if (pGPIOx == GPIOG)
    {
        GPIOG_PCLK_DI();
    }
    else if (pGPIOx == GPIOH)
    {
        GPIOH_PCLK_DI();
    }
    else if (pGPIOx == GPIOI)
    {
        GPIOI_PCLK_DI();
    }
}

/**
 *
 * @param pGPIOx    GPIO Port address
 * @return data     Return the read value
 */
uint16_t gpio_read_port (gpio_reg_t * pGPIOx)
{
    uint16_t data = 0;
    data = (uint16_t) pGPIOx->IDR;
    return data;
}

/**
 *
 * @param pGPIOx    GPIO Port address
 * @param pinNumber Pin to read
 * @return data     Return the read value
 */
uint8_t gpio_read_pin (gpio_reg_t * pGPIOx, uint8_t pinNumber)
{
    uint8_t data = 0;
    data = (uint8_t) ((pGPIOx->IDR >> pinNumber) & 0x1);
    return data;
}

/**
 *
 * @param pGPIOx    GPIO Port address
 * @param data      Data to be written
 */
void gpio_write_port (gpio_reg_t * pGPIOx, uint16_t data)
{
    pGPIOx->ODR = data;
}

/**
 *
 * @param pGPIOx    GPIO Port address
 * @param pinNumber Pin to write
 * @param data      Data to be written
 */
void gpio_write_pin (gpio_reg_t * pGPIOx, uint8_t pinNumber, uint8_t data)
{
    if (data == GPIO_PIN_SET)
    {
        pGPIOx->ODR |= (1 << pinNumber);
    }
    else
    {
        pGPIOx->ODR &= ~(1 << pinNumber);
    }
}

/**
 *
 * @param pGPIOx    GPIO Port address
 * @param pinNumber Pin to toggle
 */
void gpio_toggle_pin (gpio_reg_t * pGPIOx, uint8_t pinNumber)
{
    pGPIOx->ODR ^= (1 << pinNumber);
}

void gpio_irq_config (uint8_t IRQ_No, uint8_t ENorDI)
{
    uint8_t tmp1 = IRQ_No / 32;
    uint8_t tmp2 = IRQ_No % 32;
    if (ENorDI == ENABLE)
    {

        NVIC->ISER[tmp1] |= (1 << tmp2);
    }
    else
    {
        NVIC->ICER[tmp1] |= (1 << tmp2);
    }
}

void gpio_irq_priority (uint8_t IRQ_No, uint8_t IRQ_Priority)
{
    uint8_t tmp1 = IRQ_No / 4;
    uint8_t tmp2 = IRQ_No % 4;
    NVIC->IPR[tmp1] = (IRQ_Priority << ((tmp2 * 8) + 4));
}
void gpio_irq_handler (uint8_t Pin_No)
{
    if (EXTI->PR & (1 << Pin_No))
    {
        EXTI->PR |= (1 << Pin_No);
    }
}

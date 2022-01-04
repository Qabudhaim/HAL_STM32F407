/*
 * 006spi_interrupt.c
 *
 *  Created on: Nov 20, 2021
 *      Author: obscure
 */

#include <string.h>
#include "stm32f407xx.h"

#define COMMENT_OUT         0

uint8_t rcv = 0;
spi_handle_t spi;

static void mdelay (uint32_t cnt)
{
    for (uint32_t i = 0; i < (cnt * 1000); i++);
}

static void init_gpio_button ()
{
    gpio_handle_t button;
    button.pGPIOx = GPIOA;
    button.GPIOConf.pinMode = GPIO_MODE_IT_RE;
    button.GPIOConf.pinNumber = GPIO_PIN_NO_0;
    button.GPIOConf.pinOPType = GPIO_OP_TYPE_PP;
    button.GPIOConf.pinPUPD = GPIO_PIN_NO_PUPD;
    button.GPIOConf.pinSpeed = GPIO_SPEED_HIGH;

    gpio_init(&button);

    gpio_irq_config(IRQ_NO_EXTI0, ENABLE);
}

static void init_gpio_spi ()
{
    gpio_handle_t gpio_spi;
    gpio_spi.pGPIOx = GPIOB;
    gpio_spi.GPIOConf.pinMode = GPIO_MODE_ALTFUN;
    gpio_spi.GPIOConf.pinAltFun = GPIO_ALTFUN_AF5;
    gpio_spi.GPIOConf.pinOPType = GPIO_OP_TYPE_PP;
    gpio_spi.GPIOConf.pinPUPD = GPIO_PIN_NO_PUPD;
    gpio_spi.GPIOConf.pinSpeed = GPIO_SPEED_HIGH;

    // SCK
    gpio_spi.GPIOConf.pinNumber = GPIO_PIN_NO_13;
    gpio_init(&gpio_spi);

    // MISO
    gpio_spi.GPIOConf.pinNumber = GPIO_PIN_NO_14;
    gpio_init(&gpio_spi);

    // MOSI
    gpio_spi.GPIOConf.pinNumber = GPIO_PIN_NO_15;
    gpio_init(&gpio_spi);

    // NSS
    gpio_spi.GPIOConf.pinNumber = GPIO_PIN_NO_12;
    gpio_init(&gpio_spi);

}

static void init_spi ()
{
    spi.pSPIx = SPI2;
    spi.SPIConf.busConfig = SPI_BUS_FULL_DUPLEX;
    spi.SPIConf.deviceMode = SPI_MODE_MASTER;
    spi.SPIConf.spiCPHA = SPI_CPHA_LOW;
    spi.SPIConf.spiCPOL = SPI_CPOL_LOW;
    spi.SPIConf.spiDFF = SPI_DFF_8_BIT;
    spi.SPIConf.spiSSM = SPI_SSM_DI;
    spi.SPIConf.spiSpeed = SPI_SPEED_DIV_8;

    spi_init(&spi);
}

int main (void)
{

    char msg[] = "My name is Qusai Abudhaim!";

    init_gpio_button();
    init_gpio_spi();
    init_spi();
    spi_irq_config(IRQ_NO_SPI2, ENABLE);

    //spi_ssi_control(SPI2, ENABLE);

    spi_ssoe_control(SPI2, ENABLE);

    for (;;)
    {
        while (!rcv);

        gpio_irq_config(IRQ_NO_EXTI0, DISABLE);

        mdelay(100);

        spi_peripheral_control(SPI2, ENABLE);

        uint8_t len = strlen(msg);

        spi_interrupt_transmit(&spi, &len, 1);

        spi_interrupt_transmit(&spi, (uint8_t*) msg, len);

        while (spi_get_status_flag(SPI2, SPI_FLAG_BSY));

        spi_peripheral_control(SPI2, DISABLE);

        rcv = 0;

        gpio_irq_config(IRQ_NO_EXTI0, ENABLE);


    }
}

void SPI2_IRQHandler ()
{
    spi_irq_handler(&spi);
}

void EXTI0_IRQHandler ()
{
    gpio_irq_handler(GPIO_PIN_NO_0);
    rcv = 1;
}


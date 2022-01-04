/*
 * 004spi_master.c
 *
 *  Created on: Nov 17, 2021
 *      Author: obscure
 */

#include <string.h>
#include "stm32f407xx.h"

#define COMMENT_OUT         0

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
}

static void init_gpio_spi ()
{
    gpio_handle_t spi;
    spi.pGPIOx = GPIOB;
    spi.GPIOConf.pinMode = GPIO_MODE_ALTFUN;
    spi.GPIOConf.pinAltFun = GPIO_ALTFUN_AF5;
    spi.GPIOConf.pinOPType = GPIO_OP_TYPE_PP;
    spi.GPIOConf.pinPUPD = GPIO_PIN_NO_PUPD;
    spi.GPIOConf.pinSpeed = GPIO_SPEED_HIGH;

    // SCK
    spi.GPIOConf.pinNumber = GPIO_PIN_NO_13;
    gpio_init(&spi);

    // MISO
    //spi.GPIOConf.pinNumber = GPIO_PIN_NO_14;
    //gpio_init(&spi);

    // MOSI
    spi.GPIOConf.pinNumber = GPIO_PIN_NO_15;
    gpio_init(&spi);

    // NSS
    spi.GPIOConf.pinNumber = GPIO_PIN_NO_12;
    gpio_init(&spi);

}

static void init_spi ()
{
    spi_handle_t spi;
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

    //spi_ssi_control(SPI2, ENABLE);

    spi_ssoe_control(SPI2, ENABLE);

    for (;;)
    {
        while (!gpio_read_pin(GPIOA, GPIO_PIN_NO_0));

        mdelay(100);

        spi_peripheral_control(SPI2, ENABLE);

        uint8_t len = strlen(msg);

        spi_transmit(SPI2, &len, 1);

        spi_transmit(SPI2, (uint8_t*) msg, strlen(msg));

        while (spi_get_status_flag(SPI2, SPI_FLAG_BSY));

        spi_peripheral_control(SPI2, DISABLE);

    }
}

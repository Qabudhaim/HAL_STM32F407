/*
 * 005spi_cmd.c
 *
 *  Created on: Nov 18, 2021
 *      Author: obscure
 */



#include <string.h>
#include "stm32f407xx.h"

#define COMMENT_OUT         0

#define NACK 0xA5
#define ACK 0xF5

//command codes
#define COMMAND_LED_CTRL        0x50
#define COMMAND_SENSOR_READ     0x51
#define COMMAND_LED_READ        0x52
#define COMMAND_PRINT           0x53
#define COMMAND_ID_READ         0x54

#define LED_ON                  1
#define LED_OFF                 0

//arduino analog pins
#define ANALOG_PIN0             0
#define ANALOG_PIN1             1
#define ANALOG_PIN2             2
#define ANALOG_PIN3             3
#define ANALOG_PIN4             4

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
    spi.GPIOConf.pinNumber = GPIO_PIN_NO_14;
    gpio_init(&spi);

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

    //char msg[] = "My name is Qusai!";

    uint8_t dummy_read = 0x0;
    uint8_t dummy_write = 0xFF;

    init_gpio_button();
    init_gpio_spi();
    init_spi();

    //spi_ssi_control(SPI2, ENABLE);

    spi_ssoe_control(SPI2, ENABLE);


    for (;;)
    {

        while (!gpio_read_pin(GPIOA, GPIO_PIN_NO_0));

        mdelay(100);

        uint8_t cmd = COMMAND_LED_CTRL;
        uint8_t ack = 0x0;
        uint8_t arg[2];

        spi_peripheral_control(SPI2, ENABLE);

        spi_transmit(SPI2, &cmd, 1);

        spi_receive(SPI2, &dummy_read, 1);

        spi_transmit(SPI2, &dummy_write, 1);

        spi_receive(SPI2, &ack, 1);

        if(ack == ACK){
            arg[0] = 9;
            arg[1] = LED_ON;

            spi_transmit(SPI2, arg, 2);
        }

        spi_peripheral_control(SPI2, DISABLE);



    }
}

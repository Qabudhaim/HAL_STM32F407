/*
 * 009i2c_master_send.c
 *
 *  Created on: Dec 4, 2021
 *      Author: obscure
 */

#include <string.h>
#include "stm32f407xx.h"

#define MY_ADDR     0x61
#define SLAVE_ADDR  0x68

i2c_handle_t i2c;

static void mdelay (uint32_t cnt)
{
    for (uint32_t i = 0; i < (cnt * 1000); i++);
}

static void init_gpio_button ()
{
    gpio_handle_t button;
    button.pGPIOx = GPIOA;
    button.GPIOConf.pinMode = GPIO_MODE_INPUT;
    button.GPIOConf.pinNumber = GPIO_PIN_NO_0;
    button.GPIOConf.pinOPType = GPIO_OP_TYPE_PP;
    button.GPIOConf.pinPUPD = GPIO_PIN_NO_PUPD;
    button.GPIOConf.pinSpeed = GPIO_SPEED_HIGH;

    gpio_init(&button);
}

static void init_gpio_led ()
{
    gpio_handle_t led;
    led.pGPIOx = GPIOD;
    led.GPIOConf.pinMode = GPIO_MODE_OUTPUT;
    led.GPIOConf.pinNumber = GPIO_PIN_NO_12;
    led.GPIOConf.pinOPType = GPIO_OP_TYPE_PP;
    led.GPIOConf.pinPUPD = GPIO_PIN_NO_PUPD;
    led.GPIOConf.pinSpeed = GPIO_SPEED_HIGH;

    gpio_init(&led);
}

static void init_gpio_i2c ()
{
    gpio_handle_t gpio_i2c;
    gpio_i2c.pGPIOx = GPIOB;
    gpio_i2c.GPIOConf.pinMode = GPIO_MODE_ALTFUN;
    gpio_i2c.GPIOConf.pinAltFun = GPIO_ALTFUN_AF4;
    gpio_i2c.GPIOConf.pinOPType = GPIO_OP_TYPE_OD;
    gpio_i2c.GPIOConf.pinPUPD = GPIO_PIN_PU;
    gpio_i2c.GPIOConf.pinSpeed = GPIO_SPEED_HIGH;

    // TX
    gpio_i2c.GPIOConf.pinNumber = GPIO_PIN_NO_10;
    gpio_init(&gpio_i2c);

    // RX
    gpio_i2c.GPIOConf.pinNumber = GPIO_PIN_NO_11;
    gpio_init(&gpio_i2c);

}

static void init_i2c ()
{

    i2c.pI2Cx = I2C2;
    i2c.i2cConf.i2cACKControl = I2C_ACK_EN;
    i2c.i2cConf.i2cFMDutyCycle = I2C_FM_DUTY_2;
    i2c.i2cConf.i2cSclSpeed = I2C_SCL_SPEED_SM;
    i2c.i2cConf.i2cDeviceAddress = MY_ADDR;

    i2c_init(&i2c);
}

int main (void)
{

    char data[] = "Qusai Abudhaim!";

    init_gpio_button();
    init_gpio_led();
    init_gpio_i2c();
    init_i2c();

    i2c_peripheral_control(I2C2, ENABLE);

    for (;;)
    {
        while (!gpio_read_pin(GPIOA, GPIO_PIN_NO_0));

        mdelay(200);

        i2c_transmit(&i2c, (uint8_t*) data, strlen(data), SLAVE_ADDR);

    }

    return 0;
}


/*
 * 001blink_led.c
 *
 *  Created on: Nov 14, 2021
 *      Author: obscure
 */

#include "stm32f407xx_gpio_driver.h"

static void mdelay (uint32_t cnt)
{
    for (uint32_t i = 0; i < (cnt * 1000); i++);
}

static void init ()
{
    gpio_handle_t led;
    led.pGPIOx = GPIOD;
    led.GPIOConf.pinMode = GPIO_MODE_OUTPUT;
    led.GPIOConf.pinNumber = GPIO_PIN_NO_12;
    led.GPIOConf.pinOPType = GPIO_OP_TYPE_PP;
    led.GPIOConf.pinPUPD = GPIO_PIN_NO_PUPD;
    led.GPIOConf.pinSpeed = GPIO_SPEED_HIGH;

    gpio_init(&led);

    led.GPIOConf.pinNumber = GPIO_PIN_NO_13;
    gpio_init(&led);

    led.GPIOConf.pinNumber = GPIO_PIN_NO_14;
    gpio_init(&led);

    led.GPIOConf.pinNumber = GPIO_PIN_NO_15;
    gpio_init(&led);

}

int main ()
{
    init();

    for (;;)
    {
        gpio_toggle_pin(GPIOD, GPIO_PIN_NO_12);
        mdelay(1000);
        gpio_toggle_pin(GPIOD, GPIO_PIN_NO_12);
        gpio_toggle_pin(GPIOD, GPIO_PIN_NO_13);
        mdelay(1000);
        gpio_toggle_pin(GPIOD, GPIO_PIN_NO_13);
        gpio_toggle_pin(GPIOD, GPIO_PIN_NO_14);
        mdelay(1000);
        gpio_toggle_pin(GPIOD, GPIO_PIN_NO_14);
        gpio_toggle_pin(GPIOD, GPIO_PIN_NO_15);
        mdelay(1000);
        gpio_toggle_pin(GPIOD, GPIO_PIN_NO_15);
    }

    return 0;
}

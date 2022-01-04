/*
 * 002button_toggle_led.c
 *
 *  Created on: Nov 14, 2021
 *      Author: obscure
 */


#include "stm32f407xx_gpio_driver.h"

static void mdelay (uint32_t cnt)
{
    for (uint32_t i = 0; i < (cnt * 1000); i++);
}

static void init_led ()
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

static void init_button ()
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

int main ()
{
    init_led();
    init_button();

    for (;;)
    {
        if(gpio_read_pin(GPIOA, GPIO_PIN_NO_0) == GPIO_PIN_SET){
            mdelay(150);
            gpio_toggle_pin(GPIOD, GPIO_PIN_NO_12);
        }
    }

    return 0;
}

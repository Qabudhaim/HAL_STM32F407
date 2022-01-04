/*
 * 003button_toggle_led_it.c
 *
 *  Created on: Nov 15, 2021
 *      Author: obscure
 */

#include <string.h>
#include "stm32f407xx.h"

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
    button.GPIOConf.pinMode = GPIO_MODE_IT_RE;
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

    gpio_irq_config(IRQ_NO_EXTI0, ENABLE);

    for (;;);

    return 0;
}

void EXTI0_IRQHandler(){
    mdelay(100);
    gpio_irq_handler(GPIO_PIN_NO_0);
    gpio_toggle_pin(GPIOD, GPIO_PIN_NO_12);
}



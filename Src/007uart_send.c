/*
 * 007uart_send.c
 *
 *  Created on: Nov 23, 2021
 *      Author: obscure
 */

#include <string.h>
#include "stm32f407xx.h"

uart_handle_t uart;

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

static void init_gpio_uart ()
{
    gpio_handle_t gpio_uart;
    gpio_uart.pGPIOx = GPIOB;
    gpio_uart.GPIOConf.pinMode = GPIO_MODE_ALTFUN;
    gpio_uart.GPIOConf.pinAltFun = GPIO_ALTFUN_AF7;
    gpio_uart.GPIOConf.pinOPType = GPIO_OP_TYPE_PP;
    gpio_uart.GPIOConf.pinPUPD = GPIO_PIN_NO_PUPD;
    gpio_uart.GPIOConf.pinSpeed = GPIO_SPEED_HIGH;

    // TX
    gpio_uart.GPIOConf.pinNumber = GPIO_PIN_NO_6;
    gpio_init(&gpio_uart);

    // RX
    gpio_uart.GPIOConf.pinNumber = GPIO_PIN_NO_7;
    gpio_init(&gpio_uart);

}

static void init_uart ()
{
    uart.pUARTx = UART1;
    uart.UARTConf.uartMode = UART_MODE_TXRX;
    uart.UARTConf.uartParityControl = UART_PARITY_DI;
    uart.UARTConf.uartStopBits = UART_STOP_BITS_1;
    uart.UARTConf.uartWordLength = UART_WORD_8_BITS;
    uart.UARTConf.uartHWFlowControl = UART_HW_FLOW_CTRL_NONE;
    uart.UARTConf.uartBaudRate = UART_BAUD_RATE_115200;

    uart_init(&uart);
}

int main (void)
{

    char tr_buf[] = "My name is Qusai! ";
    char rcv_buf[10];
    char data = '2';

    init_gpio_button();
    init_gpio_led();
    init_gpio_uart();
    init_uart();

    for (;;)
    {

        while (!gpio_read_pin(GPIOA, GPIO_PIN_NO_0));

        mdelay(200);

        uart_peripheral_control(UART1, ENABLE);

        uart_transmit(&uart, (uint8_t*) tr_buf, strlen(tr_buf));

        uart_receive(&uart, (uint8_t*) rcv_buf, 1);

        if (rcv_buf[0] == data)
        {
            gpio_toggle_pin(GPIOD, 12);
        }

        //uart_peripheral_control(UART1, DISABLE);

    }
}

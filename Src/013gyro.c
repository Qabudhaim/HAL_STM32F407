/*
 * 013gyro.c
 *
 *  Created on: Dec 14, 2021
 *      Author: obscure
 */

#include <string.h>
#include "stm32f407xx.h"
#include "l3gd20.h"

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

static void init_gpio_uart ()
{
    gpio_handle_t gpio_uart;
    gpio_uart.pGPIOx = GPIOC;
    gpio_uart.GPIOConf.pinMode = GPIO_MODE_ALTFUN;
    gpio_uart.GPIOConf.pinAltFun = GPIO_ALTFUN_AF7;
    gpio_uart.GPIOConf.pinOPType = GPIO_OP_TYPE_PP;
    gpio_uart.GPIOConf.pinPUPD = GPIO_PIN_NO_PUPD;
    gpio_uart.GPIOConf.pinSpeed = GPIO_SPEED_HIGH;

    // TX
    gpio_uart.GPIOConf.pinNumber = GPIO_PIN_NO_10;
    gpio_init(&gpio_uart);

    // RX
    gpio_uart.GPIOConf.pinNumber = GPIO_PIN_NO_11;
    gpio_init(&gpio_uart);

}

static void init_uart ()
{
    uart.pUARTx = UART3;
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

    int16_t values[3];

    int16_t X, Y, Z;

    init_gpio_button();
    init_gpio_uart();
    init_uart();

    l3gd20_init();

    for (;;)
    {
        while (!gpio_read_pin(GPIOA, GPIO_PIN_NO_0));

        mdelay(200);

        l3gd20_read(values);

        X = values[0];
        Y = values[1];
        Z = values[2];

        /**
         uart_peripheral_control(UART3, ENABLE);

         uart_transmit(&uart, (uint8_t*) (date_to_string(&current_date)),
         strlen(date_to_string(&current_date)));

         uart_peripheral_control(UART3, DISABLE);
         **/
    }
}

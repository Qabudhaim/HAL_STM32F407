/*
 * 012rtc_uart.c
 *
 *  Created on: Dec 12, 2021
 *      Author: obscure
 */

#include <string.h>
#include "stm32f407xx.h"
#include "ds1307.h"

uart_handle_t uart;

static void mdelay (uint32_t cnt)
{
    for (uint32_t i = 0; i < (cnt * 1000); i++);
}

void number_to_string (uint8_t num, char * buf)
{
    if (num < 10)
    {
        buf[0] = '0';
        buf[1] = num + 48;
    }
    else if (num >= 10 && num < 99)
    {
        buf[0] = (num / 10) + 48;
        buf[1] = (num % 10) + 48;
    }
}

//hh:mm:ss
char* time_to_string (rtc_time_t * rtc_time)
{
    static char buf[9];

    buf[2] = ':';
    buf[5] = ':';

    number_to_string(rtc_time->hours, buf);
    number_to_string(rtc_time->minutes, &buf[3]);
    number_to_string(rtc_time->seconds, &buf[6]);

    buf[8] = '\0';

    return buf;
}

char* date_to_string (rtc_date_t * rtc_date)
{
    static char buf[9];

    buf[2] = '/';
    buf[5] = '/';

    number_to_string(rtc_date->date, buf);
    number_to_string(rtc_date->month, &buf[3]);
    number_to_string(rtc_date->year, &buf[6]);

    buf[8] = '\0';

    return buf;
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
    rtc_time_t current_time;
    rtc_date_t current_date;

    if (ds1307_init())
    {
        while (1);
    }

    current_date.day = SUNDAY;
    current_date.date = 12;
    current_date.month = 12;
    current_date.year = 21;

    current_time.hours = 23;
    current_time.minutes = 59;
    current_time.seconds = 35;
    current_time.time_format = DS1307_TIME_FORMAT_24;



    init_gpio_button();
    init_gpio_uart();
    init_uart();

    while (!gpio_read_pin(GPIOA, GPIO_PIN_NO_0));

    ds1307_set_current_date(&current_date);
    ds1307_set_current_time(&current_time);

    for (;;)
    {

        mdelay(1000);

        ds1307_get_current_date(&current_date);
        ds1307_get_current_time(&current_time);

        uart_peripheral_control(UART3, ENABLE);

        uart_transmit(&uart, (uint8_t*) (date_to_string(&current_date)),
                strlen(date_to_string(&current_date)));

        uart_transmit(&uart, (uint8_t*) " ", 1);

        uart_transmit(&uart, (uint8_t*) time_to_string(&current_time),
                strlen(time_to_string(&current_time)));

        uart_transmit(&uart, (uint8_t*) "\n\r", 1);

        uart_peripheral_control(UART3, DISABLE);
    }
}

/*
 * 015imu.c
 *
 *  Created on: Dec 15, 2021
 *      Author: obscure
 */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"
#include "l3gd20.h"
#include "lsm303.h"

#define M_PI 3.14159265358979323846

const float G_GAIN = 0.07;
const float DT = 0.007;
const float RAD_TO_DEG = 57.29578;
const float AA = 0.98;

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

static void init_gpio_timer ()
{
    gpio_handle_t timer;
    timer.pGPIOx = GPIOD;
    timer.GPIOConf.pinMode = GPIO_MODE_OUTPUT;
    timer.GPIOConf.pinNumber = GPIO_PIN_NO_9;
    timer.GPIOConf.pinOPType = GPIO_OP_TYPE_PP;
    timer.GPIOConf.pinPUPD = GPIO_PIN_NO_PUPD;
    timer.GPIOConf.pinSpeed = GPIO_SPEED_HIGH;

    gpio_init(&timer);
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

void read (float * x, float * y)
{
    int16_t gyr_raw[3];
    int16_t acc_raw[3];

    float rate_gyr_x, rate_gyr_y, rate_gyr_z;
    float gyroXangle = 0;
    float gyroYangle = 0;
    float gyroZangle = 0;
    float accXangle, accYangle;
    float CFangleX = 0;
    float CFangleY = 0;
    //int16_t gyro_x, gyro_y, gyro_z;
    //int16_t acc_x, acc_y, acc_z;

    l3gd20_read(gyr_raw);
    lsm303_read(acc_raw);

    rate_gyr_x = (float) *gyr_raw * G_GAIN;
    rate_gyr_y = (float) (*(gyr_raw + 1) * G_GAIN);
    rate_gyr_z = (float) (*(gyr_raw + 2) * G_GAIN);

    gyroXangle += rate_gyr_x * DT;
    gyroYangle += rate_gyr_y * DT;
    gyroZangle += rate_gyr_z * DT;

    accXangle = (float) ((atan2(*(acc_raw + 1), *(acc_raw + 2)) + M_PI)
            * RAD_TO_DEG);
    accYangle = (float) ((atan2(*(acc_raw + 2), *acc_raw) + M_PI) * RAD_TO_DEG);

    *x = CFangleX = AA * (CFangleX + rate_gyr_x * DT) + (1 - AA) * accXangle;
    *y = CFangleY = AA * (CFangleY + rate_gyr_y * DT) + (1 - AA) * accYangle;
}

int main (void)
{

    init_gpio_button();
    init_gpio_timer();

    init_gpio_uart();
    init_uart();

    l3gd20_init();
    lsm303_init();

    int16_t gyr_raw[3];
    int16_t acc_raw[3];

    float x, y;
    char x_str[5];
    char y_str[5];
    uart_peripheral_control(UART3, ENABLE);

    for (;;)
    {
        gpio_write_pin(GPIOD, 9, SET);

        //mdelay(100);

        l3gd20_read(gyr_raw);
        lsm303_read(acc_raw);

        read(&x, &y);

        gcvt(x, 5, x_str);
        gcvt(y, 5, y_str);

        uart_transmit(&uart, "x: ", 3);
        uart_transmit(&uart, (uint8_t*) x_str, strlen(x_str));
        uart_transmit(&uart, "\n", 1);

        uart_transmit(&uart, " y: ", 4);
        uart_transmit(&uart, (uint8_t*) y_str, strlen(y_str));
        uart_transmit(&uart, "\r", 1);

        gpio_write_pin(GPIOD, 9, RESET);

        //uart_peripheral_control(UART3, DISABLE);
    }
}


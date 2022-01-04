/*
 * 011i2c_interrupt.c
 *
 *  Created on: Dec 10, 2021
 *      Author: obscure
 */

#include <string.h>
#include "stm32f407xx.h"

#define MY_ADDR     0x61
#define SLAVE_ADDR  0x68

i2c_handle_t i2c;

uint8_t rcv_buf[32];

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

    // SCL
    gpio_i2c.GPIOConf.pinNumber = GPIO_PIN_NO_8;
    gpio_init(&gpio_i2c);

    // SDA
    gpio_i2c.GPIOConf.pinNumber = GPIO_PIN_NO_9;
    gpio_init(&gpio_i2c);

}

static void init_i2c ()
{

    i2c.pI2Cx = I2C1;
    i2c.i2cConf.i2cACKControl = I2C_ACK_EN;
    i2c.i2cConf.i2cFMDutyCycle = I2C_FM_DUTY_2;
    i2c.i2cConf.i2cSclSpeed = I2C_SCL_SPEED_SM;
    i2c.i2cConf.i2cDeviceAddress = MY_ADDR;

    i2c_init(&i2c);
}

int main (void)
{
    uint8_t commandCode = 0x51;
    uint8_t Len = 0;

    init_gpio_button();
    init_gpio_led();
    init_gpio_i2c();
    init_i2c();

    i2c_irq_config(IRQ_NO_I2C1_EV, ENABLE);
    i2c_irq_config(IRQ_NO_I2C1_ER, ENABLE);

    i2c_enable_disable_slave(I2C1, ENABLE);

    i2c_peripheral_control(I2C1, ENABLE);

    for (;;)
    {
        while (!gpio_read_pin(GPIOA, GPIO_PIN_NO_0));

        mdelay(200);

        commandCode = 0x51;
        while (i2c_master_transmit_interrupt(&i2c, &commandCode, 1,
        SLAVE_ADDR, I2C_SR_EN) != I2C_READY);
        while (i2c_master_receive_interrupt(&i2c, &Len, 1, SLAVE_ADDR,
        I2C_SR_EN) != I2C_READY);

        commandCode = 0x52;
        while (i2c_master_transmit_interrupt(&i2c, &commandCode, 1,
        SLAVE_ADDR, I2C_SR_EN) != I2C_READY);
        while (i2c_master_receive_interrupt(&i2c, rcv_buf, Len, SLAVE_ADDR,
        I2C_SR_DI) != I2C_READY);

    }
}

void I2C1_EV_IRQHandler (void)
{
    i2c_event_irq_handling(&i2c);
}

void I2C1_ER_IRQHandler (void)
{
    i2c_error_irq_handling(&i2c);
}


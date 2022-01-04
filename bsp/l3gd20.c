/*
 * l3gd20.c
 *
 *  Created on: Dec 12, 2021
 *      Author: obscure
 */

#include "l3gd20.h"

i2c_handle_t g_l3gd20Handle;

static void l3gd20_i2c_config (void)
{
    g_l3gd20Handle.pI2Cx = I2C1;
    g_l3gd20Handle.i2cConf.i2cACKControl = I2C_ACK_EN;
    g_l3gd20Handle.i2cConf.i2cSclSpeed = I2C_SCL_SPEED_SM;

    i2c_init(&g_l3gd20Handle);
}

static void l3gd20_i2c_pin_config (void)
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

static void l3gd20_config (uint8_t value, uint8_t reg_addr)
{
    uint8_t tx[2];
    tx[0] = reg_addr;
    tx[1] = value;

    i2c_master_transmit(&g_l3gd20Handle, tx, 2, L3GD20_SAD,
    I2C_SR_DI);
}

void l3gd20_read (int16_t * data)
{
    uint8_t buffer[6];
    uint8_t SUB = 0x80 | L3GD20_OUT_X_L;

    i2c_master_transmit(&g_l3gd20Handle, &SUB, 1, L3GD20_SAD,
    I2C_SR_EN);

    i2c_master_receive(&g_l3gd20Handle, buffer, 1, L3GD20_SAD, I2C_SR_EN);
    i2c_master_receive(&g_l3gd20Handle, buffer + 1, 1, L3GD20_SAD, I2C_SR_EN);
    i2c_master_receive(&g_l3gd20Handle, buffer + 2, 1, L3GD20_SAD, I2C_SR_EN);
    i2c_master_receive(&g_l3gd20Handle, buffer + 3, 1, L3GD20_SAD, I2C_SR_EN);
    i2c_master_receive(&g_l3gd20Handle, buffer + 4, 1, L3GD20_SAD, I2C_SR_EN);
    i2c_master_receive(&g_l3gd20Handle, buffer + 5, 1, L3GD20_SAD, I2C_SR_DI);

    *data = (int16_t) (buffer[0] | (buffer[1] << 8));
    *(data + 1) = (int16_t) (buffer[2] | (buffer[3] << 8));
    *(data + 2) = (int16_t) (buffer[4] | (buffer[5] << 8));
}

uint8_t l3gd20_init ()
{
    l3gd20_i2c_pin_config();

    l3gd20_i2c_config();

    i2c_peripheral_control(I2C1, ENABLE);

    l3gd20_config(0x0F, L3GD20_CTRL_REG1);
    l3gd20_config(0x30, L3GD20_CTRL_REG4);

    return 0;
}

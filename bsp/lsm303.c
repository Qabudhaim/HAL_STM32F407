/*
 * lsm303.c
 *
 *  Created on: Dec 15, 2021
 *      Author: obscure
 */

#include "lsm303.h"

i2c_handle_t g_lsm303Handle;

static void lsm303_i2c_config (void)
{
    g_lsm303Handle.pI2Cx = I2C1;
    g_lsm303Handle.i2cConf.i2cACKControl = I2C_ACK_EN;
    g_lsm303Handle.i2cConf.i2cSclSpeed = I2C_SCL_SPEED_SM;

    i2c_init(&g_lsm303Handle);
}

static void lsm303_i2c_pin_config (void)
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

static void lsm303_config (uint8_t value, uint8_t reg_addr)
{
    uint8_t tx[2];
    tx[0] = reg_addr;
    tx[1] = value;

    i2c_master_transmit(&g_lsm303Handle, tx, 2, LSM303_SAD, I2C_SR_DI);
}

void lsm303_read (int16_t * data)
{
    uint8_t buffer[6];
    uint8_t SUB = 0x80 | LSM303_OUT_X_L;

    i2c_master_transmit(&g_lsm303Handle, &SUB, 1, LSM303_SAD, I2C_SR_EN);

    i2c_master_receive(&g_lsm303Handle, buffer, 1, LSM303_SAD, I2C_SR_EN);
    i2c_master_receive(&g_lsm303Handle, buffer + 1, 1, LSM303_SAD, I2C_SR_EN);
    i2c_master_receive(&g_lsm303Handle, buffer + 2, 1, LSM303_SAD, I2C_SR_EN);
    i2c_master_receive(&g_lsm303Handle, buffer + 3, 1, LSM303_SAD, I2C_SR_EN);
    i2c_master_receive(&g_lsm303Handle, buffer + 4, 1, LSM303_SAD, I2C_SR_EN);
    i2c_master_receive(&g_lsm303Handle, buffer + 5, 1, LSM303_SAD, I2C_SR_DI);

    *data = (int16_t) (buffer[0] | (buffer[1] << 8));
    *(data + 1) = (int16_t) (buffer[2] | (buffer[3] << 8));
    *(data + 2) = (int16_t) (buffer[4] | (buffer[5] << 8));
}

uint8_t lsm303_init ()
{
    lsm303_i2c_pin_config();

    lsm303_i2c_config();

    i2c_peripheral_control(I2C1, ENABLE);

    lsm303_config(0x57, LSM303_CTRL_REG1);
    lsm303_config(0x28, LSM303_CTRL_REG4);

    return 0;
}

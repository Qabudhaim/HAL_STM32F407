/*
 * ds1307.c
 *
 *  Created on: Dec 11, 2021
 *      Author: obscure
 */
#include <stdint.h>
#include <string.h>
#include "ds1307.h"

i2c_handle_t g_ds1307Handle;

static uint8_t ds1307_binary_to_bcd (uint8_t value);
static uint8_t ds1307_bcd_to_binary (uint8_t value);
static uint8_t ds1307_read (uint8_t reg_addr);
static void ds1307_write (uint8_t value, uint8_t reg_addr);
static void ds1307_i2c_config (void);
static void ds1307_i2c_pin_config (void);

static uint8_t ds1307_binary_to_bcd (uint8_t value)
{
    uint8_t m, n;
    uint8_t bcd;

    bcd = value;
    if (value >= 10)
    {
        m = value / 10;
        n = value % 10;
        bcd = ((m << 4) | n);
    }
    return bcd;
}

static uint8_t ds1307_bcd_to_binary (uint8_t value)
{
    uint8_t m, n;
    m = (uint8_t) ((value >> 4) * 10);
    n = value & (uint8_t) 0x0F;
    return (m + n);
}

static void ds1307_i2c_config (void)
{
    g_ds1307Handle.pI2Cx = I2C1;
    g_ds1307Handle.i2cConf.i2cACKControl = I2C_ACK_EN;
    g_ds1307Handle.i2cConf.i2cSclSpeed = I2C_SCL_SPEED_SM;

    i2c_init(&g_ds1307Handle);
}

static void ds1307_i2c_pin_config (void)
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

static void ds1307_write (uint8_t value, uint8_t reg_addr)
{
    uint8_t tx[2];
    tx[0] = reg_addr;
    tx[1] = value;

    i2c_master_transmit(&g_ds1307Handle, tx, 2, DS1307_I2C_ADDR,
    I2C_SR_DI);
}

static uint8_t ds1307_read (uint8_t reg_addr)
{
    uint8_t data;
    i2c_master_transmit(&g_ds1307Handle, &reg_addr, 1, DS1307_I2C_ADDR,
    I2C_SR_DI);
    i2c_master_receive(&g_ds1307Handle, &data, 1, DS1307_I2C_ADDR, I2C_SR_DI);
    return data;
}

void ds1307_set_current_time (rtc_time_t * rtc_time)
{
    uint8_t seconds, hrs;

    seconds = ds1307_binary_to_bcd(rtc_time->seconds);
    seconds &= ~(1 << 7);
    ds1307_write(seconds, DS1307_ADDR_SECONDS);

    ds1307_write(ds1307_binary_to_bcd(rtc_time->minutes), DS1307_ADDR_MINUTES);

    hrs = ds1307_binary_to_bcd(rtc_time->hours);

    if (rtc_time->time_format == DS1307_TIME_FORMAT_24)
    {
        hrs &= ~(1 << 6);
    }
    else
    {
        hrs |= (1 << 6);
        hrs = (rtc_time->time_format == DS1307_TIME_FORMAT_12_AM) ?
                hrs & ~(1 << 5) : hrs | (1 << 5);
    }
    ds1307_write(hrs, DS1307_ADDR_HOURS);

}

void ds1307_set_current_date (rtc_date_t * rtc_date)
{
    ds1307_write(ds1307_binary_to_bcd(rtc_date->day), DS1307_ADDR_DAY);
    ds1307_write(ds1307_binary_to_bcd(rtc_date->date), DS1307_ADDR_DATE);
    ds1307_write(ds1307_binary_to_bcd(rtc_date->month), DS1307_ADDR_MONTH);
    ds1307_write(ds1307_binary_to_bcd(rtc_date->year), DS1307_ADDR_YEAR);
}

void ds1307_get_current_time (rtc_time_t * rtc_time)
{
    uint8_t seconds, hrs;
    seconds = ds1307_read(DS1307_ADDR_SECONDS);
    seconds &= ~(1 << 7);
    rtc_time->seconds = ds1307_bcd_to_binary(seconds);
    rtc_time->minutes = ds1307_bcd_to_binary(ds1307_read(DS1307_ADDR_MINUTES));

    hrs = ds1307_read(DS1307_ADDR_HOURS);
    if (hrs & (1 << 6))
    {
        // TIME FORMAT 12
        rtc_time->time_format = !((hrs & (1 << 5)) == 0);
        hrs &= ~(0x3 << 5);
    }
    else
    {
        // TIME FORMAT 24
        rtc_time->time_format = DS1307_TIME_FORMAT_24;
    }
    rtc_time->hours = ds1307_bcd_to_binary(hrs);

}

void ds1307_get_current_date (rtc_date_t * rtc_date)
{
    rtc_date->day = ds1307_bcd_to_binary(ds1307_read(DS1307_ADDR_DAY));
    rtc_date->date = ds1307_bcd_to_binary(ds1307_read(DS1307_ADDR_DATE));
    rtc_date->month = ds1307_bcd_to_binary(ds1307_read(DS1307_ADDR_MONTH));
    rtc_date->year = ds1307_bcd_to_binary(ds1307_read(DS1307_ADDR_YEAR));
}

uint8_t ds1307_init ()
{
    ds1307_i2c_pin_config();

    ds1307_i2c_config();

    i2c_peripheral_control(I2C1, ENABLE);

    ds1307_write(0x00, DS1307_ADDR_SECONDS);

    uint8_t clock_state = ds1307_read(DS1307_ADDR_SECONDS);

    return ((clock_state >> 7) & 0x1);
}


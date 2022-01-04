/*
 * ds1307.h
 *
 *  Created on: Dec 11, 2021
 *      Author: obscure
 */

#ifndef DS1307_H_
#define DS1307_H_

#include "stm32f407xx.h"

typedef struct
{
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t time_format;
} rtc_time_t;

typedef struct
{
    uint8_t day;
    uint8_t date;
    uint8_t month;
    uint8_t year;
} rtc_date_t;

/**
 * RTC device address
 */
#define DS1307_I2C_ADDR             0x68

/**
 * Register addresses
 */
#define DS1307_ADDR_SECONDS         0X00
#define DS1307_ADDR_MINUTES         0X01
#define DS1307_ADDR_HOURS           0X02
#define DS1307_ADDR_DAY             0X03
#define DS1307_ADDR_DATE            0X04
#define DS1307_ADDR_MONTH           0X05
#define DS1307_ADDR_YEAR            0X06

/**
 *  Options for time_format
 */
#define DS1307_TIME_FORMAT_12_AM    0
#define DS1307_TIME_FORMAT_12_PM    1
#define DS1307_TIME_FORMAT_24       2

/**
 *  Options for day
 */
#define SUNDAY                      1
#define MONDAY                      2
#define TUESDAY                     3
#define WEDNESDAY                   4
#define THURSDAY                    5
#define FRIDAY                      6
#define SATURDAY                    7

/****************************************************************************
 *                       APIs supported by this driver                      *
 ****************************************************************************/
uint8_t ds1307_init ();

void ds1307_set_current_time (rtc_time_t*);
void ds1307_get_current_time (rtc_time_t*);

void ds1307_set_current_date (rtc_date_t*);
void ds1307_get_current_date (rtc_date_t*);

#endif /* DS1307_H_ */

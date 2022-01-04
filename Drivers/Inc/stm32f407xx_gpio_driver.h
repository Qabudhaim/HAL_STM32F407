/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Nov 14, 2021
 *      Author: obscure
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/****************************************************************************
 *                       Peripheral handle structures                       *
 ****************************************************************************/
typedef struct
{
    uint8_t pinNumber;
    uint8_t pinMode;
    uint8_t pinOPType;
    uint8_t pinSpeed;
    uint8_t pinPUPD;
    uint8_t pinAltFun;
} gpio_conf_t;

typedef struct
{
    gpio_reg_t * pGPIOx;
    gpio_conf_t GPIOConf;
} gpio_handle_t;

/****************************************************************************
 *                       Peripheral user configuration                      *
 ****************************************************************************/

/**
 * @pinNumber
 * Possible GPIO pin number
 */
#define GPIO_PIN_NO_0               0
#define GPIO_PIN_NO_1               1
#define GPIO_PIN_NO_2               2
#define GPIO_PIN_NO_3               3
#define GPIO_PIN_NO_4               4
#define GPIO_PIN_NO_5               5
#define GPIO_PIN_NO_6               6
#define GPIO_PIN_NO_7               7
#define GPIO_PIN_NO_8               8
#define GPIO_PIN_NO_9               9
#define GPIO_PIN_NO_10              10
#define GPIO_PIN_NO_11              11
#define GPIO_PIN_NO_12              12
#define GPIO_PIN_NO_13              13
#define GPIO_PIN_NO_14              14
#define GPIO_PIN_NO_15              15

/**
 * @pinMode
 * Possible GPIO pin modes
 */
#define GPIO_MODE_INPUT             0
#define GPIO_MODE_OUTPUT            1
#define GPIO_MODE_ALTFUN            2
#define GPIO_MODE_ANALOG            3
#define GPIO_MODE_IT_RE             4
#define GPIO_MODE_IT_FE             5
#define GPIO_MODE_IT_RFE            6

/**
 * @pinOPType
 * Possible GPIO Output pin types
 */
#define GPIO_OP_TYPE_PP             0
#define GPIO_OP_TYPE_OD             1

/**
 * @pinSpeed
 * Possible GPIO pin speeds
 */
#define GPIO_SPEED_LOW              0
#define GPIO_SPEED_MEDIUM           1
#define GPIO_SPEED_HIGH             2
#define GPIO_SPEED_VHIGH            3

/**
 * @pinPUPD
 * Possible GPIO enable Pull-UP/Pull Down resistors configurations
 */
#define GPIO_PIN_NO_PUPD            0
#define GPIO_PIN_PU                 1
#define GPIO_PIN_PD                 2

/**
 * @pinAltFun
 * Possible GPIO alternate functions
 */
#define GPIO_ALTFUN_AF0             0
#define GPIO_ALTFUN_AF1             1
#define GPIO_ALTFUN_AF2             2
#define GPIO_ALTFUN_AF3             3
#define GPIO_ALTFUN_AF4             4
#define GPIO_ALTFUN_AF5             5
#define GPIO_ALTFUN_AF6             6
#define GPIO_ALTFUN_AF7             7
#define GPIO_ALTFUN_AF8             8
#define GPIO_ALTFUN_AF9             9
#define GPIO_ALTFUN_AF10            10
#define GPIO_ALTFUN_AF11            11
#define GPIO_ALTFUN_AF12            12
#define GPIO_ALTFUN_AF13            13
#define GPIO_ALTFUN_AF14            14
#define GPIO_ALTFUN_AF15            15

/****************************************************************************
 *                           Generic macros                                 *
 ****************************************************************************/
#define GPIO_PIN_SET                SET
#define GPIO_PIN_RESET              RESET

#define GPIO_PORT_TO_CODE(x)        ((x == GPIOA) ? 0: \
                                     (x == GPIOB) ? 1: \
                                     (x == GPIOC) ? 2: \
                                     (x == GPIOD) ? 3: \
                                     (x == GPIOE) ? 4: \
                                     (x == GPIOF) ? 5: \
                                     (x == GPIOG) ? 6: \
                                     (x == GPIOH) ? 7: \
                                     (x == GPIOI) ? 8: 0)

/****************************************************************************
 *                       APIs supported by this driver                      *
 ****************************************************************************/
void gpio_pclk_control (gpio_reg_t * pGPIOx, uint8_t ENorDI);

void gpio_init (gpio_handle_t * pGPIOHandle);
void gpio_deinit (gpio_reg_t * pGPIOx);

uint16_t gpio_read_port (gpio_reg_t * pGPIOx);
uint8_t gpio_read_pin (gpio_reg_t * pGPIOx, uint8_t pinNumber);

void gpio_write_port (gpio_reg_t * pGPIOx, uint16_t data);
void gpio_write_pin (gpio_reg_t * pGPIOx, uint8_t pinNumber, uint8_t data);
void gpio_toggle_pin (gpio_reg_t * pGPIOx, uint8_t pinNumber);

void gpio_irq_config(uint8_t IRQ_No, uint8_t ENorDI);
void gpio_irq_priority(uint8_t IRQ_No, uint8_t IRQ_Priority);
void gpio_irq_handler(uint8_t Pin_No);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */

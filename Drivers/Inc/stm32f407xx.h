/*
 * stm32f407xx.h
 *
 *  Created on: Nov 13, 2021
 *      Author: obscure
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdio.h>

#define __vo volatile
#define __weak      __attribute__((weak))

/**
 * Memory partitions base addresses
 */
#define FLSAH_BASEADDR			    0x08000000U
#define ROM_BASEADDR                0x1FFF0000U
#define SRAM_BASEADDR			    0x20000000U
#define SRAM1_BASEADDR			    SRAM_BASEADDR
#define SRAM2_BASEADDR			    0x2001C000U

/**
 * NVIC engine base address
 */
#define NVIC_BASEADDR               0xE000E100U

/**
 * System busses addresses
 */
#define APB1PERIPH_BASEADDR         0x40000000U
#define APB2PERIPH_BASEADDR         0x40010000U
#define AHB1PERIPH_BASEADDR         0x40020000U
#define AHB2PERIPH_BASEADDR         0x50000000U
#define AHB3PERIPH_BASEADDR         0xA0000000U

/****************************************************************************
 *                          Peripheral addresses                            *
 ****************************************************************************/

/**
 * APB1 dependent peripherals
 */
#define SPI2_BASEADDR               (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR               (APB1PERIPH_BASEADDR + 0x3C00)
#define UART2_BASEADDR              (APB1PERIPH_BASEADDR + 0x4400)
#define UART3_BASEADDR              (APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR              (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR              (APB1PERIPH_BASEADDR + 0x5000)
#define I2C1_BASEADDR               (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR               (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR               (APB1PERIPH_BASEADDR + 0x5C00)
#define UART7_BASEADDR              (AHB1PERIPH_BASEADDR + 0x7800)
#define UART8_BASEADDR              (AHB1PERIPH_BASEADDR + 0x7C00)

/**
 * APB2 dependent peripherals
 */
#define UART1_BASEADDR              (APB2PERIPH_BASEADDR + 0x1000)
#define UART6_BASEADDR              (APB2PERIPH_BASEADDR + 0x1400)
#define SPI1_BASEADDR               (APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR               (APB2PERIPH_BASEADDR + 0x3400)
#define SYSCFG_BASEADDR             (APB2PERIPH_BASEADDR + 0x3800)
#define EXTI_BASEADDR               (APB2PERIPH_BASEADDR + 0x3C00)
#define SPI5_BASEADDR               (APB2PERIPH_BASEADDR + 0x5000)
#define SPI6_BASEADDR               (APB2PERIPH_BASEADDR + 0x5400)

/**
 * AHB1 dependent peripherals
 */
#define GPIOA_BASEADDR              (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR              (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR              (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR              (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR              (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR              (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR              (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR              (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR              (AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR                (AHB1PERIPH_BASEADDR + 0x3800)

/****************************************************************************
 *                 Peripheral registers definition structure                *
 ****************************************************************************/

typedef struct
{
    __vo uint32_t MODER;         //! GPIO port mode register
    __vo uint32_t OTYPER;        //! GPIO port output type register
    __vo uint32_t OSPEEDR;       //! GPIO port output speed register
    __vo uint32_t PUPDR;         //! GPIO port pull-up/pull-down register
    __vo uint32_t IDR;           //! GPIO port input data register
    __vo uint32_t ODR;           //! GPIO port output data register
    __vo uint32_t BSRR;          //! GPIO port bit set/reset register
    __vo uint32_t LCKR;          //! GPIO port configuration lock register
    __vo uint32_t AFR[2];        //! GPIO alternate function low/high registers
} gpio_reg_t;

typedef struct
{
    __vo uint32_t CR;             //! RCC clock control register
    __vo uint32_t PLLCFGR;        //! RCC PLL configuration register
    __vo uint32_t CFGR;           //! RCC clock configuration register
    __vo uint32_t CIR;            //! RCC clock interrupt register
    __vo uint32_t AHB1RSTR;       //! RCC AHB1 peripheral reset register
    __vo uint32_t AHB2RSTR;       //! RCC AHB2 peripheral reset register
    __vo uint32_t AHB3RSTR;       //! RCC AHB3 peripheral reset register
    uint32_t __reserved;     //! Reserved
    __vo uint32_t APB1RSTR;       //! RCC APB1 peripheral reset register
    __vo uint32_t APB2RSTR;       //! RCC APB2 peripheral reset register
    uint32_t __reserved1[2];  //! Reserved
    __vo uint32_t AHB1ENR;        //! RCC AHB1 peripheral clock register
    __vo uint32_t AHB2ENR;        //! RCC AHB2 peripheral clock enable register
    __vo uint32_t AHB3ENR;        //! RCC AHB3 peripheral clock enable register
    uint32_t __reserved2;    //! Reserved
    __vo uint32_t APB1ENR;        //! RCC APB1 peripheral clock enable register
    __vo uint32_t APB2ENR;        //! RCC APB2 peripheral clock enable register
    uint32_t __reserved3[2];  //! Reserved
    __vo uint32_t AHB1LPENR;  //! RCC AHB1 peripheral clock enable in low power mode register
    __vo uint32_t AHB2LPENR;  //! RCC AHB2 peripheral clock enable in low power mode register
    __vo uint32_t AHB3LPENR;  //! RCC AHB3 peripheral clock enable in low power mode register
    uint32_t __reserved4;    //! Reserved
    __vo uint32_t APB1LPENR;  //! RCC APB1 peripheral clock enable in low power mode register
    __vo uint32_t APB2LPENR;  //! RCC APB2 peripheral clock enabled in low power mode register
    uint32_t __reserved5[2];  //! Reserved
    __vo uint32_t BDCR;           //! RCC Backup domain control register
    __vo uint32_t CSR;            //! RCC clock control & status register
    uint32_t __reserved6[2];  //! Reserved
    __vo uint32_t SSCGR;       //! RCC spread spectrum clock generation register
    __vo uint32_t PLLI2SCFGR;     //! RCC PLLI2S configuration register
    __vo uint32_t PLLSAICFGR;     //! RCC PLL configuration register
    __vo uint32_t DCKCFGR;        //! RCC Dedicated Clock Configuration Register
} rcc_reg_t;

typedef struct
{
    __vo uint32_t ISER[8];      //! Interrupt Set-Enable Registers
    uint32_t __reserved[15];
    __vo uint32_t ICER[8];      //! Interrupt Clear-Enable Registers
    uint32_t __reserved1[15];
    __vo uint32_t ISPR[8];      //! Interrupt Set-Pending Registers
    uint32_t __reserved2[15];
    __vo uint32_t ICPR[8];      //! Interrupt Clear-Pending Registers
    uint32_t __reserved3[15];
    __vo uint32_t IABR[8];      //! Interrupt Active Bit Register
    uint32_t __reserved4[15];
    __vo uint32_t IPR[60];      //! Interrupt Priority Register
    __vo uint32_t STIR;         //! Software Trigger Interrupt Register
} nvic_reg_t;

typedef struct
{
    __vo uint32_t MEMRMP;       //! SYSCFG memory remap register
    __vo uint32_t PMC;         //! SYSCFG peripheral mode configuration register
    __vo uint32_t EXTICR[4];  //! SYSCFG external interrupt configuration registers
    __vo uint32_t CMPCR;        //! Compensation cell control register
} syscfg_reg_t;

typedef struct
{
    __vo uint32_t IMR;          //! Interrupt mask register
    __vo uint32_t EMR;          //! Event mask register
    __vo uint32_t RTSR;         //! Rising trigger selection register
    __vo uint32_t FTSR;         //! Falling trigger selection register
    __vo uint32_t SWIER;        //! Software interrupt event register
    __vo uint32_t PR;           //! Pending register
} exti_reg_t;

typedef struct
{
    __vo uint32_t CR1;          //! SPI control register 1
    __vo uint32_t CR2;          //! SPI control register 2
    __vo uint32_t SR;           //! SPI status register
    __vo uint32_t DR;           //! SPI data register
    __vo uint32_t CRCPR;        //! SPI CRC polynomial register
    __vo uint32_t RXCRCR;       //! SPI RX CRC register
    __vo uint32_t TXCRCR;       //! SPI TX CRC register
    __vo uint32_t I2SCFGR;      //! SPI_I2S configuration register
    __vo uint32_t I2SPR;        //! SPI_I2S pre-scaler register
} spi_reg_t;

typedef struct
{
    __vo uint32_t SR;           //! UART status register
    __vo uint32_t DR;           //! UART Data register
    __vo uint32_t BRR;          //! UART baud rate register
    __vo uint32_t CR1;          //! UART control register 1
    __vo uint32_t CR2;          //! UART control register 1
    __vo uint32_t CR3;          //! UART control register 1
    __vo uint32_t GTPR;         //! UART guard time and pre-scaler register
} uart_reg_t;

typedef struct
{
    __vo uint32_t CR1;          //! I2C Control register 1
    __vo uint32_t CR2;          //! I2C Control register 2
    __vo uint32_t OAR1;         //! I2C Own address register 1
    __vo uint32_t OAR2;         //! I2C Own address register 2
    __vo uint32_t DR;           //! I2C Data register
    __vo uint32_t SR1;          //! I2C Status Register 1
    __vo uint32_t SR2;          //! I2C Status Register 2
    __vo uint32_t CCR;          //! I2C Clock control register
    __vo uint32_t TRISE;        //! I2C TRISE register
    __vo uint32_t FLTR;         //! I2C FLTR register
} i2c_reg_t;

/****************************************************************************
 *                   Peripheral registers definition                      *
 ****************************************************************************/
#define GPIOA                       ((gpio_reg_t*) GPIOA_BASEADDR)
#define GPIOB                       ((gpio_reg_t*) GPIOB_BASEADDR)
#define GPIOC                       ((gpio_reg_t*) GPIOC_BASEADDR)
#define GPIOD                       ((gpio_reg_t*) GPIOD_BASEADDR)
#define GPIOE                       ((gpio_reg_t*) GPIOE_BASEADDR)
#define GPIOF                       ((gpio_reg_t*) GPIOF_BASEADDR)
#define GPIOG                       ((gpio_reg_t*) GPIOG_BASEADDR)
#define GPIOH                       ((gpio_reg_t*) GPIOH_BASEADDR)
#define GPIOI                       ((gpio_reg_t*) GPIOI_BASEADDR)

#define RCC                         ((rcc_reg_t*) RCC_BASEADDR)

#define EXTI                        ((exti_reg_t*) EXTI_BASEADDR)

#define SYSCFG                      ((syscfg_reg_t*) SYSCFG_BASEADDR)

#define NVIC                        ((nvic_reg_t*) NVIC_BASEADDR)

#define SPI1                        ((spi_reg_t*) SPI1_BASEADDR)
#define SPI2                        ((spi_reg_t*) SPI2_BASEADDR)
#define SPI3                        ((spi_reg_t*) SPI3_BASEADDR)
#define SPI4                        ((spi_reg_t*) SPI4_BASEADDR)
#define SPI5                        ((spi_reg_t*) SPI5_BASEADDR)
#define SPI6                        ((spi_reg_t*) SPI6_BASEADDR)

#define UART1                       ((uart_reg_t*) UART1_BASEADDR)
#define UART2                       ((uart_reg_t*) UART2_BASEADDR)
#define UART3                       ((uart_reg_t*) UART3_BASEADDR)
#define UART4                       ((uart_reg_t*) UART4_BASEADDR)
#define UART5                       ((uart_reg_t*) UART5_BASEADDR)
#define UART6                       ((uart_reg_t*) UART6_BASEADDR)
#define UART7                       ((uart_reg_t*) UART7_BASEADDR)
#define UART8                       ((uart_reg_t*) UART8_BASEADDR)

#define I2C1                        ((i2c_reg_t*) I2C1_BASEADDR)
#define I2C2                        ((i2c_reg_t*) I2C2_BASEADDR)
#define I2C3                        ((i2c_reg_t*) I2C3_BASEADDR)

/****************************************************************************
 *                        Peripheral clock enable                           *
 ****************************************************************************/
#define GPIOA_PCLK_EN()             ((RCC->AHB1ENR) |= (1 << 0))
#define GPIOB_PCLK_EN()             ((RCC->AHB1ENR) |= (1 << 1))
#define GPIOC_PCLK_EN()             ((RCC->AHB1ENR) |= (1 << 2))
#define GPIOD_PCLK_EN()             ((RCC->AHB1ENR) |= (1 << 3))
#define GPIOE_PCLK_EN()             ((RCC->AHB1ENR) |= (1 << 4))
#define GPIOF_PCLK_EN()             ((RCC->AHB1ENR) |= (1 << 5))
#define GPIOG_PCLK_EN()             ((RCC->AHB1ENR) |= (1 << 6))
#define GPIOH_PCLK_EN()             ((RCC->AHB1ENR) |= (1 << 7))
#define GPIOI_PCLK_EN()             ((RCC->AHB1ENR) |= (1 << 8))

#define SYSCFG_PCLK_EN()            ((RCC->APB2ENR) |= (1 << 14))

#define SPI1_PCLK_EN()              ((RCC->APB2ENR) |= (1 << 12))
#define SPI2_PCLK_EN()              ((RCC->APB1ENR) |= (1 << 14))
#define SPI3_PCLK_EN()              ((RCC->APB1ENR) |= (1 << 15))
#define SPI4_PCLK_EN()              ((RCC->APB2ENR) |= (1 << 13))
#define SPI5_PCLK_EN()              ((RCC->APB2ENR) |= (1 << 20))
#define SPI6_PCLK_EN()              ((RCC->APB2ENR) |= (1 << 21))

#define UART1_PCLK_EN()             ((RCC->APB2ENR) |= (1 << 4))
#define UART2_PCLK_EN()             ((RCC->APB1ENR) |= (1 << 17))
#define UART3_PCLK_EN()             ((RCC->APB1ENR) |= (1 << 18))
#define UART4_PCLK_EN()             ((RCC->APB1ENR) |= (1 << 19))
#define UART5_PCLK_EN()             ((RCC->APB1ENR) |= (1 << 20))
#define UART6_PCLK_EN()             ((RCC->APB2ENR) |= (1 << 5))
#define UART7_PCLK_EN()             ((RCC->APB1ENR) |= (1 << 30))
#define UART8_PCLK_EN()             ((RCC->APB1ENR) |= (1 << 31))

#define I2C1_PCLK_EN()              ((RCC->APB1ENR) |= (1 << 21))
#define I2C2_PCLK_EN()              ((RCC->APB1ENR) |= (1 << 22))
#define I2C3_PCLK_EN()              ((RCC->APB1ENR) |= (1 << 23))

/****************************************************************************
 *                        Peripheral clock disable                          *
 ****************************************************************************/
#define GPIOA_PCLK_DI()             ((RCC->AHB1ENR) &= ~(1 << 0))
#define GPIOB_PCLK_DI()             ((RCC->AHB1ENR) &= ~(1 << 1))
#define GPIOC_PCLK_DI()             ((RCC->AHB1ENR) &= ~(1 << 2))
#define GPIOD_PCLK_DI()             ((RCC->AHB1ENR) &= ~(1 << 3))
#define GPIOE_PCLK_DI()             ((RCC->AHB1ENR) &= ~(1 << 4))
#define GPIOF_PCLK_DI()             ((RCC->AHB1ENR) &= ~(1 << 5))
#define GPIOG_PCLK_DI()             ((RCC->AHB1ENR) &= ~(1 << 6))
#define GPIOH_PCLK_DI()             ((RCC->AHB1ENR) &= ~(1 << 7))
#define GPIOI_PCLK_DI()             ((RCC->AHB1ENR) &= ~(1 << 8))

#define SPI1_PCLK_DI()              ((RCC->APB2ENR) &= ~(1 << 12))
#define SPI2_PCLK_DI()              ((RCC->APB1ENR) &= ~(1 << 14))
#define SPI3_PCLK_DI()              ((RCC->APB1ENR) &= ~(1 << 15))
#define SPI4_PCLK_DI()              ((RCC->APB2ENR) &= ~(1 << 13))
#define SPI5_PCLK_DI()              ((RCC->APB2ENR) &= ~(1 << 20))
#define SPI6_PCLK_DI()              ((RCC->APB2ENR) &= ~(1 << 21))

#define UART1_PCLK_DI()             ((RCC->APB2ENR) &= ~(1 << 4))
#define UART2_PCLK_DI()             ((RCC->APB1ENR) &= ~(1 << 17))
#define UART3_PCLK_DI()             ((RCC->APB1ENR) &= ~(1 << 18))
#define UART4_PCLK_DI()             ((RCC->APB1ENR) &= ~(1 << 19))
#define UART5_PCLK_DI()             ((RCC->APB1ENR) &= ~(1 << 20))
#define UART6_PCLK_DI()             ((RCC->APB2ENR) &= ~(1 << 5))
#define UART7_PCLK_DI()             ((RCC->APB1ENR) &= ~(1 << 30))
#define UART8_PCLK_DI()             ((RCC->APB1ENR) &= ~(1 << 31))

#define I2C1_PCLK_DI()              ((RCC->APB1ENR) &= ~(1 << 21))
#define I2C2_PCLK_DI()              ((RCC->APB1ENR) &= ~(1 << 22))
#define I2C3_PCLK_DI()              ((RCC->APB1ENR) &= ~(1 << 23))

/****************************************************************************
 *                           Peripheral reset                               *
 ****************************************************************************/
#define GPIOA_REG_RESET()           do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()           do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()           do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()           do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()           do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()           do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()           do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()           do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()           do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)

#define SPI1_REG_RESET()            do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()            do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()            do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)
#define SPI4_REG_RESET()            do{ (RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13)); }while(0)
#define SPI5_REG_RESET()            do{ (RCC->APB2RSTR |= (1 << 20)); (RCC->APB2RSTR &= ~(1 << 20)); }while(0)
#define SPI6_REG_RESET()            do{ (RCC->APB2RSTR |= (1 << 21)); (RCC->APB2RSTR &= ~(1 << 21)); }while(0)

#define UART1_REG_RESET()           do{ (RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4)); }while(0)
#define UART2_REG_RESET()           do{ (RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 17)); }while(0)
#define UART3_REG_RESET()           do{ (RCC->APB1RSTR |= (1 << 18)); (RCC->APB1RSTR &= ~(1 << 18)); }while(0)
#define UART4_REG_RESET()           do{ (RCC->APB1RSTR |= (1 << 19)); (RCC->APB1RSTR &= ~(1 << 19)); }while(0)
#define UART5_REG_RESET()           do{ (RCC->APB1RSTR |= (1 << 20)); (RCC->APB1RSTR &= ~(1 << 20)); }while(0)
#define UART6_REG_RESET()           do{ (RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~(1 << 5)); }while(0)
#define UART7_REG_RESET()           do{ (RCC->APB1RSTR |= (1 << 30)); (RCC->APB1RSTR &= ~(1 << 30)); }while(0)
#define UART8_REG_RESET()           do{ (RCC->APB1RSTR |= (1 << 31)); (RCC->APB1RSTR &= ~(1 << 31)); }while(0)

#define I2C1_REG_RESET()            do{ (RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21)); }while(0)
#define I2C2_REG_RESET()            do{ (RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22)); }while(0)
#define I2C3_REG_RESET()            do{ (RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23)); }while(0)

/****************************************************************************
 *                              IRQ number                                  *
 ****************************************************************************/
#define IRQ_NO_EXTI0                6
#define IRQ_NO_EXTI1                7
#define IRQ_NO_EXTI2                8
#define IRQ_NO_EXTI3                9
#define IRQ_NO_EXTI4                10
#define IRQ_NO_EXTI9_5              23
#define IRQ_NO_EXTI15_10            40

#define IRQ_NO_SPI1                 35
#define IRQ_NO_SPI2                 36
#define IRQ_NO_SPI3                 51

#define IRQ_NO_UART1                37
#define IRQ_NO_UART2                38
#define IRQ_NO_UART3                39
#define IRQ_NO_UART4                52
#define IRQ_NO_UART5                53
#define IRQ_NO_UART6                71
#define IRQ_NO_UART7                82
#define IRQ_NO_UART8                83

#define IRQ_NO_I2C1_EV              31
#define IRQ_NO_I2C1_ER              32
#define IRQ_NO_I2C2_EV              33
#define IRQ_NO_I2C2_ER              34
#define IRQ_NO_I2C3_EV              72
#define IRQ_NO_I2C3_ER              73

/****************************************************************************
 *                           Generic macros                                 *
 ****************************************************************************/
#define RESET                       0
#define SET                         1
#define DISABLE                     RESET
#define ENABLE                      SET
#define FLAG_SET                    SET
#define FLAG_RESET                  RESET

/****************************************************************************
 *                           Include drivers                                *
 ****************************************************************************/
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_uart_driver.h"
#include "stm32f407xx_rcc_driver.h"
#include "stm32f407xx_i2c_driver.h"

#endif /* INC_STM32F407XX_H_ */

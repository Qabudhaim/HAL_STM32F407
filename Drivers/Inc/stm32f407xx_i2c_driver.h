/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: Nov 28, 2021
 *      Author: obscure
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
    uint32_t i2cSclSpeed;
    uint8_t i2cDeviceAddress;
    uint8_t i2cACKControl;
    uint16_t i2cFMDutyCycle;
} i2c_conf_t;

typedef struct
{
    i2c_reg_t * pI2Cx;
    i2c_conf_t i2cConf;
    uint8_t TxRxState;
    uint8_t * pTxBuffer;
    uint8_t * pRxBuffer;
    uint32_t TxLen;
    uint32_t RxLen;
    uint32_t RxSize;
    uint8_t DevAddr;
    uint8_t Sr;

} i2c_handle_t;

/****************************************************************************
 *                       Peripheral user configuration                      *
 ****************************************************************************/

/**
 * @i2cSclSpeed
 * Possible options for i2cSclSpeed
 */
#define I2C_SCL_SPEED_SM            100000
#define I2C_SCL_SPEED_FM4K          200000
#define I2C_SCL_SPEED_FM2k          400000

/**
 * @i2cACKControl
 * Possible options for i2cACKControl
 */
#define I2C_ACK_DI                  0
#define I2C_ACK_EN                  1

/**
 * @i2cFMDutyCycle
 * Possible options for i2cFMDutyCycle
 */
#define I2C_FM_DUTY_2               0
#define I2C_FM_DUTY_16_9            1

/**
 * Repeated start
 */
#define I2C_SR_DI                   DISABLE
#define I2C_SR_EN                   ENABLE

/****************************************************************************
 *                           i2c registers bits                             *
 ****************************************************************************/

/**
 * I2C CR1 bits
 */
#define I2C_CR1_PE                  0
#define I2C_CR1_SMBUS               1
#define I2C_CR1_SMBTYPE             3
#define I2C_CR1_ENARP               4
#define I2C_CR1_ENPEC               5
#define I2C_CR1_ENGC                6
#define I2C_CR1_NOSTRETCH           7
#define I2C_CR1_START               8
#define I2C_CR1_STOP                9
#define I2C_CR1_ACK                 10
#define I2C_CR1_POS                 11
#define I2C_CR1_PEC                 12
#define I2C_CR1_ALERT               13
#define I2C_CR1_SWRST               15

/**
 * I2C CR2 bits
 */
#define I2C_CR2_FREQ                0
#define I2C_CR2_ITERREN             8
#define I2C_CR2_ITEVTEN             9
#define I2C_CR2_ITBUFEN             10
#define I2C_CR2_DMAEN               11
#define I2C_CR2_LAST                12

/**
 * I2C SR1 bits
 */
#define I2C_SR1_SB                  0
#define I2C_SR1_ADDR                1
#define I2C_SR1_BTF                 2
#define I2C_SR1_ADD10               3
#define I2C_SR1_STOPF               4
#define I2C_SR1_RXNE                6
#define I2C_SR1_TXE                 7
#define I2C_SR1_BERR                8
#define I2C_SR1_ARLO                9
#define I2C_SR1_AF                  10
#define I2C_SR1_OVR                 11
#define I2C_SR1_PECERR              12
#define I2C_SR1_TIMEOUT             14
#define I2C_SR1_SMBALERT            15

/**
 * I2C SR2 bits
 */
#define I2C_SR2_MSL                 0
#define I2C_SR2_BUSY                1
#define I2C_SR2_TRA                 2
#define I2C_SR2_GENCALL             4
#define I2C_SR2_SMBDEFAULT          5
#define I2C_SR2_SMBHOST             6
#define I2C_SR2_DUALF               7
#define I2C_SR2_PEC                 8

/**
 * I2C CCR bits
 */
#define I2C_CCR_CCR                 0
#define I2C_CCR_DUTY                14
#define I2C_CCR_FS                  15

/**
 * I2C TRISE bits
 */
#define I2C_TRISE_TRISE             0

/**
 * I2C OAR1 bits
 */
#define I2C_OAR1_ADD_10             0
#define I2C_OAR1_ADD_7              1
#define I2C_OAR1_BIT_14             14
#define I2C_OAR1_ADDMODE            15

/**
 * I2C Status flags definitions
 */
#define I2C_FLAG_SB                 (1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR               (1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF                (1 << I2C_SR1_BTF)
#define I2C_FLAG_ADD10              (1 << I2C_SR1_ADD10)
#define I2C_FLAG_STOPF              (1 << I2C_SR1_STOPF)
#define I2C_FLAG_RXNE               (1 << I2C_SR1_RXNE)
#define I2C_FLAG_TXE                (1 << I2C_SR1_TXE)
#define I2C_FLAG_BERR               (1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO               (1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF                 (1 << I2C_SR1_AF)
#define I2C_FLAG_OVR                (1 << I2C_SR1_OVR)
#define I2C_FLAG_PECERR             (1 << I2C_SR1_PECERR)
#define I2C_FLAG_TIMEOUT            (1 << I2C_SR1_TIMEOUT)
#define I2C_FLAG_SMBALERT           (1 << I2C_SR1_SMBALERT)

/**
 *
 * I2C Application states
 */
#define I2C_READY                   0
#define I2C_BUSY_IN_TX              1
#define I2C_BUSY_IN_RX              2

/**
 *
 * I2C application event macros
 */
#define I2C_EV_TX_CMPLT             0
#define I2C_EV_RX_CMPLT             1
#define I2C_EV_STOP                 2
#define I2C_EV_DATA_REQ             3
#define I2C_EV_DATA_RCV             4
#define I2C_ERR_BERR                5
#define I2C_ERR_ARLO                6
#define I2C_ERR_AF                  7
#define I2C_ERR_OVR                 8
#define I2C_ERR_TIMEOUT             9

/****************************************************************************
 *                       APIs supported by this driver                      *
 ****************************************************************************/
void i2c_pclk_control (i2c_reg_t * pI2Cx, uint8_t ENorDI);

void i2c_init (i2c_handle_t * pI2CHandle);
void i2c_deinit (i2c_reg_t * pI2Cx);

void i2c_master_receive (i2c_handle_t * pI2CHandle, uint8_t * pRxBuffer,
        uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void i2c_master_transmit (i2c_handle_t * pI2CHandle, uint8_t * pTxBuffer,
        uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

uint8_t i2c_slave_receive (i2c_reg_t * pI2Cx);
void i2c_slave_transmit (i2c_reg_t * pI2Cx, uint8_t data);

uint8_t i2c_master_receive_interrupt (i2c_handle_t * pI2CHandle,
        uint8_t * pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t i2c_master_transmit_interrupt (i2c_handle_t * pI2CHandle,
        uint8_t * pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

void i2c_close_transmission (i2c_handle_t * pI2CHandle);
void i2c_close_reception (i2c_handle_t * pI2CHandle);

void i2c_irq_config (uint8_t IRQ_No, uint8_t ENorDI);
void i2c_irq_priority (uint8_t IRQ_No, uint8_t IRQ_Priority);
void i2c_event_irq_handling (i2c_handle_t * pI2CHandle);
void i2c_error_irq_handling (i2c_handle_t * pI2CHandle);

uint8_t i2c_get_status_flag (i2c_reg_t * pI2Cx, uint32_t flagName);
void i2c_peripheral_control (i2c_reg_t * pI2Cx, uint8_t ENorDI);
void i2c_manage_acking (i2c_reg_t * pI2Cx, uint8_t ENorDI);
void i2c_enable_disable_slave (i2c_reg_t * pI2Cx, uint8_t ENorDI);
void i2c_generate_stop_condition (i2c_reg_t * pI2Cx);

__weak void i2c_application_event_callback (i2c_handle_t * pI2CHandle,
        uint8_t AppEv);

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */

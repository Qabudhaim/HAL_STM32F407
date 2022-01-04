/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Nov 15, 2021
 *      Author: obscure
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

//Note that the code for 16-bit data frame is buggy for odd number of characters
//and it is recommended to not use it

/****************************************************************************
 *                       Peripheral handle structures                       *
 ****************************************************************************/
typedef struct
{
    uint8_t deviceMode;
    uint8_t busConfig;
    uint8_t spiDFF;
    uint8_t spiSSM;
    uint8_t spiSpeed;
    uint8_t spiCPOL;
    uint8_t spiCPHA;
} spi_conf_t;

typedef struct
{
    spi_reg_t * pSPIx;
    spi_conf_t SPIConf;
    uint8_t * pTxBuffer;
    uint8_t * pRxBuffer;
    uint32_t TxLen;
    uint32_t RxLen;
    uint8_t TxState;
    uint8_t RxState;
} spi_handle_t;

/****************************************************************************
 *                       Peripheral user configuration                      *
 ****************************************************************************/

/**
 * @deviceMode
 * Master/Slave mode
 */
#define SPI_MODE_SLAVE              0
#define SPI_MODE_MASTER             1

/**
 * @busConfig
 * Half-/Full-Duplex, RX only
 */
#define SPI_BUS_FULL_DUPLEX         0
#define SPI_BUS_HALF_DUPLEX         1
#define SPI_BUS_SIMPLEX             2

/**
 * @spiDFF
 * 8-/16-bit data frame
 */
#define SPI_DFF_8_BIT               0
#define SPI_DFF_16_BIT              1

/**
 * @spiSSM
 * Software slave management
 */
#define SPI_SSM_DI                  0
#define SPI_SSM_EN                  1

/**
 * @spiSpeed
 * SPI baud rate
 */
#define SPI_SPEED_DIV_2             0
#define SPI_SPEED_DIV_4             1
#define SPI_SPEED_DIV_8             2
#define SPI_SPEED_DIV_16            3
#define SPI_SPEED_DIV_32            4
#define SPI_SPEED_DIV_64            5
#define SPI_SPEED_DIV_128           6
#define SPI_SPEED_DIV_256           7

/**
 * @spiCPOL
 * SPI clock polarity
 */
#define SPI_CPOL_HIGH               1
#define SPI_CPOL_LOW                0

/**
 * @spiCPHA
 * SPI clock phase
 */
#define SPI_CPHA_HIGH               1
#define SPI_CPHA_LOW                0

/****************************************************************************
 *                           SPI registers bits                             *
 ****************************************************************************/
/**
 * SPI CR1 bits
 */
#define SPI_CR1_CPHA                0
#define SPI_CR1_CPOL                1
#define SPI_CR1_MSTR                2
#define SPI_CR1_BR                  3
#define SPI_CR1_SPE                 6
#define SPI_CR1_LSBFIRST            7
#define SPI_CR1_SSI                 8
#define SPI_CR1_SSM                 9
#define SPI_CR1_RXONLY              10
#define SPI_CR1_DFF                 11
#define SPI_CR1_CRCNEXT             12
#define SPI_CR1_CRCEN               13
#define SPI_CR1_BIDIOE              14
#define SPI_CR1_BIDIMODE            15

/**
 * SPI CR2 bits
 */
#define SPI_CR2_RXDMAEN             0
#define SPI_CR2_TXDMAEN             1
#define SPI_CR2_SSOE                2
#define SPI_CR2_FRF                 4
#define SPI_CR2_ERRIE               5
#define SPI_CR2_RXNEIE              6
#define SPI_CR2_TXEIE               7

/**
 * SPI SR bits
 */
#define SPI_SR_RXNE                 0
#define SPI_SR_TXE                  1
#define SPI_SR_CHSIDE               2
#define SPI_SR_UDR                  3
#define SPI_SR_CRCERR               4
#define SPI_SR_MODF                 5
#define SPI_SR_OVR                  6
#define SPI_SR_BSY                  7
#define SPI_SR_FRE                  8

/**
 * SPI Status flags definitions
 */
#define SPI_FLAG_RXNE               (1 << SPI_SR_RXNE)
#define SPI_FLAG_TXE                (1 << SPI_SR_TXE)
#define SPI_FLAG_CHSIDE             (1 << SPI_SR_CHSIDE)
#define SPI_FLAG_UDR                (1 << SPI_SR_UDR)
#define SPI_FLAG_CRCERR             (1 << SPI_SR_CRCERR)
#define SPI_FLAG_MODF               (1 << SPI_SR_MODF)
#define SPI_FLAG_OVR                (1 << SPI_SR_OVR)
#define SPI_FLAG_BSY                (1 << SPI_SR_BSY)
#define SPI_FLAG_FRE                (1 << SPI_SR_FRE)

/**
 *
 * SPI application states
 */
#define SPI_BUSY_IN_RX              0
#define SPI_BUSY_IN_TX              1
#define SPI_READY                   2

/**
 *
 * SPI application events
 */
#define SPI_EVENT_TX_CMPLT          1
#define SPI_EVENT_RX_CMPLT          2
#define SPI_EVENT_OVR_ERR           3
#define SPI_EVENT_CRC_ERR           4

/****************************************************************************
 *                           Generic macros                                 *
 ****************************************************************************/

/****************************************************************************
 *                       APIs supported by this driver                      *
 ****************************************************************************/
void spi_pclk_control (spi_reg_t * pSPIx, uint8_t ENorDI);

void spi_init (spi_handle_t * pSPIHandle);
void spi_deinit (spi_reg_t * pSPIx);

void spi_receive (spi_reg_t * pSPIx, uint8_t * pRxBuffer, uint32_t Len);
void spi_transmit (spi_reg_t * pSPIx, uint8_t * pTxBuffer, uint32_t Len);
void spi_interrupt_receive (spi_handle_t * pSPIHandle, uint8_t * pRxBuffer,
        uint32_t Len);
void spi_interrupt_transmit (spi_handle_t * pSPIHandle, uint8_t * pTxBuffer,
        uint32_t Len);

void spi_irq_config (uint8_t IRQ_No, uint8_t ENorDI);
void spi_irq_priority (uint8_t IRQ_No, uint8_t IRQ_Priority);
void spi_irq_handler (spi_handle_t * pSPIHandle);

uint8_t spi_get_status_flag (spi_reg_t * pSPIx, uint32_t flagName);
void spi_peripheral_control (spi_reg_t * pSPIx, uint8_t ENorDI);
void spi_ssi_control (spi_reg_t * pSPIx, uint8_t ENorDI);
void spi_ssoe_control (spi_reg_t * pSPIx, uint8_t ENorDI);

void spi_close_reception (spi_handle_t * pSPIHandle);
void spi_close_transmission (spi_handle_t * pSPIHandle);
void spi_clear_ovr_flag (spi_reg_t * pSPIx);

__weak void spi_application_event_callback (spi_handle_t * pSPIHandle,
        uint8_t AppEv);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */

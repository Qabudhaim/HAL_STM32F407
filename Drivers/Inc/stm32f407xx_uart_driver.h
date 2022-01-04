/*
 * stm32f407xx_UART_driver.h
 *
 *  Created on: Nov 21, 2021
 *      Author: obscure
 */

#ifndef INC_STM32F407XX_UART_DRIVER_H_
#define INC_STM32F407XX_UART_DRIVER_H_

#include "stm32f407xx.h"

/****************************************************************************
 *                       Peripheral handle structures                       *
 ****************************************************************************/
typedef struct
{
    uint8_t uartMode;
    uint32_t uartBaudRate;
    uint8_t uartStopBits;
    uint8_t uartWordLength;
    uint8_t uartParityControl;
    uint8_t uartHWFlowControl;
} uart_conf_t;

typedef struct
{
    uart_reg_t * pUARTx;
    uart_conf_t UARTConf;
    uint8_t * pTxBuffer;
    uint8_t * pRxBuffer;
    uint32_t TxLen;
    uint32_t RxLen;
    uint8_t TxState;
    uint8_t RxState;
} uart_handle_t;

/****************************************************************************
 *                       Peripheral user configuration                      *
 ****************************************************************************/

/**
 * @uartMode
 * Possible options for uartMode
 */
#define UART_MODE_ONLY_TX           0
#define UART_MODE_ONLY_RX           1
#define UART_MODE_TXRX              2

/**
 * @uartBaudRate
 * Possible options for USART_Baud
 */
#define UART_BAUD_RATE_1200         1200
#define UART_BAUD_RATE_2400         2400
#define UART_BAUD_RATE_9600         9600
#define UART_BAUD_RATE_19200        19200
#define UART_BAUD_RATE_38400        38400
#define UART_BAUD_RATE_57600        57600
#define UART_BAUD_RATE_115200       115200
#define UART_BAUD_RATE_230400       230400
#define UART_BAUD_RATE_460800       460800
#define UART_BAUD_RATE_921600       921600
#define UART_BAUD_RATE_2M           2000000
#define UART_BAUD_RATE_3M           3000000

/**
 * @uartStopBits
 * Possible options for uartStopBits
 */
#define UART_STOP_BITS_1            0
#define UART_STOP_BITS_0_5          1
#define UART_STOP_BITS_2            2
#define UART_STOP_BITS_1_5          3

/**
 * @uartWordLength
 * Possible options for uartWordLength
 */
#define UART_WORD_8_BITS            0
#define UART_WORD_9_BITS            1

/**
 * @uartParityControl
 * Possible options for uartParityControl
 */
#define UART_PARITY_DI              0
#define UART_PARITY_EN_ODD          1
#define UART_PARITY_EN_EVEN         2

/**
 * @uartHWFlowControl
 * Possible options for uartHWFlowControl
 */
#define UART_HW_FLOW_CTRL_NONE      0
#define UART_HW_FLOW_CTRL_CTS       1
#define UART_HW_FLOW_CTRL_RTS       2
#define UART_HW_FLOW_CTRL_CTS_RTS   3

/****************************************************************************
 *                           UART registers bits                             *
 ****************************************************************************/

/**
 * UART SR bits
 */
#define UART_SR_PE                  0
#define UART_SR_FE                  1
#define UART_SR_NF                  2
#define UART_SR_ORE                 3
#define UART_SR_IDLE                4
#define UART_SR_RXNE                5
#define UART_SR_TC                  6
#define UART_SR_TXE                 7
#define UART_SR_LBD                 8
#define UART_SR_CTS                 9

/**
 * UART SR bits
 */
#define UART_BRR_DIV_FR             0
#define UART_BRR_DIV_MNTSA          4

/**
 * UART CR1 bits
 */
#define UART_CR1_SBK                0
#define UART_CR1_RWU                1
#define UART_CR1_RE                 2
#define UART_CR1_TE                 3
#define UART_CR1_IDLEIE             4
#define UART_CR1_RXNEIE             5
#define UART_CR1_TCIE               6
#define UART_CR1_TXEIE              7
#define UART_CR1_PEIE               8
#define UART_CR1_PS                 9
#define UART_CR1_PCE                10
#define UART_CR1_WAKE               11
#define UART_CR1_M                  12
#define UART_CR1_UE                 13
#define UART_CR1_OVER8              15

/**
 * UART CR2 bits
 */
#define UART_CR2_ADD                0
#define UART_CR2_LBDL               5
#define UART_CR2_LBDIE              6
#define UART_CR2_LBCL               8
#define UART_CR2_CPHA               9
#define UART_CR2_CPOL               10
#define UART_CR2_CLKEN              11
#define UART_CR2_STOP               12
#define UART_CR2_LINEN              14

/**
 * UART CR3 bits
 */
#define UART_CR3_EIE                0
#define UART_CR3_IREN               1
#define UART_CR3_IRLP               2
#define UART_CR3_HDSEL              3
#define UART_CR3_NACK               4
#define UART_CR3_SCEN               5
#define UART_CR3_DMAR               6
#define UART_CR3_DMAT               7
#define UART_CR3_RTSE               8
#define UART_CR3_CTSE               9
#define UART_CR3_CTSIE              10
#define UART_CR3_ONEBIT             11

/**
 * UART Status flags definitions
 */
#define UART_FLAG_PE                (1 << UART_SR_PE)
#define UART_FLAG_FE                (1 << UART_SR_FE)
#define UART_FLAG_NF                (1 << UART_SR_NF)
#define UART_FLAG_ORE               (1 << UART_SR_ORE)
#define UART_FLAG_IDLE              (1 << UART_SR_IDLE)
#define UART_FLAG_RXNE              (1 << UART_SR_RXNE)
#define UART_FLAG_TC                (1 << UART_SR_TC)
#define UART_FLAG_TXE               (1 << UART_SR_TXE)
#define UART_FLAG_LBD               (1 << UART_SR_LBD)
#define UART_FLAG_CTS               (1 << UART_SR_CTS)

/**
 *
 * UART application states
 */
#define UART_READY                  0
#define UART_BUSY_IN_RX             1
#define UART_BUSY_IN_TX             2

/**
 *
 * UART application events
 */
#define UART_EVENT_TX_CMPLT         0
#define UART_EVENT_RX_CMPLT         1
#define UART_EVENT_IDLE             2
#define UART_EVENT_CTS              3
#define UART_EVENT_PE               4
#define UART_ERR_FE                 5
#define UART_ERR_NF                 6
#define UART_ERR_ORE                7

/****************************************************************************
 *                           Generic macros                                 *
 ****************************************************************************/

/****************************************************************************
 *                       APIs supported by this driver                      *
 ****************************************************************************/
void uart_pclk_control (uart_reg_t * pUARTx, uint8_t ENorDI);

void uart_init (uart_handle_t * pUARTHandle);
void uart_deinit (uart_reg_t * pUARTx);

void uart_receive (uart_handle_t * pUARTHandle, uint8_t * pRxBuffer,
        uint32_t Len);
void uart_transmit (uart_handle_t * pUARTHandle, uint8_t * pTxBuffer,
        uint32_t Len);
uint8_t uart_interrupt_receive (uart_handle_t * pUARTHandle,
        uint8_t * pRxBuffer, uint32_t Len);
uint8_t uart_interrupt_transmit (uart_handle_t * pUARTHandle,
        uint8_t * pTxBuffer, uint32_t Len);

void uart_irq_config (uint8_t IRQ_No, uint8_t ENorDI);
void uart_irq_priority (uint8_t IRQ_No, uint8_t IRQ_Priority);
void uart_irq_handler (uart_handle_t * pUARTHandle);

uint8_t uart_get_status_flag (uart_reg_t * pUARTx, uint32_t flagName);
void uart_peripheral_control (uart_reg_t * pUARTx, uint8_t ENorDI);
void uart_ssi_control (uart_reg_t * pUARTx, uint8_t ENorDI);
void uart_ssoe_control (uart_reg_t * pUARTx, uint8_t ENorDI);

void uart_close_reception (uart_handle_t * pUARTHandle);
void uart_close_transmission (uart_handle_t * pUARTHandle);
void uart_clear_ovr_flag (uart_reg_t * pUARTx);

__weak void uart_application_event_callback (uart_handle_t * pUARTHandle,
        uint8_t AppEv);

#endif /* INC_STM32F407XX_UART_DRIVER_H_ */

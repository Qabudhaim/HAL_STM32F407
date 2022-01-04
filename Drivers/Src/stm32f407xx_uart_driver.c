/*
 * stm32f407xx_uart_driver.c
 *
 *  Created on: Nov 21, 2021
 *      Author: obscure
 */

#include "stm32f407xx_uart_driver.h"

static void uart_set_baudrate (uart_reg_t * pUARTx, uint32_t baudrate);

void uart_pclk_control (uart_reg_t * pUARTx, uint8_t ENorDI)
{
    if (ENorDI == ENABLE)
    {
        if (pUARTx == UART1)
        {
            UART1_PCLK_EN();
        }
        else if (pUARTx == UART2)
        {
            UART2_PCLK_EN();
        }
        else if (pUARTx == UART3)
        {
            UART3_PCLK_EN();
        }
        else if (pUARTx == UART4)
        {
            UART4_PCLK_EN();
        }
        else if (pUARTx == UART5)
        {
            UART5_PCLK_EN();
        }
        else if (pUARTx == UART6)
        {
            UART6_PCLK_EN();
        }
    }
    else
    {
        if (pUARTx == UART1)
        {
            UART1_PCLK_DI();
        }
        else if (pUARTx == UART2)
        {
            UART2_PCLK_DI();
        }
        else if (pUARTx == UART3)
        {
            UART3_PCLK_DI();
        }
        else if (pUARTx == UART4)
        {
            UART4_PCLK_DI();
        }
        else if (pUARTx == UART5)
        {
            UART5_PCLK_DI();
        }
        else if (pUARTx == UART6)
        {
            UART6_PCLK_DI();
        }
    }
}

void uart_init (uart_handle_t * pUARTHandle)
{
    //1. Enable the peripheral
    uart_pclk_control(pUARTHandle->pUARTx, ENABLE);

    uint32_t tmp = 0;

    // Configure CR1
    //2. Configure peripheral mode
    if (pUARTHandle->UARTConf.uartMode == UART_MODE_ONLY_TX)
    {
        tmp |= (1 << UART_CR1_TE);
        tmp &= ~(1 << UART_CR1_RE);
    }
    else if (pUARTHandle->UARTConf.uartMode == UART_MODE_ONLY_RX)
    {
        tmp &= ~(1 << UART_CR1_TE);
        tmp |= (1 << UART_CR1_RE);
    }
    else if (pUARTHandle->UARTConf.uartMode == UART_MODE_TXRX)
    {
        tmp |= (1 << UART_CR1_RE);
        tmp |= (1 << UART_CR1_TE);
    }

    //3. Configure parity bit
    if (pUARTHandle->UARTConf.uartParityControl == UART_PARITY_DI)
    {
        tmp &= ~(1 << UART_CR1_PCE);
    }
    else if (pUARTHandle->UARTConf.uartParityControl == UART_PARITY_EN_ODD)
    {
        tmp |= (1 << UART_CR1_PCE);
        tmp |= (1 << UART_CR1_PS);
    }
    else if (pUARTHandle->UARTConf.uartParityControl == UART_PARITY_EN_EVEN)
    {
        tmp |= (1 << UART_CR1_PCE);
        tmp &= ~(1 << UART_CR1_PS);
    }

    //4. Configure word length
    tmp |= (pUARTHandle->UARTConf.uartWordLength << UART_CR1_M);

    pUARTHandle->pUARTx->CR1 = tmp;

    //Configure CR2
    tmp = 0;

    //5. Configure stop bits
    tmp = (pUARTHandle->UARTConf.uartStopBits << UART_CR2_STOP);

    pUARTHandle->pUARTx->CR2 = tmp;

    //Configure CR3
    tmp = 0;

    if (pUARTHandle->UARTConf.uartHWFlowControl == UART_HW_FLOW_CTRL_CTS)
    {
        tmp |= (1 << UART_CR3_CTSE);
    }
    else if (pUARTHandle->UARTConf.uartHWFlowControl == UART_HW_FLOW_CTRL_RTS)
    {
        tmp |= (1 << UART_CR3_RTSE);
    }
    else if (pUARTHandle->UARTConf.uartHWFlowControl
            == UART_HW_FLOW_CTRL_CTS_RTS)
    {
        tmp |= (1 << UART_CR3_CTSE);
        tmp |= (1 << UART_CR3_RTSE);
    }

    pUARTHandle->pUARTx->CR3 = tmp;

    //Configure the Baud rate
    uart_set_baudrate(pUARTHandle->pUARTx, pUARTHandle->UARTConf.uartBaudRate);

}

static void uart_set_baudrate (uart_reg_t * pUARTx, uint32_t baudrate)
{
    uint32_t pclk = 0;
    if ((pUARTx == UART1) || (pUARTx == UART6))
    {
        pclk = rcc_get_pclk(RCC_SYSBUS_APB2);
    }
    else
    {
        pclk = rcc_get_pclk(RCC_SYSBUS_APB1);
    }

    uint32_t USARTDIV = 0;
    if (pUARTx->CR1 & (1 << UART_CR1_OVER8))
    {
        //Oversampling by 8
        USARTDIV = ((pclk * 25) / (2 * baudrate));
    }
    else
    {
        //Oversampling by 16
        USARTDIV = ((pclk * 25) / (4 * baudrate));
    }

    uint32_t tmp = 0;
    uint32_t mantissa;
    mantissa = USARTDIV / 100;
    tmp |= mantissa << UART_BRR_DIV_MNTSA;

    uint32_t fraction;
    fraction = USARTDIV - (mantissa * 100);

    if (pUARTx->CR1 & (1 << UART_CR1_OVER8))
    {
        //Oversampling by 8
        fraction = (((fraction * 8) + 50) / 100) & ((uint8_t) 0x07);  //Refer to refernce manual seciton 30.6.3 Baud rate register
    }
    else
    {
        //Oversampling by 16
        fraction = (((fraction * 16) + 50) / 100) & ((uint8_t) 0x0F);
    }

    tmp |= fraction << UART_BRR_DIV_FR;

    pUARTx->BRR = tmp;

}

void uart_deinit (uart_reg_t * pUARTx)
{
    if (pUARTx == UART1)
    {
        UART1_REG_RESET();
    }
    else if (pUARTx == UART2)
    {
        UART2_REG_RESET();
    }
    else if (pUARTx == UART3)
    {
        UART3_REG_RESET();
    }
    else if (pUARTx == UART4)
    {
        UART4_REG_RESET();
    }
    else if (pUARTx == UART5)
    {
        UART5_REG_RESET();
    }
    else if (pUARTx == UART6)
    {
        UART6_REG_RESET();
    }
}

void uart_receive (uart_handle_t * pUARTHandle, uint8_t * pRxBuffer,
        uint32_t Len)
{
    for (uint32_t i = 0; i < Len; i++)
    {
        while (!uart_get_status_flag(pUARTHandle->pUARTx, UART_FLAG_RXNE));
        if (pUARTHandle->UARTConf.uartWordLength == UART_WORD_9_BITS)
        {
            if (pUARTHandle->UARTConf.uartParityControl == UART_PARITY_DI)
            {
                *((uint16_t*) pRxBuffer) = pUARTHandle->pUARTx->DR;
                pRxBuffer++;
                pRxBuffer++;
            }
            else
            {
                *pRxBuffer = (pUARTHandle->pUARTx->DR & (uint8_t) 0xFF);
                pRxBuffer++;
            }
        }
        else
        {
            if (pUARTHandle->UARTConf.uartParityControl == UART_PARITY_DI)
            {
                *pRxBuffer = pUARTHandle->pUARTx->DR;
            }
            else
            {
                *pRxBuffer = pUARTHandle->pUARTx->DR & 0x7F;
            }
            pRxBuffer++;
        }
    }
    while (!uart_get_status_flag(pUARTHandle->pUARTx, UART_FLAG_TC));
}

void uart_transmit (uart_handle_t * pUARTHandle, uint8_t * pTxBuffer,
        uint32_t Len)
{
    uint16_t * pData;
    for (uint32_t i = 0; i < Len; i++)
    {
        while (!uart_get_status_flag(pUARTHandle->pUARTx, UART_FLAG_TXE));
        if (pUARTHandle->UARTConf.uartWordLength == UART_WORD_9_BITS)
        {

            pData = (uint16_t*) pTxBuffer;
            pUARTHandle->pUARTx->DR = (*pData & (uint16_t) 0x01FF);

            if (pUARTHandle->UARTConf.uartParityControl == UART_PARITY_DI)
            {
                pTxBuffer++;
                pTxBuffer++;
            }
            else
            {
                pTxBuffer++;
            }
        }
        else
        {
            pUARTHandle->pUARTx->DR = (*pTxBuffer & (uint8_t) 0xFF);
            pTxBuffer++;
        }
    }
    while (!uart_get_status_flag(pUARTHandle->pUARTx, UART_FLAG_TC));
}

uint8_t uart_interrupt_receive (uart_handle_t * pUARTHandle,
        uint8_t * pRxBuffer, uint32_t Len)
{
    uint8_t state = pUARTHandle->RxState;

    if (state != UART_BUSY_IN_RX)
    {
        pUARTHandle->RxLen = Len;
        pUARTHandle->pRxBuffer = pRxBuffer;
        pUARTHandle->RxState = UART_BUSY_IN_RX;

        //(void) pUARTHandle->pUARTx->DR;

        pUARTHandle->pUARTx->CR1 |= (1 << UART_CR1_RXNEIE);
    }
    return state;
}

uint8_t uart_interrupt_transmit (uart_handle_t * pUARTHandle,
        uint8_t * pTxBuffer, uint32_t Len)
{
    uint8_t state = pUARTHandle->TxState;

    if (state != UART_BUSY_IN_TX)
    {
        pUARTHandle->TxLen = Len;
        pUARTHandle->pTxBuffer = pTxBuffer;
        pUARTHandle->TxState = UART_BUSY_IN_TX;

        pUARTHandle->pUARTx->CR1 |= (1 << UART_CR1_TXEIE);

        pUARTHandle->pUARTx->CR1 |= (1 << UART_CR1_TCIE);
    }
    return state;
}

void uart_irq_config (uint8_t IRQ_No, uint8_t ENorDI)
{
    uint8_t tmp1 = IRQ_No / 32;
    uint8_t tmp2 = IRQ_No % 32;
    if (ENorDI == ENABLE)
    {

        NVIC->ISER[tmp1] |= (1 << tmp2);
    }
    else
    {
        NVIC->ICER[tmp1] |= (1 << tmp2);
    }
}
void uart_irq_priority (uint8_t IRQ_No, uint8_t IRQ_Priority)
{
    uint8_t tmp1 = IRQ_No / 4;
    uint8_t tmp2 = IRQ_No % 4;
    NVIC->IPR[tmp1] = (IRQ_Priority << ((tmp2 * 8) + 4));
}

void uart_irq_handler (uart_handle_t * pUARTHandle)
{

    uint32_t tmp1, tmp2, tmp3;

    /**
     * Check for TC flag
     */
    tmp1 = pUARTHandle->pUARTx->SR & (1 << UART_SR_TC);
    tmp2 = pUARTHandle->pUARTx->CR1 & (1 << UART_CR1_TCIE);

    if (tmp1 && tmp2)
    {
        if (pUARTHandle->TxState == UART_BUSY_IN_TX)
        {
            if (!pUARTHandle->TxLen)
            {
                pUARTHandle->pUARTx->SR &= ~(1 << UART_SR_TC);

                pUARTHandle->pUARTx->CR1 &= ~(1 << UART_CR1_TCIE);

                pUARTHandle->TxState = UART_READY;

                pUARTHandle->pTxBuffer = NULL;

                pUARTHandle->TxLen = 0;

                uart_application_event_callback(pUARTHandle,
                UART_EVENT_TX_CMPLT);

            }
        }
    }

    /**
     * Check for TXE flag
     */
    tmp1 = pUARTHandle->pUARTx->SR & (1 << UART_SR_TXE);
    tmp2 = pUARTHandle->pUARTx->CR1 & (1 << UART_CR1_TXEIE);

    if (tmp1 && tmp2)
    {
        if (pUARTHandle->TxState == UART_BUSY_IN_TX)
        {
            uint16_t * pData;
            while (pUARTHandle->TxLen > 0)
            {
                while (!uart_get_status_flag(pUARTHandle->pUARTx, UART_FLAG_TXE));
                if (pUARTHandle->UARTConf.uartWordLength == UART_WORD_9_BITS)
                {

                    pData = (uint16_t*) pUARTHandle->pTxBuffer;
                    pUARTHandle->pUARTx->DR = (*pData & (uint16_t) 0x01FF);

                    //pData = ((uint16_t*) pTxBuffer) & (0x1FF);
                    //pUARTHandle->pUARTx->DR = *pData;

                    if (pUARTHandle->UARTConf.uartParityControl
                            == UART_PARITY_DI)
                    {
                        pUARTHandle->pTxBuffer++;
                        pUARTHandle->pTxBuffer++;
                        pUARTHandle->TxLen -= 2;
                    }
                    else
                    {
                        pUARTHandle->pTxBuffer++;
                        pUARTHandle->TxLen -= 1;
                    }
                }
                else
                {
                    pUARTHandle->pUARTx->DR = (*pUARTHandle->pTxBuffer
                            & (uint8_t) 0xFF);
                    pUARTHandle->pTxBuffer++;
                    pUARTHandle->TxLen -= 1;
                }
            }
            if (pUARTHandle->TxLen == 0)
            {
                pUARTHandle->pUARTx->CR1 &= ~(1 << UART_CR1_TXEIE);
            }
        }

    }

    /**
     * Check for RXNE flag
     */
    tmp1 = pUARTHandle->pUARTx->SR & (1 << UART_SR_RXNE);
    tmp2 = pUARTHandle->pUARTx->CR1 & (1 << UART_CR1_RXNEIE);

    if (tmp1 && tmp2)
    {
        if (pUARTHandle->RxState == UART_BUSY_IN_RX)
        {
            while (pUARTHandle->RxLen > 0)
            {
                while (!uart_get_status_flag(pUARTHandle->pUARTx,
                UART_FLAG_RXNE));
                if (pUARTHandle->UARTConf.uartWordLength == UART_WORD_9_BITS)
                {
                    if (pUARTHandle->UARTConf.uartParityControl
                            == UART_PARITY_DI)
                    {
                        //This case is problematic, don't use it
                        *((uint16_t*) pUARTHandle->pRxBuffer) =
                                pUARTHandle->pUARTx->DR;
                        pUARTHandle->pRxBuffer++;
                        pUARTHandle->pRxBuffer++;
                        pUARTHandle->RxLen -= 2;
                    }
                    else
                    {
                        *pUARTHandle->pRxBuffer = (pUARTHandle->pUARTx->DR
                                & (uint8_t) 0xFF);
                        pUARTHandle->pRxBuffer++;
                        pUARTHandle->RxLen -= 1;
                    }
                }
                else
                {
                    if (pUARTHandle->UARTConf.uartParityControl
                            == UART_PARITY_DI)
                    {
                        *pUARTHandle->pRxBuffer = pUARTHandle->pUARTx->DR;
                    }
                    else
                    {
                        *pUARTHandle->pRxBuffer = pUARTHandle->pUARTx->DR
                                & 0x7F;
                    }
                    pUARTHandle->pRxBuffer++;
                    pUARTHandle->RxLen -= 1;
                }
            }
            if (pUARTHandle->RxLen == 0)
            {
                pUARTHandle->pUARTx->CR1 &= ~(1 << UART_CR1_RXNEIE);
                pUARTHandle->RxState = UART_READY;
                uart_application_event_callback(pUARTHandle,
                UART_EVENT_RX_CMPLT);
            }
        }

    }

    /**
     * Check for CTS flag
     * Note: CTS feature is not applicable for UART4 and UART5
     */
    tmp1 = pUARTHandle->pUARTx->SR & (1 << UART_SR_CTS);
    tmp2 = pUARTHandle->pUARTx->CR3 & (1 << UART_CR3_CTSE);
    tmp3 = pUARTHandle->pUARTx->CR3 & (1 << UART_CR3_CTSIE);

    if (tmp1 && tmp2)
    {
        pUARTHandle->pUARTx->SR &= ~(1 << UART_SR_CTS);
        uart_application_event_callback(pUARTHandle, UART_EVENT_CTS);
    }

    /**
     * Check for IDLE detection flag
     */
    tmp1 = pUARTHandle->pUARTx->SR & (1 << UART_SR_IDLE);
    tmp2 = pUARTHandle->pUARTx->CR1 & (1 << UART_CR1_IDLEIE);

    if (tmp1 && tmp2)
    {
        //Refer to RM for clear sequence Section 30.6.1
        pUARTHandle->pUARTx->SR &= ~(1 << UART_SR_IDLE);
        tmp3 = pUARTHandle->pUARTx->DR;
        uart_application_event_callback(pUARTHandle, UART_EVENT_IDLE);
    }
    (void) tmp3;    //To eliminate 'unused' warning

    /**
     * Check for Overrun detection flag
     */
    tmp1 = pUARTHandle->pUARTx->SR & (1 << UART_SR_IDLE);
    tmp2 = pUARTHandle->pUARTx->CR1 & (1 << UART_CR1_RXNEIE);

    if (tmp1 & tmp2)
    {
        uart_application_event_callback(pUARTHandle, UART_ERR_ORE);
    }

    /**
     * Check for Error flag
     * The code below will only get executed when multibuffer mode is used
     */
    tmp2 = pUARTHandle->pUARTx->CR3 & (1 << UART_CR3_EIE);
    if (tmp2)
    {
        tmp1 = pUARTHandle->pUARTx->SR;

        if (tmp1 & (1 << UART_SR_FE))
        {
            uart_application_event_callback(pUARTHandle, UART_ERR_FE);
        }

        if (tmp1 & (1 << UART_SR_NF))
        {
            uart_application_event_callback(pUARTHandle, UART_ERR_NF);
        }

        if (tmp1 & (1 << UART_SR_ORE))
        {
            uart_application_event_callback(pUARTHandle, UART_ERR_ORE);
        }

    }

}

uint8_t uart_get_status_flag (uart_reg_t * pUARTx, uint32_t flagName)
{
    if (pUARTx->SR & flagName)
    {
        return FLAG_SET;
    }
    else
    {
        return FLAG_RESET;
    }
}
void uart_peripheral_control (uart_reg_t * pUARTx, uint8_t ENorDI)
{
    if (ENorDI == ENABLE)
    {
        pUARTx->CR1 |= (1 << UART_CR1_UE);
    }
    else
    {
        pUARTx->CR1 &= ~(1 << UART_CR1_UE);
    }
}

__weak void uart_application_event_callback (uart_handle_t * pUARTHandle,
        uint8_t AppEv);

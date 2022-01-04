/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Nov 15, 2021
 *      Author: obscure
 */

#include "stm32f407xx_spi_driver.h"

static void spi_rx_interrupt_handler (spi_handle_t * pSPIHandle);
static void spi_tx_interrupt_handler (spi_handle_t * pSPIHandle);
static void spi_ovr_interrupt_handler (spi_handle_t * pSPIHandle);

/**
 *
 * @param pSPIx     SPI peripheral address
 * @param ENorDI    Enabling/Disabling the peripheral's clock
 */
void spi_pclk_control (spi_reg_t * pSPIx, uint8_t ENorDI)
{
    if (ENorDI == ENABLE)
    {
        if (pSPIx == SPI1)
        {
            SPI1_PCLK_EN();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLK_EN();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PCLK_EN();
        }
        else if (pSPIx == SPI4)
        {
            SPI4_PCLK_EN();
        }
        else if (pSPIx == SPI5)
        {
            SPI5_PCLK_EN();
        }
        else if (pSPIx == SPI6)
        {
            SPI6_PCLK_EN();
        }
    }
    else
    {
        if (pSPIx == SPI1)
        {
            SPI1_PCLK_DI();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLK_DI();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PCLK_DI();
        }
        else if (pSPIx == SPI4)
        {
            SPI4_PCLK_DI();
        }
        else if (pSPIx == SPI5)
        {
            SPI5_PCLK_DI();
        }
        else if (pSPIx == SPI6)
        {
            SPI6_PCLK_DI();
        }
    }
}

/**
 *
 * @param pSPIHandle    SPI peripheral address
 */
void spi_init (spi_handle_t * pSPIHandle)
{
    // Enable SPI clock
    spi_pclk_control(pSPIHandle->pSPIx, ENABLE);

    uint16_t tmp = 0;

    // 1. Configure Master/Slave mode
    tmp |= (pSPIHandle->SPIConf.deviceMode << SPI_CR1_MSTR);

    // 2. Configure Bus
    if (pSPIHandle->SPIConf.busConfig == SPI_BUS_FULL_DUPLEX)
    {
        tmp &= ~(1 << SPI_CR1_BIDIMODE);
    }
    else if (pSPIHandle->SPIConf.busConfig == SPI_BUS_HALF_DUPLEX)
    {
        tmp |= (1 << SPI_CR1_BIDIMODE);
    }
    else if (pSPIHandle->SPIConf.busConfig == SPI_BUS_SIMPLEX)
    {
        tmp &= ~(1 << SPI_CR1_BIDIMODE);
        tmp |= (1 << SPI_CR1_RXONLY);
    }

    // 3. Configure Data frame
    tmp |= (pSPIHandle->SPIConf.spiDFF << SPI_CR1_DFF);

    // 4. Configure software slave management
    tmp |= (pSPIHandle->SPIConf.spiSSM << SPI_CR1_SSM);

    // 5. Configure speed
    tmp |= (pSPIHandle->SPIConf.spiSpeed << SPI_CR1_BR);

    // 6. Configure CPOL
    tmp |= (pSPIHandle->SPIConf.spiCPOL << SPI_CR1_CPOL);

    // 7. Configure CPHA
    tmp |= (pSPIHandle->SPIConf.spiCPHA << SPI_CR1_CPHA);

    pSPIHandle->pSPIx->CR1 = tmp;
}

/**
 *
 * @param pSPIx SPI peripheral address
 */
void spi_deinit (spi_reg_t * pSPIx)
{
    if (pSPIx == SPI1)
    {
        SPI1_REG_RESET();
    }
    else if (pSPIx == SPI2)
    {
        SPI2_REG_RESET();
    }
    else if (pSPIx == SPI3)
    {
        SPI3_REG_RESET();
    }
    else if (pSPIx == SPI4)
    {
        SPI4_REG_RESET();
    }
    else if (pSPIx == SPI5)
    {
        SPI5_REG_RESET();
    }
    else if (pSPIx == SPI6)
    {
        SPI6_REG_RESET();
    }
}

void spi_receive (spi_reg_t * pSPIx, uint8_t * pRxBuffer, uint32_t Len)
{
    while (Len > 0)
    {
        while (!(spi_get_status_flag(pSPIx, SPI_FLAG_RXNE)));

        if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
        {
            *((uint16_t*) pRxBuffer) = pSPIx->DR;
            Len--;
            Len--;
            pRxBuffer += 2;
        }
        else
        {
            *(pRxBuffer) = pSPIx->DR;
            Len--;
            pRxBuffer++;
        }
    }
}

/**
 *
 * @param pSPIx SPI peripheral address
 * @param data  Data to be send
 */
void spi_transmit (spi_reg_t * pSPIx, uint8_t * pTxBuffer, uint32_t Len)
{
    while (Len > 0)
    {

        while (!(spi_get_status_flag(pSPIx, SPI_FLAG_TXE)));

        if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
        {

            uint16_t tmp = *((uint16_t*) pTxBuffer);
            tmp = (tmp << 8) | (tmp >> 8);  //Little endian, endian detection can be implemented later

            pSPIx->DR = tmp;
            Len--;
            Len--;
            pTxBuffer += 2;
        }
        else
        {
            pSPIx->DR = *(pTxBuffer);
            Len--;
            pTxBuffer++;
        }
    }
}

void spi_interrupt_receive (spi_handle_t * pSPIHandle, uint8_t * pRxBuffer,
        uint32_t Len)
{
    uint8_t state = pSPIHandle->RxState;

    if ((state != SPI_BUSY_IN_RX))
    {
        //1. Save the Rx buffer address and Len information in some global variables
        pSPIHandle->RxLen = Len;
        pSPIHandle->pRxBuffer = pRxBuffer;

        //2. Mark the SPI state as busy in reception so that no other code
        //  can take over same SPI peripheral until reception is over
        pSPIHandle->RxState = SPI_BUSY_IN_RX;

        //3. Enable the RXNEIE control bit to get interrupt whenever RXNEIE flag is set in SR
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

        //4. Enable SPI error handling
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_ERRIE);
    }
}

void spi_interrupt_transmit (spi_handle_t * pSPIHandle, uint8_t * pTxBuffer,
        uint32_t Len)
{
    uint8_t state = pSPIHandle->TxState;

    if ((state != SPI_BUSY_IN_TX))
    {
        //1. Save the Tx buffer address and Len information in some global variables
        pSPIHandle->TxLen = Len;
        pSPIHandle->pTxBuffer = pTxBuffer;

        //2. Mark the SPI state as busy in transmission so that no other code
        //  can take over same SPI peripheral until transmission is over
        pSPIHandle->TxState = SPI_BUSY_IN_TX;

        //3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

        //4. Enable SPI error handling
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_ERRIE);
    }
}

void spi_irq_config (uint8_t IRQ_No, uint8_t ENorDI)
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

void spi_irq_priority (uint8_t IRQ_No, uint8_t IRQ_Priority)
{
    uint8_t tmp1 = IRQ_No / 4;
    uint8_t tmp2 = IRQ_No % 4;
    NVIC->IPR[tmp1] = (IRQ_Priority << ((tmp2 * 8) + 4));
}

void spi_irq_handler (spi_handle_t * pSPIHandle)
{
    uint8_t tmp1, tmp2;

    tmp1 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
    tmp2 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);

    if (tmp1 && tmp2)
    {
        spi_rx_interrupt_handler(pSPIHandle);
    }

    tmp1 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
    tmp2 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);

    if (tmp1 && tmp2)
    {
        spi_tx_interrupt_handler(pSPIHandle);
    }

    tmp1 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
    tmp2 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);

    if (tmp1 && tmp2)
    {
        spi_ovr_interrupt_handler(pSPIHandle);
    }
}

uint8_t spi_get_status_flag (spi_reg_t * pSPIx, uint32_t flagName)
{
    if (pSPIx->SR & flagName)
    {
        return FLAG_SET;
    }
    else
    {
        return FLAG_RESET;
    }
}

void spi_peripheral_control (spi_reg_t * pSPIx, uint8_t ENorDI)
{
    if (ENorDI == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
    }
}

void spi_ssi_control (spi_reg_t * pSPIx, uint8_t ENorDI)
{
    if (ENorDI == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SSI);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
    }
}

void spi_ssoe_control (spi_reg_t * pSPIx, uint8_t ENorDI)
{
    if (ENorDI == ENABLE)
    {
        pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
    }
    else
    {
        pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
    }
}

static void spi_rx_interrupt_handler (spi_handle_t * pSPIHandle)
{
    if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
    {
        *((uint16_t*) pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLen--;
        pSPIHandle->RxLen--;
        pSPIHandle->pRxBuffer += 2;
    }
    else
    {
        *(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLen--;
        pSPIHandle->pRxBuffer++;
    }

    if (!pSPIHandle->RxLen)
    {
        spi_close_reception(pSPIHandle);
        spi_application_event_callback(pSPIHandle, SPI_EVENT_RX_CMPLT);
    }
}

static void spi_tx_interrupt_handler (spi_handle_t * pSPIHandle)
{
    if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
    {
        uint16_t tmp = *((uint16_t*) pSPIHandle->pTxBuffer);
        tmp = (tmp << 8) | (tmp >> 8);  //Little endian, endian detection can be implemented later

        pSPIHandle->pSPIx->DR = tmp;
        pSPIHandle->TxLen--;
        pSPIHandle->TxLen--;
        pSPIHandle->pTxBuffer += 2;
    }
    else
    {
        pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
        pSPIHandle->TxLen--;
        pSPIHandle->pTxBuffer++;
    }

    if (!pSPIHandle->TxLen)
    {
        spi_close_transmission(pSPIHandle);
        spi_application_event_callback(pSPIHandle, SPI_EVENT_TX_CMPLT);
    }
}

static void spi_ovr_interrupt_handler (spi_handle_t * pSPIHandle)
{
    uint8_t tmp;
    tmp = pSPIHandle->pSPIx->DR;
    tmp = pSPIHandle->pSPIx->DR;
    (void) tmp;
    spi_application_event_callback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void spi_close_reception (spi_handle_t * pSPIHandle)
{
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
    pSPIHandle->RxLen = 0;
    pSPIHandle->RxState = SPI_READY;
    pSPIHandle->pRxBuffer = NULL;

}

void spi_close_transmission (spi_handle_t * pSPIHandle)
{
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
    pSPIHandle->TxLen = 0;
    pSPIHandle->TxState = SPI_READY;
    pSPIHandle->pTxBuffer = NULL;
}

void spi_clear_ovr_flag (spi_reg_t * pSPIx)
{
    uint8_t tmp;
    tmp = pSPIx->DR;
    tmp = pSPIx->DR;
    (void) tmp;
}

__weak void spi_application_event_callback (spi_handle_t * pSPIHandle,
        uint8_t AppEv)
{

    //This is a weak implementation. The application may override this function.
}

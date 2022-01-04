/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Nov 28, 2021
 *      Author: obscure
 */

#include "stm32f407xx_i2c_driver.h"

static void i2c_clear_addr_flag (i2c_handle_t * pI2CHandle);
static void i2c_generate_start_condition (i2c_reg_t * pI2Cx);
static void i2c_address_write (i2c_reg_t * pI2Cx, uint8_t SlaveAddr);
static void i2c_address_read (i2c_reg_t * pI2Cx, uint8_t SlaveAddr);
static void i2c_master_handle_txe_interrupt (i2c_handle_t * pI2CHandle);
static void i2c_master_handle_rxne_interrupt (i2c_handle_t * pI2CHandle);

static void i2c_clear_addr_flag (i2c_handle_t * pI2CHandle)
{
    if (pI2CHandle->pI2Cx->SR2 == I2C_SR2_MSL)
    {
        if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
        {
            if (pI2CHandle->RxSize == 1)
            {
                i2c_manage_acking(pI2CHandle->pI2Cx, DISABLE);

                uint32_t dummy_read;
                dummy_read = pI2CHandle->pI2Cx->SR1;
                dummy_read = pI2CHandle->pI2Cx->SR2;
                (void) dummy_read;
            }
        }
        else
        {
            uint32_t dummy_read;
            dummy_read = pI2CHandle->pI2Cx->SR1;
            dummy_read = pI2CHandle->pI2Cx->SR2;
            (void) dummy_read;
        }
    }
    else
    {
        uint32_t dummy_read;
        dummy_read = pI2CHandle->pI2Cx->SR1;
        dummy_read = pI2CHandle->pI2Cx->SR2;
        (void) dummy_read;
    }

}

static void i2c_generate_start_condition (i2c_reg_t * pI2Cx)
{
    pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

void i2c_generate_stop_condition (i2c_reg_t * pI2Cx)
{
    pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

static void i2c_address_write (i2c_reg_t * pI2Cx, uint8_t SlaveAddr)
{
    SlaveAddr = SlaveAddr << 1;
    SlaveAddr &= ~(1);
    pI2Cx->DR = SlaveAddr;
}

static void i2c_address_read (i2c_reg_t * pI2Cx, uint8_t SlaveAddr)
{
    SlaveAddr = SlaveAddr << 1;
    SlaveAddr |= 1;
    pI2Cx->DR = SlaveAddr;
}

void i2c_enable_disable_slave (i2c_reg_t * pI2Cx, uint8_t ENorDI)
{
    if (ENorDI == ENABLE)
    {
        pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
        pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
        pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    }
    else
    {
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
    }
}

void i2c_pclk_control (i2c_reg_t * pI2Cx, uint8_t ENorDI)
{
    if (ENorDI == ENABLE)
    {
        if (pI2Cx == I2C1)
        {
            I2C1_PCLK_EN();
        }
        else if (pI2Cx == I2C2)
        {
            I2C2_PCLK_EN();
        }
        else if (pI2Cx == I2C3)
        {
            I2C3_PCLK_EN();
        }
    }
    else
    {
        if (pI2Cx == I2C1)
        {
            I2C1_PCLK_DI();
        }
        else if (pI2Cx == I2C2)
        {
            I2C2_PCLK_DI();
        }
        else if (pI2Cx == I2C3)
        {
            I2C3_PCLK_DI();
        }
    }
}

void i2c_init (i2c_handle_t * pI2CHandle)
{
    i2c_pclk_control(pI2CHandle->pI2Cx, ENABLE);

    uint32_t tmp = 0;
    uint32_t pclk = 0;
    pclk = (rcc_get_pclk(RCC_SYSBUS_APB1) / 1000000U);
    tmp |= pclk;
    pI2CHandle->pI2Cx->CR2 |= (tmp & 0x3F);

    uint16_t ccr_value = 0;
    tmp = 0;
    if (pI2CHandle->i2cConf.i2cSclSpeed <= I2C_SCL_SPEED_SM)
    {
        tmp &= ~(1 << I2C_CCR_FS);
        ccr_value = rcc_get_pclk(RCC_SYSBUS_APB1)
                / (2 * pI2CHandle->i2cConf.i2cSclSpeed);
        tmp |= (ccr_value & 0xFFF);
    }
    else
    {
        tmp |= (1 << I2C_CCR_FS);
        tmp |= (pI2CHandle->i2cConf.i2cFMDutyCycle << I2C_CCR_DUTY);

        if (pI2CHandle->i2cConf.i2cFMDutyCycle == I2C_FM_DUTY_2)
        {
            ccr_value |= rcc_get_pclk(RCC_SYSBUS_APB1)
                    / (3 * pI2CHandle->i2cConf.i2cSclSpeed);
        }
        else
        {
            ccr_value |= rcc_get_pclk(RCC_SYSBUS_APB1)
                    / (3 * pI2CHandle->i2cConf.i2cSclSpeed);
        }
        tmp |= (ccr_value & 0xFFF);
    }
    pI2CHandle->pI2Cx->CCR = tmp;

    tmp = 0;
    if (pI2CHandle->i2cConf.i2cSclSpeed <= I2C_SCL_SPEED_SM)
    {
        tmp = (rcc_get_pclk(RCC_SYSBUS_APB1) / 1000000U) + 1;
    }
    else
    {
        tmp = ((rcc_get_pclk(RCC_SYSBUS_APB1) * 300) / 1000000000U) + 1;
    }
    pI2CHandle->pI2Cx->TRISE = (tmp & 0x3F);

    tmp &= ~(1 << I2C_OAR1_ADDMODE);
    tmp |= (1 << I2C_OAR1_BIT_14);
    tmp |= (pI2CHandle->i2cConf.i2cDeviceAddress << I2C_OAR1_ADD_7);
    pI2CHandle->pI2Cx->OAR1 = tmp;
}

void i2c_deinit (i2c_reg_t * pI2Cx)
{
    if (pI2Cx == I2C1)
    {
        I2C1_REG_RESET();
    }
    else if (pI2Cx == I2C2)
    {
        I2C2_REG_RESET();
    }
    else if (pI2Cx == I2C3)
    {
        I2C3_REG_RESET();
    }
}

void i2c_master_transmit (i2c_handle_t * pI2CHandle, uint8_t * pTxBuffer,
        uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
    //1. Send START bit
    i2c_generate_start_condition(pI2CHandle->pI2Cx);

    //2. Clear SB
    while (!i2c_get_status_flag(pI2CHandle->pI2Cx, I2C_FLAG_SB));
    i2c_address_write(pI2CHandle->pI2Cx, SlaveAddr);

    //3. Clear ADDR flag
    while (!i2c_get_status_flag(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));
    i2c_clear_addr_flag(pI2CHandle);

    //4. Send data
    while (Len > 0)
    {
        while (!i2c_get_status_flag(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

        pI2CHandle->pI2Cx->DR = *pTxBuffer;
        Len--;
        pTxBuffer++;
    }

    while (!i2c_get_status_flag(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
    while (!i2c_get_status_flag(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

    //5. Generate STOP condition
    if (Sr == I2C_SR_DI)
    {
        i2c_generate_stop_condition(pI2CHandle->pI2Cx);
    }
}

void i2c_master_receive (i2c_handle_t * pI2CHandle, uint8_t * pRxBuffer,
        uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
    //1. Send START bit
    i2c_generate_start_condition(pI2CHandle->pI2Cx);

    //2. Clear SB
    while (!i2c_get_status_flag(pI2CHandle->pI2Cx, I2C_FLAG_SB));
    i2c_address_read(pI2CHandle->pI2Cx, SlaveAddr);

    //3. Clear ADDR flag
    while (!i2c_get_status_flag(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

    if (Len == 1)
    {
        i2c_manage_acking(pI2CHandle->pI2Cx, DISABLE);

        i2c_clear_addr_flag(pI2CHandle);

        while (!i2c_get_status_flag(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

        // Generate STOP condition
        if (Sr == I2C_SR_DI)
        {
            i2c_generate_stop_condition(pI2CHandle->pI2Cx);
        }

        *pRxBuffer = pI2CHandle->pI2Cx->DR;
    }

    //4. Send data
    if (Len > 1)
    {
        i2c_clear_addr_flag(pI2CHandle);

        while (Len > 0)
        {
            while (!i2c_get_status_flag(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

            if (Len == 2)
            {
                i2c_manage_acking(pI2CHandle->pI2Cx, DISABLE);

                // Generate STOP condition
                if (Sr == I2C_SR_DI)
                {
                    i2c_generate_stop_condition(pI2CHandle->pI2Cx);
                }
            }

            *pRxBuffer = pI2CHandle->pI2Cx->DR;
            Len--;
            pRxBuffer++;
        }
    }

    if (pI2CHandle->i2cConf.i2cACKControl == I2C_ACK_EN)
    {
        i2c_manage_acking(pI2CHandle->pI2Cx, ENABLE);
    }
}

uint8_t i2c_slave_receive (i2c_reg_t * pI2Cx)
{
    return (uint8_t) pI2Cx->DR;
}

void i2c_slave_transmit (i2c_reg_t * pI2Cx, uint8_t data)
{
    pI2Cx->DR = data;
}

uint8_t i2c_master_receive_interrupt (i2c_handle_t * pI2CHandle,
        uint8_t * pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
    uint8_t busyState = pI2CHandle->TxRxState;

    if ((busyState != I2C_BUSY_IN_TX) && (busyState != I2C_BUSY_IN_RX))
    {
        pI2CHandle->RxLen = Len;
        pI2CHandle->RxSize = Len;
        pI2CHandle->pRxBuffer = pRxBuffer;
        pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
        pI2CHandle->DevAddr = SlaveAddr;
        pI2CHandle->Sr = Sr;

        i2c_generate_start_condition(pI2CHandle->pI2Cx);

        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
    }
    return busyState;
}

uint8_t i2c_master_transmit_interrupt (i2c_handle_t * pI2CHandle,
        uint8_t * pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
    uint8_t busyState = pI2CHandle->TxRxState;

    if ((busyState != I2C_BUSY_IN_TX) && (busyState != I2C_BUSY_IN_RX))
    {
        pI2CHandle->TxLen = Len;
        pI2CHandle->pTxBuffer = pTxBuffer;
        pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
        pI2CHandle->DevAddr = SlaveAddr;
        pI2CHandle->Sr = Sr;

        i2c_generate_start_condition(pI2CHandle->pI2Cx);

        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
    }
    return busyState;
}

static void i2c_master_handle_txe_interrupt (i2c_handle_t * pI2CHandle)
{
    if (pI2CHandle->TxLen > 0)
    {
        pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
        pI2CHandle->pTxBuffer++;
        pI2CHandle->TxLen--;
    }
}

static void i2c_master_handle_rxne_interrupt (i2c_handle_t * pI2CHandle)
{
    if (pI2CHandle->RxSize == 1)
    {
        *(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
        pI2CHandle->RxLen--;
    }

    if (pI2CHandle->RxSize > 1)
    {
        if (pI2CHandle->RxLen == 2)
        {
            i2c_manage_acking(pI2CHandle->pI2Cx, DISABLE);
        }

        *(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
        pI2CHandle->pRxBuffer++;
        pI2CHandle->RxLen--;
    }

    if (pI2CHandle->RxLen == 0)
    {

        if (pI2CHandle->Sr == I2C_SR_DI)
        {
            i2c_generate_stop_condition(pI2CHandle->pI2Cx);
        }

        i2c_close_reception(pI2CHandle);
        i2c_application_event_callback(pI2CHandle, I2C_EV_RX_CMPLT);
    }

}

void i2c_event_irq_handling (i2c_handle_t * pI2CHandle)
{
    uint32_t tmp1, tmp2, tmp3;

    tmp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);
    tmp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);

    tmp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);
    if (tmp2 && tmp3)
    {
        if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
        {
            i2c_address_write(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
        }
        else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
        {
            i2c_address_read(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
        }
    }

    tmp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
    if (tmp2 && tmp3)
    {
        i2c_clear_addr_flag(pI2CHandle);
    }

    //BTF
    tmp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
    if (tmp2 && tmp3)
    {
        if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
        {
            if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE))
            {
                if (pI2CHandle->TxLen == 0)
                {

                    if (pI2CHandle->Sr == I2C_SR_DI)
                    {
                        i2c_generate_stop_condition(pI2CHandle->pI2Cx);
                    }

                    i2c_close_transmission(pI2CHandle);

                    i2c_application_event_callback(pI2CHandle, I2C_EV_TX_CMPLT);
                }
            }
        }
        else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
        {
            ;
        }
    }

    tmp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
    if (tmp2 && tmp3)
    {
        pI2CHandle->pI2Cx->CR1 |= 0x0000;
        i2c_application_event_callback(pI2CHandle, I2C_EV_STOP);
    }

    tmp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
    if (tmp1 && tmp2 && tmp3)
    {
        if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
        {
            if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
            {
                i2c_master_handle_txe_interrupt(pI2CHandle);
            }
        }
        else
        {
            if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
            {
                i2c_application_event_callback(pI2CHandle, I2C_EV_DATA_REQ);
            }
        }
    }

    tmp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
    if (tmp1 && tmp2 && tmp3)
    {
        if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
        {
            if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
            {
                i2c_master_handle_rxne_interrupt(pI2CHandle);
            }
        }
        else
        {
            if (!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)))
            {
                i2c_application_event_callback(pI2CHandle, I2C_EV_DATA_RCV);
            }
        }
    }

}

void i2c_error_irq_handling (i2c_handle_t * pI2CHandle)
{
    uint32_t tmp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITERREN);

    //! BERR handling
    uint32_t tmp2 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BERR);
    if (tmp1 && tmp2)
    {
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);
        i2c_application_event_callback(pI2CHandle, I2C_ERR_BERR);
    }

    //! ARLO handling
    tmp2 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ARLO);
    if (tmp1 && tmp2)
    {
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);
        i2c_application_event_callback(pI2CHandle, I2C_ERR_ARLO);
    }

    //! AF handling
    tmp2 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_AF);
    if (tmp1 && tmp2)
    {
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);
        i2c_application_event_callback(pI2CHandle, I2C_ERR_AF);
    }

    //! OVR handling
    tmp2 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_OVR);
    if (tmp1 && tmp2)
    {
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);
        i2c_application_event_callback(pI2CHandle, I2C_ERR_OVR);
    }

    //! TIMEOUT handling
    tmp2 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TIMEOUT);
    if (tmp1 && tmp2)
    {
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);
        i2c_application_event_callback(pI2CHandle, I2C_ERR_TIMEOUT);
    }
}

void i2c_irq_config (uint8_t IRQ_No, uint8_t ENorDI)
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

void i2c_irq_priority (uint8_t IRQ_No, uint8_t IRQ_Priority)
{
    uint8_t tmp1 = IRQ_No / 4;
    uint8_t tmp2 = IRQ_No % 4;
    NVIC->IPR[tmp1] = (IRQ_Priority << ((tmp2 * 8) + 4));
}

void i2c_close_transmission (i2c_handle_t * pI2CHandle)
{
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

    pI2CHandle->TxLen = 0;
    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pTxBuffer = NULL;
}

void i2c_close_reception (i2c_handle_t * pI2CHandle)
{
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

    pI2CHandle->RxLen = 0;
    pI2CHandle->RxSize = 0;
    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pRxBuffer = NULL;

    if (pI2CHandle->i2cConf.i2cACKControl == I2C_ACK_EN)
    {
        i2c_manage_acking(pI2CHandle->pI2Cx, ENABLE);
    }
}

uint8_t i2c_get_status_flag (i2c_reg_t * pI2Cx, uint32_t flagName)
{
    if (pI2Cx->SR1 & flagName)
    {
        return FLAG_SET;
    }
    else
    {
        return FLAG_RESET;
    }
}

void i2c_peripheral_control (i2c_reg_t * pI2Cx, uint8_t ENorDI)
{
    if (ENorDI == ENABLE)
    {
        pI2Cx->CR1 |= (1 << I2C_CR1_PE);
    }
    else
    {
        pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
    }
}

void i2c_manage_acking (i2c_reg_t * pI2Cx, uint8_t ENorDI)
{
    if (ENorDI == ENABLE)
    {
        pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
    }
    else
    {
        pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
    }
}
__weak void i2c_application_event_callback (i2c_handle_t * pI2CHandle,
        uint8_t AppEv);

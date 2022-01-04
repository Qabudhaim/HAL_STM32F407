/*
 * l3gd20.h
 *
 *  Created on: Dec 12, 2021
 *      Author: obscure
 */

#ifndef L3GD20_H_
#define L3GD20_H_

#include "stm32f407xx.h"

/****************************************************************************
 *                     Peripheral registers definition                      *
 ****************************************************************************/
#define L3GD20_CTRL_REG1                0x20
#define L3GD20_CTRL_REG2                0x21
#define L3GD20_CTRL_REG3                0x22
#define L3GD20_CTRL_REG4                0x23
#define L3GD20_CTRL_REG5                0x24
#define L3GD20_REFERENCE                0x25
#define L3GD20_OUT_TEMP                 0x26
#define L3GD20_STATUS_REG               0x27
#define L3GD20_OUT_X_L                  0x28
#define L3GD20_OUT_X_H                  0x29
#define L3GD20_OUT_Y_L                  0x2A
#define L3GD20_OUT_Y_H                  0x2B
#define L3GD20_OUT_Z_L                  0x2C
#define L3GD20_OUT_Z_H                  0x2D
#define L3GD20_FIFO_CTRL_REG            0x2E
#define L3GD20_FIFO_SRC_REG             0x2F
#define L3GD20_INT1_CFG                 0x30
#define L3GD20_INT1_SRC                 0x31
#define L3GD20_INT1_TSH_XH              0x32
#define L3GD20_INT1_TSH_XL              0x33
#define L3GD20_INT1_TSH_YH              0x34
#define L3GD20_INT1_TSH_YL              0x35
#define L3GD20_INT1_TSH_ZH              0x36
#define L3GD20_INT1_TSH_ZL              0x37
#define L3GD20_INT1_DURATION            0x38

/****************************************************************************
 *                           L3GD20 user configuration                      *
 ****************************************************************************/

/**
 * @L3GD20_POWER_MODE
 * Possible options for L3GD20 Power Mode
 */
#define L3GD20_PD_POWER_DOWN            0
#define L3GD20_PD_NORMAL                1
#define L3GD20_PD_SLEEP                 1   // Refer to datasheet Section 7.2 (CTRL_REG1) Page. 31

/**
 * @L3GD20_FS
 * Possible options for L3GD20 Full-Scale selection
 */
#define L3GD20_FS_250DPS                0
#define L3GD20_FS_500DPS                1
#define L3GD20_FS_2000PS                2

/**
 * @L3GD20_FIFO_EN
 * Enable/Disable L3GD20 FIFO
 */
#define L3GD20_FIFO_DI                  DISABLE
#define L3GD20_FIFO_EN                  ENABLE

/**
 * @L3GD20_FIFO_MODE
 * Possible options for L3GD20 FIFO mode selection
 */
#define L3GD20_BYPASS_MODE              0
#define L3GD20_FIFO_MODE                1
#define L3GD20_STREAM_MODE              2
#define L3GD20_STREAM_FIFO_MODE         3
#define L3GD20_BYPASS_STREAM_MODE       4

/****************************************************************************
 *                           L3GD20 registers bits                          *
 ****************************************************************************/
/**
 * L3GD20 CTRL_REG1
 */
#define L3GD20_CTRL_REG1_YEN            0
#define L3GD20_CTRL_REG1_XEN            1
#define L3GD20_CTRL_REG1_ZEN            2
#define L3GD20_CTRL_REG1_PD             3
#define L3GD20_CTRL_REG1_BW             4
#define L3GD20_CTRL_REG1_DR             6

/**
 * L3GD20 CTRL_REG4
 */
#define L3GD20_CTRL_REG4_SIM            0
#define L3GD20_CTRL_REG4_FS             4
#define L3GD20_CTRL_REG4_BLE            6
#define L3GD20_CTRL_REG4_BDU            7

/**
 * L3GD20 CTRL_REG5
 */
#define L3GD20_CTRL_REG5_OUT_SEL        0
#define L3GD20_CTRL_REG5_INT1_SEL       2
#define L3GD20_CTRL_REG5_HPEN           4
#define L3GD20_CTRL_REG5_FIFO_EN        6
#define L3GD20_CTRL_REG5_BOOT           7

/**
 * L3GD20 STATUS_REG
 */
#define L3GD20_STATUS_REG_XDA           0
#define L3GD20_STATUS_REG_YDA           1
#define L3GD20_STATUS_REG_ZDA           2
#define L3GD20_STATUS_REG_ZYXDA         3
#define L3GD20_STATUS_REG_XOR           4
#define L3GD20_STATUS_REG_YOR           5
#define L3GD20_STATUS_REG_ZOR           6
#define L3GD20_STATUS_REG_ZYXOR         7

/**
 * L3GD20 FIFO_CTRL_REG
 */
#define L3GD20_FIFO_CTRL_REG_WTM        0
#define L3GD20_FIFO_CTRL_REG_FM         5

/****************************************************************************
 *                       L3GD20 default configurations                      *
 ****************************************************************************/
#define L3GD20_PD                       L3GD20_PD_NORMAL
#define L3GD20_FS                       L3GD20_FS_2000PS
#define L3GD20_FIFO                     L3GD20_BYPASS_MODE
#define L3GD20_SAD                      0x6B

/****************************************************************************
 *                       APIs supported by this driver                      *
 ****************************************************************************/
uint8_t l3gd20_init ();
void l3gd20_read (int16_t * data);

#endif /* L3GD20_H_ */

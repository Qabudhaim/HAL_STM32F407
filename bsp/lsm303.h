/*
 * lsm303.h
 *
 *  Created on: Dec 15, 2021
 *      Author: obscure
 */

#ifndef LSM303_H_
#define LSM303_H_

#include "stm32f407xx.h"

#define LSM303_SAD                      0x19

/****************************************************************************
 *                     Peripheral registers definition                      *
 ****************************************************************************/
#define LSM303_CTRL_REG1                0x20
#define LSM303_CTRL_REG2                0x21
#define LSM303_CTRL_REG3                0x22
#define LSM303_CTRL_REG4                0x23
#define LSM303_CTRL_REG5                0x24
#define LSM303_CTRL_REG6                0x25
#define LSM303_REFERENCE                0x26
#define LSM303_STATUS_REG               0x27
#define LSM303_OUT_X_L                  0x28
#define LSM303_OUT_X_H                  0x29
#define LSM303_OUT_Y_L                  0x2A
#define LSM303_OUT_Y_H                  0x2B
#define LSM303_OUT_Z_L                  0x2C
#define LSM303_OUT_Z_H                  0x2D
#define LSM303_FIFO_CTRL_REG            0x2E
#define LSM303_FIFO_SRC_REG             0x2F

/****************************************************************************
 *                       APIs supported by this driver                      *
 ****************************************************************************/
uint8_t lsm303_init ();
void lsm303_read (int16_t * data);

#endif /* LSM303_H_ */

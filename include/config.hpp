#ifndef ASERIAL_CONFIG_HPP
#define ASERIAL_CONFIG_HPP


/******************************************************************************
 *                  GLOBAL
 *****************************************************************************/

#define MCU_ARCH_ATMEGA         1

/*****************************************************************************/


/******************************************************************************
 *                  I2C
 *****************************************************************************/

#define I2C_DEFAULT_BAUDRATE    NORMAL    // SLOW(100,000), NORMAL(400,000) or FAST(1,000,000)
#define I2C_BUFFER_SIZE         16        // 
#define I2C_EXTERNAL_PULLUP     true      //

#define LOG_INTERRUPTS          false

//#define I2C_REMOVE_INTERRUPTS             //

/*****************************************************************************/

#endif  //  ASERIAL_CONFIG_HPP

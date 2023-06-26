
/******************************************************************************
 *                  INCLUDES
 *****************************************************************************/

 // DEFINITIONS
#include "buses/i2c.hpp"
#include "config.hpp"

// Arduinos
#include <Arduino.h>

/*****************************************************************************/


/******************************************************************************
 *                  BUFFER
 *****************************************************************************/

static I2C::Task buffer[I2C_BUFFER_SIZE];

#if I2C_BUFFER_SIZE == 255
static uint8_t head = 0;
static uint8_t index = 0;
#define INC(I) (I++)

#else
static uintmax_t head = 0;
static uintmax_t index = 0;
#define INC(I) (I = (I==I2C_BUFFER_SIZE-1) ? 0 : I+1)

#endif

/*****************************************************************************/


/******************************************************************************
 *                  IMPLEMENTATIONS
 *****************************************************************************/

void I2C::initBus(void) {
  I2C::initBus(I2C::I2C_DEFAULT_BAUDRATE);
}

void I2C::initBus(I2C::Rate baudRate) {
  TWI0_MSTATUS = TWI_BUSSTATE_IDLE_gc;		// Force to IDLE state on reset.

#ifdef NO_EXTERNAL_I2C_PULLUP
  pinMode(PIN_WIRE_SDA, INPUT_PULLUP);
  pinMode(PIN_WIRE_SCL, INPUT_PULLUP);
#endif

  // Set BAUD rate 
  I2C::setBusRate(baudRate);

  TWI0_CTRLA = TWI_SDAHOLD_500NS_gc;		// SMBus conform.
  // Clear Master state, MADDR and MDATA.
  TWI0_MCTRLB = TWI_FLUSH_bm;
  // Read interrupt, Write interrupt, TimeOut 200us, Smart mode.
  // TWI0_MCTRLA = TWI_RIEN_bm | TWI_WIEN_bm | TWI_TIMEOUT_200US_gc | TWI_SMEN_bm;
  // Read interrupt, Write interrupt, TimeOut 200us, Smart mode, Enable.
  TWI0_MCTRLA = TWI_RIEN_bm | TWI_WIEN_bm | TWI_TIMEOUT_200US_gc | TWI_SMEN_bm | TWI_ENABLE_bm;
}


void I2C::setBusRate(I2C::Rate baudRate) {

}


uint8_t I2C::addTask(I2C::Task newTask) {

}

/*****************************************************************************/


/******************************************************************************
 *                  INTERRUPTS
 *****************************************************************************/

ISR(TWI0_TWIM_vect) {

}

ISR(TWI0_TWIS_vect) {

}

/*****************************************************************************/
#ifndef ASERIAL_BUSES_I2C_HPP
#define ASERIAL_BUSES_I2C_HPP

/******************************************************************************
 *                  INCLUDES
 *****************************************************************************/

// Types
#include <stdint.h>

// AVR
#include <avr/io.h>
#include <avr/interrupt.h>

/*****************************************************************************/


/******************************************************************************
 *                  I2C BUS CONTROLS
 *****************************************************************************/

namespace I2C {
  enum Rate { SLOW, NORMAL, FAST };

  struct Task {
    uint8_t* buffs;
    uint8_t nWrite, nRead;
  };


  /// @brief 
  void initBus(void);

  /// @brief 
  /// @param baudRate 
  void initBus(Rate baudRate);


  /// @brief 
  /// @param baudRate 
  void setBusRate(Rate baudRate);


  /// @brief 
  /// @param newTask 
  /// @return 
  uint8_t addTask(Task newTask);
}

/*****************************************************************************/

#endif // ASERIAL_BUSES_I2C_HPP

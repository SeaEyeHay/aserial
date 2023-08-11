#ifndef ASERIAL_BUSES_I2C_HPP
#define ASERIAL_BUSES_I2C_HPP

/******************************************************************************
 *                  INCLUDES
 *****************************************************************************/

// Settings
#include "config.hpp"

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
  enum Rate             { SLOW, NORMAL, FAST        };
  enum Result: uint8_t  { IDLE, BUSY,   OK,  ERROR  };

  // General TWI Master status codes
  enum MasterAction : uint8_t {
  	WRITE_DATA	= 0x00,
  	READ_DATA		= 0x01,
  	LAST_WRITE	= 0x02,
  	LAST_READ		= 0x03,

  	SEND_ADDR       = 0x04,
  	SEND_ADDR_READ  = 0x05,

    AWAIT_READ,
    AWAIT_LAST_READ,

  	STOP,
  };

  struct Task {
    uint8_t* buffs;
    uint8_t nWrite, nRead;

    MasterAction action;
  };

  typedef uint8_t   Id;



  /// @brief 
  ///     Configure the hardware I2C bus
  ///
  void initBus(void);

  /// @brief 
  ///     Configure the hardware I2C bus with
  ///     the given speed
  ///
  /// @param baudRate Speed of the transmission
  void initBus(Rate baudRate);


  /// @brief 
  ///     Set number of bits per secondes transmitted by the I2C bus
  ///
  ///         SLOW    = 100,000 bps
  ///         NORMAL  = 400,000 bps
  ///         FAST    = 1,000,000 bps
  ///
  /// @param baudRate Speed of the transmission
  void setBusRate(Rate baudRate);


  /// @brief 
  ///
  ///
  /// @param newTask 
  /// @return Id of the task in the buffer (ONLY VALIDE WHILE THE TASK IS BUSY)
  Id addTask(Task newTask);

  /// @brief 
  /// @param taskId 
  void cancelTask(Id taskId);


  /// @brief 
  /// 
  void outputInterruptLogs(void);
}

/*****************************************************************************/

#endif // ASERIAL_BUSES_I2C_HPP

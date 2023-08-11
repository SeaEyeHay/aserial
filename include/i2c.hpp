#ifndef ASERIAL_I2C_HPP
#define ASERIAL_I2C_HPP

/******************************************************************************
 *                  INCLUDES
 *****************************************************************************/

// DEFINITIONS
#include "buses/i2c.hpp"
#include "config.hpp"

// Types
#include <stdint.h>
#include <stddef.h>

#include "microTuple.h"

/*****************************************************************************/


/******************************************************************************
 *                  CONSTANTS
 *****************************************************************************/

template<class... T>
using Tuple = MicroTuple<T...>;

/*****************************************************************************/


/******************************************************************************
 *                  HELPERS
 *****************************************************************************/

//
#include "utils.hpp"

/*****************************************************************************/



/******************************************************************************
 *                  I2C CLASSES
 *****************************************************************************/

namespace I2C {
  template<class... T>
  class Base {
  protected:
    uint8_t buffer[ aserial::packSize<uint8_t, T..., Result>() ];
    Id      id;

    static constexpr uint8_t STATUS = sizeof(buffer)-1;
    static constexpr uint8_t DEVICE = 0;

    Base(uint8_t device) {
      buffer[DEVICE] = device;
      buffer[STATUS] = IDLE; 
    }

    ~Base() { if (buffer[STATUS] == BUSY) cancelTask(id); }

  public:
    void await() { while ( isDone() ) { ; } }

    bool isDone() { return !isBusy();                           }
    bool isBusy() { return this->buffer[this->STATUS] == BUSY;  }
    bool isIdle() { return this->buffer[this->STATUS] == IDLE;  }
    bool isOk()   { return this->buffer[this->STATUS] == OK;    }
    bool isErr()  { return this->buffer[this->STATUS] == ERROR; }
  };


  template<class... T>
  class Out : public Base<T...> {
  public:
    Out(uint8_t device) : Base<T...>(device) {}


    bool write(T... data) {
      if (this->isBusy()) return false;

      aserial::explode<T...>( this->buffer+1, data..., BUSY );
      this->id = addTask({ this->buffer, sizeof(this->buffer)-1, 0, SEND_ADDR });
      return true;
    }
  };


  template<class... T>
  class In : public Base<T...> {
  public:
    In(uint8_t device) : Base<T...>(device) {}


    bool read() {
      if (this->isBusy()) return false;

      this->buffer[this->STATUS] = BUSY;
      addTask({ this->buffer, 0, sizeof(this->buffer)-1, SEND_ADDR_READ });
      return true;
    }

    Tuple<T...> await() {
      Base<T...>::await();
      return aserial::implode<T...>(this->buffer+1);
    }
  };

  template<class T>
  class In<T> : public Base<T> {
  public:
    In(uint8_t device) : Base<T>(device) {}


    bool read() {
      if (this->isBusy()) return false;

      this->buffer[this->STATUS] = BUSY;
      addTask({ this->buffer, 0, sizeof(this->buffer)-1, SEND_ADDR_READ });
      return true;
    }

    T await() {
      Base<T>::await();
      return *(T*) this->buffer+1;
    }
  };


  template<class R, class... T>
  class BaseQuery : public Base<T...> {
  protected:
    BaseQuery(uint8_t device) : Base<T...>(device) {}

  public:
    bool get(R reg) {
      if (this->isBusy()) return false;

      aserial::explode( this->buffer+1, reg );
      this->buffer[this->STATUS] = BUSY;
      this->id = addTask({ this->buffer, sizeof(R)+1, aserial::packSize<T...>(), SEND_ADDR });
      return true;
    }

    bool set(R reg, T... data) {
      if (this->isBusy()) return false;

      aserial::explode( this->buffer+1, reg, data..., BUSY );
      this->id = addTask({ this->buffer, sizeof(this->buffer)-1, 0, SEND_ADDR });
      return true;
    }
  };

  template<class R, class... T>
  class Query : public BaseQuery<R, T...> {
  public:
    Query(uint8_t device) : BaseQuery<R, T...>(device) {}

    Tuple<T...> await() {
      Base<R, T...>::await();
      return aserial::implode<T...>(this->buffer + 1 + sizeof(R));
    }
  };

  template<class R, class T>
  class Query<R, T> : public BaseQuery<R, T> {
  public:
    Query(uint8_t device) : BaseQuery<R, T>(device) {}

    T await() {
      Base<R, T>::await();
      return *(T*)(this->buffer + 1 + sizeof(R));
    }
  };
}

/*****************************************************************************/

#endif
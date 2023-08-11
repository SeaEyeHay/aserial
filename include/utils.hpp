#ifndef ASERIAL_UTILS_HPP
#define ASERIAL_UTILS_HPP

/******************************************************************************
 *                  INCLUDES
 *****************************************************************************/

// Types
#include <stdint.h>
#include <stddef.h>
#include "microTuple.h"

// Memory Usage
#include <string.h>

/*****************************************************************************/


/******************************************************************************
 *                  FUNCTIONS
 *****************************************************************************/

namespace aserial {
  // https://stackoverflow.com/questions/57246592/
  // getting-total-size-in-bytes-of-c-template-parameter-pack#comment101001790_57246704
  template<class... T>
  constexpr uint8_t packSize() { return (sizeof(T) + ... + 0); }



  template<class F, class... T>
  inline void explode(uint8_t* target, F first, T... vals);
  template<class T>
  inline void explode(uint8_t* target, T val);

  template<class F, class... T>
  inline void explode(uint8_t* target, F first, T... vals) {
    explode( target,             first   );
    explode( target + sizeof(F), vals... );
  }
  
  template<class T>
  inline void explode(uint8_t* target, T val) {
    memcpy( target, &val, sizeof(T) );
  }


  template<class T>
  inline T implodeInner(uint8_t*& src) {
    T val = *(T*)src;
    src += sizeof(T);

    return val;
  }

  template<class... T>
  inline MicroTuple<T...> implode(uint8_t* src) {
    return MicroTuple<T...>( implodeInner<T>(src)... );
  } 
}

/*****************************************************************************/

#endif
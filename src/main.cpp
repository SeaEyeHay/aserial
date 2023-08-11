#include <Arduino.h>

#include "buses/i2c.hpp"
#include "i2c.hpp"


void setup() {
  I2C::initBus(I2C::SLOW);
  pinMode(13, OUTPUT);

  uint8_t buff[] = { 0x40, 0x09, 0x00, I2C::BUSY };

  I2C::Task task = {
    .buffs  = buff,
    .nWrite = 2,
    .nRead  = 1,
    .action = I2C::SEND_ADDR,
  };


  delay(100);
  I2C::addTask(task);
}

void loop() {
  //I2C::outputInterruptLogs();
}
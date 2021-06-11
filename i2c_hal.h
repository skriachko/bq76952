/*
  i2c_hal.h is I2C library for STM32 platform
  Copyright (c) 2011 BroLab.  All right reserved.
  Author: Sergii Kriachko
*/

#ifndef I2C_HAL_H
#define I2C_HAL_H

#include <inttypes.h>
#include "stm32wbxx_hal.h"

#define BUFFER_LENGTH 32

class I2C_HAL
{
  private:
    static uint8_t rxBuffer[];
    static uint8_t rxBufferIndex;
    static uint8_t rxBufferLength;

    static uint8_t txAddress;
    static uint8_t txBuffer[];
    static uint8_t txBufferIndex;
    static uint8_t txBufferLength;

    static uint8_t transmitting;
    void setWriteError(){}
    I2C_HandleTypeDef *hi2c;
  public:
    I2C_HAL(I2C_HandleTypeDef *hi2c);
    void begin();
    void begin(uint8_t);
    void begin(int);
    void setClock(uint32_t);
    void beginTransmission(uint8_t);
    void beginTransmission(int);
    uint8_t endTransmission(void);
    uint8_t endTransmission(uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t, uint8_t);
    uint8_t requestFrom(int, int);
    uint8_t requestFrom(int, int, int);
    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t *, size_t);
    virtual int available(void);
    virtual int read(void);

    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }
};


#endif


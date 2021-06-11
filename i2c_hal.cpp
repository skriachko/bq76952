/*
 * Wrapper class for i2c communication on stm32
 * Copyright (c) 2011 BroLab.  All right reserved.
 * Author      :   Sergii Kriachko
 * Description :   Source file of BQ76952 10-series multicell.
 */

extern "C" {
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
}
#include "main.h"
#include "i2c_hal.h"

// Initialize Class Variables //////////////////////////////////////////////////

uint8_t I2C_HAL::rxBuffer[BUFFER_LENGTH];
uint8_t I2C_HAL::rxBufferIndex = 0;
uint8_t I2C_HAL::rxBufferLength = 0;

uint8_t I2C_HAL::txAddress = 0;
uint8_t I2C_HAL::txBuffer[BUFFER_LENGTH];
uint8_t I2C_HAL::txBufferIndex = 0;
uint8_t I2C_HAL::txBufferLength = 0;

uint8_t I2C_HAL::transmitting = 0;

// Constructors ////////////////////////////////////////////////////////////////

I2C_HAL::I2C_HAL(I2C_HandleTypeDef *i2c) :
		hi2c(i2c) {
}

// Public Methods //////////////////////////////////////////////////////////////


void I2C_HAL::begin(void) {
	rxBufferIndex = 0;
	rxBufferLength = 0;

	txBufferIndex = 0;
	txBufferLength = 0;

}

void I2C_HAL::begin(int address) {
	begin((uint8_t) address);
}

static int Timeout = 1000;
uint8_t I2C_HAL::requestFrom(uint8_t address, uint8_t quantity,
		uint8_t sendStop) {
	// clamp to buffer length
	if (quantity > BUFFER_LENGTH) {
		quantity = BUFFER_LENGTH;
	}
	// perform blocking read into buffer
	HAL_I2C_Master_Receive(hi2c, address, rxBuffer, quantity, Timeout);
	// set rx buffer iterator vars
	rxBufferIndex = 0;
	rxBufferLength = quantity;

	return quantity;
}

uint8_t I2C_HAL::requestFrom(uint8_t address, uint8_t quantity) {
	return requestFrom((uint8_t) address, (uint8_t) quantity, (uint8_t) true);
}

uint8_t I2C_HAL::requestFrom(int address, int quantity) {
	return requestFrom((uint8_t) address, (uint8_t) quantity, (uint8_t) true);
}

uint8_t I2C_HAL::requestFrom(int address, int quantity, int sendStop) {
	return requestFrom((uint8_t) address, (uint8_t) quantity,
			(uint8_t) sendStop);
}

void I2C_HAL::beginTransmission(uint8_t address) {
	// indicate that we are transmitting
	transmitting = 1;
	// set address of targeted slave
	txAddress = address;
	// reset tx buffer iterator vars
	txBufferIndex = 0;
	txBufferLength = 0;
}

void I2C_HAL::beginTransmission(int address) {
	beginTransmission((uint8_t) address);
}


// Transmits data to battery monitor
uint8_t I2C_HAL::endTransmission(uint8_t sendStop) {
	// transmit buffer (blocking)
	auto status = HAL_I2C_Master_Transmit(hi2c, txAddress, txBuffer,
			txBufferLength, Timeout);
	if (status != HAL_OK) {
	    if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF)
	    {
	      Error_Handler();
	    }
	}
	txBufferIndex = 0;
	txBufferLength = 0;
	// indicate that we are done transmitting
	transmitting = 0;
	return status==HAL_OK;
}

// Calls data transmission to battery monitor
uint8_t I2C_HAL::endTransmission(void) {
	return endTransmission(true);
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
size_t I2C_HAL::write(uint8_t data) {
	if (transmitting) {
		// don't care if buffer is full
		if (txBufferLength >= BUFFER_LENGTH) {
			setWriteError();
			return 0;
		}
		// add byte in tx buffer
		txBuffer[txBufferIndex] = data;
		++txBufferIndex;
		txBufferLength = txBufferIndex;
	} else {
		// Send response to master
		HAL_I2C_Slave_Transmit_IT(hi2c, &data, 1);
	}
	return 1;
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
size_t I2C_HAL::write(const uint8_t *data, size_t quantity) {
	if (transmitting) {
		// in master transmitter mode
		for (size_t i = 0; i < quantity; ++i) {
			write(data[i]);
		}
	} else {
		// reply to master
		HAL_I2C_Slave_Transmit_IT(hi2c, (uint8_t *)data, quantity);
	}
	return quantity;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int I2C_HAL::available(void) {
	return rxBufferLength - rxBufferIndex;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int I2C_HAL::read(void) {
	int value = -1;

	// get each successive byte on each call
	if (rxBufferIndex < rxBufferLength) {
		value = rxBuffer[rxBufferIndex];
		++rxBufferIndex;
	}

	return value;
}



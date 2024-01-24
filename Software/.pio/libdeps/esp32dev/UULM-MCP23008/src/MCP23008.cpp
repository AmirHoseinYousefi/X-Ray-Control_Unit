//
// Created by Falko Schmidt on 04.08.21.
//

#include "MCP23008.h"
#include <Wire.h>

#define                 MCP23008_DEFAULT_I2C_ADDRESS                    0x20

#define                 MCP23008_REGISTER_IODIR                         0x00
#define                 MCP23008_REGISTER_IPOL                          0x01
#define                 MCP23008_REGISTER_GPINTEN                       0x02
#define                 MCP23008_REGISTER_DEFVAL                        0x03
#define                 MCP23008_REGISTER_INTCON                        0x04
#define                 MCP23008_REGISTER_IOCON                         0x05
#define                 MCP23008_REGISTER_GPPU                          0x06
#define                 MCP23008_REGISTER_INTF                          0x07
#define                 MCP23008_REGISTER_INTCAP                        0x08
#define                 MCP23008_REGISTER_GPIO                          0x09
#define                 MCP23008_REGISTER_OLAT                          0x0A


MCP23008::MCP23008(uint8_t i2cAddress) {
    this->i2cAddress = i2cAddress;
        Wire.begin();
}

MCP23008::MCP23008() : MCP23008::MCP23008(MCP23008_DEFAULT_I2C_ADDRESS) {
}


uint8_t MCP23008::readRegister(const uint8_t reg) const {
    // Send register address to device, but keep connection alive
    Wire.beginTransmission(this->i2cAddress);
    Wire.write(reg);
    Wire.endTransmission(false);

    // Wait for response (one byte) and close connection after
    // receiving data
    Wire.requestFrom((uint8_t)this->i2cAddress, (uint8_t)1, (uint8_t)true);
    return (Wire.available() >= 1) ? Wire.read() : -1;
}

bool MCP23008::writeRegister(const uint8_t reg, const uint8_t val) const {
    Wire.beginTransmission(this->i2cAddress);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission();
}


bool MCP23008::setPins(uint8_t gpios) {
    return writeRegister(MCP23008_REGISTER_IODIR, gpios);
}

bool MCP23008::setInvertInputs(uint8_t ii) {
    return writeRegister(MCP23008_REGISTER_IPOL, ii);
}

bool MCP23008::setInterruptOnChange(uint8_t ioc) {
    return writeRegister(MCP23008_REGISTER_GPINTEN, ioc);
}

bool MCP23008::setDefaultComparisonValue(uint8_t dv) {
    return writeRegister(MCP23008_REGISTER_DEFVAL, dv);
}

bool MCP23008::setInterruptControl(uint8_t ic) {
    return writeRegister(MCP23008_REGISTER_INTCON, ic);
}

bool MCP23008::setConfiguration(uint8_t conf) {
    return writeRegister(MCP23008_REGISTER_IOCON, conf);
}

bool MCP23008::setPullups(uint8_t pu) {
    return writeRegister(MCP23008_REGISTER_GPPU, pu);
}

bool MCP23008::setOutputs(uint8_t val) {
    return writeRegister(MCP23008_REGISTER_OLAT, val);
}

uint8_t MCP23008::readInputs() {
    return readRegister(MCP23008_REGISTER_GPIO);
}


uint8_t MCP23008::readInterrupt() {
    uint8_t regVal = readRegister(MCP23008_REGISTER_INTCAP);
    uint8_t interruptPin = 0;
    while (regVal > 1) {
        regVal >>= 1;
        interruptPin++;
    }
    return interruptPin;
}

uint8_t MCP23008::readAllInterrupts() {
    return readRegister(MCP23008_REGISTER_INTF);
}



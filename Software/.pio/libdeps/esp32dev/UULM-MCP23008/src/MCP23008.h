//
// Created by Falko Schmidt on 04.08.21.
//

#ifndef UULM_MCP23008_MCP23008_H
#define UULM_MCP23008_MCP23008_H


#include <stdint-gcc.h>

class MCP23008 {
private:
    uint8_t i2cAddress;

public:
    // Construct object
    MCP23008();
    explicit MCP23008(uint8_t i2cAddress);

    // Set pins to be inputs (1) or outputs (0)
    // DATASHEET - PAGE 8
    bool setPins(uint8_t gpios);

    // Sets if the input should be read as is (0)
    // or be inverted (1)
    // DATASHEET - PAGE 9
    bool setInvertInputs(uint8_t ii);

    // Enables or disables interrupts on input pins.
    // The registers DEFVAL and INTCON must also be set.
    // DATASHEET - PAGE 10
    bool setInterruptOnChange(uint8_t ioc);

    // Sets the default value to compare with. If the pin
    // level is the opposite of this register value, then an
    // interrupt will be triggered.
    // DATASHEET - PAGE 11
    bool setDefaultComparisonValue(uint8_t dv);


    // Sets the interrupt control. If set to '1', the
    // pin value is compared against the set value in
    // the DEFVAL value. Otherwise the pin value is set
    // to the previous pin value.
    // DATASHEET - PAGE 12
    bool setInterruptControl(uint8_t ic);


    // Set multiple settings with this function.
    // conf[5]: sequential operation mode:  1 disable,      0 enable
    // conf[4]: slew rate:                  1 disable,      0 enable
    // conf[3]: hardware address:           1 disable,      0 enable
    // conf[2]: configures INT-pin:         1 open-drain,   0 active driver
    // conf[1]: sets INT-pin polarity:      1 active-HIGH,  0 active-LOW
    // DATASHEET - PAGE 13
    bool setConfiguration(uint8_t conf);


    // Activates or deactivates internal pullup resistors
    // Set bit to 1 to enable and to 0 to disable internal 100k
    // resistor.
    // DATASHEET - Page 14
    bool setPullups(uint8_t pu);


    // Set output pins. 1 means they are logic-HIGH,
    // 0 means logic-LOW output.
    // DATASHEET - PAGE 18
    bool setOutputs(uint8_t val);


    // Reads all input pins and returns the raw value.
    // DATASHEET - PAGE 17
    uint8_t readInputs();


    // Reads the interrupt. After reading, the interrupt
    // register will be cleared.
    // DATASHEET - PAGE 16
    uint8_t readInterrupt();


    // Reads all pins, that created an interrupt.
    // DATASHEET - PAGE 15
    uint8_t readAllInterrupts();

private:
    bool writeRegister(uint8_t reg, uint8_t val) const;
    uint8_t readRegister(uint8_t reg) const;
};


#endif //UULM_MCP23008_MCP23008_H

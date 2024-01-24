#include <Arduino.h>
#include <MCP23008.h>

#define                 PORT_EXPANDER_INTERRUPT_PIN                 (3)

MCP23008 portExpander = MCP23008();


void port_expander_isr() {
    uint8_t pin = portExpander.readInterrupt();
    Serial.println((String)"Interrupt on pin " + pin);
}


void setup() {

    Serial.begin(9600);
    Serial.print("Configuring MCP23008...");

    // Set [0...3] to inputs, [4...7] to outputs
    portExpander.setPins(0b00001111);

    // Activate pullups on input pins [0...3]
    portExpander.setPullups(0b00001111);

    // Activate interrupts on input pins
    portExpander.setInterruptOnChange(0b00001111);

    // Trigger on opposite states of this register.
    // We write 1 meaning ACTIVE HIGH, so interrupt will
    // be triggered if pin goes low.
    portExpander.setDefaultComparisonValue(0b00001111);

    // Enable comparison to DEFVAL register instead of
    // previous value of the pin itself.
    portExpander.setInterruptControl(0b00001111);

    // Set multiple settings with this function.
    // conf[5]: sequential operation mode:  1 disable,      0 enable
    // conf[4]: slew rate:                  1 disable,      0 enable
    // conf[3]: hardware address:           1 disable,      0 enable
    // conf[2]: configures INT-pin:         1 open-drain,   0 active driver
    // conf[1]: sets INT-pin polarity:      1 active-HIGH,  0 active-LOW
    portExpander.setConfiguration(0b00111000);

    Serial.println("ok");

    Serial.print("Configure ISR...");
    pinMode(PORT_EXPANDER_INTERRUPT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PORT_EXPANDER_INTERRUPT_PIN), port_expander_isr, FALLING);
    Serial.println("ok");
}

void loop() {
// write your code here
}

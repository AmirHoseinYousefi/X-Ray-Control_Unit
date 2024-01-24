/* Auther : AmirHosein Yousefi
   Date : 22/4/2023
   Descriptaion :
    Control Board of Hospital Room. 
    It Has 8 Digital I/O and 4 Analog I/O
    It Can Connect to Another Control Board Via ModBus Protocol ,Ethernet ,WiFi ,Bluetooth.
    It Has Onboard Boostup Voltage for LED 5V String.
    
   MCP23008 i2c Address :
   A2   A1   A0   
   -------------------- 
   L  |  L  |  L    0X20
   L  |  L  |  H    0X21
   L  |  H  |  L    0X22
   L  |  H  |  H    0X23
   H  |  L  |  L    0X24
   H  |  L  |  H    0X25
   H  |  H  |  L    0X26
   H  |  H  |  H    0X27
*/

#include <Arduino.h>
#include <MCP23008.h>
#include <SoftwareSerial.h>

#define ASCII_0 0x30
#define ASCII_1 0x31
#define ASCII_2 0x32
#define ASCII_3 0x33
#define ASCII_4 0x34
#define ASCII_5 0x35
#define ASCII_6 0x36
#define ASCII_7 0x37
#define ASCII_8 0x38
#define ASCII_9 0x39

#define ASCII_STX 0x02
#define ASCII_SP 0x20
#define ASCII_SEMI 0x3B
#define ASCII_CR 0x0D
#define ASCII_LF 0x0A

#define ASCII_A 0x41
#define ASCII_B 0x42
#define ASCII_C 0x43
#define ASCII_D 0x44
#define ASCII_E 0x45
#define ASCII_F 0x46
#define ASCII_G 0x47
#define ASCII_H 0x48
#define ASCII_I 0x49
#define ASCII_J 0x4A
#define ASCII_K 0x4B
#define ASCII_L 0x4C
#define ASCII_M 0x4D
#define ASCII_N 0x4E
#define ASCII_O 0x4F
#define ASCII_P 0x50
#define ASCII_Q 0x51
#define ASCII_R 0x52
#define ASCII_S 0x53
#define ASCII_T 0x54
#define ASCII_U 0x55
#define ASCII_V 0x56
#define ASCII_W 0x57
#define ASCII_X 0x58
#define ASCII_Y 0x59
#define ASCII_Z 0x5A


// Pin definition
#define diINTpin   35
#define doINTpin   34
#define RS232_RX_1 16  
#define RS232_TX_1 17  
#define RS232_RX_2 18
#define RS232_TX_2 19
#define RS232_RX_3 32
#define RS232_TX_3 25
#define RS232_RX_4 27
#define RS232_TX_4 26

// MCP23007 address
#define diPortAddr 0x20
#define doPortAddr 0x21

// MCP23007 constant
#define MCP_INPUT  0XFF
#define MCP_OUTPUT 0x00
#define MCP_PULLUP 0XFF
#define MCP_PULLUP_DISABLE 0X00
#define MCP_ACTIVE_INT 0xFF
#define MCP_DISABLE_INT 0x00

// RS232
#define rsHost Serial2            // RS232_1 ~ To PC
#define rsSource Serial1          // RS232_2 ~ To xRay Source
#define rsSourceBaudRate 115200   
#define rsHostBaudRate 57600   

// rsHost receive data packet index 
#define pcStartInd 0
#define pcCntInd 2
#define pcData1Ind 3
#define pcData2Ind 4
#define pcData3Ind 5
#define pcConstOne 12
#define pcEndInd 15

// rsSource receive data packet index 
#define S_STX_Ind 0
#define S_CMD1_Ind 1
#define S_CMD2_Ind 2
#define S_CMD3_Ind 3
#define S_CMD4_Ind 4
#define S_SP_Ind 5
#define S_ARG_Ind 6

// rsHost state definition
#define startBitRecived 0
#define pending 2
#define dataProcessing 1

#define rs232TimeOut 5

// RGB LED pin definition
#define redLedPin 14
#define greenLedPin 12
#define blueLedPin 33

#define LEDon LOW
#define LEDoff HIGH

// Digital output(relay) definition
#define Motor_Foreward 0
#define Motor_Backward 1
#define Alarm_Power 2
#define Device_Power 3
#define Xray_Power 4
#define Xray_LED 5



MCP23008 DI(diPortAddr);
MCP23008 DO(doPortAddr);
TaskHandle_t Task1;

bool doState[8] = {0};
bool lastDoState[8] = {0};
bool powerOffFlag = false;

uint8_t microSwitchState = 0;

uint8_t rsHostState = pending;
uint8_t hostSend4Buff[16] = {0xFE, 0x10, 0x25, 0xD5, 0x00 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0xFF};
uint8_t hostSend8Buff[16] = {0xFE, 0x10, 0x25, 0xD6, 0x00 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0xFF};
uint8_t hostInputBuff[16] = {0xFE, 0x00, 0x00, 0x00, 0x00 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF};
uint8_t hostSamplePacket[16] = {0xFE, 0x10, 0x00, 0x00, 0x00 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0xFF};
uint8_t hostReciveByte = 0x00;
uint8_t cnt = 0;
uint8_t diState = 0;

uint8_t rsSourceState = pending;
uint8_t sourceInputBuff[16] = {0x02, 0x00, 0x00, 0x00, 0x00 , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t sourceReciveByte = 0x00;
uint8_t sourceCnt = 0;

uint8_t xrayON_Command[11] = {ASCII_STX ,ASCII_E ,ASCII_N ,ASCII_B ,ASCII_L ,ASCII_SP ,ASCII_1 ,ASCII_SEMI ,0x53 ,ASCII_CR ,ASCII_LF};
uint8_t xrayOFF_Command[11] = {ASCII_STX ,ASCII_E ,ASCII_N ,ASCII_B ,ASCII_L ,ASCII_SP ,ASCII_0 ,ASCII_SEMI ,0x54 ,ASCII_CR ,ASCII_LF};

uint16_t sourceVoltage = 0;
uint16_t sourceCurrent = 0;

uint32_t hostMaxReciveTime = 0; 
uint32_t sourceMaxReciveTime = 0; 
uint32_t startTime = 0;
uint32_t countDownPowerOff = 0;

// Set single Digital Output pin state
void setPinState(MCP23008 handler ,int pin ,bool state) {
  uint8_t outputValue = DO.readInputs();
  if (state) {
    outputValue = outputValue | (0b00000001 << pin);
  } else {
    outputValue = outputValue & ((uint8_t)~(0b00000001 << pin));
  }
  
  
  handler.setOutputs(outputValue);

  lastDoState[pin] = state;
}

// On data receive interrupt (Host) this fonction has been called and collecting data to processing
void rsHost_isr() {
  while (rsHost.available()) {   // is while necessary?
    rsHost.readBytes(&hostReciveByte ,1);
    // Serial.print(hostReciveByte ,HEX);

    switch (rsHostState) {
      case startBitRecived: {
          hostMaxReciveTime = millis();
          hostInputBuff[cnt] = hostReciveByte;
          cnt++;
          if (cnt == 16) {
            // Serial.println("Start trasmmiting data");
            // Serial.println();

            // Data packet has been received and now it needs to be processed and the appropriate action should be taken
            cnt = 0;
            rsHostState = dataProcessing;
          }
        break;
      }

      case pending: {
        if (hostReciveByte == 0xFE) {
          // Serial.println("Got Start bit");
          cnt = 1;
          rsHostState = startBitRecived;
          hostMaxReciveTime = millis();
        } 
        break;
      }
      
      default:
        break;  
    }
  }
}

// On data receive interrupt (xRay Source) this fonction has been called and collecting data to processing
void rsSource_isr() {
  while (rsSource.available()) {   // is while necessary?
    rsSource.readBytes(&sourceReciveByte ,1);
    // Serial.print(sourceReciveByte ,HEX);

    switch (rsSourceState) {
      case startBitRecived: {
          sourceMaxReciveTime = millis();
          sourceInputBuff[sourceCnt] = sourceReciveByte;
          sourceCnt++;
          if (sourceReciveByte == ASCII_CR) {
            // Serial.println("Start trasmmiting data");
            // Serial.println();

            sourceCnt = 0;
            // Data packet has been received and now it needs to be processed and the appropriate action should be taken
            rsSourceState = dataProcessing;
          }
        break;
      }

      case pending: {
        if (sourceReciveByte == ASCII_STX) {
          // Serial.println("Got Start bit");
          sourceCnt = 1;
          rsSourceState = startBitRecived;
          sourceMaxReciveTime = millis();
        } 
        break;
      }
      
      default:
        break;  
    }
  }
  String test = String((char)hostReciveByte) + String((char)hostReciveByte);
}

// Generate CRC 
uint8_t generateCRC(uint8_t packet[]) {
  uint8_t i = 1;
  unsigned int crc = 0;

  // Add <CMD> , <sp> , <ARG> , <;> 
  while (packet[i] != 0x3B) {
    crc += packet[i];
    i++;
  }
  crc += 0x3B;
  
  // twos complement
  crc = -crc;

  // Truncate the result down to the eight least significant bits
  crc = (uint8_t)crc;

  // Clear bit7
  crc &= ~(1 << 7);

  // Set bit6
  crc |= (1 << 6);

  return crc;
}

// First initialize of mcp23007 (digital input & digital output)
void mcpInit(void) {
  /**** DI Configuration ****/
  DI.setPins(MCP_INPUT);
  DI.setPullups(MCP_PULLUP);
  DI.setInterruptOnChange(MCP_ACTIVE_INT);
  // Set comparison state to high so interrupt will be triggered if pin goes low.
  DI.setDefaultComparisonValue(0xFF);      
  // If a bit is set, the corresponding I/O pin is compared against the associated bit in the DEFVAL register. 
  // If a bit value is clear, the corresponding I/O pin is compared against the previous value.
  DI.setInterruptControl(0xFF);
  // Set multiple settings with this function.
  // conf[5]: sequential operation mode:  1 disable,      0 enable
  // conf[4]: slew rate:                  1 disable,      0 enable
  // conf[3]: hardware address:           1 disable,      0 enable
  // conf[2]: configures INT-pin:         1 open-drain,   0 active driver
  // conf[1]: sets INT-pin polarity:      1 active-HIGH,  0 active-LOW
  DI.setConfiguration(0b00111000);

  pinMode(diINTpin, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(diINTpin), di_isr, FALLING);

  /**** DO Configuration ****/
  DO.setPins(MCP_OUTPUT);
  DO.setPullups(MCP_PULLUP_DISABLE);
  DO.setInterruptOnChange(MCP_DISABLE_INT);
  // Set comparison state to high so interrupt will be triggered if pin goes low.
  DO.setDefaultComparisonValue(0x00);      
  // If a bit is set, the corresponding I/O pin is compared against the associated bit in the DEFVAL register. 
  // If a bit value is clear, the corresponding I/O pin is compared against the previous value.
  DO.setInterruptControl(0x00);
  // Set multiple settings with this function.
  // conf[5]: sequential operation mode:  1 disable,      0 enable
  // conf[4]: slew rate:                  1 disable,      0 enable
  // conf[3]: hardware address:           1 disable,      0 enable
  // conf[2]: configures INT-pin:         1 open-drain,   0 active driver
  // conf[1]: sets INT-pin polarity:      1 active-HIGH,  0 active-LOW
  DO.setConfiguration(0b00111000);
}

// initialize of RGB LED
void pinInit(void) {
  pinMode(redLedPin ,OUTPUT);
  digitalWrite(redLedPin ,LEDoff);
  pinMode(greenLedPin ,OUTPUT);
  digitalWrite(greenLedPin ,LEDoff);
  pinMode(blueLedPin ,OUTPUT);
  digitalWrite(blueLedPin ,LEDoff);
}

void rs232Init(void) {
  rsHost.begin(rsHostBaudRate);
  rsHost.onReceive(rsHost_isr ,true);

  rsSource.begin(rsSourceBaudRate ,SERIAL_8N1 ,18 ,19);
  rsSource.onReceive(rsSource_isr ,true);
}

// Run on core 0 : Receive --> ~(data of rsHost and rsSource)~ | Send --> ~(processing and respond of rsHost)~
void Task1code( void * parameter) {

  rs232Init();

  for(;;) {
    if (rsHostState == dataProcessing) {
      switch (hostInputBuff[pcData1Ind]) {
        // Read Obstacle detection sensor   ***
        case 0xD5: {
          // Send ACK to Host
          hostSamplePacket[pcCntInd] = hostInputBuff[pcCntInd];
          hostSamplePacket[pcData3Ind] =  microSwitchState;
          for (size_t i = 0; i < 16; i++) {
            rsHost.write(hostSamplePacket[i]);
            // Serial.print(hostSamplePacket[i] ,HEX);
          }

          break;
        }

        // ?
        case 0xD6: {
          // Send ACK to Host
          hostSend8Buff[pcCntInd] = hostInputBuff[pcCntInd];
          for (size_t i = 0; i < 16; i++) {
            rsHost.write(hostSend8Buff[i]);
            // Serial.print(hostSend8Buff[i] ,HEX);
          }

          break;
        }
        
        // Motor Foreward.   ***
        case 0xB1: {
          // Send ACK to Host
          hostInputBuff[pcConstOne] = 0x01;
          for (size_t i = 0; i < 16; i++) {
            rsHost.write(hostInputBuff[i]);
            // Serial.print(hostInputBuff[i] ,HEX);
          }
          
          doState[Motor_Foreward] = HIGH;
          doState[Motor_Backward] = LOW;

          break;
        }

        // Motor Backward.   ***
        case 0xB2: {
          // Send ACK to Host
          hostInputBuff[pcConstOne] = 0x01;
          for (size_t i = 0; i < 16; i++) {
            rsHost.write(hostInputBuff[i]);
            // Serial.print(hostInputBuff[i] ,HEX);
          }

          doState[Motor_Foreward] = LOW;
          doState[Motor_Backward] = HIGH;
          break;
        }

        // Motor Stop.  ***
        case 0xB3: {
          // Send ACK to Host
          hostInputBuff[pcConstOne] = 0x01;
          for (size_t i = 0; i < 16; i++) {
            rsHost.write(hostInputBuff[i]);
            // Serial.print(hostInputBuff[i] ,HEX);
          }

          doState[Motor_Foreward] = LOW;
          doState[Motor_Backward] = LOW;

          break;
        }

        // x-Ray On.    **
        case 0xC4: {
          // Send ACK to Host
          hostInputBuff[pcConstOne] = 0x01;
          size_t arraySize = sizeof(hostInputBuff);
          for (size_t i = 0; i < arraySize; i++) {
            rsHost.write(hostInputBuff[i]);
            // Serial.print(hostInputBuff[i] ,HEX);
          }

          doState[Xray_Power] = HIGH;
          doState[Xray_LED] = HIGH;

          // Send packet to Source
          arraySize = sizeof(xrayON_Command);
          for (size_t i = 0; i < arraySize; i++) {
            rsSource.write(xrayON_Command[i]);
            // Serial.print(xrayON_Command[i] ,HEX);
          }

          break;
        }

        // x-Ray off.   **
        case 0xC5: {
          // Send ACK to Host
          hostInputBuff[pcConstOne] = 0x01;
          size_t arraySize = sizeof(hostInputBuff);
          for (size_t i = 0; i < arraySize; i++) {
            rsHost.write(hostInputBuff[i]);
            // Serial.print(hostInputBuff[i] ,HEX);
          }

          doState[Xray_Power] = LOW;
          doState[Xray_LED] = LOW;

          // Send packet to Source
          arraySize = sizeof(hostInputBuff);
          for (size_t i = 0; i < arraySize; i++) {
            rsSource.write(xrayOFF_Command[i]);
            // Serial.print(xrayOFF_Command[i] ,HEX);
          }

          break;
        }

        // x-Ray Set Voltage.
        case 0xC2: {
          // Send ACK to Host
          hostInputBuff[pcConstOne] = 0x01;
          sourceVoltage = ((uint16_t)hostInputBuff[pcData2Ind] << 8) + hostInputBuff[pcData3Ind];
          // Serial.print("sourceVoltage : "); Serial.println(sourceVoltage);
          for (size_t i = 0; i < 16; i++) {
            rsHost.write(hostInputBuff[i]);
            // Serial.print(hostInputBuff[i] ,HEX);
          }

          break;
        }
        
        // x-Ray Set Current.
        case 0xC3: {
          // Send ACK to Host
          hostInputBuff[pcConstOne] = 0x01;
          sourceCurrent = ((uint16_t)hostInputBuff[pcData2Ind] << 8) + hostInputBuff[pcData3Ind];
          // Serial.print("sourceCurrent : "); Serial.println(sourceCurrent);
          for (size_t i = 0; i < 16; i++) {
            rsHost.write(hostInputBuff[i]);
            // Serial.print(hostInputBuff[i] ,HEX);
          }

          break;
        }

        // Alarm On.    ***
        case 0xA9: {
          // Send ACK to Host
          hostInputBuff[pcConstOne] = 0x01;
          for (size_t i = 0; i < 16; i++) {
            rsHost.write(hostInputBuff[i]);
            // Serial.print(hostInputBuff[i] ,HEX);
          }

          doState[Alarm_Power] = HIGH;

          break;
        }

        // Alarm Off.    ***
        case 0xAA: {
          // Send ACK to Host
          hostInputBuff[pcConstOne] = 0x01;
          for (size_t i = 0; i < 16; i++) {
            rsHost.write(hostInputBuff[i]);
            // Serial.print(hostInputBuff[i] ,HEX);
          }

          doState[Alarm_Power] = LOW;

          break;
        }

        // Other command    ***
        default: {
          // Send ACK of other command (not used) to Host
          hostInputBuff[pcConstOne] = 0x01;
          for (size_t i = 0; i < 16; i++) {
            rsHost.write(hostInputBuff[i]);
            // Serial.print(hostInputBuff[i] ,HEX);
          }

          // Serial.println("default");
          break;
        }
      }

      // Serial.println();
      // Serial.println();
      // Serial.println();

      rsHostState = pending;
    }

    // Check receive data timeout on rsHost
    if (rsHostState == startBitRecived) {
      if ((millis() - hostMaxReciveTime) > rs232TimeOut) {
        rsHostState = pending;
        Serial.println("TimeOut");
      }
    }
    
    vTaskDelay(1);
  }
}


void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinInit();

  xTaskCreatePinnedToCore(
      Task1code, /* Function to implement the task */
      "Task1", /* Name of the task */
      100000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &Task1,  /* Task handle. */
      0); /* Core where the task should run */

  mcpInit();
  
  doState[Device_Power] = HIGH;
  setPinState(DO ,Device_Power ,doState[Device_Power]);

  digitalWrite(greenLedPin ,LEDon);
  startTime = millis();

}

void loop() {
  if ((millis() - startTime) > 500) {
    diState = DI.readInputs();

    // hostSend4Buff[pcData3Ind] = (((uint8_t)~(diState)) & 0b00011111);
    // hostSend8Buff[pcData2Ind] = (((uint8_t)~(diState)) & 0b11100000);

    microSwitchState = (((uint8_t)~(diState)) >> 4);

    for (size_t i = 0; i < 8; i++) {
      if (lastDoState[i] != doState[i]) {
        setPinState(DO ,i ,doState[i]);
      }
    }

    if (hostSend8Buff[pcData2Ind] > 0) {
      powerOffFlag = HIGH;
      countDownPowerOff = millis();
    }

    startTime = millis();
  }

  if (powerOffFlag) {
    if ((millis() - countDownPowerOff) > 30000)  {
      setPinState(DO ,Device_Power ,LOW);
      Serial.println("This Never Been Run.");
      powerOffFlag = false;
    }
  }
}
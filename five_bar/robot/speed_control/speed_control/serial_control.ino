#include <Arduino.h>
#include <SPI.h>
#include "mcp_can.h"

#define CAN_INT 9
#define CS_PIN 10
MCP_CAN CAN(CS_PIN);

const int BUFFER_SIZE = 8;  // Each message is 8 bytes
const byte START_BYTE = 0xAB, STOP_BYTE = 0xBA;

byte receivedData1[BUFFER_SIZE];
byte receivedData2[BUFFER_SIZE];

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) 
    Serial.println("MCP2515 Initialized Successfully!");
  else {
    Serial.println("Error Initializing MCP2515...");
    while (1);
  }

  CAN.setMode(MCP_NORMAL);
  pinMode(CAN_INT, INPUT_PULLUP);
}

void loop() {
  // Ensure we have a full packet (1 start byte + 16 data bytes + 1 stop byte)
  if (Serial.available() >= 18) {
    byte startByte = Serial.read();
    if (startByte == START_BYTE) {
      Serial.readBytes(receivedData1, BUFFER_SIZE);
      Serial.readBytes(receivedData2, BUFFER_SIZE);
      byte stopByte = Serial.read();

      if (stopByte == STOP_BYTE) {
        // Send First Message
        if (CAN.sendMsgBuf(0x141, 0, BUFFER_SIZE, receivedData1) == CAN_OK)
          Serial.println("Motor 1 Command Sent Successfully!");
        else
          Serial.println("Error Sending Motor 1 Command...");

        // Send Second Message
        if (CAN.sendMsgBuf(0x142, 0, BUFFER_SIZE, receivedData2) == CAN_OK)
          Serial.println("Motor 2 Command Sent Successfully!");
        else
          Serial.println("Error Sending Motor 2 Command...");
      }
    }
  }

  // Read from CANBUS
  if (!digitalRead(CAN_INT)) {
    long unsigned int rxId;
    unsigned char len;
    byte rxBuf[BUFFER_SIZE];

    CAN.readMsgBuf(&rxId, &len, rxBuf);
    
    Serial.print((rxId & 0x80000000) ? "Extended ID: 0x" : "Standard ID: 0x");
    Serial.print(rxId & 0x1FFFFFFF, HEX);
    Serial.print(" DLC: ");
    Serial.print(len);
    Serial.print(" Data:");

    for (byte i = 0; i < len; i++) {
      Serial.print(" 0x");
      Serial.print(rxBuf[i], HEX);
    }
    Serial.println();
  }
}

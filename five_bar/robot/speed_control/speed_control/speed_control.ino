#include <Arduino.h>
#include <SPI.h>
#include "mcp_can.h"

#define CAN_INT 2
#define cs_pin 10
MCP_CAN CAN(cs_pin);

const int BUFFER_SIZE = 8;

const byte START_BYTE_1 = 0xAB;  // Start-of-message marker
const byte STOP_BYTE_1 = 0xBA;   // End-of-message marker

const byte START_BYTE_2 = 0xAC;  // Start-of-message marker
const byte STOP_BYTE_2 = 0xCA;   // End-of-message marker

byte receivedData[BUFFER_SIZE];
byte responseData[BUFFER_SIZE] = {10, 20, 30, 40, 50, 60, 70, 80}; // Example response data

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];
byte sndStat;
byte start;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) { // Set baud rate to 1 Mbps
    Serial.println("MCP2515 Initialized Successfully!");
  }
  else  {
    Serial.println("Error Initializing MCP2515...");
    while (1); // Detiene el programa si hay un error en la inicialización
  }
  CAN.setMode(MCP_NORMAL);
  pinMode(CAN_INT, INPUT);
}

void loop() {

  //Write on CANBUS (if there is a message from the serial port)
  if (Serial.available() >= BUFFER_SIZE + 2) { // +2 for start and stop bytes
    start=Serial.read();
    if (start == START_BYTE_1) { // Check for start byte
      Serial.readBytes(receivedData, BUFFER_SIZE); // Read message
      if (Serial.read() == STOP_BYTE_1) { // Check for stop byte
        // Send response with framing
        //Serial.write(START_BYTE);  // Start byte
        //Serial.write(receivedData, BUFFER_SIZE); // Data
        //Serial.write(STOP_BYTE);  // Stop byte

        sndStat = CAN.sendMsgBuf(0x142, 0, 8, receivedData);
        if (sndStat == CAN_OK) {
          Serial.println("Command Sent Successfully!");
        }
        else {
          Serial.println("Error Sending Speed Command...");
        }
        Serial.println();
      }
    }
    else if (start == START_BYTE_2){
      Serial.readBytes(receivedData, BUFFER_SIZE); // Read message
      if (Serial.read() == STOP_BYTE_2) { // Check for stop byte
        // Send response with framing
        //Serial.write(START_BYTE);  // Start byte
        //Serial.write(receivedData, BUFFER_SIZE); // Data
        //Serial.write(STOP_BYTE);  // Stop byte

        sndStat = CAN.sendMsgBuf(0x141, 0, 8, receivedData);
        if (sndStat == CAN_OK) {
          Serial.println("Command Sent Successfully!");
        }
        else {
          Serial.println("Error Sending Speed Command...");
        }
        Serial.println();
      }
    }
  }

  //Read from CANBUS
  if (!digitalRead(CAN_INT)) {
    CAN.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)

    if ((rxId & 0x80000000) == 0x80000000)    // Determine if ID is standard (11 bits) or extended (29 bits)
      sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
    else
      sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);

    Serial.print(msgString);

    if ((rxId & 0x40000000) == 0x40000000) {  // Determine if message is a remote request frame.
      sprintf(msgString, " REMOTE REQUEST FRAME");
      Serial.print(msgString);
    } else {
      for (byte i = 0; i < len; i++) {
        sprintf(msgString, " 0x%.2X", rxBuf[i]);
        Serial.print(msgString);
      }
    }

    Serial.println();
  }
}

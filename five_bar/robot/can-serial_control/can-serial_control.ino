#include <Arduino.h>
#include <SPI.h>
#include "mcp_can.h"
#include "SerialTransfer.h"

#define CAN_INT 9
#define CS_PIN 10
#define CAN_BUFFER_SIZE 8  // Each CAN message is 8 bytes

MCP_CAN CAN(CS_PIN);
SerialTransfer myTransfer;

const int data_size = 8; // Payload size (excluding motor ID)
uint32_t messageBytes[data_size + 1]; // messageBytes[0] = motorID, [1-8] = data
uint8_t dataBytes[data_size];

long unsigned int rxId;
unsigned char len;
byte rxBuf[CAN_BUFFER_SIZE];

const int motor1_ID = 0x00;
const int motor2_ID = 0x01;

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for serial to be ready (useful for Leonardo/ESP32)

  if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) != CAN_OK) {
    while (1); // Halt if CAN initialization fails
  }

  CAN.setMode(MCP_NORMAL);  // Set to normal operation mode
  pinMode(CAN_INT, INPUT_PULLUP);  // Configure CAN interrupt pin

  myTransfer.begin(Serial);  // Start SerialTransfer library
}

void loop() {
  uint16_t index = 0;

  // ===== Receive command from Python via SerialTransfer =====
  if (myTransfer.available()) {
    index = myTransfer.rxObj(messageBytes, index);

    // Extract dataBytes from messageBytes (skip first element: motor ID)
    for (int i = 0; i < data_size; i++) {
      dataBytes[i] = (uint8_t) messageBytes[i + 1];
    }

    // Send data to the appropriate motor ID over CAN
    if (messageBytes[0] == motor1_ID) {
      CAN.sendMsgBuf(0x141, 0, CAN_BUFFER_SIZE, dataBytes);
    } else if (messageBytes[0] == motor2_ID) {
      CAN.sendMsgBuf(0x142, 0, CAN_BUFFER_SIZE, dataBytes);
    }
  }

  // ===== Receive response from CAN devices =====
  if (!digitalRead(CAN_INT)) {
    CAN.readMsgBuf(&rxId, &len, rxBuf);

    // Add motor ID as first byte based on received CAN ID
    if (rxId == 0x241) {
      messageBytes[0] = motor1_ID;
    } else if (rxId == 0x242) {
      messageBytes[0] = motor2_ID;
    } else {
      return;  // Ignore messages not from expected IDs
    }

    // Copy CAN payload into messageBytes[1..8]
    for (int i = 0; i < data_size; i++) {
      messageBytes[i + 1] = (uint32_t) rxBuf[i];
    }

    // Send the message back to Python
    index = 0;
    index = myTransfer.txObj(messageBytes, index);
    myTransfer.sendData(index);
  }
}

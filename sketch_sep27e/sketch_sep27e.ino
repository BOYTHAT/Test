#include <Wire.h>
#include <Nextion.h>
#include "NexText.h"
#include "NexNumber.h"

// Nextion objects
NexNumber nUltra = NexNumber(0, 1, "n0");
NexNumber nLDR = NexNumber(0, 2, "n1");
NexNumber nTemp = NexNumber(0, 3, "n2");
NexNumber nHumi = NexNumber(0, 4, "n3");

NexText tData1 = NexText(0, 5, "t0");
NexText tData2 = NexText(0, 6, "t1");

// Variables
int switch_pos = 0;
byte received_data[4];
int pushButtonState = HIGH;
uint16_t valueToSend = 110;
const int pushButtonPin = 7;
const int slaveAddresses[] = { 8, 9 };

// Timing variables
uint16_t ultrasonicValue = 0;
uint16_t ldrValue = 0;
uint8_t extraData1 = 0;
unsigned long lastUpdateTime = 0;
unsigned long lastButtonCheckTime = 0;
const unsigned long updateInterval = 1000;     // 1 second for sensor updates
const unsigned long buttonCheckInterval = 10;  // 200ms for button and switch checking

void setup() {
  Serial.begin(9600);
  Wire.begin();
  nexInit();  // Initialize Nextion display
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pushButtonPin, INPUT_PULLUP);
}

void loop() {
  unsigned long currentTime = millis();

  Serial.print(ultrasonicValue);
  Serial.print(ldrValue);
  Serial.println(extraData1);
  if (currentTime - lastUpdateTime <= updateInterval) {
    lastUpdateTime = currentTime;
    received_sensor();
  }

  // Check for button press and switch control every `buttonCheckInterval`
  if (currentTime - lastButtonCheckTime >= buttonCheckInterval) {
    lastButtonCheckTime = currentTime;
    handleButtonAndSwitch();
  }

  // Check if Serial data is available and process it
  while (Serial.available() > 0) {
    switch_control();
  }
}

void received_sensor() {
  uint16_t ultrasonicValue = 0, ldrValue = 0;
  uint8_t extraData1 = 0;
  requestValuesFromSlave1(slaveAddresses[0], ultrasonicValue, ldrValue, extraData1);
  if (extraData1 > 0) {
    String message = " Slave1 Ready";
    tData1.setText(message.c_str());
  } else {
    tData1.setText("");
  }
  nUltra.setValue(ultrasonicValue);
  nLDR.setValue(ldrValue);

  // Slave 2 data (Temperature and Humidity)
  uint16_t temperature = 0, humidity = 0;
  uint8_t extraData2 = 0;
  requestValuesFromSlave2(slaveAddresses[1], temperature, humidity, extraData2);
  if (extraData2 > 0) {
    String message = " Slave2 Ready";
    tData2.setText(message.c_str());
  } else {
    tData2.setText("");
  }
  nTemp.setValue(temperature);
  nHumi.setValue(humidity);
}
// Handle push button and switch state
void handleButtonAndSwitch() {
  // Read push button state directly
  int currentButtonState = digitalRead(pushButtonPin);
  if (currentButtonState == LOW) {
    pushButtonState = 1;                                   // Set button state to 1
    digitalWrite(LED_BUILTIN, HIGH);                       // Turn LED ON
    sendValueToSlave(slaveAddresses[0], pushButtonState);  // Send button state to slave 1
  } else if (switch_pos == 1) {
    switch_pos = 1;
    pushButtonState = 1;                                   // Set pushButtonState to 1 if switch_pos is 1
    digitalWrite(LED_BUILTIN, HIGH);                       // Turn LED ON
    sendValueToSlave(slaveAddresses[0], switch_pos);       // Send switch_pos to slave 2
    sendValueToSlave(slaveAddresses[0], pushButtonState);  // Send pushButtonState to slave 2
  } else {
    switch_pos = 0;                                        // Set switch_pos to 0 if no condition is met
    pushButtonState = 0;                                   // Set pushButtonState to 0 (button not pressed)
    sendValueToSlave(slaveAddresses[0], pushButtonState);  // Send pushButtonState to slave 2
    digitalWrite(LED_BUILTIN, LOW);                        // Turn LED OFF
    sendValueToSlave(slaveAddresses[0], switch_pos);       // Send switch_pos to slave 2
  }
  Serial.println(pushButtonState);
}

void sendValueToSlave(int slaveAddress, uint16_t value) {
  Wire.beginTransmission(slaveAddress);
  Wire.write((byte)(value >> 8));
  Wire.write((byte)value);
  Wire.endTransmission();
}

// Request data from slave 1
void requestValuesFromSlave1(int slaveAddress, uint16_t &ultrasonicValue, uint16_t &ldrValue, uint8_t &extraData) {
  Wire.requestFrom(slaveAddress, 5);
  if (Wire.available() >= 5) {
    ultrasonicValue = (Wire.read() << 8) | Wire.read();
    ldrValue = (Wire.read() << 8) | Wire.read();
    extraData = Wire.read();
  } else {
    extraData = 0;
  }
}

// Request data from slave 2
void requestValuesFromSlave2(int slaveAddress, uint16_t &temperature, uint16_t &humidity, uint8_t &extraData) {
  Wire.requestFrom(slaveAddress, 5);
  if (Wire.available() >= 5) {
    temperature = (Wire.read() << 8) | Wire.read();
    humidity = (Wire.read() << 8) | Wire.read();
    extraData = Wire.read();
  } else {
    extraData = 0;
  }
}

// Handle switch control via Serial input
void switch_control() {
  int size = Serial.readBytesUntil('\n', received_data, 4);
  if (size > 0 && size <= 4) {
    switch_pos = received_data[0];
  }
}

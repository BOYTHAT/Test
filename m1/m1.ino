#include <Wire.h>
#include <Nextion.h>
#include "NexText.h"
#include "NexNumber.h"
 
NexNumber nUltra = NexNumber(0, 1, "n0");
NexNumber nLDR = NexNumber(0, 2, "n1");
NexNumber nTemp = NexNumber(0, 3, "n2");
NexNumber nHumi = NexNumber(0, 4, "n3");

NexText tData1 = NexText(0, 5, "t0");
NexText tData2 = NexText(0, 6, "t1");
 
int switch_pos = 0;
byte received_data[4];
int pushButtonState = HIGH;
uint16_t valueToSend = 110;
const int pushButtonPin = 7;
const int slaveAddresses[] = { 8, 9 };

unsigned long lastButtonCheckTime = 0;
unsigned long lastSensorUpdateTime = 0;
unsigned long lastNextionUpdateTime = 0;        
const unsigned long buttonCheckInterval = 50;     
const unsigned long sensorUpdateInterval = 1000;   
const unsigned long nextionUpdateInterval = 2000;   
 
uint16_t lastUltrasonicValue = 0, lastLdrValue = 0;
uint16_t lastTemperature = 0, lastHumidity = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  nexInit();  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pushButtonPin, INPUT_PULLUP);
}

void loop() {

  unsigned long currentTime = millis();  

  if (currentTime - lastButtonCheckTime >= buttonCheckInterval) {
    handleButtonAndSwitch();
    lastButtonCheckTime = currentTime;   
  }

   
  if (currentTime - lastSensorUpdateTime >= sensorUpdateInterval) {
    updateSensorData();
    lastSensorUpdateTime = currentTime;  
  }

 
  if (currentTime - lastNextionUpdateTime >= nextionUpdateInterval) {
    transmitToNextionIfChanged();
    lastNextionUpdateTime = currentTime;  
  }
 

  if (Serial.available() > 0) {
    switch_control();
  }

}

void handleButtonAndSwitch() {
 
  int currentButtonState = digitalRead(pushButtonPin);

  if (currentButtonState == LOW) {
    currentButtonState = 1;                                    
    digitalWrite(LED_BUILTIN, HIGH);                           
    sendValueToSlave(slaveAddresses[0], currentButtonState);  
    Serial.println(currentButtonState);                        
  } else if (switch_pos == 1) {
    currentButtonState = 1;
    switch_pos = 1;
                                                
    digitalWrite(LED_BUILTIN, HIGH);                          
    sendValueToSlave(slaveAddresses[0], switch_pos);         
    sendValueToSlave(slaveAddresses[0], currentButtonState);   
  } else {
    switch_pos = 0;                                           
    currentButtonState = 0;                                   
    sendValueToSlave(slaveAddresses[0], currentButtonState);  
    digitalWrite(LED_BUILTIN, LOW);                            
    sendValueToSlave(slaveAddresses[0], switch_pos);           
  }
}

void updateSensorData() {
 
  uint16_t ultrasonicValue = 0, ldrValue = 0;
  uint8_t extraData1 = 0;
  requestValuesFromSlave1(slaveAddresses[0], ultrasonicValue, ldrValue, extraData1);

  // Print the received data from Slave 1
  Serial.print("Slave 1 - Ultrasonic: ");
  Serial.print(ultrasonicValue);
  Serial.print(", LDR: ");
  Serial.print(ldrValue);
  Serial.print(", Extra Data: ");
  Serial.println(extraData1);

  // Slave 2 data (Temperature and Humidity)
  uint16_t temperature = 0, humidity = 0;
  uint8_t extraData2 = 0;
  requestValuesFromSlave2(slaveAddresses[1], temperature, humidity, extraData2);

  // Print the received data from Slave 2
  // Serial.print("Slave 2 - Temperature: ");
  // Serial.print(temperature);
  // Serial.print(", Humidity: ");
  // Serial.print(humidity);
  // Serial.print(", Extra Data: ");
  // Serial.println(extraData2);
}

void transmitToNextionIfChanged() {
 
  uint16_t ultrasonicValue = 0, ldrValue = 0;
  uint8_t extraData1 = 0;
  requestValuesFromSlave1(slaveAddresses[0], ultrasonicValue, ldrValue, extraData1);
  if (extraData1 > 0) {
    String message = "S1.done!";
    tData1.setText(message.c_str());
  } else {
    tData1.setText("");
  }
  // Check if values have changed   || ldrValue != lastLdrValue
  if (ultrasonicValue != lastUltrasonicValue) {
    nUltra.setValue(ultrasonicValue);
    nLDR.setValue(ldrValue);
 
    lastUltrasonicValue = ultrasonicValue;
    lastLdrValue = ldrValue;

    // Serial.println(" ");
  }

 
  uint16_t temperature = 0, humidity = 0;
  uint8_t extraData2 = 0;
  requestValuesFromSlave2(slaveAddresses[1], temperature, humidity, extraData2);
  if (extraData2 > 0) {
    String message = "S2 done!";
    tData2.setText(message.c_str());
  } else {
    tData2.setText("");
  }
 
  if (temperature != lastTemperature || humidity != lastHumidity) {
    nTemp.setValue(temperature);
    nHumi.setValue(humidity);

    lastTemperature = temperature;
    lastHumidity = humidity;

    // Serial.println(" ");
  }
}
 
void sendValueToSlave(int slaveAddress, uint16_t value) {
  Wire.beginTransmission(slaveAddress);
  Wire.write((byte)(value >> 8));
  Wire.write((byte)value);
  Wire.endTransmission();
}
 
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
 
void switch_control() {
  if (Serial.available() > 0) {
    int size = Serial.readBytesUntil('\n', received_data, 4);
    if (size > 0 && size <= 4) {
      switch_pos = received_data[0];
    }
  }
}

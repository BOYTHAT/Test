#include <Wire.h>

#define SDA_PIN 36
#define SCL_PIN 35
#define LED_PIN 5

int switch_pos = 0;
int slaveAddress = 8;
uint16_t ultrasonicValue = 0;
uint16_t ldrValue = 0;
uint8_t data = 1;
unsigned long lastUpdateTime = 0;
unsigned long lastUpdateTime1 = 0;
const unsigned long updateInterval = 1000;
const unsigned long updateInterval1 = 50;
const int trigPin = 15;
const int echoPin = 16;
const int ldrPin = 13;

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Wire.begin(slaveAddress, SDA_PIN, SCL_PIN, 400000);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastUpdateTime1 >= updateInterval1) {
    receiveData();
    lastUpdateTime1 = currentTime;
  }

  if (currentTime - lastUpdateTime >= updateInterval) {

    ultrasonicValue = getUltrasonicDistance();
    transmitLDR();
    lastUpdateTime = currentTime;
  }
}

void transmitLDR() {

  ldrValue = analogRead(ldrPin);
  Serial.print(" |LDR : ");
  Serial.print(ldrValue);
  Serial.print(", Extra Data: ");
  Serial.print(data);
}

void receiveData() {
  if (switch_pos == 0) {
    digitalWrite(LED_PIN, HIGH);
  } else if (switch_pos == 1) {
    digitalWrite(LED_PIN, LOW);
  }

  Serial.print(" | Slave 2 - received switch_pos: ");
  Serial.println(switch_pos);
}

uint16_t getUltrasonicDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  uint16_t distance = duration * 0.034 / 2;
  return distance;
  Serial.print(" Ultrasonic: ");
  Serial.print(distance);
  Serial.print(" cm");
}

void receiveEvent(int numBytes) {
  if (Wire.available() >= 2) {
    uint16_t highByte = Wire.read();
    uint16_t lowByte = Wire.read();
    uint16_t receivedValue = (highByte << 8) | lowByte;
    switch_pos = receivedValue;
  }
}

void requestEvent() {

  Wire.write((byte)(ultrasonicValue >> 8));
  Wire.write((byte)ultrasonicValue);
  Wire.write((byte)(ldrValue >> 8));
  Wire.write((byte)ldrValue);
  Wire.write(data);
}

#include <Wire.h>
#include <ESP32Servo.h>
#include <EEPROM.h>

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

// Solar tracker variables
const int ldrTopLeftPin = 9;
const int ldrTopRightPin = 13;
const int ldrBottomLeftPin = 10;
const int ldrBottomRightPin = 12;
const int servoXPin = 47;  // X-axis (Horizontal left-right movement)
const int servoYPin = 48;  // Y-axis (Vertical up-down movement)
const int eepromAddressX = 0;
const int eepromAddressY = 1;
const long ldrInterval = 200;
const long servoInterval = 20;
const int threshold = 5;
const int servoMin = 0;
const int servoMax = 180;
const int ldrMaxValue = 4095;
const int ldrThreshold = 130;

unsigned long previousLdrMillis = 0;
unsigned long previousServoMillis = 0;
int posX;
int posY;
Servo servoX;
Servo servoY;
int avgTop, avgBottom, avgLeft, avgRight;

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Wire.begin(slaveAddress, SDA_PIN, SCL_PIN, 400000);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  // Solar tracker setup
  EEPROM.begin(2);
  posX = EEPROM.read(eepromAddressX);
  posY = EEPROM.read(eepromAddressY);

  servoX.attach(servoXPin);
  servoY.attach(servoYPin);
  servoX.write(posX);
  servoY.write(posY);
}

void loop() {
  unsigned long currentTime = millis();

  // Solar tracker functionality
  solarTracker();

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
  Serial.print(" | LDR: ");
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
  Serial.print(" Ultrasonic: ");
  Serial.print(distance);
  Serial.print(" cm");
  return distance;
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

void solarTracker() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousLdrMillis >= ldrInterval) {
    previousLdrMillis = currentMillis;

    int ldrTopLeft = analogRead(ldrTopLeftPin);
    int ldrTopRight = analogRead(ldrTopRightPin);
    int ldrBottomLeft = analogRead(ldrBottomLeftPin);
    int ldrBottomRight = analogRead(ldrBottomRightPin);

    avgTop = (ldrTopLeft + ldrTopRight) / 2;
    avgBottom = (ldrBottomLeft + ldrBottomRight) / 2;
    avgLeft = (ldrTopLeft + ldrBottomLeft) / 2;
    avgRight = (ldrTopRight + ldrBottomRight) / 2;

    avgTop = map(avgTop, 0, ldrMaxValue, 0, 180);
    avgBottom = map(avgBottom, 0, ldrMaxValue, 0, 180);
    avgLeft = map(avgLeft, 0, ldrMaxValue, 0, 180);
    avgRight = map(avgRight, 0, ldrMaxValue, 0, 180);

    Serial.print("Top: ");
    Serial.print(avgTop);
    Serial.print(" | Bottom: ");
    Serial.print(avgBottom);
    Serial.print(" | Left: ");
    Serial.print(avgLeft);
    Serial.print(" | Right: ");
    Serial.print(avgRight);
    Serial.print(" | X: ");
    Serial.print(posX);
    Serial.print(" | Y: ");
    Serial.println(posY);
  }

  if (currentMillis - previousServoMillis >= servoInterval) {
    previousServoMillis = currentMillis;

    if (avgTop > ldrThreshold || avgBottom > ldrThreshold || avgLeft > ldrThreshold || avgRight > ldrThreshold) {
      if (abs(avgLeft - avgRight) > threshold) {
        if (avgLeft > avgRight && avgLeft > avgTop && avgLeft > avgBottom) {
          posX = constrain(posX - 1, servoMin, servoMax);
        } else if (avgRight > avgLeft && avgRight > avgTop && avgRight > avgBottom) {
          posX = constrain(posX + 1, servoMin, servoMax);
        }
        servoX.write(posX);
        EEPROM.write(eepromAddressX, posX);
        EEPROM.commit();
      }

      if (abs(avgTop - avgBottom) > threshold) {
        if (avgTop > avgBottom && avgTop > avgLeft && avgTop > avgRight) {
          posY = constrain(posY + 1, 65, 155);
        } else if (avgBottom > avgTop && avgBottom > avgLeft && avgBottom > avgRight) {
          posY = constrain(posY - 1, 65, 155);
        }
        servoY.write(posY);
        EEPROM.write(eepromAddressY, posY);
        EEPROM.commit();
      }
    } else if (avgTop < ldrThreshold && avgBottom < ldrThreshold && avgLeft < ldrThreshold && avgRight < ldrThreshold) {
      if (posY > 110) {
        posY--;
        posY = constrain(posY, 110, 155);
      } else if (posY < 110) {
        posY++;
        posY = constrain(posY, 65, 110);
      }
      if (posX > 90) {
        posX--;
        posX = constrain(posX, 90, 180);
      } else if (posX < 90) {
        posX++;
        posX = constrain(posX, 0, 90);
      }
      servoY.write(posY);
      servoX.write(posX);
      EEPROM.write(eepromAddressX, posX);
      EEPROM.write(eepromAddressY, posY);
      EEPROM.commit();
    }
  }
}

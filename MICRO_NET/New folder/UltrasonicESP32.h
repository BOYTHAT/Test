#ifndef ULTRASONIC_ESP32_H
#define ULTRASONIC_ESP32_H

#include <Arduino.h>

class UltrasonicESP32 {
  public:
    // Constructor to initialize with trigger and echo pins
    UltrasonicESP32(uint8_t trigPin, uint8_t echoPin);

    // Method to measure distance in centimeters
    float readDistanceCM();

    // Method to measure distance in inches
    float readDistanceInches();

  private:
    uint8_t _trigPin;
    uint8_t _echoPin;

    // Method to calculate the duration of the ultrasonic pulse
    long _measurePulseDuration();
};

#endif

#include "UltrasonicESP32.h"

UltrasonicESP32::UltrasonicESP32(uint8_t trigPin, uint8_t echoPin) {
    _trigPin = trigPin;
    _echoPin = echoPin;
    pinMode(_trigPin, OUTPUT);
    pinMode(_echoPin, INPUT);
}

long UltrasonicESP32::_measurePulseDuration() {
    // Ensure trigger pin is LOW
    digitalWrite(_trigPin, LOW);
    delayMicroseconds(2);
    
    // Trigger a 10 microsecond pulse
    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);

    // Measure the duration of the echo pulse
    return pulseIn(_echoPin, HIGH);
}

float UltrasonicESP32::readDistanceCM() {
    long duration = _measurePulseDuration();
    // Calculate distance in cm (Speed of sound is 34300 cm/s)
    return (duration * 0.0343) / 2.0;
}

float UltrasonicESP32::readDistanceInches() {
    long duration = _measurePulseDuration();
    // Calculate distance in inches (Speed of sound is 13503.9 inches/s)
    return (duration * 0.0135039) / 2.0;
}

#ifndef MICRO_NET_H
#define MICRO_NET_H

#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>

// Define pins and constants
#define BUTTON1 2
#define BUTTON2 1
#define RELAY_PIN_1 37
#define RELAY_PIN_2 38
#define LED1 5
#define LED2 6
#define BUZZER 7
#define RGB 4
#define PIN15 15
#define PIN16 16
#define PIN1 3
#define PIN2 8
#define PIN3 14
#define PIN4 21
#define PIN5 39
#define PIN6 42
#define A1PIN 9
#define A2PIN 10
#define A3PIN 11
#define A4PIN 12
#define A5PIN 13
#define SERVO1_PIN 47
#define SERVO2_PIN 48
#define NUMPIXELS 2

class MICRO_NET {
public:
    MICRO_NET();
    void setup();
    void loop();
    void toggleBuzzer();
    void cycleColors();
    void togglePins();
    void toggleServos();
    void PinOUT();
    void AnalogPIN();

    // Public setter methods for servos
    void setServo1Position(int position);
    void setServo2Position(int position);

private:
    Adafruit_NeoPixel pixels;
    Servo servo1, servo2;

    unsigned long previousMillisBuzzer = 0;
    unsigned long previousMillisRGB = 0;
    unsigned long previousMillisPins = 0;
    unsigned long previousMillisServo = 0;
    unsigned long previousMillisPinOUT = 0;
    unsigned long previousMillisAPins = 0;

    const long intervalRGB = 500;
    const long intervalBuzzer = 500;
    const long intervalPins = 500;
    const long intervalServo = 500;
    const long intervalPinOUT = 500;
    const long intervalAPins = 500;

    bool rgbState = false;

    void handleButtons();  // Method to handle button logic
};

#endif

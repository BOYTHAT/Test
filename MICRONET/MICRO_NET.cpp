#include "MICRO_NET.h"

MICRO_NET::MICRO_NET() : pixels(NUMPIXELS, RGB, NEO_GRB + NEO_KHZ800) {
}

void MICRO_NET::setup() {
    Serial.begin(115200);

    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(BUZZER, OUTPUT);
    pinMode(PIN15, OUTPUT);
    pinMode(PIN16, OUTPUT);
    pinMode(RELAY_PIN_1, OUTPUT);
    pinMode(RELAY_PIN_2, OUTPUT);
    pinMode(BUTTON1, INPUT_PULLUP);
    pinMode(BUTTON2, INPUT_PULLUP);

    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);

    pixels.begin();
    pixels.show();

    digitalWrite(RELAY_PIN_1, HIGH);
    digitalWrite(RELAY_PIN_2, HIGH);
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
    digitalWrite(BUZZER, LOW);
    digitalWrite(PIN15, LOW);
    digitalWrite(PIN16, LOW);
}

void MICRO_NET::loop() {
    handleButtons();

    if (rgbState) {
        cycleColors();
    }
}

void MICRO_NET::handleButtons() {
    static int counter1 = 0;
    static int counter = 0;
    static int lastButtonState1 = HIGH;
    static int lastButtonState = HIGH;

    int button1State = digitalRead(BUTTON1);
    int currentButtonState = digitalRead(BUTTON2);

    // Handle Button 1
    if (button1State == LOW && lastButtonState1 == HIGH) {
        counter1++;
        if (counter1 == 1) {
            toggleBuzzer();
            toggleServos();
            rgbState = true;
        } else if (counter1 == 2) {
            togglePins();
            PinOUT();
            AnalogPIN();
            digitalWrite(BUZZER, LOW);
            servo1.write(0);
            servo2.write(0);
            rgbState = false;
            pixels.clear();
            pixels.show();
        } else if (counter1 > 2) {
            digitalWrite(PIN15, LOW);
            digitalWrite(PIN16, LOW);
            counter1 = 0;
        }
    }

    // Handle Button 2
    if (currentButtonState == LOW && lastButtonState == HIGH) {
        counter++;
        if (counter == 1) {
            digitalWrite(RELAY_PIN_1, LOW);
            digitalWrite(LED1, LOW);
        } else if (counter == 2) {
            digitalWrite(RELAY_PIN_2, LOW);
            digitalWrite(LED2, LOW);
        } else if (counter > 2) {
            digitalWrite(RELAY_PIN_1, HIGH);
            digitalWrite(RELAY_PIN_2, HIGH);
            digitalWrite(LED1, HIGH);
            digitalWrite(LED2, HIGH);
            counter = 0;
        }
    }

    lastButtonState1 = button1State;
    lastButtonState = currentButtonState;
}

void MICRO_NET::toggleBuzzer() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisBuzzer >= intervalBuzzer) {
        previousMillisBuzzer = currentMillis;
        digitalWrite(BUZZER, !digitalRead(BUZZER)); // Toggle buzzer state
    }
}

void MICRO_NET::cycleColors() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisRGB >= intervalRGB) {
        previousMillisRGB = currentMillis;
        static bool toggle = false;
        for (int i = 0; i < NUMPIXELS; i++) {
            pixels.setPixelColor(i, toggle ? pixels.Color(255, 0, 255) : pixels.Color(128, 0, 128));
        }
        toggle = !toggle;
        pixels.show();
    }
}

void MICRO_NET::togglePins() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisPins >= intervalPins) {
        previousMillisPins = currentMillis;
        digitalWrite(PIN15, HIGH);
        digitalWrite(PIN16, HIGH);
    }
}

void MICRO_NET::toggleServos() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisServo >= intervalServo) {
        previousMillisServo = currentMillis;
        servo1.write(180);
        servo2.write(180);
    }
}

void MICRO_NET::PinOUT() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisPinOUT >= intervalPinOUT) {
        previousMillisPinOUT = currentMillis;
        digitalWrite(PIN1, HIGH);
        digitalWrite(PIN2, HIGH);
        digitalWrite(PIN3, HIGH);
        digitalWrite(PIN4, HIGH);
        digitalWrite(PIN5, HIGH);
        digitalWrite(PIN6, HIGH);
    }
}

void MICRO_NET::AnalogPIN() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisAPins >= intervalAPins) {
        previousMillisAPins = currentMillis;
        digitalWrite(A1PIN, HIGH);
        digitalWrite(A2PIN, HIGH);
        digitalWrite(A3PIN, HIGH);
        digitalWrite(A4PIN, HIGH);
        digitalWrite(A5PIN, HIGH);
    }
}

// Public setter methods for servos
void MICRO_NET::setServo1Position(int position) {
    servo1.write(position);
}

void MICRO_NET::setServo2Position(int position) {
    servo2.write(position);
}

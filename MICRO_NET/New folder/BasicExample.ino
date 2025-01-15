#include <UltrasonicESP32.h>

// Define trigger and echo pins
#define TRIG_PIN 15
#define ECHO_PIN 16

UltrasonicESP32 ultrasonic(TRIG_PIN, ECHO_PIN);

void setup() {
    Serial.begin(115200);
}

void loop() {
    float distanceCM = ultrasonic.readDistanceCM();
    Serial.print("Distance: ");
    Serial.print(distanceCM);
    Serial.println(" cm");

    delay(1000);
}

#include <SoftWire.h>
#include "Adafruit_BME680.h"

// Define SoftWire pins for software I2C
#define SDA_PIN 15
#define SCL_PIN 16

// Initialize SoftWire instance
SoftWire myWire(SDA_PIN, SCL_PIN);

// Initialize BME680 sensor instance with SoftWire
Adafruit_BME680 bme(&myWire);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Initialize SoftWire (Software I2C)
  myWire.begin();
  Serial.println("SoftWire initialized successfully!");

  // Initialize BME680 sensor with I2C address 0x77
  if (!bme.begin(0x77)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }
  
  Serial.println("BME680 sensor found and initialized!");

  // Configure BME680 settings
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320°C for 150 ms
}

void loop() {
  // Perform a reading
  if (!bme.performReading()) {
    Serial.println("Failed to perform reading!");
    return;
  }

  // Print sensor readings
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" °C");

  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0); // Convert Pa to hPa
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.print("Gas resistance = ");
  Serial.print(bme.readGas());
  Serial.println(" ohms");

  Serial.println();
  delay(2000);
}

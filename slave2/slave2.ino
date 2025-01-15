#include <Wire.h>
#include <DHT.h>  // Include the DHT library

#define DHTPIN 13     // Pin where the DHT11 is connected
#define DHTTYPE DHT11  // Define the type of DHT sensor

#define SDA_PIN 36
#define SCL_PIN 35
#define LED_PIN 5

DHT dht(DHTPIN, DHTTYPE);  // Create an instance of the DHT sensor

int switch_pos = 0;
int slaveAddress = 9;  // I2C address for Slave 2
uint16_t temperature = 0;
uint16_t humidity = 0;
uint8_t data = 2;  // Extra data identifier for Slave 2
unsigned long lastUpdateTime = 0;
unsigned long lastUpdateTime1 = 0;
const unsigned long updateInterval = 1000;
const unsigned long updateInterval1 = 50;

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize I2C and DHT11 sensor
  Wire.begin(slaveAddress, SDA_PIN, SCL_PIN, 400000);  // Initialize I2C for Slave 2
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  dht.begin();  // Initialize the DHT11 sensor
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastUpdateTime1 >= updateInterval1) {
    receiveData();
    lastUpdateTime1 = currentTime;
  }

  if (currentTime - lastUpdateTime >= updateInterval) {
    temperature = getTemperature();
    humidity = getHumidity();
    transmitTempHumidity();
    lastUpdateTime = currentTime;
  }
}

void transmitTempHumidity() {
  Serial.print(" | Temperature : ");
  Serial.print(temperature / 100.0);  // Convert to float for display
  Serial.print("°C, Humidity : ");
  Serial.print(humidity / 100.0);     // Convert to float for display
  Serial.print(" %, Extra Data: ");
  Serial.println(data);
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

uint16_t getTemperature() {
  float temp = dht.readTemperature();  // Read temperature from DHT11
  if (isnan(temp)) {
    Serial.println("Failed to read temperature from DHT sensor!");
    return 0;
  }
  return (uint16_t)(temp * 100);  // Convert to uint16_t by multiplying by 100
}

uint16_t getHumidity() {
  float hum = dht.readHumidity();  // Read humidity from DHT11
  if (isnan(hum)) {
    Serial.println("Failed to read humidity from DHT sensor!");
    return 0;
  }
  return (uint16_t)(hum * 100);  // Convert to uint16_t by multiplying by 100
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
  // Send temperature, humidity, and extra data back to the master
  Wire.write((byte)(temperature >> 8));
  Wire.write((byte)temperature);
  Wire.write((byte)(humidity >> 8));
  Wire.write((byte)humidity);
  Wire.write(data);
}

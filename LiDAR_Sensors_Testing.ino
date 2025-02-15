#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Create instances for each VL53L0X sensor
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

// GPIO pins to control the XSHUT (shutdown) of each lidar
const int lidar1_xshut = 15;
const int lidar2_xshut = 16;
const int lidar3_xshut = 17;

void setup() {
  // Start the Serial Monitor
  Serial.begin(115200);
  Serial.println("VL53L0X Multiple Sensors Test");

  // Initialize I2C communication
  Wire.begin();

  // Set up GPIO pins for controlling sensor XSHUT (shutdown)
  pinMode(lidar1_xshut, OUTPUT);
  pinMode(lidar2_xshut, OUTPUT);
  pinMode(lidar3_xshut, OUTPUT);

  // Set all XSHUT pins to LOW to turn off all sensors
  digitalWrite(lidar1_xshut, LOW);
  digitalWrite(lidar2_xshut, LOW);
  digitalWrite(lidar3_xshut, LOW);
  delay(10);

  // Power on Sensor 1 and set its I2C address
  digitalWrite(lidar1_xshut, HIGH); // Turn on sensor 1
  delay(10); // Wait for the sensor to power up
  if (!lox1.begin()) {
    Serial.println("Failed to detect VL53L0X sensor 1. Check wiring!");
    while (1);
  }
  lox1.setAddress(0x30); // Change address to 0x30
  Serial.println("Sensor 1 initialized at address 0x30");

  // Power on Sensor 2 and set its I2C address
  digitalWrite(lidar2_xshut, HIGH); // Turn on sensor 2
  delay(10); // Wait for the sensor to power up
  if (!lox2.begin()) {
    Serial.println("Failed to detect VL53L0X sensor 2. Check wiring!");
    while (1);
  }
  lox2.setAddress(0x31); // Change address to 0x31
  Serial.println("Sensor 2 initialized at address 0x31");

  // Power on Sensor 3 and set its I2C address
  digitalWrite(lidar3_xshut, HIGH); // Turn on sensor 3
  delay(10); // Wait for the sensor to power up
  if (!lox3.begin()) {
    Serial.println("Failed to detect VL53L0X sensor 3. Check wiring!");
    while (1);
  }
  lox3.setAddress(0x32); // Change address to 0x32
  Serial.println("Sensor 3 initialized at address 0x32");

  // Keep all sensors powered on
  digitalWrite(lidar1_xshut, HIGH); // Ensure sensor 1 is on
  digitalWrite(lidar2_xshut, HIGH); // Ensure sensor 2 is on
  digitalWrite(lidar3_xshut, HIGH); // Ensure sensor 3 is on
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;

  // Sensor 1 reading
  lox1.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    Serial.print("Sensor 1 Distance: ");
    Serial.print(measure.RangeMilliMeter / 10.0);
    Serial.println(" cm");
  } else {
    Serial.println("Sensor 1 Out of range.");
  }

  // Sensor 2 reading
  lox2.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    Serial.print("Sensor 2 Distance: ");
    Serial.print(measure.RangeMilliMeter / 10.0);
    Serial.println(" cm");
  } else {
    Serial.println("Sensor 2 Out of range.");
  }

  // Sensor 3 reading
  lox3.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    Serial.print("Sensor 3 Distance: ");
    Serial.print(measure.RangeMilliMeter / 10.0);
    Serial.println(" cm");
  } else {
    Serial.println("Sensor 3 Out of range.");
  }

  delay(500);
}

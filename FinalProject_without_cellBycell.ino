#include <Wire.h>
#include <MPU6050_tockn.h>
#include <Adafruit_VL53L0X.h>

// Motor control pins
#define IN1  4   // Left motor forward
#define IN2  5   // Left motor backward
#define IN3  35  // Right motor forward
#define IN4  36  // Right motor backward
#define ENA  47  // Left motor speed (PWM)
#define ENB  48  // Right motor speed (PWM)

// I2C and sensor pins
#define SDA_PIN 8
#define SCL_PIN 9
#define XSHUT_FRONT 15  // Front LiDAR XSHUT pin
#define XSHUT_RIGHT 17  // Right LiDAR XSHUT pin
#define XSHUT_LEFT 16   // Left LiDAR XSHUT pin

MPU6050 mpu(Wire);
Adafruit_VL53L0X lidarFront = Adafruit_VL53L0X();
Adafruit_VL53L0X lidarRight = Adafruit_VL53L0X();
Adafruit_VL53L0X lidarLeft = Adafruit_VL53L0X();

// PID variables
float Kp = 1.1, Ki = 0.2, Kd = 0.05;
float yawError = 0, previousYawError = 0;
float integralYaw = 0;
float targetYaw = 0;
float initialYaw = 0;

// State machine
enum States { MOVE_FORWARD, TURN_RIGHT, TURN_LEFT };
States state = MOVE_FORWARD;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Initialize MPU6050
  mpu.begin();
  mpu.calcGyroOffsets(true);
  Serial.println("MPU6050 Initialized");

  // Initialize LiDAR sensors
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);

  // Reset all sensors
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  delay(10);

  // Start front LiDAR
  digitalWrite(XSHUT_FRONT, HIGH);
  delay(10);
  if (!lidarFront.begin(0x30)) {
    Serial.println("Failed to detect Front VL53L0X!");
    while (1);
  }

  // Start right LiDAR
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(10);
  if (!lidarRight.begin(0x31)) {
    Serial.println("Failed to detect Right VL53L0X!");
    while (1);
  }

  // Start left LiDAR
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(10);
  if (!lidarLeft.begin(0x32)) {
    Serial.println("Failed to detect Left VL53L0X!");
    while (1);
  }

  Serial.println("All LiDAR Sensors Initialized");

  // Set motor pins as output
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
}

void loop() {
  mpu.update();
  float currentYaw = mpu.getAngleZ();

  // Read LiDAR distances
  int frontDistance = getDistance(lidarFront);
  int rightDistance = getDistance(lidarRight);
  int leftDistance = getDistance(lidarLeft);

  // Print LiDAR distances
  Serial.print("Front: ");
  Serial.print(frontDistance);
  Serial.print(" cm | Right: ");
  Serial.print(rightDistance);
  Serial.print(" cm | Left: ");
  Serial.println(leftDistance);

  switch (state) {
    case MOVE_FORWARD:
      moveForward(50);
      yawControlPID(currentYaw);

      if (frontDistance <= 8 ) {
        Serial.println("Obstacle detected! Checking directions...");
        if (rightDistance > leftDistance) {
          state = TURN_RIGHT;
        } else {
          state = TURN_LEFT;
        }
      }
      break;

    case TURN_RIGHT:
      turnToDirection(1); // Turn RIGHT
      state = MOVE_FORWARD;
      break;

    case TURN_LEFT:
      turnToDirection(0); // Turn LEFT
      state = MOVE_FORWARD;
      break;
  }
}

// Read LiDAR distance
int getDistance(Adafruit_VL53L0X &sensor) {
  VL53L0X_RangingMeasurementData_t measure;
  sensor.rangingTest(&measure, false);
  return measure.RangeMilliMeter / 10;
}

// Move forward with PID yaw control
void moveForward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void turnToDirection(int direction) {
  initialYaw = mpu.getAngleZ();
  float turnAngle = 84;  // Target turn angle

  if (direction == 1) { // Turn RIGHT
    Serial.println("Turning RIGHT...");
    while (abs(mpu.getAngleZ() - initialYaw) < turnAngle) {
      mpu.update();
      turnRight();
    }
  } else { // Turn LEFT
    Serial.println("Turning LEFT...");
    while (abs(mpu.getAngleZ() - initialYaw) < turnAngle) {
      mpu.update();
      turnLeft();
    }
  }

  stopMotors();  // Stop before alignment
  delay(500);  // Short pause to stabilize
  
  targetYaw = mpu.getAngleZ();  // Set new target angle
  Serial.println("Turn completed. Aligning...");

  alignToYaw(targetYaw);  // Centering before moving forward

  Serial.println("Alignment done. Moving forward.");
}


// Correct yaw misalignment after turning
void correctYaw() {
  float currentYaw = mpu.getAngleZ();
  float error = targetYaw - currentYaw;

  while (abs(error) > 2.0) { // Allowable error margin ±2°
    mpu.update();
    currentYaw = mpu.getAngleZ();
    error = targetYaw - currentYaw;

    if (error > 0) {
      turnSlightRight();
    } else {
      turnSlightLeft();
    }
  }
  stopMotors();
}

// PID yaw control while moving forward
void yawControlPID(float currentYaw) {
  yawError = targetYaw - currentYaw;

  if (abs(yawError) < 2.0) {
    integralYaw += yawError;
  } else {
    integralYaw = 0;
  }

  integralYaw = constrain(integralYaw, -100, 100);
  float derivativeYaw = yawError - previousYawError;
  previousYawError = yawError;

  float correction = (Kp * yawError) + (Ki * integralYaw) + (Kd * derivativeYaw);

  int leftSpeed = 50 - correction;
  int rightSpeed = 50 + correction;

  leftSpeed = constrain(leftSpeed, 50, 150);
  rightSpeed = constrain(rightSpeed, 50, 150);

  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
}

// Right turn
void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 60);
  analogWrite(ENB, 60);
}

// Left turn
void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 60);
  analogWrite(ENB, 60);
}

// Slight right for fine-tuning
void turnSlightRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 60);
  analogWrite(ENB, 60);
  delay(50);
}

// Slight left for fine-tuning
void turnSlightLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 60);
  analogWrite(ENB, 60);
  delay(50);
}

// Stop motors
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
void alignToYaw(float targetYaw) {
  float currentYaw = mpu.getAngleZ();
  float error = targetYaw - currentYaw;

  while (abs(error) > 1.5) {  // Allowable error margin ±1.5°
    mpu.update();
    currentYaw = mpu.getAngleZ();
    error = targetYaw - currentYaw;

    if (error > 0) {
      turnSlightRight();
    } else {
      turnSlightLeft();
    }
  }

  stopMotors();  // Final stop after correction
  delay(300);  // Ensure stability before moving forward
}

#include <Wire.h>
#include <LiquidCrystal.h>
#include <MPU6050.h>

int motorSpeed;
const int rs = 12, en = 11, d4 = 7, d5 = 4, d6 = 13, d7 = 10;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
const int en1_2_pin = 9;

MPU6050 mpu;

#define IN1 5
#define IN2 6

#define ENCODER_A 2
#define ENCODER_B 3

volatile int encoderPos = 0;
const int encoderCount = 415;
const float countsPerDegree = encoderCount / 360.0;

float targetAngle = 0;
int targetCounts = 0;

void setup() {
  lcd.begin(16, 2);
  lcd.print("Motor Control");

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(en1_2_pin, OUTPUT);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, CHANGE);

  Wire.begin();
  mpu.initialize();
  if (mpu.testConnection()) {
    lcd.setCursor(0, 1);
    lcd.print("MPU6050 Ready");
  } else {
    lcd.setCursor(0, 1);
    lcd.print("MPU6050 Error");
    while (true)
      ;
  }

  Serial.begin(9600);
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float pitch = atan2(ay, az) * 180 / PI;

  targetAngle = constrain(pitch, -90, 90);
  targetCounts = targetAngle * countsPerDegree;

  moveToPosition(targetCounts);

  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print(" | Encoder Pos: ");
  Serial.print(encoderPos);
  Serial.print(" | Target: ");
  Serial.println(targetCounts);

  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 0);
  lcd.print("Pos: ");
  lcd.print(encoderPos);

  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("Target: ");
  lcd.print(targetCounts);

  delay(100);
}

void moveToPosition(int target) {
  int error = target - encoderPos;

  int motorSpeed = map(abs(error), 0, encoderCount, 80, 255);
  motorSpeed = constrain(motorSpeed, 50, 255);

  if (abs(error) > 6) {
    if (error > 0) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(en1_2_pin, motorSpeed);
    } else if (error < 0) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(en1_2_pin, motorSpeed);
    }
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(en1_2_pin, 0);
  }
}

void updateEncoder() {
  if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) {
    encoderPos++;
  } else {
    encoderPos--;
  }
}

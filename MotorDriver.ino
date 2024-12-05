#include <LiquidCrystal.h>

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 13, d7 = 10;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

const int potPin = A0;

#define IN1 6
#define IN2 7
#define ENCODER_A 2
#define ENCODER_B 3

int encoderPos = 0;
const int countsPerRevolution = 360;

void setup() {
  lcd.begin(16, 2);
  lcd.print("Motor Control");

  // Motor pins setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);


  pinMode(ENCODER_A, INPUT_PULLUP);
pinMode(ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, CHANGE);

  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  Serial.begin(9600);
}

void loop() {
  int potValue = analogRead(A0);
  int motorSpeed = map(abs(potValue - 512), 0, 511, 0, 255);
  motorSpeed = constrain(motorSpeed, 0, 255);

  if (potValue > 530) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (potValue < 490) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else { // Stop motor
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
  }
  Serial.print("Encoder Position: ");
  Serial.println(encoderPos);

  // Update LCD
  lcd.setCursor(0, 0);
  lcd.print("                "); 
  lcd.setCursor(0, 0);
  lcd.print("Position: ");
  lcd.print(encoderPos, 1);

  lcd.setCursor(0, 1);
  lcd.print("                "); 
  lcd.setCursor(0, 1);
  lcd.print("Speed: ");
  lcd.print(motorSpeed);

  delay(1000);
}

void updateEncoder() {
  if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) {
    encoderPos++;
  } else {
    encoderPos--;
  }
}

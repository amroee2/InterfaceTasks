#define trig 11
#define echo 10
#define ledg 7
#define ledy 6
#define ledr 5
#define pot A4

void setup() {
    Serial.begin(9600);
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
    pinMode(ledg, OUTPUT);
    pinMode(ledy, OUTPUT);
    pinMode(ledr, OUTPUT);
}

void loop() {
    long duration, distance;
    int potValue = analogRead(pot);
    int maxDistance = map(potValue, 0, 1023, 40, 100);
    int yellowThreshold = maxDistance / 2;
    int redThreshold = maxDistance / 4;
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    duration = pulseIn(echo, HIGH);
    distance = (duration / 2) / 29.1;
    if (distance < redThreshold) {
        digitalWrite(ledr, HIGH);
        digitalWrite(ledy, LOW);
        digitalWrite(ledg, LOW);
    } else if (distance < yellowThreshold) {
        digitalWrite(ledr, LOW);
        digitalWrite(ledy, HIGH);
        digitalWrite(ledg, LOW);
    } else {
        digitalWrite(ledr, LOW);
        digitalWrite(ledy, LOW);
        digitalWrite(ledg, HIGH);
    }
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" cm, Max Distance: ");
    Serial.print(maxDistance);
    Serial.print(" cm, Yellow Threshold: ");
    Serial.print(yellowThreshold);
    Serial.print(" cm, Red Threshold: ");
    Serial.println(redThreshold);
    delay(500);
}

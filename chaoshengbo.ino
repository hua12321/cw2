#include <Arduino.h>

#define IN1 A5
#define IN2 3
#define IN3 6
#define IN4 7
#define ENA 2
#define ENB 5

#define TRIG_PIN A1
#define ECHO_PIN A4

#define DISTANCE_THRESHOLD 20

void setup() {
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

 
  analogWrite(ENA, 200);  
  analogWrite(ENB, 200);  

  Serial.begin(9600);  
}

void loop() {
  long distance = getDistance();
  Serial.print("距离: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance < DISTANCE_THRESHOLD) {
    stopMotors();  
  } else {
    moveForward();  
  }

  delay(100);
}

long getDistance() {
  
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  
  long duration = pulseIn(ECHO_PIN, HIGH);

 
  long distance = duration * 0.034 / 2;

  return distance;
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}


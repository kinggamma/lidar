#include <AFMotor.h>


AF_DCMotor motor1(1); // Front-left
AF_DCMotor motor2(2); // Back-left
AF_DCMotor motor3(3); // Front-right
AF_DCMotor motor4(4); // Back-right


const int trigPin = 7;
const int echoPin = 8;

long duration;
float distanceCM;

String command = "";  
String currentCommand = "";

unsigned long lastUltrasonicPrint = 0;
bool ultrasonicTriggeredSent = false;

void setup() {
  Serial.begin(9600);  
  motor1.setSpeed(200);
  motor2.setSpeed(200);
  motor3.setSpeed(200);
  motor4.setSpeed(200);
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

float measureDistanceCM() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  float distance = (duration * 0.0343) / 2;
  return distance;
}

void loop() {
  // Read incoming command (terminated by newline)
  if (Serial.available() > 0) {
    command = Serial.readStringUntil('\n');
    command.trim();
    currentCommand = command;
  }
  
  
  if (command == "STOP") {
    motor1.run(RELEASE);
    motor2.run(RELEASE);
    motor3.run(RELEASE);
    motor4.run(RELEASE);
    ultrasonicTriggeredSent = false;
  }
  else if (command == "FRONT") {
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
    ultrasonicTriggeredSent = false;
  }
  else if (command == "LEFT") {
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
    ultrasonicTriggeredSent = false;
  }
  else if (command == "RIGHT") {
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);
    ultrasonicTriggeredSent = false;
  }
  else if (command == "SPIN") {
    // Spin in place.
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
    ultrasonicTriggeredSent = false;
  }
  else if (command == "BACK") {
    
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);
    // During backup, periodically check ultrasonic sensor.
    if (millis() - lastUltrasonicPrint > 500) {
      distanceCM = measureDistanceCM();
      // If trash is detected within 10 cm, send "ULTRA" to the Pi.
      if (distanceCM < 10.0 && !ultrasonicTriggeredSent) {
        Serial.println("ULTRA");
        ultrasonicTriggeredSent = true;
      }
      lastUltrasonicPrint = millis();
    }
  }
  else if (command == "DUMP") {
    
    motor1.run(RELEASE);
    motor2.run(RELEASE);
    motor3.run(RELEASE);
    motor4.run(RELEASE);
    ultrasonicTriggeredSent = false;
  }
  else {
    motor1.run(RELEASE);
    motor2.run(RELEASE);
    motor3.run(RELEASE);
    motor4.run(RELEASE);
    ultrasonicTriggeredSent = false;
  }
  
  delay(100);
}

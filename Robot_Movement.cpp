#include <AFMotor.h>

// Motor assignments for your motor shield
AF_DCMotor motor1(1); // Front-left
AF_DCMotor motor2(2); // Back-left
AF_DCMotor motor3(3); // Front-right
AF_DCMotor motor4(4); // Back-right

String command = "";  // Stores the last received command

void setup() {
  Serial.begin(9600);  // Must match the Pi's baud rate
  // Set default motor speeds (adjust as needed)
  motor1.setSpeed(200);
  motor2.setSpeed(200);
  motor3.setSpeed(200);
  motor4.setSpeed(200);
}

void loop() {
  // If serial data is available, read a full command line (ending with '\n')
  if (Serial.available() > 0) {
    command = Serial.readStringUntil('\n');
    command.trim();  // Remove extra whitespace or newline characters
  }
  
  // Update motor outputs based on the received command word
  if (command == "STOP") {
    motor1.run(RELEASE);
    motor2.run(RELEASE);
    motor3.run(RELEASE);
    motor4.run(RELEASE);
  } 
  else if (command == "LEFT") {
    // Left turn: left motors backward, right motors forward
    motor1.run(BACKWARD); // Front-left
    motor2.run(BACKWARD); // Back-left
    motor3.run(FORWARD);  // Front-right
    motor4.run(FORWARD);  // Back-right
  } 
  else if (command == "RIGHT") {
    // Right turn: left motors forward, right motors backward
    motor1.run(FORWARD);  // Front-left
    motor2.run(FORWARD);  // Back-left
    motor3.run(BACKWARD); // Front-right
    motor4.run(BACKWARD); // Back-right
  } 
  else if (command == "FRONT") {
    // Move forward: all motors forward
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
  } 
  else {
    // If command unrecognized, stop for safety.
    motor1.run(RELEASE);
    motor2.run(RELEASE);
    motor3.run(RELEASE);
    motor4.run(RELEASE);
  }
  
  delay(100);  // Short delay for stability
}

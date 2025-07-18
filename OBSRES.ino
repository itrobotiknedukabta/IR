#include <NewPing.h>
#include <Servo.h>

// Pin definitions
#define TRIG_PIN 12
#define ECHO_PIN 13
#define SERVO_PIN 3
#define IN1 9
#define IN2 8
#define IN3 2
#define IN4 7
#define ENA 10
#define ENB 5

// Constants
#define MAX_DISTANCE 200 // Max distance to ping (cm)
#define SAFE_DISTANCE 20 // Minimum safe distance from obstacles (cm)
#define TURN_SPEED 255
#define FORWARD_SPEED 255

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
Servo servo;

void setup() {
  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  // Initialize servo
  servo.attach(SERVO_PIN);
  servo.write(90); // Center position
  
  Serial.begin(9600);
  delay(1000); // Startup delay
}

void loop() {
  int distance = getDistance();
  
  if (distance < SAFE_DISTANCE) {
    stopMotors();
    int bestDirection = findBestPath();
    navigate(bestDirection);
  } else {
    moveForward();
  }
  
  delay(50); // Small delay for stability
}

// Motor control functions
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, FORWARD_SPEED);
  analogWrite(ENB, FORWARD_SPEED);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// Distance measurement with NewPing
int getDistance() {
  unsigned int distance = sonar.ping_cm();
  if (distance == 0) distance = MAX_DISTANCE; // If no echo, assume max distance
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println("cm");
  return distance;
}

// Path finding
int findBestPath() {
  int distances[3];
  
  // Check left (0°)
  servo.write(0);
  delay(400);
  distances[0] = getDistance();
  
  // Check center (90°)
  servo.write(90);
  delay(400);
  distances[1] = getDistance();
  
  // Check right (180°)
  servo.write(180);
  delay(400);
  distances[2] = getDistance();
  
  // Return to center
  servo.write(90);
  
  // Find best direction
  if (distances[0] > distances[1] && distances[0] > distances[2]) return 0; // Left
  if (distances[2] > distances[1] && distances[2] > distances[0]) return 2; // Right
  return 1; // Center (backup case)
}

void navigate(int direction) {
  if (direction == 0) { // Left
    turnLeft();
    delay(250);
  } else if (direction == 2) { // Right
    turnRight();
    delay(250);
  } else { // Center (obstacle ahead, back up)
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, FORWARD_SPEED);
    analogWrite(ENB, FORWARD_SPEED);
    delay(250);
    turnRight(); // Default turn after backup
    delay(250);
  }
  stopMotors();
}
#include "BluetoothSerial.h"
#include <Wire.h>
#include <MPU6050_light.h>

BluetoothSerial SerialBT;
MPU6050 mpu(Wire);

// Motor pins
const int IN1 = 16;
const int IN2 = 17;
const int IN3 = 19;
const int IN4 = 13;

// Ultrasonic pins
const int trigFront = 32, echoFront = 33;
const int trigLeft = 26, echoLeft = 27;
const int trigRight = 14, echoRight = 12;

// IR sensor pins
const int irFront = 4;
const int irRear = 5;

// Tilt threshold
float tiltThreshold = 50.0;

unsigned long lastCmdTime = 0;
const unsigned long timeout = 5000;

enum MovementState { STOPPED, MOVING_FORWARD, MOVING_BACKWARD, TURNING_LEFT, TURNING_RIGHT };
MovementState currentState = STOPPED;

float currentPitch = 0.0;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Car");
  Serial.println("Bluetooth Car ready. Pair with 'ESP32_Car'");

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(trigFront, OUTPUT); pinMode(echoFront, INPUT);
  pinMode(trigLeft, OUTPUT);  pinMode(echoLeft, INPUT);
  pinMode(trigRight, OUTPUT); pinMode(echoRight, INPUT);

  pinMode(irFront, INPUT);
  pinMode(irRear, INPUT);

  Wire.begin(21, 22);

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {
    Serial.println("MPU6050 not responding. Check wiring and address.");
    delay(1000);
  }

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets();
  Serial.println("MPU6050 offset calculation done!\n");

  stopCar();
}

void loop() {
  mpu.update();
  currentPitch = mpu.getAngleY();

  if (abs(currentPitch) > tiltThreshold) {
    Serial.print("Tilt detected ("); Serial.print(currentPitch); Serial.println("°)! Forcing forward.");
    moveForward();
    delay(500);
    return;
  }

  if (SerialBT.available()) {
    char cmd = SerialBT.read();
    handleCommand(cmd);
    lastCmdTime = millis();
  }

  if (millis() - lastCmdTime > timeout) {
    stopCar();
  }

  switch (currentState) {
    case MOVING_FORWARD:
      if (isFrontBlocked()) {
        Serial.println("Obstacle detected while moving forward!");
        stopCar();
      }
      break;
    case MOVING_BACKWARD:
      if (isRearBlocked()) {
        Serial.println("Obstacle detected while moving backward!");
        stopCar();
      }
      break;
    case TURNING_LEFT:
      if (isLeftBlocked()) {
        Serial.println("Obstacle detected while turning left!");
        stopCar();
      }
      break;
    case TURNING_RIGHT:
      if (isRightBlocked()) {
        Serial.println("Obstacle detected while turning right!");
        stopCar();
      }
      break;
    default:
      break;
  }

  delay(100);
}

// ---- Movement functions with state tracking ----
void moveForward() {
  if (isFrontBlocked()) {
    stopCar();
    Serial.println("Obstacle ahead! Forward blocked.");
    return;
  }
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  currentState = MOVING_FORWARD;
  Serial.println("Moving forward");
}

void moveBackward() {
  if (isRearBlocked()) {
    stopCar();
    Serial.println("Obstacle behind! Reverse blocked.");
    return;
  }
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  currentState = MOVING_BACKWARD;
  Serial.println("Moving backward");
}

void turnLeft() {
  if (isLeftBlocked()) {
    stopCar();
    Serial.println("Obstacle on left! Turn blocked.");
    return;
  }
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  currentState = TURNING_LEFT;
  Serial.println("Turning left");
}

void turnRight() {
  if (isRightBlocked()) {
    stopCar();
    Serial.println("Obstacle on right! Turn blocked.");
    return;
  }
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  currentState = TURNING_RIGHT;
  Serial.println("Turning right");
}

void stopCar() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  currentState = STOPPED;
  Serial.println("Motors stopped");
}

void handleCommand(char cmd) {
  switch (cmd) {
    case 'F': case 'f': moveForward(); break;
    case 'B': case 'b': moveBackward(); break;
    case 'L': case 'l': turnLeft(); break;
    case 'R': case 'r': turnRight(); break;
    case 'S': case 's': stopCar(); break;
    default: Serial.println("Unknown command"); break;
  }
}

// ---- Sensor utilities ----
float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  float distance = duration * 0.034 / 2;
  if (duration == 0 || distance <= 0.5) return -1.0;
  return distance;
}

bool isFrontBlocked() {
  float d = readDistance(trigFront, echoFront);
  bool irBlocked = false;
  if (currentPitch >= 30.0 && currentPitch <= 50.0) {
    irBlocked = digitalRead(irFront) == LOW;
  }
  return (d > 0 && d < 15) || irBlocked;
}

bool isRearBlocked() {
  if (currentPitch >= 30.0 && currentPitch <= 50.0) {
    return digitalRead(irRear) == LOW;
  }
  return false;
}

bool isLeftBlocked() {
  float d = readDistance(trigLeft, echoLeft);
  return (d > 0 && d < 15);
}

bool isRightBlocked() {
  float d = readDistance(trigRight, echoRight);
  return (d > 0 && d < 15);
}

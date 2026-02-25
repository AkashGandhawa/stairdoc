#include <ESP32Servo.h>     
#include <Wire.h>
#include <MPU6050_light.h>
#include <BluetoothSerial.h>

MPU6050 mpu(Wire);
BluetoothSerial SerialBT; 

// Motor control pins (BTS7960)
#define LPWM1 16
#define RPWM1 17
#define LPWM2 19
#define RPWM2 13
#define EN1_L 23
#define EN1_R 5
#define EN2_L 2
#define EN2_R 0

// Ultrasonic sensor pins
#define trigFront 327
#define echoFront 33
#define trigRear 15
#define echoRear 4
#define trigLeft 26
#define echoLeft 27
#define trigRight 14
#define echoRight 12

// Bump sensor pins
#define bumpLeft 34
#define bumpRight 35

// Servo pin
#define SERVO_PIN 18
Servo servo1;
Servo servo2;

char command = '\0';  // Stores the latest Bluetooth command

float pitch = 0.0;  // Y-angle from MPU6050
bool pitchOverride = false;
unsigned long lastCmdTime = 0;
const unsigned long timeout = 10000;  // 10-second command timeout

// Movement state tracking
enum MovementState { STOPPED, FORWARD, BACKWARD, LEFT, RIGHT };
MovementState currentState = STOPPED;

int armCount = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing robot...");

  // Initialize MPU6050
  Wire.begin(21, 22);
  byte status = mpu.begin();
  Serial.print("MPU6050 status: "); Serial.println(status);
  while (status != 0) {
    Serial.println("MPU6050 not responding. Check wiring.");
    delay(1000);
  }

  Serial.println("Calculating MPU6050 offsets...");
  delay(1000);
  mpu.calcOffsets();
  Serial.println("Offsets done.");

  // Set motor pins as outputs
  pinMode(RPWM1, OUTPUT); pinMode(LPWM1, OUTPUT);
  pinMode(RPWM2, OUTPUT); pinMode(LPWM2, OUTPUT);
  pinMode(EN1_R, OUTPUT); pinMode(EN1_L, OUTPUT);
  pinMode(EN2_R, OUTPUT); pinMode(EN2_L, OUTPUT);

  // Set ultrasonic sensor pins
  pinMode(trigFront, OUTPUT); pinMode(echoFront, INPUT);
  pinMode(trigRear, OUTPUT);  pinMode(echoRear, INPUT);
  pinMode(trigLeft, OUTPUT);  pinMode(echoLeft, INPUT);
  pinMode(trigRight, OUTPUT); pinMode(echoRight, INPUT);

  // Set bump sensor pins
  pinMode(bumpLeft, INPUT);
  pinMode(bumpRight, INPUT);

  // Initialize  Bluetooth
  SerialBT.begin("InterimRobot");  // Device name
  Serial.println("Bluetooth initialized. Robot ready.");

  // Initialize servos
  servo1.attach(SERVO_PIN);
  servo2.attach(SERVO_PIN);
  servo1.write(0);
  servo2.write(0);

  stopMotors();  // Ensure motors are off at startup
  delay(500);

  // Enable motor drivers
  digitalWrite(EN1_R, HIGH); digitalWrite(EN1_L, HIGH);
  digitalWrite(EN2_R, HIGH); digitalWrite(EN2_L, HIGH);
}

void loop() {
  mpu.update();
  pitch = mpu.getAngleY();  // Read pitch from Y-axis

  // Read distances from all ultrasonic sensors
  long frontDist = readUltrasonic(trigFront, echoFront);
  long rearDist  = readUltrasonic(trigRear, echoRear);
  long leftDist  = readUltrasonic(trigLeft, echoLeft);
  long rightDist = readUltrasonic(trigRight, echoRight);

  // Print sensor readings
  Serial.print("Pitch (Y-angle): "); Serial.println(pitch);
  Serial.print("Front: "); Serial.print(frontDist);
  Serial.print(" cm | Rear: "); Serial.print(rearDist);
  Serial.print(" cm | Left: "); Serial.print(leftDist);
  Serial.print(" cm | Right: "); Serial.print(rightDist);
  Serial.println(" cm");

  // Handle incoming Bluetooth commands
  if (SerialBT.available()) {
    command = SerialBT.read(); // read until newline
    lastCmdTime = millis();
    Serial.print("BT command received: ");
    Serial.println(command);
  }

  // Bump sensor override
  if (digitalRead(bumpLeft) == LOW || digitalRead(bumpRight) == LOW) {
    Serial.println("Bump detected! Reversing...");
    stopMotors();
    delay(200);
    moveBackward();
    delay(2000);
    stopMotors();
    return;
  }

  // Pitch override
  if (abs(pitch) > 50 && frontDist > 20) {
    Serial.println("Pitch override: moving forward");
    moveForward();
    pitchOverride = true;
    return;
  }

  if (pitchOverride && abs(pitch) <= 50) {
    Serial.println("Pitch normalized: stopping");
    stopMotors();
    pitchOverride = false;
  }

  // Obstacle override
  if ((command == 'f' && frontDist <= 20) ||
      (command == 'b' && rearDist <= 20) ||
      (command == 'l' && leftDist <= 20) ||
      (command == 'r' && rightDist <= 20)) {
    Serial.println("Obstacle detected. Stopping.");
    stopMotors();
    return;
  }

  // Timeout
  if (millis() - lastCmdTime > timeout) {
    Serial.println("Command timeout. Stopping.");
    stopMotors();
    return;
  }

  // Execute Bluetooth command
  if (command == 'f') moveForward();
  else if (command == 'b') moveBackward();
  else if (command == 'l') turnLeft();
  else if (command == 'r') turnRight();
  else if (command == 's') stopMotors();
  else if (command == 'u') {
    if(armCount == 0){
      Serial.println("Servo sweep up (0→180)");
      armCount++;
      for (int pos = 0; pos <= 180; pos += 5) {
        servo1.write(pos);
        servo2.write(pos);
        delay(50);
      }
    }else if(armCount == 75){
      armCount = 0;
    }else{
      armCount++;
    }
  }
  else if (command == 'd') {
    if(armCount == 0){
      Serial.println("Servo sweep down (180→0)");
      armCount++;
      for (int pos = 180; pos >= 0; pos -= 5) {
        servo1.write(pos);
        servo2.write(pos);
        delay(50);
      }
    }else if(armCount == 75){
      armCount = 0;
    }else{
      armCount++;
    }
  }
}

// Motor control functions
void moveForward() {
  Serial.println("Moving forward");
  analogWrite(RPWM1, 0); analogWrite(LPWM1, 255);
  analogWrite(RPWM2, 0); analogWrite(LPWM2, 255);
  currentState = FORWARD;
}

void moveBackward() {
  Serial.println("Moving backward");
  analogWrite(RPWM1, 255); analogWrite(LPWM1, 0);
  analogWrite(RPWM2, 255); analogWrite(LPWM2, 0);
  currentState = BACKWARD;
}

void turnLeft() {
  Serial.println("Turning left");
  analogWrite(RPWM1, 0); analogWrite(LPWM1, 255);
  analogWrite(RPWM2, 255); analogWrite(LPWM2, 0);
  currentState = LEFT;
}

void turnRight() {
  Serial.println("Turning right");
  analogWrite(RPWM1, 255); analogWrite(LPWM1, 0);
  analogWrite(RPWM2, 0); analogWrite(LPWM2, 255);
  currentState = RIGHT;
}

void stopMotors() {
  Serial.println("Stopping motors");
  analogWrite(RPWM1, 0); analogWrite(LPWM1, 0);
  analogWrite(RPWM2, 0); analogWrite(LPWM2, 0);
  currentState = STOPPED;
}

// Ultrasonic sensor read
long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  return duration * 0.034 / 2;
}

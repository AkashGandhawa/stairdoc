/*
===========================================================
FULL ROBOT CODE (Safe Initialization + Staircase Flag, Analog Speed Control)

Included:
 - Servo control (DS3218 + TD8120) with sweeps
 - Motor driver pins, initialization, and movement functions
 - Bluetooth commands for servos + motors
 - Ultrasonic obstacle detection (front/rear)
 - Bump sensor logic (disabled near staircase)
 - Pitch override using MPU6050
 - Command timeout safety
 - Staircase flag alters thresholds and bump behavior
 - Global speed percentage (default 60%)
 - Servo angle persistence in ESP32 flash (restored only if tilted >10°)
 - HX711 load cell initialization
===========================================================
*/

#define DEBUG
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BluetoothSerial.h>
#include <MPU6050_light.h>
#include <Preferences.h>
#include "HX711.h"

// Shared objects
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
BluetoothSerial SerialBT;
MPU6050 mpu(Wire);
Preferences prefs;

// --- Motor control pins (BTS7960) ---
#define LPWM1 16
#define RPWM1 17
#define LPWM2 19
#define RPWM2 13
#define EN1_L 23
#define EN1_R 5
#define EN2_L 2
#define EN2_R 0

// --- Servo channels ---
#define FRONT_LEFT_CH 0
#define FRONT_RIGHT_CH 1
#define REAR_LEFT_CH 2
#define REAR_RIGHT_CH 3

// --- Servo limits ---
#define DS3218_USMIN 500
#define DS3218_USMAX 2500
#define TD8120_MIN 150
#define TD8120_MAX 600

// --- Ultrasonic sensor pins ---
#define trigFront 32
#define echoFront 33
#define trigRear 15
#define echoRear 4

// --- Bump sensor pins ---
#define bumpLeft 34
#define bumpRight 35

// --- HX711 pins ---
#define LOADCELL_DOUT_PIN 36
#define LOADCELL_SCK_PIN 25
HX711 hx711;

// Globals
char command = '\0';
bool robotMoving = false;
enum MovementState { STOPPED, FORWARD, BACKWARD, LEFT, RIGHT };
MovementState currentState = STOPPED;

float pitch = 0.0;
bool pitchOverride = false;
bool mpuActive = false;

long frontDist = 0, rearDist = 0;
unsigned long lastUltrasonicCheck = 0;
unsigned long lastCmdTime = 0;
const unsigned long timeout = 10000;

unsigned long bumpTime = 0;
bool bumpRecovering = false;

// Staircase flag
bool nearStaircase = true;
int obstacleThreshold = 20;

// Speed control
int speedPercent = 60;  // default speed percentage

// Servo persistence
unsigned long lastServoChangeTime = 0;

// Sweep state
bool sweepingFront = false;
int frontSweepPos = 90;
int frontSweepEnd = 90;
unsigned long lastFrontUpdate = 0;

bool sweepingRear = false;
int rearSweepPos = 90;
int rearSweepEnd = 90;
unsigned long lastRearUpdate = 0;

const unsigned long sweepInterval = 30;

// ---------------- Servo helpers ----------------
void setDS3218Angle(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);
  uint32_t pulseWidthMicros = map(angle, 0, 180, DS3218_USMIN, DS3218_USMAX);
  pwm.writeMicroseconds(channel, pulseWidthMicros);
}

void setTD8120Angle(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, TD8120_MIN, TD8120_MAX);
  pwm.setPWM(channel, 0, pulse);
}

void updateSweeps() {
  unsigned long now = millis();
  if (sweepingFront && (now - lastFrontUpdate >= sweepInterval)) {
    setDS3218Angle(FRONT_LEFT_CH, frontSweepPos);
    setDS3218Angle(FRONT_RIGHT_CH, 180 - frontSweepPos);
    if (frontSweepPos < frontSweepEnd) frontSweepPos++;
    else if (frontSweepPos > frontSweepEnd) frontSweepPos--;
    else sweepingFront = false;
    lastFrontUpdate = now;
  }
  if (sweepingRear && (now - lastRearUpdate >= sweepInterval)) {
    setDS3218Angle(REAR_LEFT_CH, rearSweepPos);
    setTD8120Angle(REAR_RIGHT_CH, 180 - rearSweepPos);
    if (rearSweepPos < rearSweepEnd) rearSweepPos++;
    else if (rearSweepPos > rearSweepEnd) rearSweepPos--;
    else sweepingRear = false;
    lastRearUpdate = now;
  }
}

// ---------------- Motor control (PWM analog speed) ----------------
void enableMotors() {
  digitalWrite(EN1_L, HIGH);
  digitalWrite(EN1_R, HIGH);
  digitalWrite(EN2_L, HIGH);
  digitalWrite(EN2_R, HIGH);
}

void stopMotors() {
  ledcWrite(RPWM1, 0);
  ledcWrite(LPWM1, 0);
  ledcWrite(RPWM2, 0);
  ledcWrite(LPWM2, 0);
  digitalWrite(EN1_L, LOW);
  digitalWrite(EN1_R, LOW);
  digitalWrite(EN2_L, LOW);
  digitalWrite(EN2_R, LOW);
  currentState = STOPPED;
  robotMoving = false;
}

void moveForward() {
  enableMotors();
  int duty = map(speedPercent, 0, 100, 0, 255);
  ledcWrite(RPWM1, 0);       // RPWM1 off
  ledcWrite(LPWM1, duty);    // LPWM1 forward
  ledcWrite(RPWM2, duty);    // RPWM2 forward
  ledcWrite(LPWM2, 0);       // LPWM2 off
  currentState = FORWARD;
  robotMoving = true;
}

void moveBackward() {
  enableMotors();
  int duty = map(speedPercent, 0, 100, 0, 255);
  ledcWrite(RPWM1, duty);    // RPWM1 backward
  ledcWrite(LPWM1, 0);
  ledcWrite(RPWM2, 0);
  ledcWrite(LPWM2, duty);    // LPWM2 backward
  currentState = BACKWARD;
  robotMoving = true;
}

void turn(int direction) {
  enableMotors();
  int duty = map(speedPercent, 0, 100, 0, 255);
  if (direction == LEFT) {
    ledcWrite(RPWM1, duty);  // Right motor backward
    ledcWrite(LPWM1, 0);
    ledcWrite(RPWM2, duty);  // Left motor forward
    ledcWrite(LPWM2, 0);
    currentState = LEFT;
  } else {
    ledcWrite(RPWM1, 0);
    ledcWrite(LPWM1, duty);  // Right motor forward
    ledcWrite(RPWM2, 0);
    ledcWrite(LPWM2, duty);  // Left motor backward
    currentState = RIGHT;
  }
  robotMoving = true;
}

// ---------------- Ultrasonic helper ----------------
long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  return duration * 0.034 / 2;
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  Serial.println("Robot setup starting...");

  // --- Servo driver ---
  pwm.begin();
  pwm.setPWMFreq(50);  // 50 Hz for servos

  // --- Motor driver enable pins ---
  pinMode(EN1_L, OUTPUT);
  pinMode(EN1_R, OUTPUT);
  pinMode(EN2_L, OUTPUT);
  pinMode(EN2_R, OUTPUT);

  // --- Setup PWM channels for motor pins (analog speed control) ---
  ledcAttach(RPWM1, 1000, 8);
  ledcAttach(LPWM1, 1000, 8);
  ledcAttach(RPWM2, 1000, 8);
  ledcAttach(LPWM2, 1000, 8);

  // --- Ultrasonic pins ---
  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);
  pinMode(trigRear, OUTPUT);
  pinMode(echoRear, INPUT);

  // --- Bump sensors ---
  pinMode(bumpLeft, INPUT_PULLUP);
  pinMode(bumpRight, INPUT_PULLUP);

  // --- Bluetooth ---
  SerialBT.begin("ESP32Robot");
  Serial.println("Bluetooth ready.");

  // --- Preferences (servo persistence) ---
  prefs.begin("servoStore", false);

  // --- MPU6050 ---
  Wire.begin();
  byte status = mpu.begin();
  if (status == 0) {
    mpuActive = true;
    mpu.calcOffsets();
    Serial.println("MPU6050 initialized");
  } else {
    Serial.println("MPU6050 not found");
  }

  // --- Staircase flag ---
  if (nearStaircase) {
    obstacleThreshold = 3;
    Serial.println("Near staircase: threshold set to 3 cm");
  }

  // --- Restore servo angles only if flat ---
  if (mpuActive) {
    mpu.update();
    pitch = mpu.getAngleY();
    if (abs(pitch) < 10) {
      frontSweepPos = prefs.getInt("frontPos", 90);
      rearSweepPos = prefs.getInt("rearPos", 90);
      setDS3218Angle(FRONT_LEFT_CH, frontSweepPos);
      setDS3218Angle(FRONT_RIGHT_CH, 180 - frontSweepPos);
      setDS3218Angle(REAR_LEFT_CH, rearSweepPos);
      setTD8120Angle(REAR_RIGHT_CH, 180 - rearSweepPos);
      Serial.println("Servo angles restored from flash");
    } else {
      Serial.println("Robot not flat, skipping servo restore");
    }
  } else {
    frontSweepPos = 90;
    rearSweepPos = 90;
    setDS3218Angle(FRONT_LEFT_CH, frontSweepPos);
    setDS3218Angle(FRONT_RIGHT_CH, 180 - frontSweepPos);
    setDS3218Angle(REAR_LEFT_CH, rearSweepPos);
    setTD8120Angle(REAR_RIGHT_CH, 180 - rearSweepPos);
    Serial.println("Servo angles set to default (90°)");
  }

  // --- HX711 initialization ---
  hx711.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  hx711.set_scale();
  hx711.tare();
  Serial.println("HX711 initialized and tared");

  stopMotors();  // ensure motors are off at startup
  Serial.println("Setup complete");
}

// ---------------------- Loop ----------------------
void loop() {
  unsigned long now = millis();

  // --- Update MPU6050 ---
  if (mpuActive) {
    mpu.update();
    pitch = mpu.getAngleY();
#ifdef DEBUG
    Serial.printf("Pitch: %.2f\n", pitch);
#endif
  }

  // --- Ultrasonic obstacle check ---
  if (robotMoving) {
    if (now - lastUltrasonicCheck >= 200) {  // faster when moving
      frontDist = readUltrasonic(trigFront, echoFront);
      rearDist = readUltrasonic(trigRear, echoRear);
#ifdef DEBUG
      Serial.printf("Front: %ld cm | Rear: %ld cm\n", frontDist, rearDist);
#endif
      lastUltrasonicCheck = now;
    }
  } else {
    if (now - lastUltrasonicCheck >= 1000) {  // slower when idle
      frontDist = readUltrasonic(trigFront, echoFront);
      rearDist = readUltrasonic(trigRear, echoRear);
      lastUltrasonicCheck = now;
    }
  }

  // --- Bluetooth commands ---
  if (SerialBT.available()) {
    command = SerialBT.read();
    lastCmdTime = now;
#ifdef DEBUG
    Serial.printf("BT command: %c\n", command);
#endif

    if (command == 'u') {
      sweepingFront = true;
      frontSweepEnd = frontSweepPos + 15;
    } else if (command == 'd') {
      sweepingFront = true;
      frontSweepEnd = frontSweepPos - 15;
    } else if (command == 'v') {
      sweepingRear = true;
      rearSweepEnd = rearSweepPos + 15;
    } else if (command == 'e') {
      sweepingRear = true;
      rearSweepEnd = rearSweepPos - 15;
    } else if (command == 'f') {
      moveForward();
    } else if (command == 'b') {
      moveBackward();
    } else if (command == 'l') {
      turn(LEFT);
    } else if (command == 'r') {
      turn(RIGHT);
    } else if (command == 's') {
      stopMotors();
    }
  }

  // --- Safety: obstacle detection ---
  if (robotMoving) {
    if (frontDist > 0 && frontDist < obstacleThreshold && currentState == FORWARD) {
      stopMotors();
#ifdef DEBUG
      Serial.println("Obstacle in front! Stopping.");
#endif
    }
    if (rearDist > 0 && rearDist < obstacleThreshold && currentState == BACKWARD) {
      stopMotors();
#ifdef DEBUG
      Serial.println("Obstacle in rear! Stopping.");
#endif
    }
  }

  // --- Safety: bump sensors (disabled near staircase) ---
  if (!nearStaircase) {
    if (digitalRead(bumpLeft) == LOW || digitalRead(bumpRight) == LOW) {
      if (!bumpRecovering) {
        stopMotors();
        moveBackward();
        bumpTime = now;
        bumpRecovering = true;
      }
    }
    if (bumpRecovering && now - bumpTime >= 1000) {
      stopMotors();
      bumpRecovering = false;
    }
  }

  // --- Safety: pitch override ---
  static unsigned long pitchStart = 0;
  if (mpuActive) {
    if (abs(pitch) > 50 && !pitchOverride) {
      moveForward();
      pitchOverride = true;
      pitchStart = now;
    }
    if (pitchOverride && now - pitchStart >= 1000) {
      stopMotors();
      pitchOverride = false;
    }
  }

  // --- Safety: command timeout ---
  if (now - lastCmdTime > timeout) {
    stopMotors();
#ifdef DEBUG
    Serial.println("Command timeout triggered");
#endif
  }

  // --- Sweep updates ---
  updateSweeps();
}

/*
===========================================================
FULL ROBOT CODE (Safe Initialization + Staircase Flag)

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

Commented out (not needed yet):
 - Stair detection ultrasonic sensors (left/right)
 - HX711 load cell code
===========================================================
*/

#define DEBUG  // comment this out to disable debug prints
//TODO: comment out the DEBUG definition before testing with battery

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
#define FRONT_LEFT_CH 0   // DS3218
#define FRONT_RIGHT_CH 1  // DS3218
#define REAR_LEFT_CH 2    // DS3218
#define REAR_RIGHT_CH 3   // TD8120

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

// --- Globals ---
char command = '\0';
bool robotMoving = false;
enum MovementState { STOPPED,
                     FORWARD,
                     BACKWARD,
                     LEFT,
                     RIGHT };
MovementState currentState = STOPPED;

// HX711 pins (your actual wiring)
#define LOADCELL_DOUT_PIN 36  // DT pin
#define LOADCELL_SCK_PIN 25   // SCK pin

// HX711 object (same name as old code)
HX711 hx711;

// Calibration values
long tareOffset = 0;
float calibrationFactor = 1.0;  // adjust after calibration

float pitch = 0.0;
bool pitchOverride = false;
bool mpuActive = false;

// Timing and distance variables
long frontDist = 0, rearDist = 0;
unsigned long lastMotionUltrasonicRead = 0;
unsigned long lastCmdTime = 0;
unsigned long lastUltrasonicCheck = 0; 
unsigned long lastServoSave = 0;
const unsigned long timeout = 10000;  // 10s safety timeout

unsigned long bumpTime = 0;
bool bumpRecovering = false;

// --- Staircase flag ---
bool nearStaircase = true;   // For testing, set true in setup
int obstacleThreshold = 20;  // Will be set to 3 if near staircase

// --- Speed control ---
int speedPercent = 60;  // default speed percentage

// --- Servo persistence ---
unsigned long servoSaveTimeout = 10000;  // 10s timeout
unsigned long lastServoChangeTime = 0;

// --- Sweep state ---
bool sweepingFront = false;
int frontSweepPos = 90;  // current angle
int frontSweepEnd = 90;  // target angle
unsigned long lastFrontUpdate = 0;

bool sweepingRear = false;
int rearSweepPos = 90;  // current angle
int rearSweepEnd = 90;  // target angle
unsigned long lastRearUpdate = 0;

// --- Sweep timing ---
const unsigned long sweepInterval = 30;  // ms between updates

// ---------------------- Servo helpers ----------------------

void setDS3218Angle(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);
  uint32_t pulseWidthMicros = map(angle, 0, 180, DS3218_USMIN, DS3218_USMAX);
  pwm.writeMicroseconds(channel, pulseWidthMicros);

#ifdef DEBUG
  Serial.printf("DS3218 channel %d -> %d° (pulse %lu)\n", channel, angle, pulseWidthMicros);
#endif
}

// TD8120 servo (rear)
void setTD8120Angle(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, TD8120_MIN, TD8120_MAX);
  pwm.setPWM(channel, 0, pulse);

#ifdef DEBUG
  Serial.printf("TD8120 channel %d -> %d° (pulse %d)\n", channel, angle, pulse);
#endif
}

// --- Step rules ---
// Front servos: range 0–90
// Rear servos: range 90–0
// Steps: 15° in outer ranges, 5° in 30–60 range

// NEW VERSION (simplified with constrain)
int getTargetStep(int current, bool front, bool increase) {
  if (front) {
    if (current < 30) return constrain(current + (increase ? 15 : -15), 0, 30);
    else if (current < 60) return constrain(current + (increase ? 5 : -5), 30, 60);
    else if (current < 90) return constrain(current + (increase ? 15 : -15), 60, 90);
    else return current;
  } else {  // rear: range 90–0
    if (current > 60) return constrain(current + (increase ? -15 : 15), 60, 90);
    else if (current > 30) return constrain(current + (increase ? -5 : 5), 30, 60);
    else if (current > 0) return constrain(current + (increase ? -15 : 15), 0, 30);
    else return current;
  }
}

/* 
// ORIGINAL VERSION (kept for reference)
int getTargetStep(int current, bool front, bool increase) {
  if (front) {
    if (current < 30) return increase ? min(current + 15, 30) : max(current - 15, 0);
    else if (current >= 30 && current < 60) return increase ? min(current + 5, 60) : max(current - 5, 30);
    else if (current >= 60 && current < 90) return increase ? min(current + 15, 90) : max(current - 15, 60);
    else return current;
  } else {  // rear: range 90–0
    if (current > 60) return increase ? max(current - 15, 60) : min(current + 15, 90);
    else if (current <= 60 && current > 30) return increase ? max(current - 5, 30) : min(current + 5, 60);
    else if (current <= 30 && current > 0) return increase ? max(current - 15, 0) : min(current + 15, 30);
    else return current;
  }
}
*/

// --- Sweep update ---
// Moves 1° per iteration toward target
// NEW VERSION (using constrain, moves 1° per iteration)
void updateSweeps() {
  unsigned long now = millis();

  if (sweepingFront && (now - lastFrontUpdate >= sweepInterval)) {
    setDS3218Angle(FRONT_LEFT_CH, frontSweepPos);
    setDS3218Angle(FRONT_RIGHT_CH, 180 - frontSweepPos);

    // Move one degree toward target
    frontSweepPos = constrain(frontSweepPos + (frontSweepPos < frontSweepEnd ? 1 : (frontSweepPos > frontSweepEnd ? -1 : 0)), 0, 90);

    if (frontSweepPos == frontSweepEnd) sweepingFront = false;

    lastFrontUpdate = now;
    lastServoChangeTime = now;  // mark change for persistence
  }

  if (sweepingRear && (now - lastRearUpdate >= sweepInterval)) {
    setDS3218Angle(REAR_LEFT_CH, rearSweepPos);
    setTD8120Angle(REAR_RIGHT_CH, 180 - rearSweepPos);

    // Move one degree toward target
    rearSweepPos = constrain(rearSweepPos + (rearSweepPos < rearSweepEnd ? 1 : (rearSweepPos > rearSweepEnd ? -1 : 0)), 0, 90);

    if (rearSweepPos == rearSweepEnd) sweepingRear = false;

    lastRearUpdate = now;
    lastServoChangeTime = now;  // mark change for persistence
  }
}

/* 
// --- ORIGINAL VERSION (kept for reference) ---
void updateSweeps() {
  unsigned long now = millis();

  if (sweepingFront && (now - lastFrontUpdate >= sweepInterval)) {
    setDS3218Angle(FRONT_LEFT_CH, frontSweepPos);
    setDS3218Angle(FRONT_RIGHT_CH, 180 - frontSweepPos);

    if (frontSweepPos < frontSweepEnd) frontSweepPos++;
    else if (frontSweepPos > frontSweepEnd) frontSweepPos--;
    else sweepingFront = false;

    lastFrontUpdate = now;
    lastServoChangeTime = now;
  }

  if (sweepingRear && (now - lastRearUpdate >= sweepInterval)) {
    setDS3218Angle(REAR_LEFT_CH, rearSweepPos);
    setTD8120Angle(REAR_RIGHT_CH, 180 - rearSweepPos);

    if (rearSweepPos < rearSweepEnd) rearSweepPos++;
    else if (rearSweepPos > rearSweepEnd) rearSweepPos--;
    else sweepingRear = false;

    lastRearUpdate = now;
    lastServoChangeTime = now;
  }
}
*/

// ---------------------- Motor control ----------------------
// Enable motor driver pins
void enableMotors() {
  digitalWrite(EN1_L, HIGH);
  digitalWrite(EN1_R, HIGH);
  digitalWrite(EN2_L, HIGH);
  digitalWrite(EN2_R, HIGH);

#ifdef DEBUG
  Serial.println("Motor drivers enabled");
#endif
}

// Stop all motors and disable drivers
void stopMotors() {
  // Stop PWM outputs
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);

  // Disable motor driver pins
  digitalWrite(EN1_L, LOW);
  digitalWrite(EN1_R, LOW);
  digitalWrite(EN2_L, LOW);
  digitalWrite(EN2_R, LOW);

  currentState = STOPPED;
  robotMoving = false;

#ifdef DEBUG
  Serial.println("Motors stopped and drivers disabled");
#endif
}

// Move robot forward
void moveForward() {
  enableMotors();  // ensure drivers are active

  int duty = (speedPercent * 255) / 100;  // scale duty cycle

  // Right motor forward
  ledcWrite(0, 0);
  ledcWrite(1, duty);

  // Left motor forward
  ledcWrite(2, duty);
  ledcWrite(3, 0);

  currentState = FORWARD;
  robotMoving = true;

#ifdef DEBUG
  Serial.printf("Moving forward at %d%% speed (duty %d)\n", speedPercent, duty);
#endif
}

// Move robot backward
void moveBackward() {
  enableMotors();  // ensure drivers are active

  int duty = (speedPercent * 255) / 100;  // scale duty cycle

  // Right motor backward
  ledcWrite(0, duty);
  ledcWrite(1, 0);

  // Left motor backward
  ledcWrite(2, 0);
  ledcWrite(3, duty);

  currentState = BACKWARD;
  robotMoving = true;

#ifdef DEBUG
  Serial.printf("Moving backward at %d%% speed (duty %d)\n", speedPercent, duty);
#endif
}

// Turn robot left or right
// direction = LEFT or RIGHT (enum values)
void turn(int direction) {
  enableMotors();  // ensure drivers are active

  int duty = (speedPercent * 255) / 100;  // scale duty cycle

  if (direction == LEFT) {
    // Both motors backward on one side, forward on the other
    ledcWrite(0, duty);  // Right motor backward
    ledcWrite(1, 0);
    ledcWrite(2, duty);  // Left motor forward
    ledcWrite(3, 0);
    currentState = LEFT;
  } else {
    // Opposite configuration for right turn
    ledcWrite(0, 0);
    ledcWrite(1, duty);  // Right motor forward
    ledcWrite(2, 0);
    ledcWrite(3, duty);  // Left motor backward
    currentState = RIGHT;
  }

  robotMoving = true;

#ifdef DEBUG
  Serial.printf("Turning %s at %d%% speed (duty %d)\n",
                direction == LEFT ? "LEFT" : "RIGHT",
                speedPercent, duty);
#endif
}

// ------------------ Servo persistence helper -------------------
// Save current servo positions to flash
void saveServoAngles() {
  prefs.putInt("frontPos", frontSweepPos);
  prefs.putInt("rearPos", rearSweepPos);

#ifdef DEBUG
  Serial.printf("Saved servo angles: front=%d, rear=%d\n", frontSweepPos, rearSweepPos);
#endif
}

// Restore servo positions from flash
void restoreServoAngles() {
  frontSweepPos = prefs.getInt("frontPos", 90);
  rearSweepPos = prefs.getInt("rearPos", 90);

  setDS3218Angle(FRONT_LEFT_CH, frontSweepPos);
  setDS3218Angle(FRONT_RIGHT_CH, 180 - frontSweepPos);
  setDS3218Angle(REAR_LEFT_CH, rearSweepPos);
  setTD8120Angle(REAR_RIGHT_CH, 180 - rearSweepPos);

#ifdef DEBUG
  Serial.printf("Restored servo angles: front=%d, rear=%d\n", frontSweepPos, rearSweepPos);
#endif
}

// ---------------------- Ultrasonic helper ----------------------
// Read distance from an ultrasonic sensor
long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);  // timeout 30ms
  long distance = duration * 0.034 / 2;           // convert to cm

#ifdef DEBUG
  Serial.printf("Ultrasonic trig=%d echo=%d -> %ld cm\n", trigPin, echoPin, distance);
#endif

  return distance;
}

// ---------------------- Setup ----------------------
void setup() {
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("Robot setup starting...");
#endif

  // --- PWM driver ---
  pwm.begin();
  pwm.setPWMFreq(50);  // 50 Hz for servos

  // --- Motor driver pins ---
  pinMode(EN1_L, OUTPUT);
  pinMode(EN1_R, OUTPUT);
  pinMode(EN2_L, OUTPUT);
  pinMode(EN2_R, OUTPUT);

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

  // --- Preferences ---
  prefs.begin("servoStore", false);

  // --- MPU6050 ---
  Wire.begin();
  byte status = mpu.begin();
  if (status == 0) {
    mpuActive = true;
    mpu.calcOffsets();
#ifdef DEBUG
    Serial.println("MPU6050 initialized");
#endif
  } else {
#ifdef DEBUG
    Serial.println("MPU6050 not found");
#endif
  }

  // --- Staircase flag ---
  if (nearStaircase) {
    obstacleThreshold = 3;
#ifdef DEBUG
    Serial.println("Near staircase: threshold set to 3 cm");
#endif
  }

  // --- Restore servo angles only if flat ---
  if (mpuActive) {
    mpu.update();
    pitch = mpu.getAngleY();
    if (abs(pitch) < 10) {
      restoreServoAngles();
    } else {
#ifdef DEBUG
      Serial.println("Robot not flat, skipping servo restore");
#endif
    }
  } else {
    restoreServoAngles();  // fallback if MPU not active
  }

  // --- HX711 initialization ---
  hx711.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  hx711.set_scale();
  hx711.tare();
#ifdef DEBUG
  Serial.println("HX711 initialized and tared");
#endif

  stopMotors();  // ensure motors are off at startup

#ifdef DEBUG
  Serial.println("Setup complete");
#endif

  //   // --- HX711 initialization ---
  // hx711.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  // hx711.set_scale(calibrationFactor);
  // hx711.tare();
  // #ifdef DEBUG
  // Serial.println("HX711 initialized and tared");
  // #endif
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

    // Servo sweep commands set target step
    if (command == 'u') {
      sweepingFront = true;
      frontSweepEnd = getTargetStep(frontSweepPos, true, true);
    } else if (command == 'd') {
      sweepingFront = true;
      frontSweepEnd = getTargetStep(frontSweepPos, true, false);
    } else if (command == 'v') {
      sweepingRear = true;
      rearSweepEnd = getTargetStep(rearSweepPos, false, true);
    } else if (command == 'e') {
      sweepingRear = true;
      rearSweepEnd = getTargetStep(rearSweepPos, false, false);
    } else if (command == 'f') {
      moveForward();
    } else if (command == 'b') {
      moveBackward();
    } else if (command == 'l') {
      turn(LEFT);  // unified turn helper
    } else if (command == 'r') {
      turn(RIGHT);  // unified turn helper
    } else if (command == 's') {
      stopMotors();
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

    // --- Safety: bump sensors ---
    if (!nearStaircase) {
      if (digitalRead(bumpLeft) == LOW || digitalRead(bumpRight) == LOW) {
        if (!bumpRecovering) {
          stopMotors();
          moveBackward();  // recovery motion preserved
          bumpTime = now;
          bumpRecovering = true;
        }
      }
      if (bumpRecovering && now - bumpTime >= 1000) {  // 1s recovery
        stopMotors();
        bumpRecovering = false;
      }
    }

    // --- Safety: pitch override ---
    static unsigned long pitchStart = 0;
    if (mpuActive) {
      if (abs(pitch) > 50 && !pitchOverride) {
        moveForward();  // old behavior preserved
        pitchOverride = true;
        pitchStart = now;
      }
      if (pitchOverride && now - pitchStart >= 1000) {  // 1s forward
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
}

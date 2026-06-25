/*
===========================================================
FULL ROBOT CODE (Safe Initialization)

Included:
 - Servo control (DS3218 + TD8120) with sweeps
 - Motor driver pins, initialization, and movement functions
 - Bluetooth commands for servos + motors
 - Ultrasonic obstacle detection (front/rear)
 - Bump sensor logic
 - Pitch override using MPU6050
 - Command timeout safety

Commented out (not needed yet):
 - Stair detection ultrasonic sensors (left/right)
 - HX711 load cell code
===========================================================
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BluetoothSerial.h>
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

Commented out (not needed yet):
 - Stair detection ultrasonic sensors (left/right)
 - HX711 load cell code
===========================================================
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BluetoothSerial.h>
#include <MPU6050_light.h>

// --- Shared objects ---
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
BluetoothSerial SerialBT;
MPU6050 mpu(Wire);

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

float pitch = 0.0;
bool pitchOverride = false;
bool mpuActive = false;

long frontDist = 0, rearDist = 0;
unsigned long lastMotionUltrasonicRead = 0;
unsigned long lastCmdTime = 0;
const unsigned long timeout = 10000;  // 10s safety timeout

unsigned long bumpTime = 0;
bool bumpRecovering = false;

// --- Staircase flag ---
bool nearStaircase = true;   // For testing, set true in setup
int obstacleThreshold = 20;  // Will be set to 3 if near staircase

// --- Sweep state ---
bool sweepingFront = false;
int frontSweepPos = 90;        // current angle
int frontSweepEnd = 90;        // target angle
unsigned long lastFrontUpdate = 0;

bool sweepingRear = false;
int rearSweepPos = 90;         // current angle
int rearSweepEnd = 90;         // target angle
unsigned long lastRearUpdate = 0;

// --- Sweep timing ---
const unsigned long sweepInterval = 30;  // ms between updates

// ---------------------- Servo helpers ----------------------

// DS3218 servo (front)
void setDS3218Angle(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);
  uint32_t pulseWidthMicros = map(angle, 0, 180, DS3218_USMIN, DS3218_USMAX);
  pwm.writeMicroseconds(channel, pulseWidthMicros);
}

// TD8120 servo (rear)
void setTD8120Angle(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, TD8120_MIN, TD8120_MAX);
  pwm.setPWM(channel, 0, pulse);
}

// --- Step rules ---
// Front servos: range 0–90
// Rear servos: range 90–0
// Steps: 15° in outer ranges, 5° in 30–60 range
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

// --- Sweep update ---
// Moves 1° per iteration toward target
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

// ---------------------- Motor control ----------------------
void enableMotors() {
  digitalWrite(EN1_R, HIGH);
  digitalWrite(EN1_L, HIGH);
  digitalWrite(EN2_R, HIGH);
  digitalWrite(EN2_L, HIGH);
}

void stopMotors() {
  digitalWrite(RPWM1, LOW);
  digitalWrite(LPWM1, LOW);
  digitalWrite(RPWM2, LOW);
  digitalWrite(LPWM2, LOW);

  digitalWrite(EN1_R, LOW);
  digitalWrite(EN1_L, LOW);
  digitalWrite(EN2_R, LOW);
  digitalWrite(EN2_L, LOW);

  currentState = STOPPED;
  robotMoving = false;
}

void moveForward() {
  enableMotors();
  digitalWrite(RPWM1, LOW);
  digitalWrite(LPWM1, HIGH);
  digitalWrite(RPWM2, HIGH);
  digitalWrite(LPWM2, LOW);
  currentState = FORWARD;
  robotMoving = true;
}

void moveBackward() {
  enableMotors();
  digitalWrite(RPWM1, HIGH);
  digitalWrite(LPWM1, LOW);
  digitalWrite(RPWM2, LOW);
  digitalWrite(LPWM2, HIGH);
  currentState = BACKWARD;
  robotMoving = true;
}

void turnLeft() {
  enableMotors();
  digitalWrite(RPWM1, HIGH);
  digitalWrite(LPWM1, LOW);
  digitalWrite(RPWM2, HIGH);
  digitalWrite(LPWM2, LOW);
  currentState = LEFT;
  robotMoving = true;
}

void turnRight() {
  enableMotors();
  digitalWrite(RPWM1, LOW);
  digitalWrite(LPWM1, HIGH);
  digitalWrite(RPWM2, LOW);
  digitalWrite(LPWM2, HIGH);
  currentState = RIGHT;
  robotMoving = true;
}

// ---------------------- Ultrasonic helper ----------------------
long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  return duration * 0.034 / 2;
}

// ---------------------- Setup ----------------------
void setup() {
  Serial.begin(115200);
  Serial.println("Robot Init with Staircase Flag...");

  // Motor pins
  pinMode(RPWM1, OUTPUT);
  digitalWrite(RPWM1, LOW);
  pinMode(LPWM1, OUTPUT);
  digitalWrite(LPWM1, LOW);
  pinMode(RPWM2, OUTPUT);
  digitalWrite(RPWM2, LOW);
  pinMode(LPWM2, OUTPUT);
  digitalWrite(LPWM2, LOW);

  pinMode(EN1_R, OUTPUT);
  digitalWrite(EN1_R, LOW);
  pinMode(EN1_L, OUTPUT);
  digitalWrite(EN1_L, LOW);
  pinMode(EN2_R, OUTPUT);
  digitalWrite(EN2_R, LOW);
  pinMode(EN2_L, OUTPUT);
  digitalWrite(EN2_L, LOW);

  // Servo driver
  pwm.begin();
  pwm.setPWMFreq(50);

  // Initialize servos to safe mid positions
  setDS3218Angle(FRONT_LEFT_CH, 90);
  setDS3218Angle(FRONT_RIGHT_CH, 90);
  setDS3218Angle(REAR_LEFT_CH, 90);
  setTD8120Angle(REAR_RIGHT_CH, 90);

  // Ultrasonic pins
  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);
  pinMode(trigRear, OUTPUT);
  pinMode(echoRear, INPUT);

  // Bump sensors
  pinMode(bumpLeft, INPUT);
  pinMode(bumpRight, INPUT);

  // Bluetooth
  SerialBT.begin("Robot | StairDoc");
  Serial.println("Bluetooth ready. Motors OFF, servos initialized.");

  // MPU6050 init
  Wire.begin(21, 22, 400000);
  byte status = mpu.begin();
  if (status == 0) {
    mpuActive = true;
    mpu.calcOffsets();
    Serial.println("MPU6050 active.");
  } else {
    Serial.println("MPU6050 not responding. Pitch override disabled.");
  }

  // Staircase flag logic
  if (nearStaircase) {
    obstacleThreshold = 3;  // tighter threshold
    Serial.println("Near staircase: threshold set to 3 cm, bump disabled.");
  } else {
    obstacleThreshold = 20;  // normal threshold
    Serial.println("Normal mode: threshold set to 20 cm, bump enabled.");
  }

  // HX711 init (commented out)
  // scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  // scale.set_scale();
  // scale.tare();
}

// ---------------------- Loop ----------------------
void loop() {
  unsigned long now = millis();

  // MPU6050 pitch update
  if (mpuActive) {
    mpu.update();
    pitch = mpu.getAngleY();
    Serial.print("Pitch: ");
    Serial.println(pitch);
  }

  // Ultrasonic obstacle check (every 100ms when moving)
  if (robotMoving && now - lastMotionUltrasonicRead >= 100) {
    frontDist = readUltrasonic(trigFront, echoFront);
    rearDist = readUltrasonic(trigRear, echoRear);
    Serial.printf("Front: %ld cm | Rear: %ld cm\n", frontDist, rearDist);
    lastMotionUltrasonicRead = now;
  }

  // Bluetooth commands
  if (SerialBT.available()) {
    command = SerialBT.read();
    lastCmdTime = now;
    Serial.printf("BT command: %c\n", command);

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
    } else if (command == 'f') moveForward();
    else if (command == 'b') moveBackward();
    else if (command == 'l') turnLeft();
    else if (command == 'r') turnRight();
    else if (command == 's') stopMotors();
  }

  // Safety: obstacle detection
  if (robotMoving) {
    if (frontDist > 0 && frontDist < obstacleThreshold && currentState == FORWARD) {
      Serial.println("Obstacle in front! Stopping.");
      stopMotors();
    }
    if (rearDist > 0 && rearDist < obstacleThreshold && currentState == BACKWARD) {
      Serial.println("Obstacle in rear! Stopping.");
      stopMotors();
    }
  }

  // Safety: bump sensors (disabled near staircase)
  if (!nearStaircase) {
    if (digitalRead(bumpLeft) == LOW || digitalRead(bumpRight) == LOW) {
      if (!bumpRecovering) {
        stopMotors();
        moveBackward();
        bumpTime = now;
        bumpRecovering = true;
      }
    }
    if (bumpRecovering && now - bumpTime >= 1000) {  // 1s recovery
      stopMotors();
      bumpRecovering = false;
    }
  }

  // Safety: pitch override (non-blocking, 1s forward)
  static unsigned long pitchStart = 0;
  if (mpuActive) {
    if (abs(pitch) > 50 && !pitchOverride) {
      moveForward();
      pitchOverride = true;
      pitchStart = now;
    }
    if (pitchOverride && now - pitchStart >= 1000) {  // 1s forward
      stopMotors();
      pitchOverride = false;
    }
  }

  // Safety: command timeout
  if (now - lastCmdTime > timeout) {
    stopMotors();
  }

  // Update sweeps
  updateSweeps();
}

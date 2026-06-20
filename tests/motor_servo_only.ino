/*
===========================================================
NOTE: This version includes:
 - Servo control (DS3218 + TD8120) with sweeps
 - Motor driver pin definitions, initialization, and functions
 - Bluetooth commands for both servos and motors
 - Ultrasonic helper function

EXCLUDED PARTS (relative to the original full robot code):
 - Bump sensor logic (digitalRead of bumpLeft/bumpRight)
 - Pitch override logic using MPU6050 (tilt-based auto movement)
 - Stair detection ultrasonic sensors (left/right stair reads)
 - HX711 load cell code (weight measurement)
 - Command timeout safety logic
 - Non-blocking timing helpers for stair/HX711 reads

Reason: These were removed to simplify and isolate the servo + motor + Bluetooth core,
while keeping motors safely initialized OFF at startup to avoid backward spin.
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

// --- Servo channels ---
#define FRONT_LEFT_CH 0   // DS3218
#define FRONT_RIGHT_CH 1  // DS3218
#define REAR_LEFT_CH 2    // DS3218
#define REAR_RIGHT_CH 3   // TD8120

// --- Servo limits ---
#define DS3218_USMIN 500
#define DS3218_USMAX 2500
#define TD8120_MIN   150
#define TD8120_MAX   600

// --- Motor control pins (BTS7960) ---
#define LPWM1 16
#define RPWM1 17
#define LPWM2 19
#define RPWM2 13
#define EN1_L 23
#define EN1_R 5
#define EN2_L 2
#define EN2_R 0

// --- Ultrasonic sensor pins ---
#define trigFront 32
#define echoFront 33
#define trigRear 15
#define echoRear 4

// --- Globals ---
char command = '\0';
bool robotMoving = false;
enum MovementState { STOPPED, FORWARD, BACKWARD, LEFT, RIGHT };
MovementState currentState = STOPPED;

// --- Sweep state ---
bool sweepingFront = false;
int frontSweepPos = 0;
int frontSweepEnd = 0;
unsigned long lastFrontUpdate = 0;

bool sweepingRear = false;
int rearSweepPos = 0;
int rearSweepEnd = 0;
unsigned long lastRearUpdate = 0;

// --- Sweep timing ---
const unsigned long sweepInterval = 20;  // ms between updates

// ---------------------- Servo helpers ----------------------
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
  Serial.println("Robot init starting...");

  // Motor pins
  pinMode(RPWM1, OUTPUT); digitalWrite(RPWM1, LOW);
  pinMode(LPWM1, OUTPUT); digitalWrite(LPWM1, LOW);
  pinMode(RPWM2, OUTPUT); digitalWrite(RPWM2, LOW);
  pinMode(LPWM2, OUTPUT); digitalWrite(LPWM2, LOW);

  pinMode(EN1_R, OUTPUT); digitalWrite(EN1_R, LOW);
  pinMode(EN1_L, OUTPUT); digitalWrite(EN1_L, LOW);
  pinMode(EN2_R, OUTPUT); digitalWrite(EN2_R, LOW);
  pinMode(EN2_L, OUTPUT); digitalWrite(EN2_L, LOW);

  // Servo driver
  pwm.begin();
  pwm.setPWMFreq(50);

  setDS3218Angle(FRONT_LEFT_CH, 0);
  setDS3218Angle(FRONT_RIGHT_CH, 180);
  setDS3218Angle(REAR_LEFT_CH, 0);
  setTD8120Angle(REAR_RIGHT_CH, 180);

  // Ultrasonic pins
  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);
  pinMode(trigRear, OUTPUT);
  pinMode(echoRear, INPUT);

  // Bluetooth
  SerialBT.begin("FullRobotTester");
  Serial.println("Bluetooth ready. Motors OFF, servos initialized.");
}

// ---------------------- Loop ----------------------
void loop() {
  // Bluetooth commands
  if (SerialBT.available()) {
    char cmd = SerialBT.read();
    Serial.printf("BT command: %c\n", cmd);

    if (cmd == 'u') { // front sweep up
      sweepingFront = true; frontSweepPos = 0; frontSweepEnd = 180;
    } else if (cmd == 'd') { // front sweep down
      sweepingFront = true; frontSweepPos = 180; frontSweepEnd = 0;
    } else if (cmd == 'v') { // rear sweep up
      sweepingRear = true; rearSweepPos = 0; rearSweepEnd = 180;
    } else if (cmd == 'e') { // rear sweep down
      sweepingRear = true; rearSweepPos = 180; rearSweepEnd = 0;
    } else if (cmd == 'f') moveForward();
    else if (cmd == 'b') moveBackward();
    else if (cmd == 'l') turnLeft();
    else if (cmd == 'r') turnRight();
    else if (cmd == 's') stopMotors();
  }

  // Update sweeps
  updateSweeps();
}

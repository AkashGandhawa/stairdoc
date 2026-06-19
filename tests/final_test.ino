#include <Wire.h>
#include <MPU6050_light.h>
#include <BluetoothSerial.h>
#include <Adafruit_PWMServoDriver.h>
// #include <HX711.h>

// --- Shared objects ---
MPU6050 mpu(Wire);
BluetoothSerial SerialBT;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// HX711 scale;

// --- Motor control pins (BTS7960) ---
#define LPWM1 16
#define RPWM1 17
#define LPWM2 19
#define RPWM2 13
#define EN1_L 23
#define EN1_R 5
#define EN2_L 2
#define EN2_R 0

// --- PCA9685 servo channels ---
#define FRONT_LEFT_CH 0   // DS3218
#define FRONT_RIGHT_CH 1  // DS3218
#define REAR_LEFT_CH 2    // DS3218
#define REAR_RIGHT_CH 3   // TD8120

// --- Servo limits ---
#define TD8120_MIN 150
#define TD8120_MAX 600
#define DS3218_USMIN 500
#define DS3218_USMAX 2500

// --- HX711 ---
// const int LOADCELL_DOUT_PIN = 36;
// const int LOADCELL_SCK_PIN = 25;

// --- Globals ---
char command = '\0';
float pitch = 0.0;
bool pitchOverride = false;
unsigned long lastCmdTime = 0;
const unsigned long timeout = 10000;
float calibration_factor = 433.65;
bool mpuActive = false;
bool robotMoving = false;

enum MovementState { STOPPED,
                     FORWARD,
                     BACKWARD,
                     LEFT,
                     RIGHT };
MovementState currentState = STOPPED;

// --- Ultrasonic sensor pins ---
#define trigFront 32
#define echoFront 33
#define trigRear 15
#define echoRear 4
#define trigLeft 26
#define echoLeft 27
#define trigRight 14
#define echoRight 12

// --- Bump sensor pins ---
#define bumpLeft 34
#define bumpRight 35

// --- Non-blocking timing helpers ---
// unsigned long lastHX711Read = 0;
unsigned long lastStairRead = 0;
unsigned long lastMotionUltrasonicRead = 0;

// --- Sweep state ---
bool sweepingFront = false;
int frontSweepPos = 0;
int frontSweepEnd = 0;  // target angle for front sweep
unsigned long lastFrontUpdate = 0;

bool sweepingRear = false;
int rearSweepPos = 0;
int rearSweepEnd = 0;  // target angle for rear sweep
unsigned long lastRearUpdate = 0;

// --- Sweep timing ---
const unsigned long sweepInterval = 20;  // ms between servo updates

// ---------------------- Setup ----------------------
void setup() {
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

  pwm.begin();
  pwm.setPWMFreq(50);
  stopMotors();
  delay(200);

  // scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  // scale.set_scale();
  // scale.tare();

  // Initialize servos to safe positions
  // Front arms (channels 0,1)
  setDS3218Angle(FRONT_LEFT_CH, frontSweepPos);
  setDS3218Angle(FRONT_RIGHT_CH, 180 - frontSweepPos);

  // Rear arms (channels 2,3)
  setDS3218Angle(REAR_LEFT_CH, rearSweepPos);
  setTD8120Angle(REAR_RIGHT_CH, 180 - rearSweepPos);

  SerialBT.begin("StairdocRobot");

  Serial.begin(500000);  // Faster baud rate
  Serial.println("Initializing Stairdoc Robot...");

  Wire.begin(21, 22, 400000);  // Fast I2C

  byte status = mpu.begin();
  if (status == 0) {
    mpuActive = true;
    mpu.calcOffsets();
  } else {
    Serial.println("MPU6050 not responding. Servo arms disabled");
    Serial.println("Check MPU6050 connection and reset to enable servo arms");
  }
}

// ---------------------- Loop ----------------------
void loop() {
  unsigned long now = millis();

  // MPU6050 pitch update
  if (mpuActive) {
    mpu.update();
    pitch = mpu.getAngleY();
  }

  // Motion ultrasonic sensors (only when moving, every 100ms)
  if (robotMoving && now - lastMotionUltrasonicRead >= 100) {
    long frontDist = readUltrasonic(trigFront, echoFront);
    long rearDist = readUltrasonic(trigRear, echoRear);

    Serial.printf("Front: %ld cm | Rear: %ld cm\n", frontDist, rearDist);
    lastMotionUltrasonicRead = now;
  }

  // Stair sensors (once per minute)
  if (now - lastStairRead >= 60000) {
    long frontLeftStair = readUltrasonic(trigLeft, echoLeft);
    long rearStair = readUltrasonic(trigRight, echoRight);

    Serial.printf("Front-Left Stair: %ld cm | Rear Stair: %ld cm\n", frontLeftStair, rearStair);
    lastStairRead = now;
  }

  // HX711 (only when stopped, every 5s)
  // if (!robotMoving && now - lastHX711Read >= 5000) {
  //   scale.set_scale(calibration_factor);
  //   Serial.printf("Weight: %.1f g | Cal: %.2f\n", scale.get_units(), calibration_factor);
  //   lastHX711Read = now;
  // }

  // Bluetooth commands
  if (SerialBT.available()) {
    command = SerialBT.read();
    lastCmdTime = now;
    Serial.printf("BT command: %c\n", command);

    // Front arm sweeps
    if (command == 'u') {
      sweepingFront = true;
      frontSweepPos = 0;
      frontSweepEnd = 180;
      lastFrontUpdate = now;
    } else if (command == 'd') {
      sweepingFront = true;
      frontSweepPos = 180;
      frontSweepEnd = 0;
      lastFrontUpdate = now;
    }
    // Rear arm sweeps
    else if (command == 'v') {
      sweepingRear = true;
      rearSweepPos = 0;
      rearSweepEnd = 180;
      lastRearUpdate = now;
    } else if (command == 'e') {
      sweepingRear = true;
      rearSweepPos = 180;
      rearSweepEnd = 0;
      lastRearUpdate = now;
    }
  }

  // Safety checks while moving
  if (robotMoving) {
    long frontDist = readUltrasonic(trigFront, echoFront);
    long rearDist = readUltrasonic(trigRear, echoRear);

    if (frontDist > 0 && frontDist < 20 && currentState == FORWARD) {
      Serial.println("Obstacle in front! Stopping.");
      stopMotors();
    }
    if (rearDist > 0 && rearDist < 20 && currentState == BACKWARD) {
      Serial.println("Obstacle in rear! Stopping.");
      stopMotors();
    }
  }

  // Bump sensors
  if (digitalRead(bumpLeft) == LOW || digitalRead(bumpRight) == LOW) {
    stopMotors();
    // Non-blocking recovery: schedule backward motion for 2s
    moveBackward();
    if (now - lastCmdTime >= 2000) {
      stopMotors();
    }
  }

  // Pitch override
  if (mpuActive) {
    if (abs(pitch) > 50) {
      moveForward();
      pitchOverride = true;
    } else if (pitchOverride && abs(pitch) <= 50) {
      stopMotors();
      pitchOverride = false;
    }
  }

  // Command timeout
  if (now - lastCmdTime > timeout) {
    stopMotors();
  }

  // Command execution
  if (command == 'f') moveForward();
  else if (command == 'b') moveBackward();
  else if (command == 'l') turnLeft();
  else if (command == 'r') turnRight();
  else if (command == 's') stopMotors();

  // Calibration factor adjustment
  // if (Serial.available()) {
  //   char temp = Serial.read();
  //   if (temp == '+' || temp == 'a') calibration_factor += 10;
  //   else if (temp == '-' || temp == 'z') calibration_factor -= 10;
  //   else if (temp == 's') calibration_factor += 100;
  //   else if (temp == 'x') calibration_factor -= 100;
  // }

  // Update sweeps (non-blocking)
  updateSweeps();
}
// ---------------------- Motor control functions ----------------------
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

// ---------------------- Servo helpers ----------------------

// DS3218 positional style
void setDS3218Angle(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);
  uint32_t pulseWidthMicros = map(angle, 0, 180, DS3218_USMIN, DS3218_USMAX);
  pwm.writeMicroseconds(channel, pulseWidthMicros);
}

// TD8120 positional style
void setTD8120Angle(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, TD8120_MIN, TD8120_MAX);
  pwm.setPWM(channel, 0, pulse);
}

// Non-blocking sweep updates
void updateSweeps() {
  unsigned long now = millis();

  // Only disable motors if a sweep is active AND robot is not supposed to be moving
  if ((sweepingFront || sweepingRear) && !robotMoving) {
    stopMotors();
  }

  // --- Front arms (DS3218 pair: channels 0,1) ---
  if (sweepingFront && (now - lastFrontUpdate >= sweepInterval)) {
    setDS3218Angle(FRONT_LEFT_CH, frontSweepPos);
    setDS3218Angle(FRONT_RIGHT_CH, 180 - frontSweepPos);

    if (frontSweepPos < frontSweepEnd) {
      frontSweepPos += 3;
    } else if (frontSweepPos > frontSweepEnd) {
      frontSweepPos -= 3;
    } else {
      sweepingFront = false;
    }
    lastFrontUpdate = now;
  }

  // --- Rear arms (DS3218 left channel 2 + TD8120 right channel 3) ---
  if (sweepingRear && (now - lastRearUpdate >= sweepInterval)) {
    setDS3218Angle(REAR_LEFT_CH, rearSweepPos);
    setTD8120Angle(REAR_RIGHT_CH, 180 - rearSweepPos);

    if (rearSweepPos < rearSweepEnd) {
      rearSweepPos += 1;
    } else if (rearSweepPos > rearSweepEnd) {
      rearSweepPos -= 1;
    } else {
      sweepingRear = false;
    }
    lastRearUpdate = now;
  }
}

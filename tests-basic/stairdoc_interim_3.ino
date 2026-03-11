#include <Wire.h>
#include <MPU6050_light.h>
#include <BluetoothSerial.h>
#include <Adafruit_PWMServoDriver.h>

// --- Shared objects ---
MPU6050 mpu(Wire);
BluetoothSerial SerialBT;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

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
#define trigLeft 26
#define echoLeft 27
#define trigRight 14
#define echoRight 12

// --- Bump sensor pins ---
#define bumpLeft 34
#define bumpRight 35

// --- PCA9685 servo driver channels (four DS3218 servos) ---
#define FRONT_LEFT_CH   0
#define FRONT_RIGHT_CH  1
#define REAR_LEFT_CH    2
#define REAR_RIGHT_CH   3

// --- Servo pulse endpoints (PCA9685 ticks) ---
#define SERVO_MIN  150   // conservative endpoint (ticks)
#define SERVO_MAX  600   // conservative endpoint (ticks)

// --- Globals ---
char command = '\0';               // latest Bluetooth command
float pitch = 0.0;                 // Y-angle from MPU6050
bool pitchOverride = false;
unsigned long lastCmdTime = 0;
const unsigned long timeout = 10000;  // 10-second command timeout

// Movement state tracking
enum MovementState { STOPPED, FORWARD, BACKWARD, LEFT, RIGHT };
MovementState currentState = STOPPED;

// --- Setup / Loop ---
void setup() {
  Serial.begin(115200);
  Serial.println("Initializing robot + servos...");

  // I2C for MPU6050 and PCA9685
  Wire.begin(21, 22);

  // Initialize MPU6050
  byte status = mpu.begin();
  Serial.print("MPU6050 status: "); Serial.println(status);
  while (status != 0) {
    Serial.println("MPU6050 not responding. Check wiring.");
    delay(1000);
    status = mpu.begin();
  }
  Serial.println("Calculating MPU6050 offsets...");
  delay(1000);
  mpu.calcOffsets();
  Serial.println("MPU offsets done.");

  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(50);   // Standard servo frequency
  delay(10);

  // Initialize servos to neutral (0°)
  setFrontAngle(0);
  setRearAngle(0);

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

  // Initialize Bluetooth (single device name)
  SerialBT.begin("InterimRobot");  // Device name
  Serial.println("Bluetooth initialized. Robot + servos ready.");

  stopMotors();  // Ensure motors are off at startup
  delay(500);

  // Enable motor drivers
  digitalWrite(EN1_R, HIGH); digitalWrite(EN1_L, HIGH);
  digitalWrite(EN2_R, HIGH); digitalWrite(EN2_L, HIGH);
}

void loop() {
  // Update IMU
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

  // Handle incoming Bluetooth commands (single stream for both subsystems)
  if (SerialBT.available()) {
    command = SerialBT.read();
    lastCmdTime = millis();
    Serial.print("BT command received: ");
    Serial.println(command);

    // Servo commands (u,d for front; v,e for rear)
    if (command == 'u') {
      Serial.println("Servo: Sweeping front servos up (0 -> 180)");
      sweepFront(0, 180);
    } else if (command == 'd') {
      Serial.println("Servo: Sweeping front servos down (180 -> 0)");
      sweepFront(180, 0);
    } else if (command == 'v') {
      Serial.println("Servo: Sweeping rear servos up (0 -> 180)");
      sweepRear(0, 180);
    } else if (command == 'e') {
      Serial.println("Servo: Sweeping rear servos down (180 -> 0)");
      sweepRear(180, 0);
    }
    // Robot movement commands handled below (non-blocking decision)
  }

  // Bump sensor override (immediate)
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

  // Obstacle override for movement commands
  if ((command == 'f' && frontDist <= 20) ||
      (command == 'b' && rearDist <= 20) ||
      (command == 'l' && leftDist <= 20) ||
      (command == 'r' && rightDist <= 20)) {
    Serial.println("Obstacle detected. Stopping.");
    stopMotors();
    return;
  }

  // Timeout for movement commands
  if (millis() - lastCmdTime > timeout) {
    Serial.println("Command timeout. Stopping.");
    stopMotors();
    return;
  }

  // Execute movement Bluetooth command (if any)
  if (command == 'f') moveForward();
  else if (command == 'b') moveBackward();
  else if (command == 'l') turnLeft();
  else if (command == 'r') turnRight();
  else if (command == 's') stopMotors();

  // small loop delay to avoid flooding serial
  delay(50);
}

// ---------------------- Motor control functions ----------------------
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

// ---------------------- Servo helper functions ----------------------
// Set a single servo channel to an angle (0..180)
void setServoAngle(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(channel, 0, pulse);
}

// Set front pair: front-left = angle, front-right = 180 - angle
void setFrontAngle(int angle) {
  angle = constrain(angle, 0, 180);
  int comp = 180 - angle;
  setServoAngle(FRONT_LEFT_CH, angle);
  setServoAngle(FRONT_RIGHT_CH, comp);
}

// Set rear pair: rear-left = angle, rear-right = 180 - angle
void setRearAngle(int angle) {
  angle = constrain(angle, 0, 180);
  int comp = 180 - angle;
  setServoAngle(REAR_LEFT_CH, angle);
  setServoAngle(REAR_RIGHT_CH, comp);
}

// Sweep front servos from startAngle to endAngle (inclusive)
void sweepFront(int startAngle, int endAngle) {
  if (startAngle < endAngle) {
    for (int pos = startAngle; pos <= endAngle; pos += 5) {
      setFrontAngle(pos);
      delay(50);
    }
  } else {
    for (int pos = startAngle; pos >= endAngle; pos -= 5) {
      setFrontAngle(pos);
      delay(50);
    }
  }
}

// Sweep rear servos from startAngle to endAngle (inclusive)
void sweepRear(int startAngle, int endAngle) {
  if (startAngle < endAngle) {
    for (int pos = startAngle; pos <= endAngle; pos += 5) {
      setRearAngle(pos);
      delay(50);
    }
  } else {
    for (int pos = startAngle; pos >= endAngle; pos -= 5) {
      setRearAngle(pos);
      delay(50);
    }
  }
}

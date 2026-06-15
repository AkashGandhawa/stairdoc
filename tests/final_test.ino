#include <Wire.h>
#include <MPU6050_light.h>
#include <BluetoothSerial.h>
#include <Adafruit_PWMServoDriver.h>
#include <HX711.h>

// --- Shared objects ---
MPU6050 mpu(Wire);
BluetoothSerial SerialBT;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
HX711 scale;

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

// --- PCA9685 servo driver channels ---
#define FRONT_LEFT_CH 0   // DS3218
#define FRONT_RIGHT_CH 1  // DS3218
#define REAR_LEFT_CH 2    // DS3218
#define REAR_RIGHT_CH 3   // TD8120

// --- Servo pulse endpoints ---
#define SERVO_MIN 150
#define SERVO_MAX 600

// --- HX711 ---
const int LOADCELL_DOUT_PIN = 36;  // DT
const int LOADCELL_SCK_PIN = 25;   // SCK

// --- Globals ---
char command = '\0';
float pitch = 0.0;
bool pitchOverride = false;
unsigned long lastCmdTime = 0;
const unsigned long timeout = 10000;
float calibration_factor = 433.65;
bool mpuActive = false;
bool robotMoving = false;

enum MovementState { STOPPED, FORWARD, BACKWARD, LEFT, RIGHT };
MovementState currentState = STOPPED;

// --- Non-blocking timing helpers ---
unsigned long lastHX711Read = 0;
unsigned long lastStairRead = 0;
unsigned long lastMotionUltrasonicRead = 0;

// --- Unified sweep state ---
bool sweepingRear = false;
int sweepStartAngle = 0;
int sweepEndAngle = 0;
int sweepPos = 0;
unsigned long lastSweepUpdate = 0;
const unsigned long sweepInterval = 50; // ms between steps

// DS3218 timing
unsigned long ds3218StartTime = 0;
const unsigned long ds3218Duration = 1000; // ms to approximate 180° rotation
bool ds3218Active = false;
int ds3218Direction = 0; // +1 = CCW, -1 = CW

// ---------------------- Setup ----------------------
void setup() {
  Serial.begin(115200);
  Serial.println("Initializing robot + servos...");

  Wire.begin(21, 22);

  byte status = mpu.begin();
  if (status != 0) {
    Serial.println("MPU6050 not responding. Servo Arms Disabled\nCheck MPU6050 connection and reset to enable servo arms");
  } else {
    mpuActive = true;
    mpu.calcOffsets();
  }

  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  pinMode(RPWM1, OUTPUT);
  pinMode(LPWM1, OUTPUT);
  pinMode(RPWM2, OUTPUT);
  pinMode(LPWM2, OUTPUT);
  pinMode(EN1_R, OUTPUT);
  pinMode(EN1_L, OUTPUT);
  pinMode(EN2_R, OUTPUT);
  pinMode(EN2_L, OUTPUT);

  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);
  pinMode(trigRear, OUTPUT);
  pinMode(echoRear, INPUT);
  pinMode(trigLeft, OUTPUT);
  pinMode(echoLeft, INPUT);
  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);

  pinMode(bumpLeft, INPUT);
  pinMode(bumpRight, INPUT);

  SerialBT.begin("StairdocRobot");

  stopMotors();
  delay(500);

  digitalWrite(EN1_R, HIGH);
  digitalWrite(EN1_L, HIGH);
  digitalWrite(EN2_R, HIGH);
  digitalWrite(EN2_L, HIGH);

  Serial.println("HX711 Calibration");
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale();
  scale.tare();
}

// ---------------------- Loop ----------------------
void loop() {
  if(mpuActive){ 
    mpu.update();
    pitch = mpu.getAngleY();
  }

  // Motion ultrasonic sensors (only when moving)
  if(robotMoving && millis() - lastMotionUltrasonicRead >= 100) {
    long frontDist = readUltrasonic(trigFront, echoFront);
    long rearDist = readUltrasonic(trigRear, echoRear);

    Serial.print("Front: ");
    Serial.print(frontDist);
    Serial.print(" cm | Rear: ");
    Serial.print(rearDist);
    Serial.println(" cm");

    lastMotionUltrasonicRead = millis();
  }

  // Stair sensors (once per minute)
  if(millis() - lastStairRead >= 60000) {
    long frontLeftStair = readUltrasonic(trigLeft, echoLeft);
    long rearStair = readUltrasonic(trigRight, echoRight);

    Serial.print("Front-Left Stair Sensor: ");
    Serial.print(frontLeftStair);
    Serial.print(" cm | Front-Right Stair Sensor: ");
    Serial.print(rearStair);
    Serial.println(" cm");

    lastStairRead = millis();
  }

  // HX711 (only when stopped, every 5s)
  if(!robotMoving && millis() - lastHX711Read >= 5000) {
    scale.set_scale(calibration_factor);
    Serial.print("Reading: ");
    Serial.print(scale.get_units(), 1);
    Serial.print(" g | Calibration Factor: ");
    Serial.println(calibration_factor);
    lastHX711Read = millis();
  }

  // Bluetooth commands
  if (SerialBT.available()) {
    command = SerialBT.read();
    lastCmdTime = millis();
    Serial.print("BT command received: ");
    Serial.println(command);

    if (command == 'u') {
      Serial.println("Sweeping front servos up");
      startDS3218Rotation(false); // CCW timed rotation
    } else if (command == 'd') {
      Serial.println("Sweeping front servos down");
      startDS3218Rotation(true);  // CW timed rotation
    } else if (command == 'v') {
      Serial.println("Sweeping rear servos up");
      startDS3218Rotation(false);
      sweepingRear = true;
      sweepStartAngle = 0;
      sweepEndAngle = 180;
      sweepPos = 0;
      lastSweepUpdate = millis();
    } else if (command == 'e') {
      Serial.println("Sweeping rear servos down");
      startDS3218Rotation(true);
      sweepingRear = true;
      sweepStartAngle = 180;
      sweepEndAngle = 0;
      sweepPos = 180;
      lastSweepUpdate = millis();
    }
  }

  // Bump sensors
  if (digitalRead(bumpLeft) == LOW || digitalRead(bumpRight) == LOW) {
    stopMotors();
    delay(200);
    moveBackward();
    delay(2000);
    stopMotors();
    return;
  }

  // Pitch override
  if(mpuActive){
    if (abs(pitch) > 50) {
      moveForward();
      pitchOverride = true;
    } else if (pitchOverride && abs(pitch) <= 50) {
      stopMotors();
      pitchOverride = false;
    }
  }

  if (millis() - lastCmdTime > timeout) {
    stopMotors();
    return;
  }

  if (command == 'f') moveForward();
  else if (command == 'b') moveBackward();
  else if (command == 'l') turnLeft();
  else if (command == 'r') turnRight();
  else if (command == 's') stopMotors();

  // Calibration factor adjustment
  if (Serial.available()) {
    char temp = Serial.read();
    if (temp == '+' || temp == 'a') calibration_factor += 10;
    else if (temp == '-' || temp == 'z') calibration_factor -= 10;
    else if (temp == 's') calibration_factor += 100;
    else if (temp == 'x') calibration_factor -= 100;
  }

  // Update sweeps
  updateSweeps();
}

 // ---------------------- Motor control functions ----------------------
void stopMotors() {
  analogWrite(RPWM1, 0);
  analogWrite(LPWM1, 0);
  analogWrite(RPWM2, 0);
  analogWrite(LPWM2, 0);
  currentState = STOPPED;
  robotMoving = false;
}

void moveForward() {
  analogWrite(RPWM1, 0);
  analogWrite(LPWM1, 255);
  analogWrite(RPWM2, 255);
  analogWrite(LPWM2, 0);
  currentState = FORWARD;
  robotMoving = true;
}

void moveBackward() {
  analogWrite(RPWM1, 255);
  analogWrite(LPWM1, 0);
  analogWrite(RPWM2, 0);
  analogWrite(LPWM2, 255);
  currentState = BACKWARD;
  robotMoving = true;
}

void turnLeft() {
  analogWrite(RPWM1, 255);
  analogWrite(LPWM1, 0);
  analogWrite(RPWM2, 255);
  analogWrite(LPWM2, 0);
  currentState = LEFT;
  robotMoving = true;
}

void turnRight() {
  analogWrite(RPWM1, 0);
  analogWrite(LPWM1, 255);
  analogWrite(RPWM2, 0);
  analogWrite(LPWM2, 255);
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

// DS3218 continuous rotation style (timed relative motion)
void setDS3218Speed(uint8_t channel, int speed) {
  int pulse = map(speed, -100, 100, 1000, 2000); 
  pwm.setPWM(channel, 0, pulse);
}

void startDS3218Rotation(bool clockwise) {
  ds3218Active = true;
  ds3218Direction = clockwise ? -1 : 1;
  ds3218StartTime = millis();
  int speed = clockwise ? -100 : 100;
  setDS3218Speed(FRONT_LEFT_CH, speed);
  setDS3218Speed(FRONT_RIGHT_CH, -speed); // complementary
  setDS3218Speed(REAR_LEFT_CH, speed);
}

// TD8120 positional style
void setTD8120Angle(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(channel, 0, pulse);
}

// Unified sweep update
void updateSweeps() {
  // DS3218 timed rotation
  if(ds3218Active && millis() - ds3218StartTime >= ds3218Duration) {
    setDS3218Speed(FRONT_LEFT_CH, 0);
    setDS3218Speed(FRONT_RIGHT_CH, 0);
    setDS3218Speed(REAR_LEFT_CH, 0);
    ds3218Active = false;
  }

  // TD8120 positional sweep
  if(sweepingRear && millis() - lastSweepUpdate >= sweepInterval) {
    if(sweepStartAngle < sweepEndAngle) {
      if(sweepPos <= sweepEndAngle) {
        setTD8120Angle(REAR_RIGHT_CH, sweepPos);
        sweepPos += 5;
      } else sweepingRear = false;
    } else {
      if(sweepPos >= sweepEndAngle) {
        setTD8120Angle(REAR_RIGHT_CH, sweepPos);
        sweepPos -= 5;
      } else sweepingRear = false;
    }
    lastSweepUpdate = millis();
  }
}

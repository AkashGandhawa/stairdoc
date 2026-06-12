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
#define trigLeft 26  // repurposed as front-left stair sensor
#define echoLeft 27
#define trigRight 14  // repurposed as rear stair sensor
#define echoRight 12

// --- Bump sensor pins ---
#define bumpLeft 34
#define bumpRight 35

// --- PCA9685 servo driver channels ---
#define FRONT_LEFT_CH 0
#define FRONT_RIGHT_CH 1
#define REAR_LEFT_CH 2
#define REAR_RIGHT_CH 3

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

enum MovementState { STOPPED,
                     FORWARD,
                     BACKWARD,
                     LEFT,
                     RIGHT };
MovementState currentState = STOPPED;

// ---------------------- Setup ----------------------
void setup() {
  Serial.begin(115200);
  Serial.println("Initializing robot + servos...");

  Wire.begin(21, 22);

  byte status = mpu.begin();
  if (status != 0) {
    Serial.println("MPU6050 not responding. Servo Arms Disabled\nCheck MPU6050 connection and reset to enable servo arms");
  }else{
    mpuActive = true;
  }

  if(mpuActive){
    mpu.calcOffsets();
  }

  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  setFrontAngle(0);
  setRearAngle(0);

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

  long frontDist = readUltrasonic(trigFront, echoFront);
  long rearDist = readUltrasonic(trigRear, echoRear);

  // Repurposed ultrasonic sensors for stair detection
  long frontLeftStair = readUltrasonic(trigLeft, echoLeft);
  long rearStair = readUltrasonic(trigRight, echoRight);

  if(mpuActive){
    Serial.print("Pitch (Y-angle): ");
    Serial.println(pitch);
  }
  Serial.print("Front: ");
  Serial.print(frontDist);
  Serial.print(" cm | Rear: ");
  Serial.print(rearDist);
  Serial.println(" cm");

  Serial.print("Front-Left Stair Sensor: ");
  Serial.print(frontLeftStair);
  Serial.print(" cm | Rear Stair Sensor: ");
  Serial.println(rearStair);

  if (SerialBT.available()) {
    command = SerialBT.read();
    lastCmdTime = millis();
    Serial.print("BT command received: ");
    Serial.println(command);

    if (command == 'u') {
      Serial.println("Sweeping front servos up");
      sweepFrontPair(0, 180);
    } else if (command == 'd') {
      Serial.println("Sweeping front servos down");
      sweepFrontPair(180, 0);
    } else if (command == 'v') {
      Serial.println("Sweeping rear servos up");
      sweepRearPair(0, 180);
    } else if (command == 'e') {
      Serial.println("Sweeping rear servos down");
      sweepRearPair(180, 0);
    }
  }

  if (digitalRead(bumpLeft) == LOW || digitalRead(bumpRight) == LOW) {
    stopMotors();
    delay(200);
    moveBackward();
    delay(2000);
    stopMotors();
    return;
  }

  if(mpuActive){
    if (abs(pitch) > 50 && frontDist > 20) {
      moveForward();
      pitchOverride = true;
      return;
    }

    if (pitchOverride && abs(pitch) <= 50) {
      stopMotors();
      pitchOverride = false;
    }
  }

  if ((command == 'f' && frontDist <= 20) || (command == 'b' && rearDist <= 20)) {
    stopMotors();
    return;
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

  delay(50);

  scale.set_scale(calibration_factor);
  Serial.print("Reading: ");
  Serial.print(scale.get_units(), 1);
  Serial.print(" g | Calibration Factor: ");
  Serial.println(calibration_factor);

  if (Serial.available()) {
    char temp = Serial.read();
    if (temp == '+' || temp == 'a') calibration_factor += 10;
    else if (temp == '-' || temp == 'z') calibration_factor -= 10;
    else if (temp == 's') calibration_factor += 100;
    else if (temp == 'x') calibration_factor -= 100;
  }
}

// ---------------------- Motor control functions ----------------------
void stopMotors() {
  analogWrite(RPWM1, 0);
  analogWrite(LPWM1, 0);
  analogWrite(RPWM2, 0);
  analogWrite(LPWM2, 0);
  currentState = STOPPED;
}

void moveForward() {
  analogWrite(RPWM1, 0);
  analogWrite(LPWM1, 255);
  analogWrite(RPWM2, 255);
  analogWrite(LPWM2, 0);
  currentState = FORWARD;
}

void moveBackward() {
  analogWrite(RPWM1, 255);
  analogWrite(LPWM1, 0);
  analogWrite(RPWM2, 0);
  analogWrite(LPWM2, 255);
  currentState = BACKWARD;
}

void turnLeft() {
  analogWrite(RPWM1, 255);
  analogWrite(LPWM1, 0);
  analogWrite(RPWM2, 255);
  analogWrite(LPWM2, 0);
  currentState = LEFT;
}

void turnRight() {
  analogWrite(RPWM1, 0);
  analogWrite(LPWM1, 255);
  analogWrite(RPWM2, 0);
  analogWrite(LPWM2, 255);
  currentState = RIGHT;
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
void setServoAngle(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(channel, 0, pulse);
}

void setFrontAngle(int angle) {
  angle = constrain(angle, 0, 180);
  int comp = 180 - angle;
  setServoAngle(FRONT_LEFT_CH, angle);
  setServoAngle(FRONT_RIGHT_CH, comp);
}

void setRearAngle(int angle) {
  angle = constrain(angle, 0, 180);
  int comp = 180 - angle;
  setServoAngle(REAR_LEFT_CH, angle);
  setServoAngle(REAR_RIGHT_CH, comp);
}

// Sweep both front servos together
void sweepFrontPair(int startAngle, int endAngle) {
  if (startAngle < endAngle) {
    for (int pos = startAngle; pos <= endAngle; pos += 5) {
      setServoAngle(FRONT_LEFT_CH, pos);
      setServoAngle(FRONT_RIGHT_CH, 180 - pos);
      delay(50);
    }
  } else {
    for (int pos = startAngle; pos >= endAngle; pos -= 5) {
      setServoAngle(FRONT_LEFT_CH, pos);
      setServoAngle(FRONT_RIGHT_CH, 180 - pos);
      delay(50);
    }
  }
}

// Sweep both rear servos together
void sweepRearPair(int startAngle, int endAngle) {
  if (startAngle < endAngle) {
    for (int pos = startAngle; pos <= endAngle; pos += 5) {
      setServoAngle(REAR_LEFT_CH, pos);
      setServoAngle(REAR_RIGHT_CH, 180 - pos);
      delay(50);
    }
  } else {
    for (int pos = startAngle; pos >= endAngle; pos -= 5) {
      setServoAngle(REAR_LEFT_CH, pos);
      setServoAngle(REAR_RIGHT_CH, 180 - pos);
      delay(50);
    }
  }
}

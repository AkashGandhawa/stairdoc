#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// DS3218 channels to test
#define DS3218_FRONT_LEFT   4
#define DS3218_FRONT_RIGHT  3

// PCA9685 pulse values (12-bit, out of 4096)
// These correspond to ~µs values at 50 Hz
#define PULSE_STOP   375   // ~1500 µs
#define PULSE_FWD    300   // ~1300 µs
#define PULSE_REV    450   // ~1700 µs

void setup() {
  Serial.begin(115200);
  Serial.println("DS3218 diagnostic test starting...");

  Wire.begin(21, 22);   // SDA=21, SCL=22 for ESP32
  pwm.begin();
  pwm.setPWMFreq(50);   // Standard servo frequency
  delay(10);

  // Test sequence for both DS3218 servos
  Serial.println("Sending STOP pulse (~1500 µs)...");
  pwm.setPWM(DS3218_FRONT_LEFT, 0, PULSE_STOP);
  pwm.setPWM(DS3218_FRONT_RIGHT, 0, PULSE_STOP);
  delay(4000);

  Serial.println("Sending FORWARD pulse (~1300 µs)...");
  pwm.setPWM(DS3218_FRONT_LEFT, 0, PULSE_FWD);
  pwm.setPWM(DS3218_FRONT_RIGHT, 0, PULSE_FWD);
  delay(4000);

  Serial.println("Sending REVERSE pulse (~1700 µs)...");
  pwm.setPWM(DS3218_FRONT_LEFT, 0, PULSE_REV);
  pwm.setPWM(DS3218_FRONT_RIGHT, 0, PULSE_REV);
  delay(4000);

  Serial.println("Back to STOP...");
  pwm.setPWM(DS3218_FRONT_LEFT, 0, PULSE_STOP);
  pwm.setPWM(DS3218_FRONT_RIGHT, 0, PULSE_STOP);
}

void loop() {
  // Nothing here — test runs once in setup()
}


#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); 

// PCA9685 channel assignments
#define MG995_REAR_LEFT     0
#define MG995_REAR_RIGHT    1
#define DS3218_FRONT_LEFT   2
#define DS3218_FRONT_RIGHT  3

// Pulse ranges (adjust if needed)
// DS3218: 270° capable, but we constrain to 180°
// MG995: ~180° range
#define SERVO_MIN  150   // ~500 µs
#define SERVO_MAX  600   // ~2500 µs

char command = '\0';

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ServoTest");  
  Serial.println("Servo test ready. Send 'u' or 'd' via Bluetooth.");

  Wire.begin(21, 22);   // SDA=21, SCL=22 for ESP32

  pwm.begin();
  pwm.setPWMFreq(50);   // Standard servo frequency
  delay(10);

  // Initialize all servos to 0 degrees
  setServoAngle(DS3218_FRONT_LEFT, 0);
  setServoAngle(DS3218_FRONT_RIGHT, 0);
  setServoAngle(MG995_REAR_LEFT, 0);
  setServoAngle(MG995_REAR_RIGHT, 0);
}

void loop() {
  if (SerialBT.available()) {
    command = SerialBT.read();
    Serial.print("Command received: ");
    Serial.println(command);

    if (command == 'u') {
      Serial.println("Sweeping all servos up (0→180)");
      sweepServos(0, 180);
    } else if (command == 'd') {
      Serial.println("Sweeping all servos down (180→0)");
      sweepServos(180, 0);
    }
  }
}

// Helper: set servo angle
void setServoAngle(uint8_t servo, int angle) {
  // Constrain DS3218 to 180° even though it supports 270°
  if (servo == DS3218_FRONT_LEFT || servo == DS3218_FRONT_RIGHT) {
    angle = constrain(angle, 0, 180);
  }
  // MG995 is already ~180°, so no special handling needed

  int pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(servo, 0, pulse);
}

// Helper: sweep all servos together
void sweepServos(int startAngle, int endAngle) {
  if (startAngle < endAngle) {
    for (int pos = startAngle; pos <= endAngle; pos += 5) {
      setServoAngle(DS3218_FRONT_LEFT, pos);
      setServoAngle(DS3218_FRONT_RIGHT, pos);
      setServoAngle(MG995_REAR_LEFT, pos);
      setServoAngle(MG995_REAR_RIGHT, pos);
      delay(50);
    }
  } else {
    for (int pos = startAngle; pos >= endAngle; pos -= 5) {
      setServoAngle(DS3218_FRONT_LEFT, pos);
      setServoAngle(DS3218_FRONT_RIGHT, pos);
      setServoAngle(MG995_REAR_LEFT, pos);
      setServoAngle(MG995_REAR_RIGHT, pos);
      delay(50);
    }
  }
}

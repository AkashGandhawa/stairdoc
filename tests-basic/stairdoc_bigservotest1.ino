Stairdoc bigservo

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// PCA9685 channels for 4 DS3218 servos
#define SERVO_1 0
#define SERVO_2 1
#define SERVO_3 2
#define SERVO_4 3

// Pulse range for DS3218 (adjust if needed)
#define SERVO_MIN 150   // ~500 µs
#define SERVO_MAX 600   // ~2500 µs

char command = '\0';

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ServoTest");
  Serial.println("4x DS3218 Servo Test Ready");

  Wire.begin(21, 22);  // ESP32 I2C pins

  pwm.begin();
  pwm.setPWMFreq(50);  // Standard servo frequency
  delay(10);

  // Initialize all servos to 0°
  setServoAngle(SERVO_1, 0);
  setServoAngle(SERVO_2, 0);
  setServoAngle(SERVO_3, 0);
  setServoAngle(SERVO_4, 0);
}

void loop() {

  if (SerialBT.available()) {
    command = SerialBT.read();

    Serial.print("Command: ");
    Serial.println(command);

    if (command == 'u') {
      Serial.println("Moving servos UP (0 -> 180)");
      sweepServos(0, 180);
    }

    if (command == 'd') {
      Serial.println("Moving servos DOWN (180 -> 0)");
      sweepServos(180, 0);
    }
  }
}

// Set servo angle
void setServoAngle(uint8_t servo, int angle) {

  angle = constrain(angle, 0, 180);

  int pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);

  pwm.setPWM(servo, 0, pulse);
}

// Sweep all servos together
void sweepServos(int startAngle, int endAngle) {

  if (startAngle < endAngle) {

    for (int pos = startAngle; pos <= endAngle; pos += 5) {

      setServoAngle(SERVO_1, pos);
      setServoAngle(SERVO_2, pos);
      setServoAngle(SERVO_3, pos);
      setServoAngle(SERVO_4, pos);

      delay(50);
    }

  } else {

    for (int pos = startAngle; pos >= endAngle; pos -= 5) {

      setServoAngle(SERVO_1, pos);
      setServoAngle(SERVO_2, pos);
      setServoAngle(SERVO_3, pos);
      setServoAngle(SERVO_4, pos);

      delay(50);
    }
  }
}

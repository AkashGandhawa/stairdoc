#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// PCA9685 channel assignments (four DS3218 servos)
#define FRONT_LEFT_CH   0
#define FRONT_RIGHT_CH  1
#define REAR_LEFT_CH    2
#define REAR_RIGHT_CH   3

// Pulse ranges (adjust if needed)
// DS3218: 270° capable, but we constrain to 0..180°
#define SERVO_MIN  150   // ~500 µs
#define SERVO_MAX  600   // ~2500 µs

char command = '\0';

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ServoTest");
  Serial.println("Servo test ready. Send 'u','d','v','e' via Bluetooth.");

  Wire.begin(21, 22);   // SDA=21, SCL=22 for ESP32

  pwm.begin();
  pwm.setPWMFreq(50);   // Standard servo frequency
  delay(10);

  // Initialize all servos to 0 degrees (or choose a safe neutral angle)
  setFrontAngle(0);
  setRearAngle(0);
}

void loop() {
  if (SerialBT.available()) {
    command = SerialBT.read();
    Serial.print("Command received: ");
    Serial.println(command);

    switch (command) {
      case 'u': // sweep up for front servos (0 -> 180)
        Serial.println("Sweeping front servos up (0 -> 180)");
        sweepFront(0, 180);
        break;

      case 'd': // sweep down for front servos (180 -> 0)
        Serial.println("Sweeping front servos down (180 -> 0)");
        sweepFront(180, 0);
        break;

      case 'v': // sweep up for rear servos (0 -> 180)
        Serial.println("Sweeping rear servos up (0 -> 180)");
        sweepRear(0, 180);
        break;

      case 'e': // sweep down for rear servos (180 -> 0)
        Serial.println("Sweeping rear servos down (180 -> 0)");
        sweepRear(180, 0);
        break;

      default:
        Serial.println("Unknown command");
        break;
    }
  }
}

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

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BluetoothSerial.h>

// Create PCA9685 driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
BluetoothSerial SerialBT;

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

  // --- Front arms (DS3218 pair: channels 0,1) ---
  if (sweepingFront && (now - lastFrontUpdate >= sweepInterval)) {
    setDS3218Angle(FRONT_LEFT_CH, frontSweepPos);
    setDS3218Angle(FRONT_RIGHT_CH, 180 - frontSweepPos);

    if (frontSweepPos < frontSweepEnd) {
      frontSweepPos += 1;  // smoother increments
    } else if (frontSweepPos > frontSweepEnd) {
      frontSweepPos -= 1;
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

// ---------------------- Setup ----------------------
void setup() {
  Serial.begin(115200);
  Serial.println("Servo + Bluetooth test starting...");

  pwm.begin();
  pwm.setPWMFreq(50);  // Standard 50 Hz for hobby servos

  // Initialize servos to safe positions
  setDS3218Angle(FRONT_LEFT_CH, frontSweepPos);
  setDS3218Angle(FRONT_RIGHT_CH, 180 - frontSweepPos);
  setDS3218Angle(REAR_LEFT_CH, rearSweepPos);
  setTD8120Angle(REAR_RIGHT_CH, 180 - rearSweepPos);

  // Start Bluetooth
  SerialBT.begin("ServoTester");  // Device name
  Serial.println("Bluetooth ready. Connect and send commands.");
}

// ---------------------- Loop ----------------------
void loop() {
  // Handle Bluetooth commands
  if (SerialBT.available()) {
    char cmd = SerialBT.read();
    Serial.printf("BT command: %c\n", cmd);

    if (cmd == 'u') { // front sweep up
      sweepingFront = true;
      frontSweepPos = 0;
      frontSweepEnd = 180;
      lastFrontUpdate = millis();
    } else if (cmd == 'd') { // front sweep down
      sweepingFront = true;
      frontSweepPos = 180;
      frontSweepEnd = 0;
      lastFrontUpdate = millis();
    } else if (cmd == 'v') { // rear sweep up
      sweepingRear = true;
      rearSweepPos = 0;
      rearSweepEnd = 180;
      lastRearUpdate = millis();
    } else if (cmd == 'e') { // rear sweep down
      sweepingRear = true;
      rearSweepPos = 180;
      rearSweepEnd = 0;
      lastRearUpdate = millis();
    }
  }

  // Update sweeps
  updateSweeps();
}

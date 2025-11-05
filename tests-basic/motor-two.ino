// Motor A (Left track)
const int IN1_A = 18;
const int IN2_A = 19;

// Motor B (Right track)
const int IN1_B = 21;
const int IN2_B = 22;

void setup() {
  // Motor A setup
  pinMode(IN1_A, OUTPUT);
  pinMode(IN2_A, OUTPUT);

  // Motor B setup
  pinMode(IN1_B, OUTPUT);
  pinMode(IN2_B, OUTPUT);

  Serial.begin(115200);
  Serial.println("Dual motor test with turning (no speed control) started...");
}

void loop() {
  // Forward
  digitalWrite(IN1_A, HIGH); digitalWrite(IN2_A, LOW);
  digitalWrite(IN1_B, HIGH); digitalWrite(IN2_B, LOW);
  Serial.println("Moving forward");
  delay(1500);

  // Stop
  stopMotors();

  // Reverse
  digitalWrite(IN1_A, LOW); digitalWrite(IN2_A, HIGH);
  digitalWrite(IN1_B, LOW); digitalWrite(IN2_B, HIGH);
  Serial.println("Moving backward");
  delay(1500);

  // Stop
  stopMotors();

  // Turn Left
  digitalWrite(IN1_A, LOW); digitalWrite(IN2_A, HIGH);   // Left motor reverse
  digitalWrite(IN1_B, HIGH); digitalWrite(IN2_B, LOW);   // Right motor forward
  Serial.println("Turning left");
  delay(1000);

  // Stop
  stopMotors();

  // Turn Right
  digitalWrite(IN1_A, HIGH); digitalWrite(IN2_A, LOW);   // Left motor forward
  digitalWrite(IN1_B, LOW); digitalWrite(IN2_B, HIGH);   // Right motor reverse
  Serial.println("Turning right");
  delay(1000);

  // Stop
  stopMotors();
}

void stopMotors() {
  digitalWrite(IN1_A, LOW); digitalWrite(IN2_A, LOW);
  digitalWrite(IN1_B, LOW); digitalWrite(IN2_B, LOW);
  Serial.println("Motors stopped");
  delay(1000);
}

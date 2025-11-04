// Front sensor (HC-SRF05)
const int trigFront = 18;
const int echoFront = 19;

// Left sensor (HC-SRF05)
const int trigLeft = 21;
const int echoLeft = 22;

// Right sensor (HC-SR04)
const int trigRight = 23;
const int echoRight = 25;

void setup() {
  Serial.begin(115200);

  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);

  pinMode(trigLeft, OUTPUT);
  pinMode(echoLeft, INPUT);

  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);

  Serial.println("Ultrasonic sensor array test started...");
}

float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;  // Distance in cm
}

void loop() {
  float frontDist = readDistance(trigFront, echoFront);
  float leftDist = readDistance(trigLeft, echoLeft);
  float rightDist = readDistance(trigRight, echoRight);

  Serial.print("Front: ");
  Serial.print(frontDist);
  Serial.print(" cm | Left: ");
  Serial.print(leftDist);
  Serial.print(" cm | Right: ");
  Serial.print(rightDist);
  Serial.println(" cm");

  delay(500);
}

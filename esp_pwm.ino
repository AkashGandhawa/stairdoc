//Example for providing PWM (analog) outputs from ESP32
#include <Arduino.h>

#define LEDC_PIN 2
#define LEDC_RESOLUTION 10

void setup() {
  ledcAttach(LEDC_PIN, 50, LEDC_RESOLUTION);
}

void loop() {
  // Gradually increase LED brightness
  for (int duty = 0; duty <= 255; duty++) {
    ledcWrite(LEDC_PIN, duty);
    delay(1);
  }

  // Gradually decrease LED brightness
  for (int duty = 255; duty >= 0; duty--) {
    ledcWrite(LEDC_PIN, duty);
    delay(1);
  }
}

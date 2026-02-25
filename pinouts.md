# ⚙️ Interim Edition Robot Pinouts

## Motor Drivers (BTS7960)

| Function | Pin | Purpose |
|----------|-----|---------|
| LPWM1    | 16  | Left motor 1 forward PWM |
| RPWM1    | 17  | Left motor 1 reverse PWM |
| LPWM2    | 19  | Right motor 2 forward PWM |
| RPWM2    | 13  | Right motor 2 reverse PWM |
| EN1_L    | 23  | Enable left motor driver channel |
| EN1_R    | 5   | Enable right motor driver channel |
| EN2_L    | 2 ⚠️ | Enable left motor driver channel (ESP32 strapping pin) |
| EN2_R    | 0 ⚠️ | Enable right motor driver channel (ESP32 strapping pin) |

---

## Ultrasonic Sensors (HC‑SR04)

| Sensor Position | Trig Pin | Echo Pin | Purpose |
|-----------------|----------|----------|---------|
| Front           | 32       | 33       | Detect obstacles ahead |
| Rear            | 15       | 4        | Detect obstacles behind |
| Left            | 26       | 27       | Detect obstacles to the left |
| Right           | 14       | 12       | Detect obstacles to the right |

⚠️ Echo pins must be stepped down to 3.3 V using a voltage divider (e.g., 1 kΩ + 560 Ω).

---

## Bump Sensors

| Sensor Position | Pin | Purpose |
|-----------------|-----|---------|
| Left bump       | 34  | Detect collision on left |
| Right bump      | 35  | Detect collision on right |

---

## Servos

| Servo | Pin | Purpose |
|-------|-----|---------|
| Servo1 | 18 | Arm/attachment control |
| Servo2 | 18 ⚠️ | Shares same pin as Servo1 (unusual setup) |

---

## MPU6050 (I²C)

| Signal | Pin | Purpose |
|--------|-----|---------|
| SDA    | 21  | I²C data line |
| SCL    | 22  | I²C clock line |

---

## Bluetooth

| Signal   | Pin | Purpose |
|----------|-----|---------|
| SerialBT | —   | Uses ESP32’s built‑in Bluetooth |

---

Here’s the **README‑ready section with a “Known Issues & Workarounds” subsection** added, so you can paste it directly into your GitHub repo:

---

# ⚙️ Interim Edition Robot Pinouts

## Motor Drivers (BTS7960)

| Function | Pin | Purpose |
|----------|-----|---------|
| LPWM1    | 16  | Left motor 1 forward PWM |
| RPWM1    | 17  | Left motor 1 reverse PWM |
| LPWM2    | 19  | Right motor 2 forward PWM |
| RPWM2    | 13  | Right motor 2 reverse PWM |
| EN1_L    | 23  | Enable left motor driver channel |
| EN1_R    | 5   | Enable right motor driver channel |
| EN2_L    | 2 ⚠️ | Enable left motor driver channel (ESP32 strapping pin) |
| EN2_R    | 0 ⚠️ | Enable right motor driver channel (ESP32 strapping pin) |

---

## Ultrasonic Sensors (HC‑SR04)

| Sensor Position | Trig Pin | Echo Pin | Purpose |
|-----------------|----------|----------|---------|
| Front           | 32       | 33       | Detect obstacles ahead |
| Rear            | 15       | 4        | Detect obstacles behind |
| Left            | 26       | 27       | Detect obstacles to the left |
| Right           | 14       | 12       | Detect obstacles to the right |

⚠️ Echo pins must be stepped down to 3.3 V using a voltage divider (e.g., 1 kΩ + 560 Ω).

---

## Bump Sensors

| Sensor Position | Pin | Purpose |
|-----------------|-----|---------|
| Left bump       | 34  | Detect collision on left |
| Right bump      | 35  | Detect collision on right |

---

## Servos

| Servo | Pin | Purpose |
|-------|-----|---------|
| Servo1 | 18 | Arm/attachment control |
| Servo2 | 18 ⚠️ | Shares same pin as Servo1 (unusual setup) |

---

## MPU6050 (I²C)

| Signal | Pin | Purpose |
|--------|-----|---------|
| SDA    | 21  | I²C data line |
| SCL    | 22  | I²C clock line |

---

## Bluetooth

| Signal   | Pin | Purpose |
|----------|-----|---------|
| SerialBT | —   | Uses ESP32’s built‑in Bluetooth |

---

## Known Issues & Workarounds

- **Startup motor creep**  
  - Cause: EN2 pins are mapped to ESP32 strapping pins (GPIO0 and GPIO2), which output unintended states during boot.  
  - Workaround: EN pins are held LOW during initialization, and motor drivers are only enabled after sensors and servos are configured, with an added delay to allow GPIOs to stabilize. This prevents the BTS7960 from reacting to boot‑time glitches.

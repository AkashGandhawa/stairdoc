Perfect — here’s a **README‑ready section** in Markdown format that you can paste directly into your GitHub repo to document the Interim Edition robot pinouts:

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

This section is formatted for clarity and consistency with your ultrasonic sensor table. It highlights ⚠️ notes where strapping pins or unusual configurations are used, so collaborators are aware of potential quirks.  

Would you like me to also add a **“Known Issues & Workarounds” subsection** under this pinout in the README, summarizing the startup motor creep fix and the strapping pin caveat? That way the documentation captures both the mapping and the lessons learned.

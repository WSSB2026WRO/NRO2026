# R2M Robot Connection Schematic (Textual Representation)

## Microcontroller
- Raspberry Pi 4 (main controller)
  - Runs Python control scripts
  - Controls motors, servo, and reads sensors

---

## Motor Driver (L298N)
- IN1 -> GPIO17 (DigitalOutputDevice)
- IN2 -> GPIO27 (DigitalOutputDevice)
- ENA -> GPIO13 (PWMOutputDevice)
- Motor A connects to left wheel
- Motor B connects to right wheel
- VCC of L298N -> 7.4V battery (or suitable supply)
- GND -> Common ground with Raspberry Pi

---

## Servo (Steering)
- Servo channel 0 on PCA9685
- PWM connection: SDA/SCL -> I2C on Raspberry Pi
- Power -> 5V supply from PCA9685 or Pi
- Servo angles:
  - ANGLE_CENTER = 90° (neutral)
  - MAX_ANGLE = ±30° for turning

---

## PCA9685 PWM Controller
- SDA -> Raspberry Pi SDA (GPIO2)
- SCL -> Raspberry Pi SCL (GPIO3)
- VCC -> 3.3V or 5V (depending on servo specs)
- GND -> Common ground

---

## Distance Sensors
### ultrasonic Lidar Sensors
- I2C Bus
  - SDA -> Raspberry Pi SDA (GPIO2)
  - SCL -> Raspberry Pi SCL (GPIO3)
- Addresses:
  - Sensor Left: 0x31
  - Sensor Right: 0x30
  - Sensor Rear: 0x33 (optional)
- Power -> 3.3V
- GND -> Common ground

### HC-SR04 Ultrasonic Sensor
- Trigger -> GPIO16
- Echo -> GPIO26
- VCC -> 5V
- GND -> Common ground

---

## HuskyLens Camera (Obstacle Detection)
- I2C interface
  - SDA -> Raspberry Pi SDA (GPIO2)
  - SCL -> Raspberry Pi SCL (GPIO3)
- Address: 0x32
- VCC -> 5V
- GND -> Common ground

---

## Start/Stop Button
- GPIO22 -> Button input
- Pull-up: False
- GND -> Common ground

---

## Power Supply
- Motors: 7.4V Li-ion battery 
- Raspberry Pi & sensors: 5V regulated supply (via Pi or separate voltage regulator)
- PCA9685: 5V (can share from Pi)
- Ensure common GND for all components

---

## Summary
- Raspberry Pi controls the entire system via GPIO and I2C
- L298N drives the DC motors
- PCA9685 controls the steering servo
- VL53L0X and HC-SR04 provide distance sensing for navigation
- HuskyLens detects colored obstacles for avoidance
- Start/Stop button controls the running state of the program

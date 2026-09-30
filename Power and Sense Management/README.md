# Power and Sense Management

## Power Source

The R2M robot is powered by a **lithium-ion rechargeable battery pack**, providing a stable voltage to the Raspberry Pi, motor driver, and sensors. The battery selection ensures enough capacity to complete the Open and Obstacle challenges without interruption.  

- **Battery Type:** Li-ion or LiPo (7.4V or 11.1V nominal, depending on the motor driver requirements)  
- **Voltage Regulation:** A voltage regulator or UBEC is used to supply the Raspberry Pi with 5V stable power. The motor driver receives the battery voltage directly for maximum efficiency.  
- **Power Management:** Current draw is monitored during testing to ensure the battery can sustain all three laps plus parking maneuvers. The single DC motor and servo combination minimizes overall power consumption while allowing precise control.  

### Power Distribution
- The battery powers:  
  - **Raspberry Pi** via a 5V regulator  
  - **Motor Driver** to supply the DC motor  
  - **Servo Motors** via the PCA9685 PWM driver  
  - **Sensors** (Ultrasonic and HC-SR04) via 5V pins from the Pi or regulated supply  
- **Current Consumption:**  
  - DC Motor: ~1.2–1.5A during acceleration  
  - Servo: ~0.5A during maximum steering correction  
  - ultrasonic sensors: ~20mA each  
  - HC-SR04: ~15mA during measurement pulses  
  - Raspberry Pi: ~0.7–1A depending on load  
- Total current remains within the capacity of a 2–3Ah battery pack for sustained operation during the competition.

---

## Sensor Suite

The vehicle is equipped with a combination of **distance sensors and camera vision** to navigate the challenges autonomously. Each sensor was selected based on its accuracy, update rate, and suitability for the specific task.  

### Side and Rear Distance Sensors
- **ultrasonic ToF Sensors (Side & Rear):**  
  - Two sensors on the sides detect the vehicle’s lateral distance from walls or table borders for centering.  
  - One rear sensor aids in parking maneuvers and ensures the robot aligns correctly in the designated magenta parking lot after completing the laps.  
  - **Reason for Selection:** Compact, precise, fast response (~30ms), and capable of measuring up to 2 meters.  

### Front Ultrasonic Sensor
- **HC-SR04:**  
  - Mounted at the front to detect the distance to the start/finish line and to ensure the robot can stop accurately after completing three laps.  
  - **Reason for Selection:** Cost-effective, reliable for detecting a flat reference surface, simple interface with Raspberry Pi.  

### Vision Sensor
- **Pi camera Camera:**  
  - Detects colored obstacles (red and green) to implement the obstacle avoidance logic.  
  - Allows dynamic steering adjustments based on object detection.  
  - **Reason for Selection:** Provides color recognition without requiring heavy processing on the Raspberry Pi.  

### Sensor Integration
- The Pi reads all sensors and combines the data to determine:
  - Lateral correction for centering (side ultrasonic)  
  - Corner detection and lap counting (side ultrasonic)  
  - Obstacle avoidance (Pi camera)  
  - Parking detection (rear ultrasonic)  
  - Start/finish detection (HC-SR04)

---

## Wiring and Integration

The wiring is professionally arranged to minimize noise and ensure reliable communication between components:  
- **I2C Bus:** Shared by the ultrasonic sensors and Pi camera camera. Each device has a unique address. Pull-up resistors are included as required by the I2C standard.  
- **PWM Lines:** Servo motors are controlled via PCA9685 to offload PWM generation from the Raspberry Pi.  
- **Digital Lines:** HC-SR04 echo and trigger lines are connected to GPIO pins.  
- **Motor Driver:** Controlled via two digital GPIO lines for direction and one PWM line for speed.  

### Bill of Materials (BOM)
- Raspberry Pi 4 Model B (or compatible)  
- Single DC Motor (high torque)  
- L298N Motor Driver  
- Standard Servo Motor (steering)  
- PCA9685 16-channel PWM driver  
- 3 × ultrasonic Time-of-Flight distance sensors (2 sides + 1 rear)  
- 1 × HC-SR04 ultrasonic sensor  
- 1 × Pi camera camera  
- Lithium-ion battery pack (7.4V–11.1V)  
- Voltage regulator / UBEC  
- Wires, connectors, and mounting hardware  
- 3D-printed mounts for sensors and servo  

### Notes
- Wiring diagrams and schematics are provided as separate JPEG/PNG/PDF files in this directory.  
- All power lines are clearly marked, and sensor lines are grouped to avoid interference.  
- The power distribution ensures that voltage drops do not affect sensor readings or the Raspberry Pi stability.  

This carefully designed power and sensor management system allows the R2M robot to navigate challenges autonomously, avoid obstacles, complete laps, and park precisely while maintaining optimal energy efficiency.

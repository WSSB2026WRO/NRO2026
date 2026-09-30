# R2M Robot – WRO 2026

## Project Overview

R2M is an autonomous robot designed by the team members Rashid, Mohammed, and Mansoor for the WRO 2026 Open and Obstacle Challenges. It is capable of navigating a predefined track, avoiding obstacles, performing precise cornering, completing laps, and parking autonomously in a designated magenta lot.

The robot combines multiple sensors, actuators, and AI-powered vision to make decisions in real-time, ensuring both accuracy and safety during the competition.


---

## Vehicle Components

The R2M robot integrates electromechanical components, sensors, and actuators for autonomous navigation, obstacle detection, and precise parking. Each component was selected to ensure stability, responsiveness, and accurate control.

### Drive System
- **DC Motors:** Provide propulsion; controlled via L298N motor driver with PWM for precise speed adjustment.
- **Motor Driver (L298N):** Controls forward, backward, and braking commands to the motors.

### Steering System
- **Servo Motor:** Controls steering angle through PCA9685 PWM driver.
- **Steering Range:** ±30° from neutral (90°) allows sharp cornering and smooth lateral corrections.
- **Purpose:** Accurate navigation through corners and assists in obstacle avoidance.

### Distance Sensors
- **VL53L0X Time-of-Flight Sensors (Left and Right):** Measure side distances for centering and corner detection.
- **Optional Rear VL53L0X Sensor:** Used for parking or additional obstacle detection.
- **HC-SR04 Ultrasonic Sensor (Front):** Detects start/finish line and distance for final parking.

### Computer Vision System
- **HuskyLens AI Camera:** Detects colored obstacles.
  - **Red Objects:** Robot passes to the right.
  - **Green Objects:** Robot passes to the left.
- Overrides normal navigation when obstacles are detected to ensure safe avoidance.

### Controller
- **Raspberry Pi:** Main processor that reads sensors, executes control algorithms, and handles HuskyLens communication.

### User Interface
- **Start/Stop Button:** Provides manual control to safely start or stop the robot.

### Power Supply
- **Battery Pack:** Powers motors, servo, sensors, and Raspberry Pi.
- Voltage regulators ensure stable operation of sensitive electronics.

### Chassis
- Lightweight yet sturdy to maintain stability during fast movement and sharp turns.
- Provides mounting points for all sensors, motors, and the Raspberry Pi.

**Integration:** All components work together to ensure precise navigation, obstacle avoidance, smooth cornering, and final parking in the magenta lot.

---

## Usage

1. Connect the Raspberry Pi to the motor driver, sensors, and HuskyLens camera.
2. Ensure power supply is connected.
3. Press the **Start button** to begin the challenge.
4. The robot will navigate autonomously, detect and avoid obstacles, complete three laps, and park in the magenta lot.
5. Press the **Stop button** at any time to safely stop the robot.



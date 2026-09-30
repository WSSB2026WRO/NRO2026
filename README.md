# R2M Autonomous Robot – WRO 2026

## Team Members
- Faysal
- Mohammed
- Omar

## Robot Name
**R2M** – derived from the first letters of the team members’ names.

## Project Overview
R2M is an autonomous mobile robot designed to participate in the World Robot Olympiad 2026. The robot is programmed to perform two main challenges: the **Open Challenge** and the **Obstacle Challenge**. It combines sensor fusion, precise motor control, and intelligent decision-making to navigate, detect obstacles, and complete laps with accuracy.

The robot’s primary objectives include:
1. Completing multiple laps along a pre-defined course autonomously.
2. Detecting and avoiding obstacles using computer vision and distance sensors.
3. Performing precise cornering and lateral centering.
4. Automatically parking in a designated magenta parking lot after completing the challenge.

---

## Challenges

### 1. Open Challenge
The Open Challenge focuses on autonomous navigation without obstacles. The robot completes three laps along a pre-defined route while maintaining lateral alignment between left and right boundaries. Key functions include:
- **Lateral centering** using dual VL53L0X distance sensors.
- **Corner detection** using thresholds for lateral distances.
- **Speed regulation** for consistent forward movement.
- **Lap counting** to ensure the robot completes the exact number of laps.
- **Start/stop control** via a physical button for safety.

### 2. Obstacle Challenge
The Obstacle Challenge extends the Open Challenge with dynamic obstacles. The robot must:
- Detect colored obstacles using the **HuskyLens AI camera** (Red and Green).
- Pass Red obstacles on the right and Green obstacles on the left.
- Adjust steering dynamically to avoid collisions.
- Detect corners and navigate around them.
- Perform **automatic parking** in the designated magenta parking lot after completing three laps.
- Adjust speed dynamically: moves faster when the distance ahead is large, slows down as it approaches the parking area.

---

## Robot Hardware

The robot integrates the following components:

### 1. Drive System
- **DC Motors** with L298N motor driver.
- **PWM control** for speed modulation.

### 2. Steering System
- **Servo motor** controlled via PCA9685 PWM driver.
- Steering range allows correction up to ±30° from the center (90°).

### 3. Distance Sensors
- **Two VL53L0X Time-of-Flight sensors** (left and right) for lateral distance measurement.
- **HC-SR04 ultrasonic sensor** at the front for start/finish detection and parking guidance.

### 4. Computer Vision
- **HuskyLens AI camera** for colored obstacle detection (Red and Green).

### 5. Control & Input
- **Raspberry Pi** for main computation and sensor integration.
- **Start/Stop Button** for manual control.

---

## Software Structure

The robot’s software is implemented in **Python 3**. The main modules include:

### 1. `sensors.py`
- Handles reading VL53L0X and HC-SR04 distance sensors.
- Implements averaging for smoother readings.
- Provides functions to return the current distance of each sensor.

### 2. `motors.py`
- Controls the DC motors’ forward motion via PWM signals.
- Implements functions for stopping, forward motion, and speed modulation.

### 3. `steering.py`
- Controls the servo motor for steering.
- Implements lateral centering corrections based on proportional gain (KP).
- Handles cornering adjustments during navigation and obstacle avoidance.

### 4. `obstacle_detection.py`
- Integrates HuskyLens camera to detect Red and Green obstacles.
- Determines the appropriate avoidance direction:
  - Red → pass right
  - Green → pass left
- Overrides normal navigation if obstacles are detected.

### 5. `main.py`
- Combines all modules into the main control loop.
- Implements lap and corner counting logic:
  - **Four corners = one lap**
  - **Eight corners = two laps**
  - **Twelve corners = three laps**
- Manages speed:
  - Doubles forward speed if front distance > 120 cm.
  - Reduces speed for final approach to parking.
- Performs automatic parking after completing three laps.
- Handles start/stop button functionality.

---

## Key Software Logic

1. **Initialization**
   - Set servo to neutral position (90°).
   - Read initial front distance (`start_distance_initial`) using HC-SR04.
   - Reset lap and corner counters.

2. **Main Control Loop**
   - Read lateral distances (left and right sensors).
   - Read front distance (HC-SR04).
   - Detect corners and increment `corner_count`.
   - Determine turn direction based on first corner detected.
   - Perform steering adjustments:
     - Corner detected → turn with `TURN_DURATION`.
     - No corner → lateral centering with proportional correction.
   - Check for HuskyLens obstacles and override steering if needed.
   - Adjust speed dynamically based on front distance.
   - After 12 corners, slow down and continue until front distance ≤ `start_distance_initial` → stop and park.

3. **Obstacle Avoidance**
   - Detect colored obstacles in real-time.
   - Redirect path accordingly while maintaining lateral stability.
   - Resume normal navigation once obstacles are cleared.

4. **Parking**
   - After completing three laps (12 corners), approach the magenta parking lot.
   - Use front distance to determine exact stopping point.
   - Reduce speed to `FINAL_SPEED` for smooth parking.

---

## Build, Compile & Upload Instructions

1. **Install Python 3** and required libraries:
```bash
sudo apt-get update
sudo apt-get install python3-pip python3-dev
pip3 install gpiozero adafruit-circuitpython-servokit adafruit-circuitpython-vl53l0x huskylens

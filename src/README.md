# R2M Robot – WRO 2025

This repository contains all code and artifacts developed by the R2M team (Rashid, Mohammed, Mansoor) for the WRO 2025 Open and Obstacle Challenges. It includes control software, sensor integration, vision modules, and supporting files for building, running, and reproducing the vehicle.

---

## Control Software Description

The `src` folder contains all Python code responsible for controlling the R2M robot:

- **open_challenge.py** – Autonomous navigation around the track without obstacles.
- **obstacle_challenge.py** – Autonomous navigation with HuskyLens-based obstacle detection and avoidance.
- **motor_control.py** – Functions to control DC motors (forward, backward, stop) using PWM.
- **servo_control.py** – Functions to manage the steering servo for cornering, lateral correction, and obstacle avoidance.
- **sensor_readings.py** – Functions to read ultrasonic and HC-SR04 sensors with smoothing and calibration.
- **huskylens_control.py** – Handles HuskyLens AI camera detection of colored blocks for obstacle navigation.
- **utils.py** – Helper functions for logging, speed adjustment, lap/corner counting, and miscellaneous utilities.

---

## Dependencies

The robot control software requires the following Python packages:

- `gpiozero` – Motor and sensor control
- `adafruit_servokit` – PWM servo control via PCA9685
- `adafruit_ultrasonic` – ultrasonic distance sensor
- `HUSKYLENS.huskylib` – HuskyLens AI camera
- `board` and `busio` – I2C communication
- Python 3.9+ recommended

A `requirements.txt` is included for easy installation of dependencies.



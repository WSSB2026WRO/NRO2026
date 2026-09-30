# Mobility Management

## Vehicle Movement Control

The R2M robot is designed for autonomous navigation and obstacle handling in the Open and Obstacle Challenges. The vehicle’s movement is managed through a combination of a single DC motor for driving and a steering servo, all controlled by a Raspberry Pi running Python scripts.

### Motor Selection and Implementation
- **DC Motor:** One high-torque brushed DC motor is selected to drive the main wheel assembly. This motor provides sufficient torque for smooth acceleration, precise movement, and stable navigation on the competition surface.
- **Motor Driver:** An L298N motor driver module interfaces the DC motor with the Raspberry Pi. PWM signals from the Pi control speed, while digital signals control direction.
- **Speed Control:** Forward speed and turning speed are adjustable via software parameters. The robot dynamically modifies speed based on sensor readings. For instance, it can increase speed when no obstacles are detected in front and reduce speed when approaching corners or the finish line.
- **Torque Considerations:** The motor selection ensures enough torque to carry the full weight of the robot (chassis, sensors, camera, and battery) while maintaining precise control during turning and acceleration.

### Steering Mechanism
- **Servo Motor:** A standard servo is mounted for steering control. It is connected via a PCA9685 PWM driver, which allows fine-grained control of the steering angle.
- **Steering Angle Management:** The servo is set to a neutral position for straight movement and adjusted up to ±30° for cornering. The control system uses proportional error from side distance sensors to maintain lateral alignment during straight paths.

### Vehicle Chassis
- **Chassis Design:** The chassis is designed from lightweight materials to optimize speed, torque, and stability. All components, including the motor, steering servo, sensors, and Raspberry Pi, are mounted securely to prevent wobbling during autonomous movement.
- **Mounting of Components:** 
  - The DC motor is attached to the chassis with a bracket that ensures proper alignment of the drive wheel.
  - The steering servo is centrally mounted to control the front wheel or steering linkage.
  - **HC-SR04 sensor:** Front ultrasonic sensor is mounted for start/finish detection and front obstacle distance.
  - The Pi camera camera is mounted to allow clear detection of colored obstacles. Two side sensors are mounted for lateral distance measurement, and one rear sensor is mounted for parking detection and alignment.
- **3D Printed Parts:** Custom mounts and holders for sensors, servo, and motor are designed in CAD software and 3D printed for precise fitting. CAD files are included in this directory for replication.

### Engineering Principles
- **Speed and Power:** The vehicle balances maximum speed with safe navigation. Acceleration and deceleration are controlled to prevent slippage or overshoot in corners.
- **Torque:** The single motor is sufficient to overcome friction and carry the full weight of the vehicle while maintaining precise control.
- **Energy Efficiency:** Battery selection and motor efficiency allow uninterrupted operation throughout all three laps of the challenge.
- **Control Algorithms:** The robot uses proportional control for lateral alignment, corner detection for precise turns, obstacle avoidance through the Pi camera camera, and rear distance sensing for parking maneuvers.

### Assembly Instructions
1. Mount the DC motor to the chassis using the provided bracket.
2. Attach the steering servo and connect it to the steering linkage.
3. Secure the Raspberry Pi and motor driver board onto the chassis with spacers or 3D-printed holders.
4. Mount the Ultrasound sensors: two at the sides for lateral detection, one at the rear for parking alignment.
5. Attach the HC-SR04 sensor at the front.
6. Attach the Pi camera camera at a height allowing unobstructed detection.
7. Connect all wiring according to the provided schematic, ensuring a common ground.
8. Place the battery securely and connect it to the motor driver and Raspberry Pi power inputs.
9. Test the motor, servo, and sensor readings manually before running the autonomous code.

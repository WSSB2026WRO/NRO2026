# Obstacle Management Discussion

## Overview

The **Obstacle Challenge** is designed to test the vehicle’s ability to autonomously navigate a complex track while detecting and responding to obstacles of different colors.  
Our robot **R2M** (named after team members Rashid, Mohammed, and Mansoor) is programmed to complete **three full laps (12 corners)** around the course while adapting its behavior to handle **red and green obstacles**.  
After completing the laps, it automatically **enters and parks in the magenta parking area**.

---

## Strategy

The control strategy relies on integrating **color recognition** and **distance sensing** to dynamically adjust the robot’s trajectory and speed. The steps below outline the decision-making process:

1. **Normal Navigation:**  
   - The robot follows a pre-defined path to complete three laps.  
   - During normal operation, the forward speed is constant.

2. **Obstacle Detection and Color Recognition:**  
   - When an obstacle is detected by the **Pi camera camera**, the system identifies its color.  
   - If the obstacle is **red**, the robot bypasses it from the **right**.  
   - If the obstacle is **green**, the robot bypasses it from the **left**.

3. **Dynamic Speed Adjustment:**  
   - If the distance measured by the **HC-SR04 ultrasonic sensor** is greater than **120 cm**, the robot doubles its forward speed.  
   - Otherwise, it maintains a normal cruising speed.

4. **End of Lap and Parking:**  
   - After completing three laps (12 corners), the robot reduces its speed.  
   - The robot continues moving until the HC-SR04 sensor detects an object (the parking barrier) at a distance **equal to or less than the starting distance**.  
   - The robot then stops automatically between the magenta parking marks.

---

## Flow Diagram (Text Description)
┌────────────────────────────┐
│ Start / Button Pressed │
└──────────────┬─────────────┘
│
▼
Initialize sensors
│
▼
Move Forward (normal)
│
┌────────────┼────────────┐
│ │
▼ ▼
Pi camera detects No obstacle
red or green? detected
│ │
▼ ▼
Execute avoidance Continue centering
(Red → Right, using ulktrqsound sensors
Green → Left) (PID correction)
│ │
▼ ▼
Corner detected? ────────┐
│ │
▼ │
Turn in chosen direction │
│ │
Update corner count │
│ │
12 corners reached? ────────────┘
│
▼
Slow down and approach start
│
HC-SR04 ≤ initial distance?
│
▼
Stop and park


---

## Pseudo Code

```python
Initialize motors, camera (Pi camera), ultrasonic sensor (HC-SR04)
laps = 0
corners = 0
start_distance = read_ultrasonic()

while laps < 3:
    move_forward(FORWARD_SPEED)
    
    distance = read_ultrasonic()
    if distance > 120:
        set_speed(FORWARD_SPEED * 2)
    else:
        set_speed(FORWARD_SPEED)
    
    color = get_color_from_Pi camera()
    
    if color == "red":
        turn_right_around_obstacle()
    elif color == "green":
        turn_left_around_obstacle()
    
    if corner_detected():
        corners += 1
        if corners == 12:
            laps += 1
            corners = 0

# After 3 laps
reduce_speed()
while read_ultrasonic() > start_distance:
    move_forward(SLOW_SPEED)

stop_motors()






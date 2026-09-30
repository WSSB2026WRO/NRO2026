import time
import serial
from gpiozero import DistanceSensor, Button
from picamera2 import Picamera2
import cv2
import numpy as np


# ==================================================
# WRO 2026 - OPEN CHALLENGE - E3 MPU85
#
# V3-1 STABLE NAVIGATION
# +
# E1 NAVIGATION + ROBUST CAMERA WALL-EDGE DETECTION
#
# IMPORTANT:
# CAMERA = LINE CONTROL WHEN RELIABLE
# ULTRASONIC = CORNERS + FALLBACK + FINISH
# ==================================================


# ==================================================
# CAMERA SETTINGS
# ==================================================

CAM_WIDTH = 1280
CAM_HEIGHT = 720

ROI_TOP = 180
ROI_BOTTOM = 430

BLACK_V_MAX = 80

CAM_CENTER_X = CAM_WIDTH // 2

# Rows used for corridor measurement
# relative to ROI
SCAN_ROWS = [120, 145, 170, 195, 220]

# E2 robust wall-edge scan rows (absolute frame Y coordinates)
# Calibrated from the current physical camera position.
E2_SCAN_YS = [190, 200, 210, 220, 230]
E2_MIN_WIDTH = 250
E2_MAX_WIDTH = 1250
E2_CENTER_MAD_MULTIPLIER = 2.5
E2_CENTER_MIN_TOLERANCE_PX = 45

# Minimum valid corridor width
MIN_CORRIDOR_WIDTH = 180

# Ignore tiny floor regions
MIN_COMPONENT_AREA = 5000

# E1 CAMERA + ULTRASONIC FUSION + MPU OBSERVER
CAM_DEADZONE = 35
CAM_MAX_ERROR = 220
CAM_EDGE_MARGIN = 15
CAM_MIN_WIDTH_CONTROL = 300
CAM_VALID_CONFIRMATIONS = 3


# ==================================================
# CAMERA INITIALIZATION
# ==================================================

picam2 = Picamera2()

camera_config = picam2.create_preview_configuration(
    main={
        "size": (CAM_WIDTH, CAM_HEIGHT),
        "format": "RGB888"
    },
    controls={
        "FrameRate": 60
    }
)

picam2.configure(camera_config)
picam2.start()

time.sleep(1)

kernel5 = np.ones((5, 5), np.uint8)
kernel9 = np.ones((9, 9), np.uint8)

print("CAMERA READY")


# ==================================================
# FIND RUN ON ONE ROW
# ==================================================

def find_floor_run(row_data, image_center):

    """
    Finds the white floor segment which contains the
    camera center.

    If center itself is not floor, choose the nearest
    sufficiently large white segment.
    """

    binary = row_data > 0

    runs = []

    start = None

    for x, value in enumerate(binary):

        if value and start is None:
            start = x

        elif not value and start is not None:

            end = x - 1

            if end - start >= 30:
                runs.append((start, end))

            start = None

    if start is not None:

        end = len(binary) - 1

        if end - start >= 30:
            runs.append((start, end))

    if not runs:
        return None

    # First choice:
    # floor segment containing image center
    for left, right in runs:

        if left <= image_center <= right:
            return left, right

    # Second choice:
    # closest segment to image center
    best_run = None
    best_distance = 999999

    for left, right in runs:

        center = (left + right) // 2

        distance = abs(center - image_center)

        if distance < best_distance:

            best_distance = distance
            best_run = (left, right)

    return best_run


# ==================================================
# CAMERA ANALYSIS
# ==================================================

def analyze_camera():

    frame = picam2.capture_array()

    # --------------------------------------------------
    # E2 ROBUST BLACK WALL-EDGE DETECTION
    #
    # IMPORTANT:
    # - Keep CAM_CENTER_X = 640.
    # - Do not accept the old false L=0 / R=1279 corridor.
    # - Measure the nearest black wall candidate on each
    #   side of the optical center.
    # - Reject abnormal scan rows before taking the median.
    # --------------------------------------------------

    hsv = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2HSV
    )

    lower_black = np.array(
        [0, 0, 0],
        dtype=np.uint8
    )

    upper_black = np.array(
        [179, 255, BLACK_V_MAX],
        dtype=np.uint8
    )

    black_mask = cv2.inRange(
        hsv,
        lower_black,
        upper_black
    )

    black_mask = cv2.morphologyEx(
        black_mask,
        cv2.MORPH_OPEN,
        kernel5
    )

    black_mask = cv2.morphologyEx(
        black_mask,
        cv2.MORPH_CLOSE,
        kernel9
    )

    raw_rows = []

    for y in E2_SCAN_YS:

        if y < 0 or y >= CAM_HEIGHT:
            continue

        row = black_mask[y]

        left_candidates = np.flatnonzero(
            row[:CAM_CENTER_X] > 0
        )

        right_candidates = np.flatnonzero(
            row[CAM_CENTER_X:] > 0
        )

        if (
            left_candidates.size == 0
            or right_candidates.size == 0
        ):
            continue

        left_x = int(
            left_candidates[-1]
        )

        right_x = int(
            CAM_CENTER_X
            + right_candidates[0]
        )

        corridor_width = (
            right_x - left_x
        )

        if (
            corridor_width < E2_MIN_WIDTH
            or corridor_width > E2_MAX_WIDTH
        ):
            continue

        # Explicitly reject the old false
        # full-frame corridor result.
        if (
            left_x <= CAM_EDGE_MARGIN
            and right_x >= (
                CAM_WIDTH - 1 - CAM_EDGE_MARGIN
            )
        ):
            continue

        corridor_center = (
            left_x + right_x
        ) / 2.0

        raw_rows.append(
            {
                "y": y,
                "left": left_x,
                "right": right_x,
                "center": corridor_center,
                "width": corridor_width
            }
        )

    # --------------------------------------------------
    # ROBUST ROW REJECTION
    # --------------------------------------------------

    accepted_rows = []

    if raw_rows:

        centers = np.array(
            [
                row["center"]
                for row in raw_rows
            ],
            dtype=float
        )

        median_center = float(
            np.median(centers)
        )

        deviations = np.abs(
            centers - median_center
        )

        mad = float(
            np.median(deviations)
        )

        tolerance = max(
            E2_CENTER_MIN_TOLERANCE_PX,
            E2_CENTER_MAD_MULTIPLIER
            * mad
        )

        for row in raw_rows:

            if (
                abs(
                    row["center"]
                    - median_center
                )
                <= tolerance
            ):
                accepted_rows.append(
                    row
                )

    # --------------------------------------------------
    # FINAL CAMERA RESULT
    # --------------------------------------------------

    cam_left = None
    cam_right = None
    corridor_center = None
    cam_error = None

    # Require at least 3 mutually consistent rows.
    if len(accepted_rows) >= 3:

        cam_left = int(
            np.median(
                [
                    row["left"]
                    for row in accepted_rows
                ]
            )
        )

        cam_right = int(
            np.median(
                [
                    row["right"]
                    for row in accepted_rows
                ]
            )
        )

        corridor_center = int(
            round(
                np.median(
                    [
                        row["center"]
                        for row in accepted_rows
                    ]
                )
            )
        )

        cam_error = (
            corridor_center
            - CAM_CENTER_X
        )

    # --------------------------------------------------
    # DISPLAY
    # --------------------------------------------------

    cv2.line(
        frame,
        (
            CAM_CENTER_X,
            min(E2_SCAN_YS) - 20
        ),
        (
            CAM_CENTER_X,
            max(E2_SCAN_YS) + 20
        ),
        (255, 255, 255),
        1
    )

    accepted_y = {
        row["y"]
        for row in accepted_rows
    }

    for row in raw_rows:

        y = row["y"]
        left_x = row["left"]
        right_x = row["right"]
        center_x = int(
            round(row["center"])
        )

        if y in accepted_y:
            color = (255, 255, 255)
        else:
            color = (120, 120, 120)

        cv2.line(
            frame,
            (left_x, y),
            (right_x, y),
            color,
            2
        )

        cv2.circle(
            frame,
            (left_x, y),
            5,
            color,
            -1
        )

        cv2.circle(
            frame,
            (right_x, y),
            5,
            color,
            -1
        )

        cv2.circle(
            frame,
            (center_x, y),
            5,
            color,
            -1
        )

    if cam_error is not None:

        cv2.line(
            frame,
            (
                corridor_center,
                min(E2_SCAN_YS) - 20
            ),
            (
                corridor_center,
                max(E2_SCAN_YS) + 20
            ),
            (255, 255, 255),
            3
        )

        cv2.putText(
            frame,
            f"CAM LEFT: {cam_left}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f"CAM RIGHT: {cam_right}",
            (20, 65),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f"CAM CENTER: {corridor_center}",
            (20, 95),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f"CAM ERROR: {cam_error:+d}",
            (20, 125),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f"ROWS: {len(accepted_rows)}/{len(E2_SCAN_YS)}",
            (20, 155),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

    else:

        cv2.putText(
            frame,
            "CAM: NO RELIABLE CORRIDOR",
            (20, 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2
        )

    cv2.imshow(
        "WRO E2 CAMERA",
        frame
    )

    cv2.imshow(
        "BLACK WALL MASK",
        black_mask
    )

    cv2.waitKey(1)

    return (
        cam_left,
        cam_right,
        corridor_center,
        cam_error
    )


# ==================================================
# CAMERA TEXT
# ==================================================

def camera_text(
    cam_left,
    cam_right,
    cam_center,
    cam_error
):

    if cam_error is None:

        return "CAM: NO CORRIDOR"

    return (
        f"CAM L:{cam_left:4d} | "
        f"R:{cam_right:4d} | "
        f"C:{cam_center:4d} | "
        f"ERR:{cam_error:+4d}"
    )


# ==================================================
# V3-4 CAMERA RELIABILITY / FUSION
# ==================================================

def camera_reliable(cam_left, cam_right, cam_error):
    if cam_left is None or cam_right is None or cam_error is None:
        return False
    width = cam_right - cam_left
    if width < CAM_MIN_WIDTH_CONTROL:
        return False
    if cam_left <= CAM_EDGE_MARGIN:
        return False
    if cam_right >= (CAM_WIDTH - 1 - CAM_EDGE_MARGIN):
        return False
    if abs(cam_error) > CAM_MAX_ERROR:
        return False
    return True


def ultrasonic_steering(left, right):
    difference = left - right
    if difference > SIDE_TOLERANCE:
        return "A", "US SOFT LEFT", difference
    elif difference < -SIDE_TOLERANCE:
        return "D", "US SOFT RIGHT", difference
    return "C", "US CENTER", difference


def camera_steering(cam_error):
    # Positive error = visible corridor center is to the RIGHT.
    # Therefore steer softly RIGHT (D).
    if cam_error > CAM_DEADZONE:
        return "D", "CAM SOFT RIGHT"
    elif cam_error < -CAM_DEADZONE:
        return "A", "CAM SOFT LEFT"
    return "C", "CAM CENTER"


# ==================================================
# ULTRASONIC SENSORS
# ==================================================

left_sensor = DistanceSensor(
    echo=24,
    trigger=27,
    max_distance=4.0,
    queue_len=1
)

front_sensor = DistanceSensor(
    echo=16,
    trigger=22,
    max_distance=4.0,
    queue_len=1
)

right_sensor = DistanceSensor(
    echo=23,
    trigger=17,
    max_distance=4.0,
    queue_len=1
)

# Raspberry Pi physical START button
# BCM GPIO5 (physical pin 29) -> button -> GND (physical pin 30)
start_button = Button(
    5,
    pull_up=True,
    bounce_time=0.05
)


# ==================================================
# ESP32
# ==================================================

esp32 = serial.Serial()

esp32.port = "/dev/ttyUSB0"
esp32.baudrate = 115200
esp32.timeout = 1

esp32.dtr = False
esp32.rts = False

esp32.open()

time.sleep(1)

esp32.reset_input_buffer()


# ==================================================
# NAVIGATION SETTINGS
# ==================================================

FRONT_TRIGGER = 110.0
FRONT_TRIGGER_CONFIRMATIONS = 3
FRONT_CLEAR = 130.0

SIDE_TOLERANCE = 5.0

MIN_TURN_TIME = 1.6
MAX_TURN_TIME = 4.0

CLEAR_CONFIRMATIONS = 4
MPU_TURN_TARGET = 79.0


# ==================================================
# FINISH SETTINGS
# ==================================================

TOTAL_TURNS = 12

FINISH_TOLERANCE = 5.0
FINISH_CONFIRMATIONS = 2


# ==================================================
# VARIABLES
# ==================================================

last_command = None

turn_armed = True
turn_count = 0

start_left = None
start_front = None
start_right = None

camera_valid_count = 0


# ==================================================
# READ SENSOR
# ==================================================

def read_distance(sensor):

    try:

        distance = (
            sensor.distance * 100
        )

        if (
            distance < 2
            or distance >= 400
        ):
            return None

        return distance

    except Exception:

        return None


# ==================================================
# READ ALL
# ==================================================

def read_all():

    left = read_distance(
        left_sensor
    )

    time.sleep(0.04)

    front = read_distance(
        front_sensor
    )

    time.sleep(0.04)

    right = read_distance(
        right_sensor
    )

    time.sleep(0.04)

    return left, front, right


# ==================================================
# SEND ESP32 COMMAND
# ==================================================

def send(command):

    global last_command

    if command != last_command:

        esp32.write(
            command.encode()
        )

        esp32.flush()

        last_command = command


# ==================================================
# E1 - MPU OBSERVER
# ==================================================
# CAMERA = main corridor steering
# ULTRASONIC = fallback + corners + finish
# MPU = turn-angle observer only
#
# The MPU never decides the steering and never decides
# when a turn is complete. The real corridor geometry
# remains the authority, as in stable V3-4.
# ==================================================

latest_mpu_angle = None


def reset_mpu_observer():

    global latest_mpu_angle

    latest_mpu_angle = 0.0

    # Z resets yaw only. It does not change motor speed
    # or steering in wro2026_mpu_v3.
    esp32.write(b"Z")
    esp32.flush()


def read_mpu_observer():

    global latest_mpu_angle

    try:

        while esp32.in_waiting > 0:

            line = esp32.readline().decode(
                errors="ignore"
            ).strip()

            if not line:
                continue

            if "ANGLE:" in line:

                try:

                    angle_text = (
                        line.split("ANGLE:", 1)[1]
                        .split("deg", 1)[0]
                        .strip()
                    )

                    latest_mpu_angle = float(
                        angle_text
                    )

                except Exception:
                    pass

    except Exception:
        pass

    return latest_mpu_angle


# ==================================================
# MEASURE START POSITION
# ==================================================

def measure_start_position():

    print()
    print(
        "===================================="
    )
    print(
        " MEASURING START POSITION"
    )
    print(
        "===================================="
    )
    print()

    samples_left = []
    samples_front = []
    samples_right = []

    while len(samples_left) < 5:

        left, front, right = \
            read_all()

        if (
            left is not None
            and front is not None
            and right is not None
        ):

            samples_left.append(left)
            samples_front.append(front)
            samples_right.append(right)

            print(
                f"SAMPLE "
                f"{len(samples_left)} | "
                f"L:{left:6.1f} | "
                f"F:{front:6.1f} | "
                f"R:{right:6.1f}"
            )

        time.sleep(0.1)

    start_left = (
        sum(samples_left)
        / len(samples_left)
    )

    start_front = (
        sum(samples_front)
        / len(samples_front)
    )

    start_right = (
        sum(samples_right)
        / len(samples_right)
    )

    print()
    print("START POSITION SAVED")
    print()

    print(
        f"START LEFT  = "
        f"{start_left:.1f} cm"
    )

    print(
        f"START FRONT = "
        f"{start_front:.1f} cm"
    )

    print(
        f"START RIGHT = "
        f"{start_right:.1f} cm"
    )

    print()

    return (
        start_left,
        start_front,
        start_right
    )


# ==================================================
# START POSITION CHECK
# ==================================================

def start_position_detected(
    left,
    front,
    right
):

    if (
        left is None
        or front is None
        or right is None
    ):
        return False

    return (
        abs(left - start_left)
        <= FINISH_TOLERANCE

        and

        abs(front - start_front)
        <= FINISH_TOLERANCE

        and

        abs(right - start_right)
        <= FINISH_TOLERANCE
    )


# ==================================================
# AUTOMATIC LEFT TURN
# ==================================================

def automatic_left_turn():

    global turn_count
    global latest_mpu_angle

    print()
    print(">>> CORNER DETECTED")
    print(">>> AUTOMATIC LEFT TURN - MPU 85 DEG")

    # Reset MPU yaw reference at the beginning of THIS turn.
    # Z resets yaw only; steering remains controlled by the Pi.
    latest_mpu_angle = None

    try:
        esp32.reset_input_buffer()
    except Exception:
        pass

    esp32.write(b"Z")
    esp32.flush()

    time.sleep(0.08)

    try:
        esp32.reset_input_buffer()
    except Exception:
        pass

    latest_mpu_angle = None

    # Same LEFT steering command as E3.
    send("L")

    start_time = time.monotonic()

    while True:

        (
            cam_left,
            cam_right,
            cam_center,
            cam_error
        ) = analyze_camera()

        left, front, right = read_all()

        elapsed = time.monotonic() - start_time

        # Read MPU angle from ESP32.
        # LEFT rotation is positive yaw.
        try:

            while esp32.in_waiting > 0:

                line = esp32.readline().decode(
                    errors="ignore"
                ).strip()

                if not line:
                    continue

                if "ANGLE:" in line:

                    try:
                        angle_text = (
                            line.split("ANGLE:", 1)[1]
                            .split("deg", 1)[0]
                            .strip()
                        )

                        latest_mpu_angle = float(
                            angle_text
                        )

                    except Exception:
                        pass

        except Exception:
            pass

        if (
            left is not None
            and front is not None
            and right is not None
        ):

            angle_text = (
                f"{latest_mpu_angle:6.1f} deg"
                if latest_mpu_angle is not None
                else "   --- deg"
            )

            print(
                f"TURN | "
                f"L:{left:6.1f} | "
                f"F:{front:6.1f} | "
                f"R:{right:6.1f} | "
                f"MPU:{angle_text} | "
                f"TARGET:{MPU_TURN_TARGET:.0f} | "
                f"T:{elapsed:4.2f}s | "
                + camera_text(
                    cam_left,
                    cam_right,
                    cam_center,
                    cam_error
                )
            )

        # E3 MPU85:
        # As soon as a REAL MPU report reaches 85 degrees,
        # center steering immediately.
        if (
            latest_mpu_angle is not None
            and latest_mpu_angle >= MPU_TURN_TARGET
        ):

            send("C")

            turn_count += 1

            print()
            print(">>> MPU 85 DEG REACHED")
            print(
                f">>> MPU ANGLE = "
                f"{latest_mpu_angle:.1f} deg"
            )
            print(
                f">>> TURN FINISHED AFTER "
                f"{elapsed:.2f} s"
            )
            print(
                f">>> TURN COUNT = "
                f"{turn_count}/{TOTAL_TURNS}"
            )

            if turn_count == 4:
                print(">>> LAP 1 COMPLETE")

            elif turn_count == 8:
                print(">>> LAP 2 COMPLETE")

            elif turn_count == 12:

                print(">>> LAP 3 COMPLETE")

                send("V")

                print(
                    ">>> MOTOR SLOW SPEED "
                    "= 150/255"
                )

                print()
                print(
                    ">>> FINISH MODE ACTIVATED"
                )
                print(
                    ">>> SEARCHING START POSITION"
                )

            print()

            return

        # Preserve E3's 4-second safety limit, but stop
        # instead of allowing an uncontrolled continuation.
        if elapsed >= MAX_TURN_TIME:

            print()
            print(">>> MPU TURN TIMEOUT")
            print(">>> STOP FOR SAFETY")

            send("S")
            send("C")

            raise RuntimeError(
                "MPU did not reach 85 degrees"
            )


def automatic_right_turn():

    global turn_count
    global latest_mpu_angle

    print()
    print(">>> CORNER DETECTED")
    print(">>> AUTOMATIC RIGHT TURN - MPU 85 DEG")

    # Reset MPU yaw reference at the beginning of THIS turn.
    # Z resets yaw only; steering remains controlled by the Pi.
    latest_mpu_angle = None

    try:
        esp32.reset_input_buffer()
    except Exception:
        pass

    esp32.write(b"Z")
    esp32.flush()

    time.sleep(0.08)

    try:
        esp32.reset_input_buffer()
    except Exception:
        pass

    latest_mpu_angle = None

    # RIGHT steering command for clockwise/right-hand course.
    send("R")

    start_time = time.monotonic()

    while True:

        (
            cam_left,
            cam_right,
            cam_center,
            cam_error
        ) = analyze_camera()

        left, front, right = read_all()

        elapsed = time.monotonic() - start_time

        # Read MPU angle from ESP32.
        # RIGHT rotation is negative yaw.
        try:

            while esp32.in_waiting > 0:

                line = esp32.readline().decode(
                    errors="ignore"
                ).strip()

                if not line:
                    continue

                if "ANGLE:" in line:

                    try:
                        angle_text = (
                            line.split("ANGLE:", 1)[1]
                            .split("deg", 1)[0]
                            .strip()
                        )

                        latest_mpu_angle = float(
                            angle_text
                        )

                    except Exception:
                        pass

        except Exception:
            pass

        if (
            left is not None
            and front is not None
            and right is not None
        ):

            angle_text = (
                f"{latest_mpu_angle:6.1f} deg"
                if latest_mpu_angle is not None
                else "   --- deg"
            )

            print(
                f"TURN | "
                f"L:{left:6.1f} | "
                f"F:{front:6.1f} | "
                f"R:{right:6.1f} | "
                f"MPU:{angle_text} | "
                f"TARGET:{MPU_TURN_TARGET:.0f} | "
                f"T:{elapsed:4.2f}s | "
                + camera_text(
                    cam_left,
                    cam_right,
                    cam_center,
                    cam_error
                )
            )

        # E3 MPU85:
        # As soon as a REAL MPU report reaches the negative right-turn target,
        # center steering immediately.
        if (
            latest_mpu_angle is not None
            and latest_mpu_angle <= -MPU_TURN_TARGET
        ):

            send("C")

            turn_count += 1

            print()
            print(">>> MPU RIGHT TARGET REACHED")
            print(
                f">>> MPU ANGLE = "
                f"{latest_mpu_angle:.1f} deg"
            )
            print(
                f">>> TURN FINISHED AFTER "
                f"{elapsed:.2f} s"
            )
            print(
                f">>> TURN COUNT = "
                f"{turn_count}/{TOTAL_TURNS}"
            )

            if turn_count == 4:
                print(">>> LAP 1 COMPLETE")

            elif turn_count == 8:
                print(">>> LAP 2 COMPLETE")

            elif turn_count == 12:

                print(">>> LAP 3 COMPLETE")

                send("V")

                print(
                    ">>> MOTOR SLOW SPEED "
                    "= 150/255"
                )

                print()
                print(
                    ">>> FINISH MODE ACTIVATED"
                )
                print(
                    ">>> SEARCHING START POSITION"
                )

            print()

            return

        # Preserve E3's 4-second safety limit, but stop
        # instead of allowing an uncontrolled continuation.
        if elapsed >= MAX_TURN_TIME:

            print()
            print(">>> MPU TURN TIMEOUT")
            print(">>> STOP FOR SAFETY")

            send("S")
            send("C")

            raise RuntimeError(
                "MPU did not reach 85 degrees"
            )




# ==================================================
# INITIALIZATION
# ==================================================

print()
print(
    "===================================="
)

print(
    " WRO 2026 - OPEN CHALLENGE E3 MPU85"
)

print(
    " CAMERA + ULTRASONIC FUSION"
)

print(
    "===================================="
)

print()

print(
    "CAMERA = LINE CONTROL WHEN RELIABLE"
)

print(
    "ULTRASONIC = CORNERS + FALLBACK + FINISH"
)

print(
    "MPU = TURN ANGLE OBSERVER ONLY"
)

print(
    "MPU DOES NOT CONTROL STEERING"
)

print()

print(
    "KEEP ROBOT IN START POSITION"
)

print(
    "DO NOT MOVE IT"
)

print()


# ==================================================
# CENTER STEERING
# ==================================================

send("C")

time.sleep(0.5)


# ==================================================
# WAIT FOR PHYSICAL START BUTTON - RASPBERRY PI GPIO5
# ==================================================

# Keep the robot stopped while the program is waiting.
send("S")
send("C")

print()
print("====================================")
print(" ROBOT READY")
print(" WAITING FOR START BUTTON - PI GPIO5")
print("====================================")
print()
print("You may move/place the robot now.")
print("After the click, DO NOT TOUCH IT while the start position is measured.")
print()

start_button.wait_for_press()

print(">>> START BUTTON PRESSED - PI GPIO5")
print(">>> SAVING START POSITION BEFORE MOTOR START")
print()

# ==================================================
# START POSITION - MEASURE ONLY AFTER GPIO5 CLICK
# ==================================================

(
    start_left,
    start_front,
    start_right
) = measure_start_position()

# ==================================================
# COURSE DIRECTION IS NOT CHOSEN AT START
# ==================================================
# The robot must first move forward. The FIRST side sensor
# that sees an opening >= 100 cm determines the course:
#   LEFT  >= 100 cm -> first turn LEFT, then all turns LEFT
#   RIGHT >= 100 cm -> first turn RIGHT, then all turns RIGHT
DIRECTION_THRESHOLD = 100.0
course_direction = None

print("====================================")
print(" COURSE DIRECTION: WAITING FOR FIRST SIDE OPENING")
print(" LEFT >= 100 cm  -> FIRST TURN LEFT")
print(" RIGHT >= 100 cm -> FIRST TURN RIGHT")
print("====================================")
print()


# ==================================================
# MOTOR START - IMMEDIATELY AFTER START POSITION SAVED
# ==================================================

print("STARTING ROBOT...")
print()

esp32.write(b"M")
esp32.flush()
last_command = "M"

print(
    "MOTOR NORMAL SPEED = 255/255"
)

print("ROBOT STARTED")
print()


# ==================================================
# MAIN LOOP
# ==================================================

finish_confirmation_count = 0
front_trigger_count = 0

try:

    while True:

        # ------------------------------------------
        # CAMERA
        # ------------------------------------------

        (
            cam_left,
            cam_right,
            cam_center,
            cam_error
        ) = analyze_camera()


        # ------------------------------------------
        # ULTRASOUND
        # ------------------------------------------

        left, front, right = \
            read_all()


        # ==========================================
        # INVALID SENSOR
        # ==========================================

        if (
            left is None
            or front is None
            or right is None
        ):

            send("C")

            print(
                "INVALID SENSOR -> CENTER | "
                + camera_text(
                    cam_left,
                    cam_right,
                    cam_center,
                    cam_error
                )
            )

            continue


        # ==========================================
        # FINISH MODE
        # ==========================================

        if turn_count >= TOTAL_TURNS:

            # --------------------------------------
            # Front safety
            # --------------------------------------

            if front < FRONT_TRIGGER:

                print()
                print(
                    "===================================="
                )

                print(
                    " FINISH SEARCH SAFETY STOP"
                )

                print(
                    "===================================="
                )

                print(
                    f"FRONT WALL = "
                    f"{front:.1f} cm"
                )

                send("S")
                send("C")

                print(
                    "MOTOR STOPPED"
                )

                print(
                    "STEERING CENTERED"
                )

                break


            # --------------------------------------
            # Stable ultrasonic centering
            # --------------------------------------

            difference = (
                left - right
            )

            if (
                difference
                > SIDE_TOLERANCE
            ):

                send("A")

                direction = \
                    "SOFT LEFT"

            elif (
                difference
                < -SIDE_TOLERANCE
            ):

                send("D")

                direction = \
                    "SOFT RIGHT"

            else:

                send("C")

                direction = \
                    "CENTER"


            # --------------------------------------
            # Start signature
            # --------------------------------------

            if start_position_detected(
                left,
                front,
                right
            ):

                finish_confirmation_count += 1

                print(
                    f"FINISH MATCH "
                    f"{finish_confirmation_count}/"
                    f"{FINISH_CONFIRMATIONS} | "
                    f"L:{left:6.1f} | "
                    f"F:{front:6.1f} | "
                    f"R:{right:6.1f} | "
                    + camera_text(
                        cam_left,
                        cam_right,
                        cam_center,
                        cam_error
                    )
                )

            else:

                finish_confirmation_count = 0

                print(
                    f"SEARCH START | "
                    f"L:{left:6.1f} | "
                    f"F:{front:6.1f} | "
                    f"R:{right:6.1f} | "
                    f"{direction} | "
                    + camera_text(
                        cam_left,
                        cam_right,
                        cam_center,
                        cam_error
                    )
                )


            # --------------------------------------
            # Finish confirmed
            # --------------------------------------

            if (
                finish_confirmation_count
                >= FINISH_CONFIRMATIONS
            ):

                print()
                print(
                    "===================================="
                )

                print(
                    " START POSITION FOUND"
                )

                print(
                    " 3 LAPS COMPLETE"
                )

                print(
                    "===================================="
                )

                print()

                send("S")
                send("C")

                print(
                    "MOTOR STOPPED"
                )

                print(
                    "STEERING CENTERED"
                )

                break

            continue


        # ==========================================
        # FIRST CORNER: DETECT COURSE FROM SIDE OPENING
        # ==========================================
        # Before the first turn, do NOT use the front sensor
        # to choose the course. Keep moving normally until one
        # side ultrasonic sees 100 cm or more. That opening IS
        # the first corner and locks the direction for the race.
        if course_direction is None:

            left_open = left >= DIRECTION_THRESHOLD
            right_open = right >= DIRECTION_THRESHOLD

            if left_open or right_open:

                if left_open and not right_open:
                    course_direction = "LEFT"
                elif right_open and not left_open:
                    course_direction = "RIGHT"
                else:
                    # If both cross the threshold in the same cycle,
                    # use the larger opening.
                    course_direction = "LEFT" if left >= right else "RIGHT"

                print()
                print("====================================")
                print(f">>> FIRST SIDE OPENING DETECTED: {course_direction}")
                print(f">>> LEFT={left:.1f} cm | RIGHT={right:.1f} cm")
                print(f">>> COURSE LOCKED {course_direction} FOR ALL 12 TURNS")
                print(">>> THIS IS TURN 1/12")
                print("====================================")
                print()

                if course_direction == "LEFT":
                    automatic_left_turn()
                else:
                    automatic_right_turn()

                turn_armed = False
                front_trigger_count = 0
                continue

            # No side opening yet: robot continues normal corridor control.
            # The normal steering code below remains active.

        # ==========================================
        # CORNER DETECTION AFTER DIRECTION IS LOCKED
        # ==========================================

        # E3: require 3 consecutive front readings below
        # FRONT_TRIGGER before starting a corner turn.
        # This prevents one isolated ultrasonic echo from
        # causing a false turn in the middle of a corridor.

        if course_direction is not None and turn_armed:

            if front < FRONT_TRIGGER:

                front_trigger_count += 1

                print(
                    f">>> FRONT TRIGGER CONFIRM "
                    f"{front_trigger_count}/"
                    f"{FRONT_TRIGGER_CONFIRMATIONS} "
                    f"| F:{front:.1f} cm"
                )

                if (
                    front_trigger_count
                    >= FRONT_TRIGGER_CONFIRMATIONS
                ):

                    front_trigger_count = 0

                    if course_direction == "LEFT":
                        automatic_left_turn()
                    else:
                        automatic_right_turn()

                    turn_armed = False

                    continue

            else:

                front_trigger_count = 0

        else:

            front_trigger_count = 0


        # ==========================================
        # RE-ARM
        # ==========================================

        if front > 110:

            turn_armed = True


        # ==========================================
        # NORMAL CONTROL - E1 FUSION
        #
        # Camera controls only after several consecutive
        # reliable frames. Otherwise V3-3 ultrasonic
        # steering is used immediately.
        # ==========================================

        difference = left - right

        if camera_reliable(cam_left, cam_right, cam_error):
            camera_valid_count += 1
        else:
            camera_valid_count = 0

        if camera_valid_count >= CAM_VALID_CONFIRMATIONS:
            command, direction = camera_steering(cam_error)
            control_source = "CAM"
        else:
            command, direction, difference = ultrasonic_steering(left, right)
            control_source = "US"

        send(command)


        # ==========================================
        # LOG
        # ==========================================

        print(
            f"L:{left:6.1f} cm | "
            f"F:{front:6.1f} cm | "
            f"R:{right:6.1f} cm | "
            f"US DIFF:"
            f"{difference:6.1f} | "
            f"TURN:"
            f"{turn_count:2d}/"
            f"{TOTAL_TURNS} | "
            f"{direction} | "
            f"SRC:{control_source} | "
            + camera_text(
                cam_left,
                cam_right,
                cam_center,
                cam_error
            )
        )


# ==================================================
# CTRL+C
# ==================================================

except KeyboardInterrupt:

    print()
    print(
        "CTRL+C DETECTED"
    )


# ==================================================
# SAFE SHUTDOWN
# ==================================================

finally:

    print()
    print(
        "STOPPING ROBOT..."
    )

    # ----------------------------------------------
    # MOTOR STOP
    # ----------------------------------------------

    try:

        esp32.write(b"S")
        esp32.flush()

    except Exception:

        pass


    # ----------------------------------------------
    # CENTER STEERING
    # ----------------------------------------------

    try:

        time.sleep(0.1)

        esp32.write(b"C")
        esp32.flush()

    except Exception:

        pass


    print(
        "MOTOR STOPPED"
    )

    print(
        "STEERING CENTERED"
    )

    time.sleep(0.5)


    # ----------------------------------------------
    # CAMERA
    # ----------------------------------------------

    try:

        picam2.stop()

        cv2.destroyAllWindows()

    except Exception:

        pass


    # ----------------------------------------------
    # SENSORS
    # ----------------------------------------------

    for sensor in (
        left_sensor,
        front_sensor,
        right_sensor
    ):

        try:

            sensor.close()

        except Exception:

            pass


    time.sleep(0.5)


    # ----------------------------------------------
    # SERIAL
    # ----------------------------------------------

    try:

        if esp32.is_open:

            esp32.close()

    except Exception:

        pass


    print(
        "CAMERA STOPPED"
    )

    print(
        "ROBOT SAFELY STOPPED"
    )

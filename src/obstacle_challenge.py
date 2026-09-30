from picamera2 import Picamera2
from gpiozero import DistanceSensor, Button
import cv2
import numpy as np
import serial
import time
import math


# ============================================================
# WRO 2026 - O26
#
# PREMIER COULOIR:
#   SCAN -> SPACE -> PRE-STEER -> START
#
# COULOIRS SUIVANTS:
#   TURN
#     ↓
#   BACKUP
#     ↓
#   STOP + CENTER
#     ↓
#   SCAN
#     ↓
#   PLAN FIGE
#     ↓
#   ACQUIRE OBSTACLE #1 DANS IMAGE ACTUELLE
#     ↓
#   PRE-STEER AVANT MOTEUR
#     ↓
#   START AUTOMATIQUE
#
# Pendant le couloir:
#   obstacle #1
#     ↓
#   reacquire obstacle #2 prévu
#     ↓
#   obstacle #2
#     ↓
#   turn
#
# IMPORTANT:
# - maximum 2 obstacles par scan
# - le plan est figé
# - un obstacle extérieur ne change pas le plan
# - après CLOSE, un autre objet de même couleur
#   ne peut pas remplacer l'objet actif
# ============================================================


# ============================================================
# GPIO5 START + VALIDATED PARKING / FIRST CORRIDOR POSITION
# ============================================================
START_BUTTON_GPIO = 5
REAR_TRIGGER = 25
REAR_ECHO = 4

FIRST_LEFT_FORWARD_TIME = 1.2
RIGHT_REVERSE_TIME = 0.5
FINAL_LEFT_TIME = 2.0
FINAL_STRAIGHT_TIME = 0.40
SERVO_SETTLE_TIME = 0.30
REAR_STOP_CM = 5.0
REAR_MAX_VALID_CM = 150.0

POST_EXIT_RIGHT_ANGLE = -5.0
POST_EXIT_TURN_TIMEOUT = 6.0
FRONT_STOP_CM = 20.0
FRONT_MAX_VALID_CM = 300.0

REVERSE_RIGHT_TARGET_ANGLE = 45.0
REVERSE_RIGHT_TIMEOUT = 8.0
REAR_FINAL_STOP_CM = 20.0

# ============================================================
# CAMERA
# ============================================================

WIDTH = 1280
HEIGHT = 720

ROI_TOP = 20
ROI_BOTTOM = 620
ROI_LEFT = 0
ROI_RIGHT = WIDTH


# ============================================================
# BIRD VIEW - CALIBRATION VALIDEE
# ============================================================

SRC_POINTS = np.float32([
    [486, 104],
    [826, 115],
    [1150, 289],
    [25, 282]
])

DST_POINTS = np.float32([
    [100, 40],
    [600, 40],
    [600, 660],
    [100, 660]
])

REAL_WIDTH_CM = 68.0
REAL_LENGTH_CM = 118.0

DEST_WIDTH_PX = 500.0
DEST_LENGTH_PX = 620.0

PX_TO_CM_X = REAL_WIDTH_CM / DEST_WIDTH_PX
PX_TO_CM_Y = REAL_LENGTH_CM / DEST_LENGTH_PX

CM_TO_PX_X = DEST_WIDTH_PX / REAL_WIDTH_CM


# ============================================================
# ROBOT
# ============================================================

ROBOT_WIDTH_CM = 14.0
ROBOT_HALF_WIDTH_CM = 7.0

EXTRA_SAFETY_CM = 3.0

SAFE_CENTER_DISTANCE_CM = (
    ROBOT_HALF_WIDTH_CM
    + EXTRA_SAFETY_CM
)

SAFE_MARGIN_PX = int(
    SAFE_CENTER_DISTANCE_CM
    * CM_TO_PX_X
)


# ============================================================
# SERIAL
# ============================================================

SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200

esp32 = None
last_command = None


# ============================================================
# COMMANDES ESP32
# ============================================================

CMD_LEFT_STRONG = "L"
CMD_LEFT_SOFT = "A"

CMD_CENTER = "C"

CMD_RIGHT_SOFT = "D"
CMD_RIGHT_STRONG = "R"

CMD_MOTOR_FAST = "M"
CMD_MOTOR_SLOW = "V"

CMD_STOP = "S"
CMD_BACKWARD = "B"


# ============================================================
# DEMARRAGE
# ============================================================

BOOST_TIME = 0.40

# Après un virage, l'obstacle peut être extrêmement proche.
# On braque AVANT de démarrer.
PRE_STEER_TIME = 0.15

# Si obstacle très proche après le recul, on évite un long boost.
VERY_CLOSE_HEIGHT = 300
VERY_CLOSE_BOTTOM = 500

CLOSE_START_BOOST_TIME = 0.15


# ============================================================
# DEGAGEMENTS VALIDES
# ============================================================

GREEN_PASS_EXTRA_TIME = 1.30
RED_PASS_EXTRA_TIME = 1.50


# ============================================================
# RECUL
# ============================================================

BACKUP_TIME = 0.70
BACKUP_SETTLE_TIME = 0.30


# ============================================================
# ULTRASONIC
# ============================================================

RIGHT_TRIGGER = 17
RIGHT_ECHO = 23

LEFT_TRIGGER = 27
LEFT_ECHO = 24

FRONT_TRIGGER = 22
FRONT_ECHO = 16


right_sensor = DistanceSensor(
    echo=RIGHT_ECHO,
    trigger=RIGHT_TRIGGER,
    max_distance=4.0
)

left_sensor = DistanceSensor(
    echo=LEFT_ECHO,
    trigger=LEFT_TRIGGER,
    max_distance=4.0
)

front_sensor = DistanceSensor(
    echo=FRONT_ECHO,
    trigger=FRONT_TRIGGER,
    max_distance=4.0
)

rear_sensor = DistanceSensor(
    echo=REAR_ECHO,
    trigger=REAR_TRIGGER,
    max_distance=4.0,
    queue_len=1
)

start_button = Button(
    START_BUTTON_GPIO,
    pull_up=True,
    bounce_time=0.05
)


# ============================================================
# VIRAGE
# ============================================================

FRONT_CLEAR = 130.0

MIN_TURN_TIME = 1.95
MAX_TURN_TIME = 4.00

CLEAR_CONFIRMATIONS = 4
TURN_CHECK_DELAY = 0.03

# ============================================================
# O24 - APPROCHE PHYSIQUE DE LA FIN DU COULOIR
# ============================================================
# Après le dernier obstacle, ne pas tourner immédiatement.
# Continuer centré à vitesse lente jusqu'à l'approche du mur.
TURN_START_FRONT_CM = 100.0
TURN_START_CONFIRMATIONS = 3
APPROACH_TURN_TIMEOUT = 12.0
APPROACH_TURN_CHECK_DELAY = 0.05

# O28 - après le dernier obstacle:
# après le dernier obstacle: FRONT <= 100 cm, virage GAUCHE 90 deg MPU, puis recul centré jusqu'à REAR <= 40 cm.
NEW_CORRIDOR_LEFT_ANGLE = 90.0
NEW_CORRIDOR_LEFT_TIMEOUT = 8.0
NEW_CORRIDOR_REAR_STOP_CM = 40.0
NEW_CORRIDOR_REVERSE_TIMEOUT = 8.0

# O25 - centrage latéral ultrasonique pendant APPROACH_TURN
SIDE_CENTER_DEADBAND_CM = 6.0
SIDE_MIN_VALID_CM = 12.0
SIDE_MAX_VALID_CM = 100.0
SIDE_CORRECTION_CONFIRMATIONS = 2


# ============================================================
# HSV
# ============================================================

RED_LOW_1 = np.array(
    [0, 80, 60],
    dtype=np.uint8
)

RED_HIGH_1 = np.array(
    [12, 255, 255],
    dtype=np.uint8
)

RED_LOW_2 = np.array(
    [165, 80, 60],
    dtype=np.uint8
)

RED_HIGH_2 = np.array(
    [179, 255, 255],
    dtype=np.uint8
)

GREEN_LOW = np.array(
    [35, 60, 40],
    dtype=np.uint8
)

GREEN_HIGH = np.array(
    [95, 255, 255],
    dtype=np.uint8
)


# ============================================================
# DETECTION
# ============================================================

MIN_AREA = 500
MIN_WIDTH = 8
MIN_HEIGHT = 30

MAX_WIDTH_HEIGHT_RATIO = 1.35
MIN_FILL_RATIO = 0.35


# ============================================================
# FAUX ROUGE AU SOL
# ============================================================

FLOOR_ZONE_TOP = 360
FLOOR_FRAGMENT_MAX_HEIGHT = 70
FLOOR_FRAGMENT_MAX_AREA = 2500


# ============================================================
# SCAN
# ============================================================

SCAN_CONFIRMATIONS = 5
MAX_PLAN_OBSTACLES = 2

scan_last_signature = None
scan_confirmation_count = 0

corridor_plan = []
current_plan_index = 0

# Premier couloir = SPACE.
# Après le premier virage = démarrage automatique.
auto_start_after_scan = True
scan_plan_ready_announced = False


# ============================================================
# REACQUISITION
# ============================================================

REACQUIRE_CONFIRMATIONS = 4
REACQUIRE_TIMEOUT = 1.50

reacquire_candidate_color = None
reacquire_count = 0
reacquire_start_time = None

# True seulement lorsqu'on acquiert le premier obstacle
# d'un nouveau couloir avant de redémarrer le moteur.
reacquire_for_corridor_start = False


# ============================================================
# GREEN
# ============================================================

GREEN_STRONG_ERROR_CM = 12.0
GREEN_SOFT_ERROR_CM = 4.0

GREEN_CLOSE_HEIGHT = 185
GREEN_CLOSE_BOTTOM = 260

GREEN_LOST_CONFIRMATIONS = 6
GREEN_EARLY_LOST_LIMIT = 8


# ============================================================
# RED
# ============================================================

RED_STRONG_ERROR_CM = 12.0
RED_SOFT_ERROR_CM = 4.0

RED_CLOSE_HEIGHT = 185
RED_CLOSE_BOTTOM = 260

RED_LOST_CONFIRMATIONS = 6
RED_EARLY_LOST_LIMIT = 8


# ============================================================
# TRACKING
# ============================================================

tracked_object = None

tracked_close_seen = False
tracked_lost_count = 0
tracked_early_lost_count = 0

TRACK_MAX_SCORE_BEFORE_CLOSE = 260.0
TRACK_MAX_SCORE_AFTER_CLOSE = 120.0


# ============================================================
# TURN / BACKUP
# ============================================================

turn_start_time = None
turn_clear_count = 0

# O24: état d'approche de la vraie fin physique du couloir.
approach_turn_start_time = None
approach_turn_confirm_count = 0
side_last_direction = None
side_direction_count = 0

backup_start_time = None


# ============================================================
# STATE
# ============================================================

phase = "SCAN"


# ============================================================
# PERSPECTIVE
# ============================================================

perspective_matrix = cv2.getPerspectiveTransform(
    SRC_POINTS,
    DST_POINTS
)


# ============================================================
# CAMERA
# ============================================================

picam2 = Picamera2()

camera_config = picam2.create_video_configuration(
    main={
        "size": (WIDTH, HEIGHT),
        "format": "RGB888"
    },
    controls={
        "FrameRate": 60
    }
)

picam2.configure(camera_config)


# ============================================================
# MORPHOLOGY
# ============================================================

kernel3 = np.ones((3, 3), np.uint8)
kernel5 = np.ones((5, 5), np.uint8)


# ============================================================
# SERIAL
# ============================================================

def open_serial():

    global esp32

    esp32 = serial.Serial()

    esp32.port = SERIAL_PORT
    esp32.baudrate = BAUDRATE
    esp32.timeout = 1

    esp32.dtr = False
    esp32.rts = False

    esp32.open()

    time.sleep(1)

    esp32.reset_input_buffer()

    print("ESP32 CONNECTED:", SERIAL_PORT)


def send_command(command):

    global last_command

    if esp32 is None:
        return

    if command == last_command:
        return

    try:

        esp32.write(command.encode("ascii"))
        esp32.flush()

        last_command = command

        print("CMD:", command)

    except Exception as error:

        print("SERIAL ERROR:", error)


def emergency_stop():

    send_command(CMD_STOP)
    time.sleep(0.05)
    send_command(CMD_CENTER)


# ============================================================
# ULTRASONIC
# ============================================================

def read_distance(sensor):

    try:

        value = sensor.distance * 100.0

        if value <= 0:
            return None

        return value

    except Exception:
        return None


def read_front():

    return read_distance(front_sensor)


def read_left():

    return read_distance(left_sensor)


def read_right():

    return read_distance(right_sensor)


# ============================================================
# PARKING + POSITIONING HELPERS
# ============================================================

def clear_serial():
    try:
        esp32.reset_input_buffer()
    except Exception:
        pass


def reset_mpu():
    clear_serial()
    esp32.write(b"Z")
    esp32.flush()
    time.sleep(0.10)
    clear_serial()


def read_new_mpu_angles():
    angles = []
    try:
        while esp32.in_waiting > 0:
            line = esp32.readline().decode(errors="ignore").strip()
            if "ANGLE:" not in line:
                continue
            try:
                angle_text = line.split("ANGLE:", 1)[1].split("deg", 1)[0].strip()
                angles.append(float(angle_text))
            except Exception:
                pass
    except Exception:
        pass
    return angles


def read_rear_cm():
    try:
        value = rear_sensor.distance * 100.0
        if 1.0 <= value <= REAR_MAX_VALID_CM:
            return value
    except Exception:
        pass
    return None


def timed_motion(duration, label, monitor_rear=False):
    start = time.monotonic()
    while True:
        elapsed = time.monotonic() - start
        if elapsed >= duration:
            return True
        rear = read_rear_cm()
        rear_text = f"{rear:5.1f} cm" if rear is not None else "---"
        print(f"{label} | T:{elapsed:4.2f}/{duration:.2f}s | REAR:{rear_text}")
        if monitor_rear and rear is not None and rear <= REAR_STOP_CM:
            print(f">>> REAR SAFETY STOP ({rear:.1f} cm)")
            send_command(CMD_STOP)
            return False
        time.sleep(0.05)


def parking_exit():
    print("\\n==========================================")
    print(" VALIDATED PARKING EXIT")
    print("==========================================")
    print("STARTING IN 3 SECONDS...")
    time.sleep(3.0)

    send_command(CMD_LEFT_STRONG)
    time.sleep(SERVO_SETTLE_TIME)
    send_command(CMD_MOTOR_SLOW)
    timed_motion(FIRST_LEFT_FORWARD_TIME, "LEFT FORWARD")

    send_command(CMD_STOP)
    time.sleep(0.30)

    send_command(CMD_RIGHT_STRONG)
    time.sleep(SERVO_SETTLE_TIME)
    # N = slow reverse in the validated ESP32 firmware.
    send_command("N")
    timed_motion(RIGHT_REVERSE_TIME, "RIGHT REVERSE", monitor_rear=True)

    send_command(CMD_STOP)
    time.sleep(0.30)

    send_command(CMD_LEFT_STRONG)
    time.sleep(SERVO_SETTLE_TIME)
    send_command(CMD_MOTOR_SLOW)
    timed_motion(FINAL_LEFT_TIME, "FINAL LEFT FORWARD")

    send_command(CMD_CENTER)
    send_command(CMD_MOTOR_SLOW)
    timed_motion(FINAL_STRAIGHT_TIME, "CENTER FORWARD")

    emergency_stop()
    print(">>> PARKING EXIT COMPLETE")
    return True


def position_toward_first_corridor():
    print("\\n==========================================")
    print(" POSITION TOWARD FIRST CORRIDOR")
    print("==========================================")
    emergency_stop()
    time.sleep(0.30)
    reset_mpu()

    send_command(CMD_RIGHT_STRONG)
    time.sleep(SERVO_SETTLE_TIME)
    send_command(CMD_MOTOR_SLOW)

    start = time.monotonic()
    latest_angle = None
    while True:
        elapsed = time.monotonic() - start
        for angle in read_new_mpu_angles():
            latest_angle = angle
        print("RIGHT TURN | MPU:", "---" if latest_angle is None else f"{latest_angle:.1f}")
        if latest_angle is not None and latest_angle <= POST_EXIT_RIGHT_ANGLE:
            emergency_stop()
            break
        if elapsed >= POST_EXIT_TURN_TIMEOUT:
            emergency_stop()
            print(">>> RIGHT TURN TIMEOUT")
            return False
        time.sleep(0.03)

    time.sleep(0.30)
    send_command(CMD_CENTER)
    send_command(CMD_MOTOR_SLOW)
    while True:
        front = read_front()
        print("FRONT:", "---" if front is None else f"{front:.1f} cm")
        if front is not None and front <= FRONT_STOP_CM:
            emergency_stop()
            print(">>> FRONT POSITION REACHED")
            return True
        time.sleep(0.05)


def reverse_position_first_corridor():
    print("\\n==========================================")
    print(" FINAL POSITION IN FRONT OF FIRST CORRIDOR")
    print("==========================================")
    emergency_stop()
    time.sleep(0.30)
    reset_mpu()

    send_command(CMD_RIGHT_STRONG)
    time.sleep(SERVO_SETTLE_TIME)
    send_command("N")

    start = time.monotonic()
    latest_angle = None
    while True:
        elapsed = time.monotonic() - start
        for angle in read_new_mpu_angles():
            latest_angle = angle
        rear = read_rear_cm()
        print("REVERSE RIGHT | MPU:", "---" if latest_angle is None else f"{latest_angle:.1f}",
              "| REAR:", "---" if rear is None else f"{rear:.1f} cm")
        if latest_angle is not None and abs(latest_angle) >= REVERSE_RIGHT_TARGET_ANGLE:
            send_command(CMD_CENTER)
            break
        if rear is not None and rear <= REAR_FINAL_STOP_CM:
            emergency_stop()
            print(">>> REAR SAFETY STOP DURING TURN")
            return False
        if elapsed >= REVERSE_RIGHT_TIMEOUT:
            emergency_stop()
            print(">>> REVERSE POSITION TIMEOUT")
            return False
        time.sleep(0.03)

    send_command(CMD_CENTER)
    send_command("N")
    while True:
        rear = read_rear_cm()
        print("REAR:", "---" if rear is None else f"{rear:.1f} cm")
        if rear is not None and rear <= REAR_FINAL_STOP_CM:
            emergency_stop()
            print(">>> FIRST CORRIDOR POSITION REACHED - ROBOT STOPPED")
            return True
        time.sleep(0.05)


# ============================================================
# MASK
# ============================================================

def clean_mask(mask):

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel3
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        kernel5
    )

    return mask


# ============================================================
# FIND OBJECTS
# ============================================================

def find_objects(mask, color_name):

    clean = clean_mask(mask)

    contours, _ = cv2.findContours(
        clean,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    objects = []

    for contour in contours:

        area = cv2.contourArea(contour)

        if area < MIN_AREA:
            continue

        x, y, w, h = cv2.boundingRect(contour)

        if w < MIN_WIDTH:
            continue

        if h < MIN_HEIGHT:
            continue

        ratio = w / float(h)

        if ratio > MAX_WIDTH_HEIGHT_RATIO:
            continue

        rectangle_mask = clean[
            y:y + h,
            x:x + w
        ]

        rectangle_area = w * h

        if rectangle_area <= 0:
            continue

        fill = (
            cv2.countNonZero(rectangle_mask)
            / float(rectangle_area)
        )

        if fill < MIN_FILL_RATIO:
            continue

        gx = x + ROI_LEFT
        gy = y + ROI_TOP

        cx = gx + w // 2
        cy = gy + h // 2

        bottom = gy + h

        if color_name == "RED":

            if (
                bottom >= FLOOR_ZONE_TOP
                and
                h <= FLOOR_FRAGMENT_MAX_HEIGHT
                and
                area <= FLOOR_FRAGMENT_MAX_AREA
            ):
                continue

        objects.append({
            "color": color_name,
            "x": gx,
            "y": gy,
            "w": w,
            "h": h,
            "cx": cx,
            "cy": cy,
            "bottom": bottom,
            "area": area,
            "fill": fill
        })

    return objects


# ============================================================
# DEPTH
# ============================================================

def depth_score(obj):

    return (
        obj["bottom"] * 10
        + obj["h"] * 2
        + math.sqrt(obj["area"])
    )


def order_objects(objects):

    return sorted(
        objects,
        key=depth_score,
        reverse=True
    )


def find_nearest_color(objects, wanted_color):

    candidates = [
        obj
        for obj in objects
        if obj["color"] == wanted_color
    ]

    if not candidates:
        return None

    return max(
        candidates,
        key=depth_score
    )


# ============================================================
# CAMERA -> BIRD
# ============================================================

def camera_to_bird(x, y):

    point = np.array(
        [[[float(x), float(y)]]],
        dtype=np.float32
    )

    transformed = cv2.perspectiveTransform(
        point,
        perspective_matrix
    )

    bx = float(transformed[0][0][0])
    by = float(transformed[0][0][1])

    return bx, by


# ============================================================
# BIRD -> CM
# ============================================================

def bird_to_cm(bx, by):

    real_x = (
        bx - 100.0
    ) * PX_TO_CM_X

    real_y = (
        by - 40.0
    ) * PX_TO_CM_Y

    return real_x, real_y


# ============================================================
# GROUND EDGES
# ============================================================

def get_ground_edges(obj):

    camera_left_x = obj["x"]
    camera_right_x = obj["x"] + obj["w"]
    camera_y = obj["bottom"]

    left_bx, left_by = camera_to_bird(
        camera_left_x,
        camera_y
    )

    right_bx, right_by = camera_to_bird(
        camera_right_x,
        camera_y
    )

    bird_left = min(
        left_bx,
        right_bx
    )

    bird_right = max(
        left_bx,
        right_bx
    )

    bird_y = (
        left_by + right_by
    ) / 2.0

    return (
        bird_left,
        bird_right,
        bird_y
    )


# ============================================================
# TARGET GREEN
# ============================================================

def get_green_target(obj):

    bird_left, bird_right, bird_y = (
        get_ground_edges(obj)
    )

    target_x = (
        bird_left - SAFE_MARGIN_PX
    )

    target_x = max(
        100.0,
        min(600.0, target_x)
    )

    target_y = max(
        40.0,
        min(660.0, bird_y)
    )

    cm_x, cm_y = bird_to_cm(
        target_x,
        target_y
    )

    return {
        "x": target_x,
        "y": target_y,
        "cm_x": cm_x,
        "cm_y": cm_y
    }


# ============================================================
# TARGET RED
# ============================================================

def get_red_target(obj):

    bird_left, bird_right, bird_y = (
        get_ground_edges(obj)
    )

    target_x = (
        bird_right + SAFE_MARGIN_PX
    )

    target_x = max(
        100.0,
        min(600.0, target_x)
    )

    target_y = max(
        40.0,
        min(660.0, bird_y)
    )

    cm_x, cm_y = bird_to_cm(
        target_x,
        target_y
    )

    return {
        "x": target_x,
        "y": target_y,
        "cm_x": cm_x,
        "cm_y": cm_y
    }


# ============================================================
# ROBOT REFERENCE
# ============================================================

ROBOT_BIRD_X = 350.0

ROBOT_CM_X = (
    ROBOT_BIRD_X - 100.0
) * PX_TO_CM_X


# ============================================================
# STEERING GREEN
# ============================================================

def calculate_green_steering(obj):

    target = get_green_target(obj)

    error_cm = (
        ROBOT_CM_X
        - target["cm_x"]
    )

    if error_cm > GREEN_STRONG_ERROR_CM:

        command = CMD_LEFT_STRONG
        mode = "STRONG LEFT"

    elif error_cm > GREEN_SOFT_ERROR_CM:

        command = CMD_LEFT_SOFT
        mode = "SOFT LEFT"

    else:

        command = CMD_LEFT_SOFT
        mode = "HOLD LEFT"

    return (
        command,
        mode,
        error_cm,
        target
    )


# ============================================================
# STEERING RED
# ============================================================

def calculate_red_steering(obj):

    target = get_red_target(obj)

    error_cm = (
        target["cm_x"]
        - ROBOT_CM_X
    )

    if error_cm > RED_STRONG_ERROR_CM:

        command = CMD_RIGHT_STRONG
        mode = "STRONG RIGHT"

    elif error_cm > RED_SOFT_ERROR_CM:

        command = CMD_RIGHT_SOFT
        mode = "SOFT RIGHT"

    else:

        command = CMD_RIGHT_SOFT
        mode = "HOLD RIGHT"

    return (
        command,
        mode,
        error_cm,
        target
    )


# ============================================================
# COPY OBJECT
# ============================================================

def copy_object(obj):

    if obj is None:
        return None

    return {
        "color": obj["color"],
        "x": obj["x"],
        "y": obj["y"],
        "w": obj["w"],
        "h": obj["h"],
        "cx": obj["cx"],
        "cy": obj["cy"],
        "bottom": obj["bottom"],
        "area": obj["area"],
        "fill": obj["fill"]
    }


# ============================================================
# TRACKING SCORE
# ============================================================

def tracking_score(previous, candidate):

    dx = abs(
        candidate["cx"]
        - previous["cx"]
    )

    dbottom = abs(
        candidate["bottom"]
        - previous["bottom"]
    )

    dh = abs(
        candidate["h"]
        - previous["h"]
    )

    return (
        dx * 1.0
        + dbottom * 0.8
        + dh * 0.5
    )


# ============================================================
# MATCH TRACKED OBJECT
# ============================================================

def match_tracked_object(
    previous,
    objects,
    close_seen
):

    if previous is None:
        return None

    same_color = [
        obj
        for obj in objects
        if obj["color"] == previous["color"]
    ]

    if not same_color:
        return None

    scored = []

    for obj in same_color:

        score = tracking_score(
            previous,
            obj
        )

        scored.append(
            (score, obj)
        )

    scored.sort(
        key=lambda item: item[0]
    )

    best_score = scored[0][0]
    best_object = scored[0][1]

    if close_seen:
        max_score = TRACK_MAX_SCORE_AFTER_CLOSE
    else:
        max_score = TRACK_MAX_SCORE_BEFORE_CLOSE

    if best_score > max_score:
        return None

    return best_object


# ============================================================
# O26 - CURRENT CORRIDOR SCAN FILTER
# ============================================================

def is_in_current_corridor_scan(obj):
    """
    Filtre utilisé UNIQUEMENT pendant SCAN.

    On projette le point au sol situé au centre de l'obstacle
    dans la Bird View. Un obstacle dont le centre au sol est
    hors de la largeur calibrée du couloir [100, 600] est
    considéré comme appartenant à un autre couloir / hors piste
    courante.

    Important:
    - ne modifie PAS le tracking GREEN/RED;
    - ne modifie PAS le reacquire;
    - ne modifie PAS le dépassement;
    - sert seulement à construire le corridor_plan.
    """

    try:
        ground_x = float(obj["cx"])
        ground_y = float(obj["bottom"])

        bird_x, bird_y = camera_to_bird(
            ground_x,
            ground_y
        )

    except Exception:
        return False

    return (
        100.0 <= bird_x <= 600.0
        and
        40.0 <= bird_y <= 660.0
    )


def filter_current_corridor_scan(objects):

    accepted = []
    rejected = []

    for obj in objects:

        if is_in_current_corridor_scan(obj):
            accepted.append(obj)
        else:
            rejected.append(obj)

    return accepted, rejected


# ============================================================
# SCAN SIGNATURE
# ============================================================

def build_scan_signature(
    ordered_objects
):

    selected = ordered_objects[
        :MAX_PLAN_OBSTACLES
    ]

    if not selected:
        return None

    return tuple(
        obj["color"]
        for obj in selected
    )


# ============================================================
# BUILD PLAN
# ============================================================

def build_plan(
    ordered_objects
):

    selected = ordered_objects[
        :MAX_PLAN_OBSTACLES
    ]

    plan = []

    for index, obj in enumerate(selected):

        plan.append({
            "plan_id": index + 1,
            "color": obj["color"],
            "scan_cx": obj["cx"],
            "scan_bottom": obj["bottom"],
            "scan_h": obj["h"]
        })

    return plan


# ============================================================
# START REACQUIRE
# ============================================================

def start_reacquire(
    for_corridor_start=False
):

    global phase

    global tracked_object
    global tracked_close_seen
    global tracked_lost_count
    global tracked_early_lost_count

    global reacquire_candidate_color
    global reacquire_count
    global reacquire_start_time
    global reacquire_for_corridor_start


    if (
        current_plan_index
        >= len(corridor_plan)
    ):
        start_left_turn()
        return


    planned = corridor_plan[
        current_plan_index
    ]


    tracked_object = None
    tracked_close_seen = False
    tracked_lost_count = 0
    tracked_early_lost_count = 0

    reacquire_candidate_color = None
    reacquire_count = 0

    reacquire_start_time = (
        time.monotonic()
    )

    reacquire_for_corridor_start = (
        for_corridor_start
    )


    print()
    print(
        "========================================"
    )

    if for_corridor_start:
        print(" ACQUIRE FIRST OBSTACLE BEFORE START")
    else:
        print(" REACQUIRE NEXT PLANNED OBSTACLE")

    print(
        " ID:",
        planned["plan_id"]
    )

    print(
        " COLOR:",
        planned["color"]
    )

    print(
        " USING CURRENT CAMERA POSITION"
    )

    print(
        "========================================"
    )


    phase = "REACQUIRE"


# ============================================================
# START MOTOR AFTER PRE-STEER
# ============================================================

def start_motor_after_presteer(obj):

    very_close = (
        obj["h"] >= VERY_CLOSE_HEIGHT
        or
        obj["bottom"] >= VERY_CLOSE_BOTTOM
    )


    print()
    print(
        "PRE-STEER COMPLETE"
    )


    if very_close:

        print(
            "VERY CLOSE OBSTACLE"
        )

        print(
            "SHORT BOOST:",
            CLOSE_START_BOOST_TIME,
            "s"
        )

        send_command(
            CMD_MOTOR_FAST
        )

        time.sleep(
            CLOSE_START_BOOST_TIME
        )

        send_command(
            CMD_MOTOR_SLOW
        )

    else:

        print(
            "NORMAL START BOOST:",
            BOOST_TIME,
            "s"
        )

        send_command(
            CMD_MOTOR_FAST
        )

        time.sleep(
            BOOST_TIME
        )

        send_command(
            CMD_MOTOR_SLOW
        )


# ============================================================
# LOCK REACQUIRED OBJECT
# ============================================================

def lock_reacquired_object(obj):

    global phase

    global tracked_object
    global tracked_close_seen
    global tracked_lost_count
    global tracked_early_lost_count

    global reacquire_for_corridor_start


    tracked_object = copy_object(obj)

    tracked_close_seen = False
    tracked_lost_count = 0
    tracked_early_lost_count = 0


    print()
    print(
        "========================================"
    )

    print(
        " PLANNED OBSTACLE ACQUIRED"
    )

    print(
        " ID:",
        corridor_plan[
            current_plan_index
        ]["plan_id"]
    )

    print(
        " COLOR:",
        tracked_object["color"]
    )

    print(
        " CURRENT CX:",
        tracked_object["cx"]
    )

    print(
        " CURRENT BOTTOM:",
        tracked_object["bottom"]
    )

    print(
        " CURRENT H:",
        tracked_object["h"]
    )

    print(
        " PHYSICAL TRACK LOCKED"
    )

    print(
        "========================================"
    )


    if tracked_object["color"] == "GREEN":

        (
            command,
            mode,
            error,
            target
        ) = calculate_green_steering(
            tracked_object
        )

        phase = "TRACK_GREEN"

    else:

        (
            command,
            mode,
            error,
            target
        ) = calculate_red_steering(
            tracked_object
        )

        phase = "TRACK_RED"


    print(
        "STEERING:",
        mode
    )

    print(
        "ERROR:",
        f"{error:.1f}",
        "cm"
    )


    # ========================================================
    # CRITICAL O23:
    # BRAQUER AVANT DE DEMARRER
    # ========================================================

    send_command(command)

    if reacquire_for_corridor_start:

        print(
            "PRE-STEER BEFORE MOTOR"
        )

        time.sleep(
            PRE_STEER_TIME
        )

        start_motor_after_presteer(
            tracked_object
        )


    reacquire_for_corridor_start = False


# ============================================================
# FINISH OBSTACLE
# ============================================================

def finish_current_obstacle():

    global current_plan_index
    global tracked_object


    finished = corridor_plan[
        current_plan_index
    ]


    print()
    print(
        "========================================"
    )

    print(
        " OBSTACLE",
        finished["plan_id"],
        finished["color"],
        "PASSED"
    )

    print(
        "========================================"
    )


    tracked_object = None

    current_plan_index += 1


    if (
        current_plan_index
        < len(corridor_plan)
    ):

        print()
        print(
            "NEXT PLAN =",
            corridor_plan[
                current_plan_index
            ]["color"]
        )

        print(
            "REACQUIRE IN CURRENT IMAGE"
        )

        # Moteur reste déjà en V150.
        start_reacquire(
            for_corridor_start=False
        )

    else:

        print()
        print(
            "CORRIDOR COMPLETE"
        )

        print(
            "APPROACH FRONT 100 CM ZONE"
        )

        # O24:
        # Le nombre/placement des obstacles ne détermine plus
        # l'endroit du virage. On rejoint d'abord la vraie fin
        # physique du couloir avec l'ultrason avant.
        start_approach_turn()


# ============================================================
# O24 - APPROACH REAL END OF CORRIDOR
# ============================================================

def start_approach_turn():

    global phase
    global approach_turn_start_time
    global approach_turn_confirm_count
    global side_last_direction
    global side_direction_count

    print()
    print("========================================")
    print(" LAST PLANNED OBSTACLE PASSED")
    print(" APPROACH REAL END OF CORRIDOR")
    print(" CENTER + SLOW MOTOR")
    print("========================================")

    approach_turn_confirm_count = 0
    approach_turn_start_time = time.monotonic()

    side_last_direction = None
    side_direction_count = 0

    send_command(CMD_CENTER)
    time.sleep(0.05)
    send_command(CMD_MOTOR_SLOW)

    phase = "APPROACH_TURN"


# ============================================================
# O30 - LEFT 90 DEG THEN REVERSE TO REAR 40 CM
# ============================================================

def left_90_then_reverse_to_new_corridor():
    """FRONT <= 100 -> STOP -> L + forward to 90 deg MPU -> CENTER -> reverse to rear <= 40 -> SCAN."""
    global phase

    print()
    print("========================================")
    print(" FRONT 100 CM CONFIRMED")
    print(" LEFT TURN 90 DEG WITH MPU")
    print(" THEN CENTER + REVERSE TO REAR 40 CM")
    print("========================================")

    emergency_stop()
    time.sleep(0.20)
    reset_mpu()

    # 1) Virage à gauche en marche avant jusqu'à 90 degrés MPU.
    send_command(CMD_LEFT_STRONG)
    time.sleep(SERVO_SETTLE_TIME)
    send_command(CMD_MOTOR_FAST)

    start_time = time.monotonic()
    latest_angle = None

    while True:
        for angle in read_new_mpu_angles():
            latest_angle = angle

        if latest_angle is not None:
            print(
                "LEFT 90 | MPU:",
                f"{latest_angle:+.1f} deg",
                "| TARGET:",
                f"{NEW_CORRIDOR_LEFT_ANGLE:.0f} deg"
            )

            # Le virage à gauche est normalement positif avec ce firmware.
            # On valide la rotation physique de 90 degrés par sa magnitude.
            if abs(latest_angle) >= NEW_CORRIDOR_LEFT_ANGLE:
                break

        if time.monotonic() - start_time >= NEW_CORRIDOR_LEFT_TIMEOUT:
            emergency_stop()
            print(">>> LEFT 90 DEG TIMEOUT - ROBOT STOPPED")
            phase = "ERROR"
            return False

        time.sleep(0.02)

    # 2) Fin du virage: STOP puis roues au centre.
    send_command(CMD_STOP)
    time.sleep(0.10)
    send_command(CMD_CENTER)
    time.sleep(SERVO_SETTLE_TIME)

    print(">>> 90 DEG REACHED")
    print(">>> CENTER + REVERSE UNTIL REAR <= 40 CM")

    # 3) Recul en ligne droite jusqu'à 40 cm du mur arrière.
    send_command(CMD_BACKWARD)
    reverse_start = time.monotonic()

    while True:
        rear = read_rear_cm()
        rear_text = "---" if rear is None else f"{rear:.1f} cm"
        print("REVERSE TO NEW CORRIDOR | REAR:", rear_text)

        if rear is not None and rear <= NEW_CORRIDOR_REAR_STOP_CM:
            send_command(CMD_STOP)
            time.sleep(0.10)
            send_command(CMD_CENTER)
            print(f">>> REAR <= {NEW_CORRIDOR_REAR_STOP_CM:.0f} CM - STOP")
            print(">>> START NEW CORRIDOR SCAN")
            prepare_new_scan()
            return True

        if time.monotonic() - reverse_start >= NEW_CORRIDOR_REVERSE_TIMEOUT:
            emergency_stop()
            print(">>> REVERSE TO REAR 40 CM TIMEOUT - ROBOT STOPPED")
            phase = "ERROR"
            return False

        time.sleep(0.03)


# ============================================================
# START LEFT TURN
# ============================================================

def start_left_turn():

    global phase
    global turn_start_time
    global turn_clear_count


    print()
    print(
        "========================================"
    )

    print(
        " START LEFT TURN"
    )

    print(
        " L + M255"
    )

    print(
        "========================================"
    )


    turn_clear_count = 0


    # IMPORTANT:
    # envoyer L puis M une seule fois.
    send_command(
        CMD_LEFT_STRONG
    )

    time.sleep(
        0.05
    )

    send_command(
        CMD_MOTOR_FAST
    )


    turn_start_time = (
        time.monotonic()
    )

    phase = "TURN"


# ============================================================
# START BACKUP
# ============================================================

def start_backup():

    global phase
    global backup_start_time


    print()
    print(
        "========================================"
    )

    print(
        " TURN COMPLETE"
    )

    print(
        " STOP -> CENTER -> BACKUP"
    )

    print(
        "========================================"
    )


    send_command(
        CMD_STOP
    )

    time.sleep(
        0.08
    )


    send_command(
        CMD_CENTER
    )

    time.sleep(
        0.12
    )


    send_command(
        CMD_BACKWARD
    )


    backup_start_time = (
        time.monotonic()
    )

    phase = "BACKUP"


# ============================================================
# PREPARE NEW SCAN
# ============================================================

def prepare_new_scan():

    global phase

    global corridor_plan
    global current_plan_index

    global scan_last_signature
    global scan_confirmation_count
    global scan_plan_ready_announced

    global tracked_object
    global tracked_close_seen
    global tracked_lost_count
    global tracked_early_lost_count

    global reacquire_candidate_color
    global reacquire_count
    global reacquire_start_time

    global auto_start_after_scan


    send_command(
        CMD_STOP
    )

    time.sleep(
        BACKUP_SETTLE_TIME
    )

    send_command(
        CMD_CENTER
    )


    corridor_plan = []
    current_plan_index = 0

    scan_last_signature = None
    scan_confirmation_count = 0
    scan_plan_ready_announced = False

    tracked_object = None
    tracked_close_seen = False
    tracked_lost_count = 0
    tracked_early_lost_count = 0

    reacquire_candidate_color = None
    reacquire_count = 0
    reacquire_start_time = None


    # Après le premier virage, dès que le nouveau scan
    # est confirmé, le couloir démarre automatiquement.
    auto_start_after_scan = True

    phase = "SCAN"


    print()
    print(
        "========================================"
    )

    print(
        " NEW CORRIDOR"
    )

    print(
        " ROBOT STOPPED"
    )

    print(
        " NEW SCAN"
    )

    print(
        " AUTO START AFTER CONFIRMATION"
    )

    print(
        "========================================"
    )


# ============================================================
# START FROZEN CORRIDOR
# ============================================================

def start_frozen_corridor():

    global current_plan_index


    if len(corridor_plan) == 0:
        return


    current_plan_index = 0


    print()
    print(
        "========================================"
    )

    print(
        " START CORRIDOR PLAN"
    )

    print(
        " PLAN:",
        [
            p["color"]
            for p in corridor_plan
        ]
    )

    print(
        "========================================"
    )


    # Robot reste STOP pendant acquisition.
    # Après confirmation:
    # steer -> wait -> motor.
    start_reacquire(
        for_corridor_start=True
    )


# ============================================================
# INFO
# ============================================================

print()
print(
    "================================================"
)

print(
    " WRO 2026 - O26"
)

print(
    " TURN -> BACKUP -> SCAN -> PRE-STEER -> AUTO START"
)

print(
    "================================================"
)

print()

print(
    "GREEN CLEARANCE:",
    GREEN_PASS_EXTRA_TIME
)

print(
    "RED CLEARANCE:",
    RED_PASS_EXTRA_TIME
)

print(
    "TURN MIN:",
    MIN_TURN_TIME
)

print(
    "BACKUP:",
    BACKUP_TIME
)

print(
    "VERY CLOSE H:",
    VERY_CLOSE_HEIGHT
)

print(
    "VERY CLOSE BOTTOM:",
    VERY_CLOSE_BOTTOM
)

print()

print(
    "FIRST CORRIDOR: AUTO START AFTER SCAN"
)

print(
    "NEXT CORRIDORS: AUTOMATIC"
)

print(
    "Q = EMERGENCY STOP"
)

print()


# ============================================================
# MAIN
# ============================================================

try:

    open_serial()

    send_command(
        CMD_STOP
    )

    time.sleep(
        0.1
    )

    send_command(
        CMD_CENTER
    )

    print()
    print("========================================")
    print(" ROBOT READY - WAITING GPIO5")
    print("========================================")
    start_button.wait_for_press()
    print(">>> GPIO5 PRESSED - START")

    if not parking_exit():
        raise RuntimeError("Parking exit failed")
    if not position_toward_first_corridor():
        raise RuntimeError("First corridor positioning failed")
    if not reverse_position_first_corridor():
        raise RuntimeError("Final first corridor positioning failed")

    # O26 takes over here. No SPACE is required.
    # First stable scan automatically starts the corridor.
    emergency_stop()
    auto_start_after_scan = True

    picam2.start()

    time.sleep(
        2
    )


    while True:

        # ====================================================
        # CAMERA
        # ====================================================

        frame = picam2.capture_array()

        roi = frame[
            ROI_TOP:ROI_BOTTOM,
            ROI_LEFT:ROI_RIGHT
        ]


        # VALIDATED:
        # NO RGB/BGR conversion before this.
        hsv = cv2.cvtColor(
            roi,
            cv2.COLOR_BGR2HSV
        )


        # ====================================================
        # RED MASK
        # ====================================================

        red_mask_1 = cv2.inRange(
            hsv,
            RED_LOW_1,
            RED_HIGH_1
        )

        red_mask_2 = cv2.inRange(
            hsv,
            RED_LOW_2,
            RED_HIGH_2
        )

        red_mask = cv2.bitwise_or(
            red_mask_1,
            red_mask_2
        )


        # ====================================================
        # GREEN MASK
        # ====================================================

        green_mask = cv2.inRange(
            hsv,
            GREEN_LOW,
            GREEN_HIGH
        )


        # ====================================================
        # OBJECTS
        # ====================================================

        red_objects = find_objects(
            red_mask,
            "RED"
        )

        green_objects = find_objects(
            green_mask,
            "GREEN"
        )

        objects = (
            red_objects
            + green_objects
        )

        ordered_objects = order_objects(
            objects
        )


        steering_mode = "---"
        error_cm = 0.0
        front_cm = None


        # ====================================================
        # SCAN
        # ====================================================

        if phase == "SCAN":

            send_command(
                CMD_STOP
            )

            # O26:
            # Ne construire le plan qu'avec les obstacles dont
            # le point au sol appartient réellement au couloir
            # courant dans la Bird View.
            scan_objects, rejected_scan_objects = (
                filter_current_corridor_scan(
                    ordered_objects
                )
            )

            # Afficher uniquement lorsqu'un objet est rejeté,
            # pour vérifier physiquement le filtre sur la piste.
            for rejected_obj in rejected_scan_objects:

                rejected_bx, rejected_by = camera_to_bird(
                    rejected_obj["cx"],
                    rejected_obj["bottom"]
                )

                print(
                    "SCAN REJECT OTHER CORRIDOR:",
                    rejected_obj["color"],
                    "CX",
                    rejected_obj["cx"],
                    "BOTTOM",
                    rejected_obj["bottom"],
                    "BIRD",
                    f"{rejected_bx:.1f}",
                    f"{rejected_by:.1f}"
                )


            signature = build_scan_signature(
                scan_objects
            )


            if signature is not None:

                if (
                    signature
                    == scan_last_signature
                ):

                    scan_confirmation_count += 1

                else:

                    scan_last_signature = signature
                    scan_confirmation_count = 1


                if (
                    scan_confirmation_count
                    >= SCAN_CONFIRMATIONS
                    and
                    len(corridor_plan) == 0
                ):

                    corridor_plan = build_plan(
                        scan_objects
                    )


                    print()
                    print(
                        "========================================"
                    )

                    print(
                        " CORRIDOR PLAN FROZEN"
                    )


                    for p in corridor_plan:

                        print(
                            " ID",
                            p["plan_id"],
                            p["color"],
                            "SCAN CX",
                            p["scan_cx"],
                            "BOTTOM",
                            p["scan_bottom"],
                            "H",
                            p["scan_h"]
                        )


                    print(
                        "========================================"
                    )


                    # ========================================
                    # O23:
                    # Après un virage/recul, démarrer
                    # automatiquement.
                    # ========================================

                    if auto_start_after_scan:

                        print(
                            "AUTO START REQUESTED"
                        )

                        start_frozen_corridor()


            else:

                scan_last_signature = None
                scan_confirmation_count = 0


        # ====================================================
        # REACQUIRE
        # ====================================================

        elif phase == "REACQUIRE":

            planned_color = corridor_plan[
                current_plan_index
            ]["color"]


            current_candidate = (
                find_nearest_color(
                    objects,
                    planned_color
                )
            )


            if current_candidate is not None:

                if (
                    reacquire_candidate_color
                    == planned_color
                ):

                    reacquire_count += 1

                else:

                    reacquire_candidate_color = (
                        planned_color
                    )

                    reacquire_count = 1


                print(
                    "REACQUIRE",
                    planned_color,
                    reacquire_count,
                    "/",
                    REACQUIRE_CONFIRMATIONS,
                    "CX",
                    current_candidate["cx"],
                    "H",
                    current_candidate["h"],
                    "BOTTOM",
                    current_candidate["bottom"]
                )


                # =================================================
                # PRE-STEER DURING ACQUISITION
                # =================================================

                if planned_color == "GREEN":

                    (
                        command,
                        steering_mode,
                        error_cm,
                        target
                    ) = calculate_green_steering(
                        current_candidate
                    )

                else:

                    (
                        command,
                        steering_mode,
                        error_cm,
                        target
                    ) = calculate_red_steering(
                        current_candidate
                    )


                send_command(
                    command
                )


                if (
                    reacquire_count
                    >= REACQUIRE_CONFIRMATIONS
                ):

                    lock_reacquired_object(
                        current_candidate
                    )


            else:

                reacquire_candidate_color = None
                reacquire_count = 0


            if (
                phase == "REACQUIRE"
                and
                reacquire_start_time is not None
            ):

                elapsed_reacquire = (
                    time.monotonic()
                    - reacquire_start_time
                )


                if (
                    elapsed_reacquire
                    >= REACQUIRE_TIMEOUT
                ):

                    print()
                    print(
                        "ERROR:"
                    )

                    print(
                        "PLANNED",
                        planned_color,
                        "NOT REACQUIRED"
                    )

                    emergency_stop()

                    phase = "ERROR"


        # ====================================================
        # TRACK GREEN
        # ====================================================

        elif phase == "TRACK_GREEN":

            matched = match_tracked_object(
                tracked_object,
                objects,
                tracked_close_seen
            )


            if matched is not None:

                tracked_object = copy_object(
                    matched
                )

                tracked_lost_count = 0
                tracked_early_lost_count = 0


                (
                    command,
                    steering_mode,
                    error_cm,
                    target
                ) = calculate_green_steering(
                    tracked_object
                )


                send_command(
                    command
                )


                if (
                    tracked_object["h"]
                    >= GREEN_CLOSE_HEIGHT
                    or
                    tracked_object["bottom"]
                    >= GREEN_CLOSE_BOTTOM
                ):

                    if not tracked_close_seen:

                        print()
                        print(
                            "TRACKED GREEN CLOSE"
                        )

                        print(
                            "GREEN PHYSICAL LOCK ACTIVE"
                        )

                        print(
                            "OTHER GREEN CANNOT REPLACE IT"
                        )


                    tracked_close_seen = True


            else:

                if tracked_close_seen:

                    tracked_lost_count += 1

                    send_command(
                        CMD_LEFT_SOFT
                    )


                    print(
                        "TRACKED GREEN LOST:",
                        tracked_lost_count,
                        "/",
                        GREEN_LOST_CONFIRMATIONS,
                        "- OTHER GREEN IGNORED"
                    )


                    if (
                        tracked_lost_count
                        >= GREEN_LOST_CONFIRMATIONS
                    ):

                        print()
                        print(
                            "TRACKED GREEN OUT OF CAMERA"
                        )


                        send_command(
                            CMD_CENTER
                        )

                        time.sleep(
                            0.05
                        )


                        send_command(
                            CMD_MOTOR_SLOW
                        )

                        time.sleep(
                            GREEN_PASS_EXTRA_TIME
                        )


                        finish_current_obstacle()


                else:

                    tracked_early_lost_count += 1


                    if (
                        tracked_early_lost_count
                        >= GREEN_EARLY_LOST_LIMIT
                    ):

                        print()
                        print(
                            "ERROR:"
                        )

                        print(
                            "TRACKED GREEN LOST BEFORE CLOSE"
                        )


                        emergency_stop()

                        phase = "ERROR"


        # ====================================================
        # TRACK RED
        # ====================================================

        elif phase == "TRACK_RED":

            matched = match_tracked_object(
                tracked_object,
                objects,
                tracked_close_seen
            )


            if matched is not None:

                tracked_object = copy_object(
                    matched
                )

                tracked_lost_count = 0
                tracked_early_lost_count = 0


                (
                    command,
                    steering_mode,
                    error_cm,
                    target
                ) = calculate_red_steering(
                    tracked_object
                )


                send_command(
                    command
                )


                if (
                    tracked_object["h"]
                    >= RED_CLOSE_HEIGHT
                    or
                    tracked_object["bottom"]
                    >= RED_CLOSE_BOTTOM
                ):

                    if not tracked_close_seen:

                        print()
                        print(
                            "TRACKED RED CLOSE"
                        )

                        print(
                            "RED PHYSICAL LOCK ACTIVE"
                        )

                        print(
                            "OTHER RED CANNOT REPLACE IT"
                        )


                    tracked_close_seen = True


            else:

                if tracked_close_seen:

                    tracked_lost_count += 1

                    send_command(
                        CMD_RIGHT_SOFT
                    )


                    print(
                        "TRACKED RED LOST:",
                        tracked_lost_count,
                        "/",
                        RED_LOST_CONFIRMATIONS,
                        "- OTHER RED IGNORED"
                    )


                    if (
                        tracked_lost_count
                        >= RED_LOST_CONFIRMATIONS
                    ):

                        print()
                        print(
                            "TRACKED RED OUT OF CAMERA"
                        )


                        send_command(
                            CMD_CENTER
                        )

                        time.sleep(
                            0.05
                        )


                        send_command(
                            CMD_MOTOR_SLOW
                        )

                        time.sleep(
                            RED_PASS_EXTRA_TIME
                        )


                        finish_current_obstacle()


                else:

                    tracked_early_lost_count += 1


                    if (
                        tracked_early_lost_count
                        >= RED_EARLY_LOST_LIMIT
                    ):

                        print()
                        print(
                            "ERROR:"
                        )

                        print(
                            "TRACKED RED LOST BEFORE CLOSE"
                        )


                        emergency_stop()

                        phase = "ERROR"


        # ====================================================
        # O24 - APPROACH REAL END OF CORRIDOR
        # ====================================================

        elif phase == "APPROACH_TURN":

            steering_mode = "ULTRASONIC CENTERING + SLOW"

            # Vitesse lente pendant l'approche.
            send_command(CMD_MOTOR_SLOW)

            front_cm = read_front()
            left_cm = read_left()
            right_cm = read_right()

            elapsed_approach = (
                time.monotonic()
                - approach_turn_start_time
            )

            def fmt_distance(value):
                if value is None:
                    return "---"
                return f"{value:.1f}"

            # O25: centrage doux avec les deux capteurs latéraux.
            left_valid = (
                left_cm is not None
                and SIDE_MIN_VALID_CM <= left_cm <= SIDE_MAX_VALID_CM
            )
            right_valid = (
                right_cm is not None
                and SIDE_MIN_VALID_CM <= right_cm <= SIDE_MAX_VALID_CM
            )

            desired_direction = "CENTER"
            side_error_cm = None

            if left_valid and right_valid:
                # Positif: plus d'espace à droite -> robot trop à gauche.
                side_error_cm = right_cm - left_cm

                if side_error_cm > SIDE_CENTER_DEADBAND_CM:
                    desired_direction = "RIGHT"
                elif side_error_cm < -SIDE_CENTER_DEADBAND_CM:
                    desired_direction = "LEFT"

            if desired_direction == side_last_direction:
                side_direction_count += 1
            else:
                side_last_direction = desired_direction
                side_direction_count = 1

            if desired_direction == "CENTER":
                send_command(CMD_CENTER)
                steering_mode = "CENTER"

            elif side_direction_count >= SIDE_CORRECTION_CONFIRMATIONS:
                if desired_direction == "RIGHT":
                    send_command(CMD_RIGHT_SOFT)
                    steering_mode = "SOFT RIGHT - CENTERING"
                else:
                    send_command(CMD_LEFT_SOFT)
                    steering_mode = "SOFT LEFT - CENTERING"
            else:
                send_command(CMD_CENTER)
                steering_mode = "CENTER - WAIT SIDE CONFIRM"

            if side_error_cm is None:
                side_error_text = "---"
            else:
                side_error_text = f"{side_error_cm:.1f}"

            print(
                "APPROACH TURN |",
                "L:", fmt_distance(left_cm),
                "| F:", fmt_distance(front_cm),
                "| R:", fmt_distance(right_cm),
                "| SIDE ERR:", side_error_text,
                "| STEER:", steering_mode,
                "| TURN CONF:",
                approach_turn_confirm_count,
                "/",
                TURN_START_CONFIRMATIONS
            )

            # Le capteur avant décide toujours du début du virage.
            if (
                front_cm is not None
                and front_cm <= TURN_START_FRONT_CM
            ):
                approach_turn_confirm_count += 1
            else:
                approach_turn_confirm_count = 0

            if approach_turn_confirm_count >= TURN_START_CONFIRMATIONS:
                print()
                print("========================================")
                print(" FRONT 100 CM ZONE CONFIRMED")
                print(" FRONT:", f"{front_cm:.1f}", "cm")
                print(" STOP -> LEFT 90 DEG -> CENTER -> REVERSE TO REAR 40 CM")
                print("========================================")
                left_90_then_reverse_to_new_corridor()

            elif elapsed_approach >= APPROACH_TURN_TIMEOUT:
                print()
                print("ERROR: APPROACH TURN TIMEOUT")
                emergency_stop()
                phase = "ERROR"

            time.sleep(APPROACH_TURN_CHECK_DELAY)


        # ====================================================
        # TURN
        # ====================================================

        elif phase == "TURN":

            steering_mode = (
                "LEFT TURN L + M255"
            )


            # IMPORTANT:
            # On ne renvoie plus L/M à chaque boucle.
            # Ils ont déjà été envoyés par start_left_turn().

            front_cm = read_front()


            elapsed_turn = (
                time.monotonic()
                - turn_start_time
            )


            if (
                elapsed_turn
                < MIN_TURN_TIME
            ):

                turn_clear_count = 0


            else:

                if (
                    front_cm is not None
                    and
                    front_cm > FRONT_CLEAR
                ):

                    turn_clear_count += 1


                    print(
                        "TURN CLEAR:",
                        f"{front_cm:.1f}",
                        "cm",
                        turn_clear_count,
                        "/",
                        CLEAR_CONFIRMATIONS
                    )

                else:

                    turn_clear_count = 0


                if (
                    turn_clear_count
                    >= CLEAR_CONFIRMATIONS
                ):

                    start_backup()


            if (
                phase == "TURN"
                and
                elapsed_turn >= MAX_TURN_TIME
            ):

                print()
                print(
                    "ERROR: TURN TIMEOUT"
                )

                emergency_stop()

                phase = "ERROR"


            time.sleep(
                TURN_CHECK_DELAY
            )


        # ====================================================
        # BACKUP
        # ====================================================

        elif phase == "BACKUP":

            elapsed_backup = (
                time.monotonic()
                - backup_start_time
            )


            if (
                elapsed_backup
                >= BACKUP_TIME
            ):

                print()
                print(
                    "BACKUP COMPLETE"
                )

                prepare_new_scan()


        # ====================================================
        # DRAW OBJECTS
        # ====================================================

        for obj in ordered_objects[:5]:

            if obj["color"] == "GREEN":

                draw_color = (
                    0,
                    255,
                    0
                )

            else:

                draw_color = (
                    0,
                    0,
                    255
                )


            cv2.rectangle(
                frame,
                (
                    obj["x"],
                    obj["y"]
                ),
                (
                    obj["x"] + obj["w"],
                    obj["y"] + obj["h"]
                ),
                draw_color,
                2
            )


            cv2.putText(
                frame,
                (
                    obj["color"]
                    + " H:"
                    + str(obj["h"])
                ),
                (
                    obj["x"],
                    max(
                        25,
                        obj["y"] - 8
                    )
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                draw_color,
                2
            )


        # ====================================================
        # TRACKED OBJECT
        # ====================================================

        if tracked_object is not None:

            tx = tracked_object["x"]
            ty = tracked_object["y"]
            tw = tracked_object["w"]
            th = tracked_object["h"]


            cv2.rectangle(
                frame,
                (
                    tx - 5,
                    ty - 5
                ),
                (
                    tx + tw + 5,
                    ty + th + 5
                ),
                (
                    255,
                    255,
                    255
                ),
                4
            )


            cv2.putText(
                frame,
                "TRACKED",
                (
                    tx,
                    max(
                        25,
                        ty - 25
                    )
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                (
                    255,
                    255,
                    255
                ),
                2
            )


        # ====================================================
        # UI
        # ====================================================

        cv2.putText(
            frame,
            "O27 - PARKING + O26 AUTO",
            (25, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.70,
            (255, 255, 255),
            2
        )


        cv2.putText(
            frame,
            "PHASE: " + phase,
            (25, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2
        )


        if corridor_plan:

            plan_text = (
                "PLAN: "
                + " -> ".join(
                    [
                        p["color"]
                        for p in corridor_plan
                    ]
                )
            )

        else:

            plan_text = (
                "PLAN: scanning..."
            )


        cv2.putText(
            frame,
            plan_text,
            (25, 110),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 255, 255),
            2
        )


        if (
            corridor_plan
            and
            current_plan_index
            < len(corridor_plan)
        ):

            active_text = (
                "PLANNED: "
                + corridor_plan[
                    current_plan_index
                ]["color"]
                + " "
                + str(
                    current_plan_index + 1
                )
                + "/"
                + str(
                    len(corridor_plan)
                )
            )

        else:

            active_text = (
                "PLANNED: ---"
            )


        cv2.putText(
            frame,
            active_text,
            (25, 145),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 0),
            2
        )


        cv2.putText(
            frame,
            (
                "CLOSE LOCK: "
                + str(tracked_close_seen)
            ),
            (25, 180),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 0),
            2
        )


        if phase == "SCAN":

            if auto_start_after_scan:

                scan_mode_text = (
                    "SCAN -> AUTO START "
                    + str(
                        scan_confirmation_count
                    )
                    + "/"
                    + str(
                        SCAN_CONFIRMATIONS
                    )
                )

            else:

                scan_mode_text = (
                    "SCAN -> AUTO START "
                    + str(
                        scan_confirmation_count
                    )
                    + "/"
                    + str(
                        SCAN_CONFIRMATIONS
                    )
                )


            cv2.putText(
                frame,
                scan_mode_text,
                (25, 215),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2
            )


        elif phase == "REACQUIRE":

            cv2.putText(
                frame,
                (
                    "ACQUIRE "
                    + str(
                        reacquire_count
                    )
                    + "/"
                    + str(
                        REACQUIRE_CONFIRMATIONS
                    )
                ),
                (25, 215),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2
            )


        elif phase == "APPROACH_TURN":

            cv2.putText(
                frame,
                "APPROACH FRONT 100 CM",
                (25, 215),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2
            )


        elif phase == "BACKUP":

            cv2.putText(
                frame,
                "BACKUP BEFORE NEW SCAN",
                (25, 215),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2
            )


        cv2.imshow(
            "WRO O26",
            frame
        )


        # ====================================================
        # KEYBOARD
        # ====================================================

        key = (
            cv2.waitKey(1)
            & 0xFF
        )


        # ====================================================
        # SPACE - PREMIER COULOIR SEULEMENT
        # ====================================================

        if False:

            if len(corridor_plan) == 0:

                print(
                    "WAITING FOR STABLE SCAN..."
                )

            else:

                print()
                print(
                    "MANUAL START FIRST CORRIDOR"
                )

                start_frozen_corridor()


        # ====================================================
        # Q
        # ====================================================

        if key == ord("q"):

            print()
            print(
                "MANUAL EMERGENCY STOP"
            )

            emergency_stop()

            break


# ============================================================
# CTRL+C
# ============================================================

except KeyboardInterrupt:

    print()
    print(
        "CTRL+C"
    )

    emergency_stop()


# ============================================================
# ERROR
# ============================================================

except Exception as error:

    print()
    print(
        "ERROR:",
        error
    )

    emergency_stop()


# ============================================================
# CLEANUP
# ============================================================

finally:

    try:
        emergency_stop()
    except Exception:
        pass

    try:
        picam2.stop()
    except Exception:
        pass

    try:
        start_button.close()
        rear_sensor.close()
        right_sensor.close()
        left_sensor.close()
        front_sensor.close()
    except Exception:
        pass

    try:

        if esp32 is not None:
            esp32.close()

    except Exception:
        pass

    cv2.destroyAllWindows()

    print()
    print(
        "================================================"
    )

    print(
        " O26 STOPPED"
    )

    print(
        "================================================"
    )

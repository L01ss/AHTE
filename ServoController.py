from adafruit_servokit import ServoKit
import time

# This defines the servo controller boards and assigns the correct I2C adresses
servoController_L = ServoKit(channels=16, address=0x40)
servoController_R = ServoKit(channels=16, address=0x41)

# This sets the offset for each servo
ServoOffset = [
    (60, 40, 180),  # leg 0
    (45, 25, 180),  # leg 1
    (60, 40, 180),  # leg 2
    (60, 60, 0),  # leg 3
    (60, 90, 0),  # leg 4
    (60, 40, 0),  # leg 5
    ]

# This sets the Direction for each servo
ServoDir = [
    (1, 1, -1),  # leg 0
    (1, 1, -1),  # leg 1
    (1, 1, -1),  # leg 2
    (1, 1, 1),  # leg 3
    (1, 1, 1),  # leg 4
    (1, 1, 1),  # leg 5
]

def set_angle_with_retry(servo, angle, tries=3, delay=0.005):
    """
    set_angle_with_retry tries to set the angle of a servo, if it fails it will retry a few times with a delay in between, this prevents crashing

    :param servo: The servo object to set the angle on
    :param angle: The angle to set
    :param tries: The number of attempts
    :param delay: The delay between attempts
    :return: None
    """
    for i in range(tries):
        try:
            servo.angle = angle
            return
        except OSError as e:
            if i == tries - 1:
                raise
            time.sleep(delay)

def clamp(a):
    """
    clamp limits the input value to the range 0-180

    :param a: The input value to be clamped
    :return: The clamped value
    """
    return max(0, min(180, int(a)))

def detachServos():
    """
    detachServos detaches all the servos, this is used to prevent overheating and to allow manual movement of the legs
    :return: None
    """
    for i in range(16):
        set_angle_with_retry(servoController_L.servo[i], None)
        set_angle_with_retry(servoController_R.servo[i], None)

def updateLegServos(legID :int, angle1 :int, angle2 :int, angle3 :int):
    """
    updateLegServos sets the angles of the servos for a given leg
    it puts the values in the correct servo controller (left or right) and the correct servo

    :param legID: The ID-number of the leg (0-5)
    :param angle1: The angle for servo 1 of the leg
    :param angle2: The angle for servo 2 of the leg
    :param angle3: The angle for servo 3 of the leg

    :return: None
    """
    off1, off2, off3 = ServoOffset[legID]
    dir1, dir2, dir3 = ServoDir[legID]

    s1 = clamp(off1 + dir1 * angle1)
    s2 = clamp(off2 + dir2 * angle2)
    s3 = clamp(off3 + dir3 * angle3)

    print("update", legID, s1, s2, s3)

    # matches the legID to the correct servo controller and servos, then sets the angles with retry
    match legID:
        case 0:
            set_angle_with_retry(servoController_L.servo[0], s1)
            set_angle_with_retry(servoController_L.servo[1], s2)
            set_angle_with_retry(servoController_L.servo[2], s3)

        case 1:
            set_angle_with_retry(servoController_L.servo[4], s1)
            set_angle_with_retry(servoController_L.servo[5], s2)
            set_angle_with_retry(servoController_L.servo[6], s3)

        case 2:
            set_angle_with_retry(servoController_L.servo[15], s1)
            set_angle_with_retry(servoController_L.servo[14], s2)
            set_angle_with_retry(servoController_L.servo[13], s3)

        case 3:
            set_angle_with_retry(servoController_R.servo[15], s1)
            set_angle_with_retry(servoController_R.servo[14], s2)
            set_angle_with_retry(servoController_R.servo[13], s3)

        case 4:
            set_angle_with_retry(servoController_R.servo[11], s1)
            set_angle_with_retry(servoController_R.servo[10], s2)
            set_angle_with_retry(servoController_R.servo[9], s3)

        case 5:
            set_angle_with_retry(servoController_R.servo[0], s1)
            set_angle_with_retry(servoController_R.servo[1], s2)
            set_angle_with_retry(servoController_R.servo[2], s3)
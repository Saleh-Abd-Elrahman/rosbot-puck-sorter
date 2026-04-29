#!/usr/bin/env python3
"""
Approach a red puck and grasp it with the servo gripper.
States: SEARCH -> APPROACH -> COMMIT -> GRASP -> DONE
  SEARCH:   spin in place until a red puck is found
  APPROACH: P-controller centres puck and drives forward
  COMMIT:   open-loop forward drive (puck leaves the frame at close range)
  GRASP:    stop, close gripper, wait for servo to settle
"""
import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16
import cv2
import numpy as np

RED_RANGES = [
    (np.array([0,   120, 70]),  np.array([10,  255, 255])),
    (np.array([170, 120, 70]),  np.array([180, 255, 255])),
]
MIN_AREA       = 300
AIM_OFFSET_PX  = 50      # +ve = aim point right of image centre (gripper sits left of camera)
COMMIT_RADIUS    = 45    # px — visible tip of puck reaches this size; commit and coast forward
COMMIT_LOST_MIN_R = 38   # px — min last-seen radius before "puck disappeared" triggers commit
                          # (must be close to COMMIT_RADIUS so far pucks never trigger commit)
KP_ANGULAR     = 0.005
KP_LINEAR      = 0.003
APPROACH_BASE  = 0.06    # m/s base forward during APPROACH
MAX_LINEAR     = 0.15
MAX_ANGULAR    = 0.8
COMMIT_SPEED   = 0.08    # m/s open-loop forward after losing puck
COMMIT_TIME    = 7.0    # s — coast forward to seat puck inside gripper (~6 cm)
GRASP_SETTLE   = 1.5     # s wait for servo to finish closing

GRIPPER_OPEN   = 0
GRIPPER_CLOSE  = 170

SEARCH, APPROACH, COMMIT, GRASP, DONE = range(5)
STATE_NAMES = ["SEARCH", "APPROACH", "COMMIT", "GRASP", "DONE"]

state       = SEARCH
state_t     = None
last_radius = 0
cmd_pub     = None
servo_pub   = None
last        = None


def find_largest_red(frame):
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in RED_RANGES:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_AREA:
            continue
        if best is None or area > cv2.contourArea(best):
            best = cnt
    return best


def set_state(new_state):
    global state, state_t
    if new_state != state:
        rospy.loginfo("state: %s -> %s", STATE_NAMES[state], STATE_NAMES[new_state])
    state = new_state
    state_t = rospy.Time.now()


def cb(msg):
    global last, last_radius
    arr   = np.frombuffer(msg.data, dtype=np.uint8)
    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if frame is None:
        return

    h, w    = frame.shape[:2]
    cx_img  = w // 2
    aim_x   = cx_img + AIM_OFFSET_PX
    twist   = Twist()
    vis     = frame.copy()

    cnt = find_largest_red(frame)
    cx = cy = 0
    radius = 0
    if cnt is not None:
        (fx, fy), fr = cv2.minEnclosingCircle(cnt)
        cx, cy, radius = int(fx), int(fy), int(fr)
        last_radius = radius
        cv2.circle(vis, (cx, cy), radius, (0, 0, 255), 2)
        cv2.line(vis, (aim_x, cy), (cx, cy), (0, 255, 0), 2)

    if state == SEARCH:
        if cnt is not None:
            set_state(APPROACH)
        else:
            twist.angular.z = 0.3

    elif state == APPROACH:
        if cnt is None:
            if last_radius >= COMMIT_LOST_MIN_R:
                set_state(COMMIT)
            else:
                last_radius = 0
                set_state(SEARCH)
        elif radius >= COMMIT_RADIUS:
            set_state(COMMIT)
        else:
            heading_err = cx - aim_x
            twist.angular.z = float(np.clip(-KP_ANGULAR * heading_err,
                                             -MAX_ANGULAR, MAX_ANGULAR))
            twist.linear.x  = float(np.clip(APPROACH_BASE + KP_LINEAR * (COMMIT_RADIUS - radius),
                                             0.0, MAX_LINEAR))

    elif state == COMMIT:
        if (rospy.Time.now() - state_t).to_sec() < COMMIT_TIME:
            twist.linear.x = COMMIT_SPEED
        else:
            servo_pub.publish(UInt16(data=GRIPPER_CLOSE))
            set_state(GRASP)

    elif state == GRASP:
        servo_pub.publish(UInt16(data=GRIPPER_CLOSE))   # resend each tick — rosserial drops single publishes
        if (rospy.Time.now() - state_t).to_sec() >= GRASP_SETTLE:
            set_state(DONE)

    cmd_pub.publish(twist)

    cv2.line(vis, (cx_img, 0), (cx_img, h), (120, 120, 0), 1)   # camera centre
    cv2.line(vis, (aim_x,  0), (aim_x,  h), (255, 255, 0), 1)   # gripper aim point
    cv2.putText(vis, f"{STATE_NAMES[state]}  r={radius}px  last_r={last_radius}",
                (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    last = vis


def main():
    global cmd_pub, servo_pub, state_t
    rospy.init_node("grab_red_puck")
    cmd_pub   = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    servo_pub = rospy.Publisher("/servo", UInt16, queue_size=1, latch=True)
    rospy.Subscriber("/camera/color/image_2fps/compressed",
                     CompressedImage, cb, queue_size=1)

    rospy.sleep(0.5)
    servo_pub.publish(UInt16(data=GRIPPER_OPEN))
    state_t = rospy.Time.now()
    rospy.loginfo("grab_red_puck started — Ctrl+C to stop")

    while not rospy.is_shutdown():
        if last is not None:
            cv2.imshow("grab_red_puck", last)
        if cv2.waitKey(50) & 0xFF == ord('q'):
            break

    cmd_pub.publish(Twist())
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

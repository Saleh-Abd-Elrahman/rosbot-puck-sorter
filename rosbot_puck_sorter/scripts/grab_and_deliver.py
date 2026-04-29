#!/usr/bin/env python3
"""
Grab a red puck, then drive to an ArUco tag (id 1) and release.
Pipeline: SEARCH_PUCK -> APPROACH_PUCK -> COMMIT -> GRASP
       -> SEARCH_TAG  -> APPROACH_TAG  -> RELEASE -> BACKUP -> DONE

Imports the puck detector + tuning constants from grab_red_puck.py.
"""
import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16
import cv2
import numpy as np

from grab_red_puck import (
    find_largest_red,
    AIM_OFFSET_PX,
    KP_ANGULAR, KP_LINEAR, APPROACH_BASE,
    MAX_LINEAR, MAX_ANGULAR,
    COMMIT_RADIUS, COMMIT_LOST_MIN_R, COMMIT_SPEED, COMMIT_TIME,
    GRASP_SETTLE,
    GRIPPER_OPEN, GRIPPER_CLOSE,
)

ARUCO_TARGET_ID    = 1
ARUCO_DICT_NAME    = cv2.aruco.DICT_4X4_50
TAG_TARGET_SIZE_PX = 180      # release distance — tune with the script's overlay
TAG_KP_ANGULAR     = 0.005
TAG_KP_LINEAR      = 0.002
TAG_APPROACH_BASE  = 0.06
TAG_SEARCH_OMEGA   = 0.4
RELEASE_BACKUP_T   = 1.0      # s
RELEASE_BACKUP_V   = -0.06    # m/s

(SEARCH_PUCK, APPROACH_PUCK, COMMIT, GRASP,
 SEARCH_TAG, APPROACH_TAG, RELEASE, BACKUP, DONE) = range(9)
STATE_NAMES = ["SEARCH_PUCK", "APPROACH_PUCK", "COMMIT", "GRASP",
               "SEARCH_TAG", "APPROACH_TAG", "RELEASE", "BACKUP", "DONE"]

state       = SEARCH_PUCK
state_t     = None
last_radius = 0
cmd_pub     = None
servo_pub   = None
last        = None

aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_NAME)
try:
    _aruco_params = cv2.aruco.DetectorParameters()
    _aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, _aruco_params)
    def _detect_aruco(gray):
        return _aruco_detector.detectMarkers(gray)
except AttributeError:
    _aruco_params = cv2.aruco.DetectorParameters_create()
    def _detect_aruco(gray):
        return cv2.aruco.detectMarkers(gray, aruco_dict, parameters=_aruco_params)


def find_target_tag(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = _detect_aruco(gray)
    if ids is None:
        return None
    for i, mid in enumerate(ids.flatten()):
        if int(mid) == ARUCO_TARGET_ID:
            pts = corners[i].reshape(-1, 2)
            cx = int(pts[:, 0].mean())
            cy = int(pts[:, 1].mean())
            sides = [np.linalg.norm(pts[j] - pts[(j + 1) % 4]) for j in range(4)]
            return cx, cy, int(max(sides)), pts.astype(np.int32)
    return None


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

    h, w   = frame.shape[:2]
    cx_img = w // 2
    aim_x  = cx_img + AIM_OFFSET_PX
    twist  = Twist()
    vis    = frame.copy()
    text   = STATE_NAMES[state]

    if state in (SEARCH_PUCK, APPROACH_PUCK, COMMIT, GRASP):
        cnt = find_largest_red(frame)
        cx = cy = 0
        radius = 0
        if cnt is not None:
            (fx, fy), fr = cv2.minEnclosingCircle(cnt)
            cx, cy, radius = int(fx), int(fy), int(fr)
            last_radius = radius
            cv2.circle(vis, (cx, cy), radius, (0, 0, 255), 2)

        if state == SEARCH_PUCK:
            if cnt is not None:
                set_state(APPROACH_PUCK)
            else:
                twist.angular.z = 0.3

        elif state == APPROACH_PUCK:
            if cnt is None:
                if last_radius >= COMMIT_LOST_MIN_R:
                    set_state(COMMIT)
                else:
                    last_radius = 0
                    set_state(SEARCH_PUCK)
            elif radius >= COMMIT_RADIUS:
                set_state(COMMIT)
            else:
                heading_err = cx - aim_x
                twist.angular.z = float(np.clip(-KP_ANGULAR * heading_err, -MAX_ANGULAR, MAX_ANGULAR))
                twist.linear.x  = float(np.clip(APPROACH_BASE + KP_LINEAR * (COMMIT_RADIUS - radius),
                                                 0.0, MAX_LINEAR))

        elif state == COMMIT:
            if (rospy.Time.now() - state_t).to_sec() < COMMIT_TIME:
                twist.linear.x = COMMIT_SPEED
            else:
                servo_pub.publish(UInt16(data=GRIPPER_CLOSE))
                set_state(GRASP)

        elif state == GRASP:
            servo_pub.publish(UInt16(data=GRIPPER_CLOSE))
            if (rospy.Time.now() - state_t).to_sec() >= GRASP_SETTLE:
                set_state(SEARCH_TAG)

        text += f"  r={radius}px"

    elif state in (SEARCH_TAG, APPROACH_TAG):
        tag = find_target_tag(frame)
        if tag is not None:
            tcx, tcy, size, pts = tag
            cv2.polylines(vis, [pts], True, (0, 255, 0), 2)
            cv2.circle(vis, (tcx, tcy), 4, (0, 255, 0), -1)

        if state == SEARCH_TAG:
            if tag is not None:
                set_state(APPROACH_TAG)
            else:
                twist.angular.z = TAG_SEARCH_OMEGA

        elif state == APPROACH_TAG:
            if tag is None:
                set_state(SEARCH_TAG)
            elif size >= TAG_TARGET_SIZE_PX:
                set_state(RELEASE)
            else:
                heading_err = tcx - aim_x
                twist.angular.z = float(np.clip(-TAG_KP_ANGULAR * heading_err, -MAX_ANGULAR, MAX_ANGULAR))
                twist.linear.x  = float(np.clip(TAG_APPROACH_BASE + TAG_KP_LINEAR * (TAG_TARGET_SIZE_PX - size),
                                                 0.0, MAX_LINEAR))

        text += f"  tag_size={tag[2]}px" if tag is not None else "  no tag"

    elif state == RELEASE:
        servo_pub.publish(UInt16(data=GRIPPER_OPEN))
        if (rospy.Time.now() - state_t).to_sec() >= GRASP_SETTLE:
            set_state(BACKUP)

    elif state == BACKUP:
        if (rospy.Time.now() - state_t).to_sec() < RELEASE_BACKUP_T:
            twist.linear.x = RELEASE_BACKUP_V
        else:
            set_state(DONE)

    cmd_pub.publish(twist)

    cv2.line(vis, (cx_img, 0), (cx_img, h), (120, 120, 0), 1)
    cv2.line(vis, (aim_x,  0), (aim_x,  h), (255, 255, 0), 1)
    cv2.putText(vis, text, (10, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    last = vis


def main():
    global cmd_pub, servo_pub, state_t
    rospy.init_node("grab_and_deliver")
    cmd_pub   = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    servo_pub = rospy.Publisher("/servo", UInt16, queue_size=1, latch=True)
    rospy.Subscriber("/camera/color/image_2fps/compressed",
                     CompressedImage, cb, queue_size=1)

    rospy.sleep(0.5)
    servo_pub.publish(UInt16(data=GRIPPER_OPEN))
    state_t = rospy.Time.now()
    rospy.loginfo("grab_and_deliver started — Ctrl+C to stop")

    while not rospy.is_shutdown():
        if last is not None:
            cv2.imshow("grab_and_deliver", last)
        if cv2.waitKey(50) & 0xFF == ord('q'):
            break

    cmd_pub.publish(Twist())
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

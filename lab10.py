#!/usr/bin/env python3 
import rospy
import cv2
import math
import numpy as np
import time
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from puppy_control.msg import Pose, Gait, Velocity
from puppy_control.srv import SetRunActionName
from threading import Lock, Thread

ROS_NODE_NAME = "we_ball_node"

lock = Lock()

shut = False

max_contour = None
center = None
radius = 0

pose_pub = None
gait_pub = None
vel_pub = None
action_srv = None

box_top_left = (380, 180)
box_bot_right = (4700, 300)

pose_msg = Pose(roll=math.radians(0), pitch=math.radians(0), yaw=0, height=-10, x_shift=0, stance_x=0, stance_y=0, run_time=500)
gait_msg = Gait(overlap_time=0.15, swing_time=0.15, clearance_time=0.0, z_clearance=3)
vel_msg = Velocity(x=0, y=0, yaw_rate=math.radians(0))

def filterColor(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    mask = cv2.inRange(img, np.array([0, 153, 88]), np.array([255, 255, 255]))
    mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
    mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
    return mask

def getMaxContour(mask, thresh=50):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    max_area = 0
    max_contour = None
    for c in contours:
        c_area = math.fabs(cv2.contourArea(c))
        if c_area > thresh:
            if c_area > max_area:
                max_area = c_area
                max_contour = c
    return max_contour

def drawCircularBound(img, c):
    center, radius = cv2.minEnclosingCircle(c)
    radius -= 5 
    img = cv2.circle(img, (int(center[0]), int(center[1])), int(radius), (255, 0, 0), thickness = 2)
    return img, (int(center[0]), int(center[1])), int(radius)

def img_process(img):
    global pose_pub, pose_msg
    global max_contour, center, radius
    frame = np.ndarray(shape=(img.height, img.width, 3), dtype=np.uint8, buffer=img.data)
    cv2_img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    mask = filterColor(cv2_img)
    with lock:
        max_contour = getMaxContour(mask)
        if max_contour is not None:
            cv2_img, center, radius = drawCircularBound(cv2_img, max_contour)
            cv2_img = cv2.circle(cv2_img, center, 3, (0, 0, 255), thickness = 3)
    cv2_img = cv2.rectangle(cv2_img, box_top_left, box_bot_right, (255, 255, 255))

    cv2.imshow("Frame", cv2_img)
    cv2.waitKey(1)

def cleanup():
    with lock:
        shut = True
    rospy.loginfo("Shutting down...")
    cv2.destroyWindow("Frame")
    time.sleep(0.5)
    rospy.ServiceProxy("/puppy_control/go_home", Empty)()

def move():
    global max_contour, center, radius
    global pose_pub, vel_pub, gait_pub, action_srv
    global shut, lock
    should_kick = False
    while True:
        time.sleep(0.2)
        send_pose = False
        send_vel = False
        send_yaw = False
        prev_yaw_rate = vel_msg.yaw_rate
        prev_vel = vel_msg.x
        prev_pitch = pose_msg.pitch
        c = None
        r = None
        if shut:
            break
        with lock:
            if max_contour is not None:
                c = max_contour
                r = radius
        if c is not None:
            if not should_kick:
                if radius < 50:
                    if center[1] < box_top_left[1]:
                        if pose_msg.pitch < math.radians(20):
                            pose_msg.pitch += math.radians(2)
                            send_pose = True
                    elif center[1] > box_bot_right[1]:
                        if pose_msg.pitch > math.radians(-20):
                            pose_msg.pitch -= math.radians(2)
                            send_pose = True
                else:
                    pose_msg.pitch = math.radians(-20)
                    send_pose = prev_pitch != pose_msg.pitch
                
                if center[0] < box_top_left[0]:
                    vel_msg.yaw_rate = math.radians(8)
                    send_yaw = True
                elif center[0] > box_bot_right[0]:
                    vel_msg.yaw_rate = math.radians(-8)
                    send_yaw = True
                else:
                    vel_msg.yaw_rate = math.radians(0)
                    send_yaw = prev_yaw_rate != vel_msg.yaw_rate
                
                if radius < 50:
                    vel_msg.x = 10
                    send_vel = True
                elif 50 <= radius < 100:
                    vel_msg.x = 7 
                    send_vel = True
                else:
                    vel_msg.x = 0
                    should_kick = True
                    send_vel = prev_vel != vel_msg.x
            else:
                # do the kick
                vel_msg.x = 7
                vel_msg.yaw_rate = math.radians(7)
                vel_pub.publish(vel_msg)
                time.sleep(1)

                vel_msg.x = 0
                vel_msg.yaw_rate = math.radians(0)
                vel_pub.publish(vel_msg)
                time.sleep(0.1)


                action_srv("kick_ball_right.d6ac", True)
                should_kick = False
                time.sleep(1)
        else:
            pose_msg.pitch = math.radians(0)
            vel_msg.yaw_rate = math.radians(0)
            vel_msg.x = 0        
            send_vel = prev_yaw_rate != vel_msg.yaw_rate
            send_pose = True

        if send_pose and pose_pub is not None:
            pose_pub.publish(pose_msg)
        if (not send_pose and (send_vel or send_yaw)) and vel_pub is not None:
            vel_pub.publish(vel_msg)
            time.sleep(1)

if __name__ == "__main__":
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.INFO)
    rospy.on_shutdown(cleanup)
    rospy.ServiceProxy("/puppy_control/go_home", Empty)()
    pose_pub = rospy.Publisher("/puppy_control/pose", Pose, queue_size=1)
    vel_pub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)
    gait_pub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    action_srv = rospy.ServiceProxy("/puppy_control/runActionGroup", SetRunActionName) 
    gait_pub.publish(gait_msg)
    th = Thread(target=move, daemon=True)
    th.start()
    rospy.Subscriber("/usb_cam/image_raw", Image, img_process)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


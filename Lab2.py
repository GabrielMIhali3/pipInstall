#!/usr/bin/env python3

import rospy
from sensor.msg import Led, RGB
ROS_NODE_NAME = "my_ros_node"
led_pub=None

def changeColor():
    global led_pub
    led_pub=rospy.Publisher("/sensor/rgb_led",Led,queue_size=1)
    colors=[(255,0,0),(255,100,0),(255,255,0),(0,255,0),(0,0,255),(75,0,130),(148,0,211)]
    size=len(colors)
    i=0
    rate=rospy.Rate(1)
    led=Led()
    led.index=0
    while not rospy.is_shutdown():
        print("merge")
        led.rgb.r=colors[i][0]
        led.rgb.g=colors[i][1]
        led.rgb.b=colors[i][2]
        led_pub.publish(led)
        i=(i+1)%size
        rate.sleep()


def workFunction():
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rospy.loginfo("Test")
        rate.sleep()

def cleanup():
    global led_pub
    led=Led(0,RGB(0,0,0))
    if led_pub!=None:
        led_pub.publish(led)
    rospy.loginfo("Shutting down...")

if __name__=='__main__':
    
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.INFO)
    rospy.on_shutdown(cleanup)
    try:
        changeColor()
    except KeyboardInterrupt:
        pass

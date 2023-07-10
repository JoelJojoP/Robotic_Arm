#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import math

r = 0
z = 120
g = 0
t = 0

def rvalue_callback(msg):
    global r
    r = msg.data

def zvalue_callback(msg):
    global z
    z = msg.data

def gvalue_callback(msg):
    global g
    g = msg.data

def tvalue_callback(msg):
    global t
    t = msg.data

def main():
    global r, z, g, t
    rospy.init_node('arm_controller_w_arduino', anonymous=True)

    rvalue_sub = rospy.Subscriber('/rvalue', Float64, rvalue_callback)
    zvalue_sub = rospy.Subscriber('/zvalue', Float64, zvalue_callback)
    gvalue_sub = rospy.Subscriber('/gvalue', Float64, gvalue_callback)
    tvalue_sub = rospy.Subscriber('/tvalue', Float64, tvalue_callback)

    joint1_pub = rospy.Publisher('/joint_controller/joint1_position_controller/command', Float64, queue_size=10)
    joint2_pub = rospy.Publisher('/joint_controller/joint2_position_controller/command', Float64, queue_size=10)
    joint3_pub = rospy.Publisher('/joint_controller/joint3_position_controller/command', Float64, queue_size=10)
    joint5_pub = rospy.Publisher('/joint_controller/joint5_position_controller/command', Float64, queue_size=10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        theta3 = math.acos((r**2 + z**2 - 23850)/20250)
        theta2 = math.pi/2 - math.atan2(z, r) - math.atan2(75*math.sin(theta3), 135 + 75*math.cos(theta3))

        joint1_pub.publish(math.radians(t))
        joint2_pub.publish(theta2)
        joint3_pub.publish(theta3)
        joint5_pub.publish(math.radians(g))
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
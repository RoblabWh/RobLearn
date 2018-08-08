#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def callback(data):
    turbo = 0.5
    if data.buttons[7] == 1:
        turbo = 1

    twist = Twist()
    twist.linear.x = turbo*data.axes[1]
    twist.angular.z = turbo*2.5*data.axes[2]
    pub.publish(twist)

def start():

    rospy.init_node('teleop_joy')

    global pub
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist)
    rospy.Subscriber("joy", Joy, callback)

    rospy.spin()

if __name__ == '__main__':
    start()

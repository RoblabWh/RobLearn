#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

scan_realsense = None
publisher_scan_fusion = None

RANGE_OFFFSET = 0.03 # in Meter

def callback_scan_realsense(data):
    global scan_realsense
    scan_realsense = data

def callback_scan_hokuyo(scan_hokuyo):

    if scan_realsense is not None:

        scan_hokuyo_ranges = list(scan_hokuyo.ranges)

        index_hokuyo_start = int(len(scan_hokuyo.ranges) - len(scan_realsense.ranges)) / 2


        for index_realsense in range(len(scan_realsense.ranges)):
            index_hokuyo = index_hokuyo_start + index_realsense
            range_hokuyo = scan_hokuyo.ranges[index_hokuyo]
            range_realsense = scan_realsense.ranges[index_realsense] - RANGE_OFFFSET

            if range_realsense < scan_realsense.range_max and range_realsense < range_hokuyo:
                scan_hokuyo_ranges[index_hokuyo] = range_realsense

        scan_hokuyo.ranges = scan_hokuyo_ranges

    publisher_scan_fusion.publish(scan_hokuyo)


def start():

    rospy.init_node('scan_fusion')

    global publisher_scan_fusion
    publisher_scan_fusion = rospy.Publisher('scan', LaserScan, queue_size=1)
    rospy.Subscriber("scan_hokuyo", LaserScan, callback_scan_hokuyo)
    rospy.Subscriber("scan_realsense", LaserScan, callback_scan_realsense)

    rospy.spin()

if __name__ == '__main__':
    start()

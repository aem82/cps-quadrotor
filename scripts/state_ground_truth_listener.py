#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Header

global headerTime
global twist

def isTwistChanged(twist2):
    global twist
    if abs(twist.linear.x - twist2.linear.x) > 0.4:
        return True
    if abs(twist.linear.y - twist2.linear.y) > 0.4:
        return True
    if abs(twist.linear.z - twist2.linear.z) > 0.4:
        return True
    return False

def callback(odo):
    global headerTime
    global twist
    if isTwistChanged(odo.twist.twist):
        rospy.loginfo('')
        print(headerTime)
        print(twist)
        print('----')
        twist = odo.twist.twist
        headerTime = odo.header.stamp.nsecs
        print(headerTime)
        print(twist)
    else:
        headerTime = odo.header.stamp.nsecs

def listener():

    global headerTime
    global twist
    twist = Twist()

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('state_ground_truth_listener', anonymous=True)

    rospy.Subscriber("/ground_truth/state", Odometry, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

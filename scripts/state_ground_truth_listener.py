#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Header

global headerTime
global twist
global twistLinear
global pose

def isTwistChanged(twist2):
    global twist
    delta = 0.1
    if abs(twist.linear.x - twist2.linear.x) > delta:
        return True
    if abs(twist.linear.y - twist2.linear.y) > delta:
        return True
    if abs(twist.linear.z - twist2.linear.z) > delta:
        return True
    return False

def callback(odo):
    global headerTime
    global twist
    global twistLinear
    global pose
    if isTwistChanged(odo.twist.twist):
        rospy.loginfo('')
        print('Time: %ds %dns' % (headerTime[0], headerTime[1]))
        print('Pose %.2f %.2f %.2f' % (pose.x, pose.y, pose.z))
        # print(twist)
        print('Twist %.2f %.2f %.2f' % (twistLinear.x, twistLinear.y, twistLinear.z))
        print('----')
        twist = odo.twist.twist
        twistLinear = twist.linear
        pose = odo.pose.pose.position
        headerTime = (odo.header.stamp.secs, odo.header.stamp.nsecs)
        print(headerTime)
        print('Pose %.2f %.2f %.2f' % (pose.x, pose.y, pose.z))
        # print(twist)
        print('Twist %.2f %.2f %.2f' % (twistLinear.x, twistLinear.y, twistLinear.z))
        print('')
    else:
        headerTime = (odo.header.stamp.secs, odo.header.stamp.nsecs)

def listener():

    global headerTime
    global twist
    global twistLinear
    global pose
    headerTime = (0, 0)
    twist = Twist()
    twistLinear = twist.linear
    pose = Point()

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

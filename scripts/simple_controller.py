#!/usr/bin/env python

import rospy
import std_msgs.msg
from geometry_msgs.msg import Twist, TwistStamped
#from std_msgs.msg import Header, TwistStamped

def simple_controller():
    pub = rospy.Publisher('/command/twist', TwistStamped, queue_size=10)
    rospy.init_node('simple_controller', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
	h = std_msgs.msg.Header()
	h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
	h.frame_id = rospy.get_param('base_stabilized_frame', 'base_stabilized')
	msg = TwistStamped()
	msg.header = h
	msg.twist.linear.x = 3.0
	msg.twist.linear.y = 0.0
	msg.twist.linear.z = 0.0
	msg.twist.angular.x = 0.0
	msg.twist.angular.y = 0.0
	msg.twist.angular.z = 0.0
        #rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
        #rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
    pub.publish(msg)
    rate.sleep()

if __name__ == '__main__':
    try:
        simple_controller()
    except rospy.ROSInterruptException:
        pass

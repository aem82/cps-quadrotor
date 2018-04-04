#!/usr/bin/env python
#import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import Twist, TwistStamped
from hector_uav_msgs.srv import EnableMotors

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
  w:	 +y
  s:	 -y
  d:	 +x
  a:	 -x
  e:	 +z
  c:	 -z

anything else : stop

i/o : increase/decrease max speeds by 10%
k/l : increase/decrease only linear speed by 10%
,/. : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
		'w':(0,1,0,0),
		's':(0,-1,0,0),
		'd':(1,0,0,0),
		'a':(-1,0,0,0),
		'e':(0,0,1,0),
		'c':(0,0,-1,0),
	       }

speedBindings={
		'o':(1.1,1.1),
		'i':(.9,.9),
		'l':(1.1,1),
		'k':(.9,1),
		',':(1,1.1),
		'.':(1,.9),
	      }

def disableMotors():
    SERVICE_ENABLE_MOTORS = 'enable_motors'
    #print "Waiting for service", SERVICE_ENABLE_MOTORS
    #rospy.wait_for_service(SERVICE_ENABLE_MOTORS)
    try:
        enable_motors = rospy.ServiceProxy(SERVICE_ENABLE_MOTORS, EnableMotors)
        res = enable_motors(False)
        if res:
            print "Motors disabled!"
        else:
            print "Failed to disable motors..."
    except rospy.ServiceException, e:
        print "Disable service", SERVICE_ENABLE_MOTORS, "call failed: %s"%e

def enableMotors():
    SERVICE_ENABLE_MOTORS = 'enable_motors'
    print "Waiting for service", SERVICE_ENABLE_MOTORS
    rospy.wait_for_service(SERVICE_ENABLE_MOTORS)
    try:
        enable_motors = rospy.ServiceProxy(SERVICE_ENABLE_MOTORS, EnableMotors)
        res = enable_motors(True)
        if res:
            print "Motors enabled!"
        else:
            print "Failed to enable motors..."
    except rospy.ServiceException, e:
        print "Enable service", SERVICE_ENABLE_MOTORS, "call failed: %s"%e


def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

speed = .5
turn = 1

def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":

    enableMotors()

    BASE_STABILIZED_FRAME = rospy.get_param('base_stabilized_frame', 'stabilized_frame')
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('/command/twist', TwistStamped, queue_size = 1)
    rospy.init_node('teleop_twist_keyboard')

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
    	print msg
    	print vels(speed,turn)
    	while(1):
    		key = getKey()
    		header = Header()
    		header.stamp = rospy.Time.now()
    		header.frame_id = BASE_STABILIZED_FRAME
    		if key in moveBindings.keys():
    			x = moveBindings[key][0]
    			y = moveBindings[key][1]
    			z = moveBindings[key][2]
    			th = moveBindings[key][3]
    		elif key in speedBindings.keys():
    			speed = speed * speedBindings[key][0]
    			turn = turn * speedBindings[key][1]

    			print vels(speed,turn)
    			if (status == 14):
    				print msg
    			status = (status + 1) % 15
    		else:
    			x = 0
    			y = 0
    			z = 0
    			th = 0
    			if (key == '\x03'):
    				break

    		twistStamped = TwistStamped()
    		twistStamped.header = header
    		twist = Twist()
    		twist.linear.x = x*speed
    		twist.linear.y = y*speed
    		twist.linear.z = z*speed
    		twist.angular.x = 0
    		twist.angular.y = 0
    		twist.angular.z = th*turn
    		twistStamped.twist = twist
    		pub.publish(twistStamped)

    except:
    	print e

    finally:
    	header = Header()
    	header.stamp = rospy.Time.now()
    	header.frame_id = BASE_STABILIZED_FRAME
    	twistStamped = TwistStamped()
    	twistStamped.header = header
    	twist = Twist()
    	twist.linear.x = 0
    	twist.linear.y = 0
    	twist.linear.z = 0
    	twist.angular.x = 0
    	twist.angular.y = 0
    	twist.angular.z = 0
    	twistStamped.twist = twist
    	pub.publish(twistStamped)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    disableMotors()

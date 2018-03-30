#!/usr/bin/env python
#import roslib; roslib.load_manifest('teleop_pose_keyboard')
import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped
from hector_uav_msgs.srv import EnableMotors
from nav_msgs.msg import Odometry

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

global poseX
global poseY
global poseZ
global quatW

def stateCb(data):
    global poseX
    global poseY
    global poseZ
    global quatW
    poseX = data.pose.pose.position.x
    poseY = data.pose.pose.position.y
    poseZ = data.pose.pose.position.z
    quatW = data.pose.pose.orientation.x

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
    global poseX
    global poseY
    global poseZ
    global quatW

    enableMotors()

    WORLD_FRAME = rospy.get_param('world_frame', '/world')
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('/command/pose', PoseStamped, queue_size = 1)
    rospy.init_node('teleop_pose_keyboard')

    x = poseX = 0
    y = poseY = 0
    z = poseZ = 0
    w = quatW = 1
    status = 0

    rospy.Subscriber('/ground_truth/state', Odometry, stateCb)

    try:
        print msg
        print vels(speed,turn)
        while(1):
            key = getKey()
            if key in moveBindings.keys():
            	x = poseX + (moveBindings[key][0]*speed)
            	y = poseY + (moveBindings[key][1]*speed)
            	z = poseZ + (moveBindings[key][2]*speed)
            	w = 1
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
            	w = 1
            	if (key == '\x03'):
            		break

            poseStamped = PoseStamped()
            poseStamped.pose = Pose()
            poseStamped.pose.position.x = x
            poseStamped.pose.position.y = y
            poseStamped.pose.position.z = z
            poseStamped.pose.orientation.w = w
            poseStamped.header = Header()
            poseStamped.header.frame_id = WORLD_FRAME
            poseStamped.header.stamp = rospy.Time.now()
            pub.publish(poseStamped)

    except:
        print e

    # finally:
    # 	header = Header()
    # 	header.stamp = rospy.Time.now()
    # 	header.frame_id = BASE_STABILIZED_FRAME
    # 	twistStamped = TwistStamped()
    # 	twistStamped.header = header
    # 	twist = Twist()
    # 	twist.linear.x = 0
    # 	twist.linear.y = 0
    # 	twist.linear.z = 0
    # 	twist.angular.x = 0
    # 	twist.angular.y = 0
    # 	twist.angular.z = 0
    # 	twistStamped.twist = twist
    # 	pub.publish(twistStamped)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    disableMotors()

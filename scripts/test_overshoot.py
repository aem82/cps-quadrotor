#!/usr/bin/env python
import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, Point
from hector_uav_msgs.srv import EnableMotors
from nav_msgs.msg import Odometry

import sys, select, termios, tty, time, datetime, math

CPS_DIR = "/home/saeed/cps_ws/src/cps_quadrotor/"

msg = """
Reading from the keyboard and Publishing to Pose!
---------------------------
Moving around:
  w:	 +y
  s:	 -y
  d:	 +x
  a:	 -x
  e:	 +z
  c:	 -z

anything else : go to origin

i/o : increase/decrease max speeds by 1
k/l : increase/decrease only linear speed by 1
,/. : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

WRITE_PERIOD = 5 # write 1 message every WRITE_PERIOD
DIST_THRESHOLD = 0.01 # threshold for distance from target
VEL_THRESHOLD = 0.001 # threshold for target velocity

moveBindings = {
		'w':(0,1,0,0),
		's':(0,-1,0,0),
		'd':(1,0,0,0),
		'a':(-1,0,0,0),
		'e':(0,0,1,0),
		'c':(0,0,-1,0),
	       }

speedBindings={
		'o':(1,1.1),
		'i':(-1,.9),
		'l':(1,1),
		'k':(-1,1),
		',':(1,1.1),
		'.':(1,.9),
	      }

global position
global cmdPosition
global count
global dataFile
global isMoving


def dist(p1, p2):
    d =  math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)
    # print d
    return d

def speed(v):
    s = math.sqrt((v.x)**2 + (v.y)**2 + (v.z)**2)
    # print s
    return s

def isTargetReached(pos, target, vel):
    if dist(pos,target) < DIST_THRESHOLD:
        if speed(vel) < VEL_THRESHOLD:
            return True
    return False

def stateCb(odo):
    global position
    global cmdPosition
    global count
    global dataFile
    global isMoving

    position = odo.pose.pose.position

    if isMoving and not dataFile.closed:
        if not count%WRITE_PERIOD:
            twist = odo.twist.twist
            twistLinear = twist.linear
            dataFile.write('%d.%d %.2f %.2f %.2f %.2f %.2f %.2f\n'
                            % (odo.header.stamp.secs, odo.header.stamp.nsecs/10000000,
                            position.x, position.y, position.z,
                            twistLinear.x, twistLinear.y, twistLinear.z))
            if isTargetReached(position, cmdPosition, twistLinear):
                isMoving = False
                dataFile.write('\n')
                print 'Reached target!'
        count += 1

def disableMotors():
    SERVICE_ENABLE_MOTORS = 'enable_motors'
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

dist_travel = 1
turn = 1

def vels(dist_travel,turn):
	return "currently:\tdist_travel %s\tturn %s " % (dist_travel,turn)

def publishPoseCommand(topic, frame, x, y, z):
    poseStamped = PoseStamped()
    poseStamped.pose = Pose()
    poseStamped.pose.position.x = x
    poseStamped.pose.position.y = y
    poseStamped.pose.position.z = z
    poseStamped.pose.orientation.w = 1
    poseStamped.header = Header()
    poseStamped.header.frame_id = frame
    poseStamped.header.stamp = rospy.Time.now()
    topic.publish(poseStamped)
    return poseStamped

if __name__=="__main__":
    global position
    global cmdPosition
    global count
    global dataFile
    global isMoving

    enableMotors()

    WORLD_FRAME = rospy.get_param('world_frame', '/world')
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('/command/pose', PoseStamped, queue_size = 1)
    rospy.init_node('test_overshoot')

    position = Point()
    cmdPosition = Point()
    count = 0
    status = 0
    isMoving = False

    now = datetime.datetime.now()
    filename = "test_files/%d-%d-%d-%d-%d-%d.txt" %(now.year, now.month, now.day, now.hour, now.minute, now.second)
    dataFile = open(CPS_DIR + filename,"w+")
    # dataFile.write('t(s) x(m) y(m) z(m) vx(m/s) vy(m/s) vz(m/s)\n')

    rospy.Subscriber('/ground_truth/state', Odometry, stateCb)

    print msg
    print vels(dist_travel,turn)

    # Hover
    publishPoseCommand(pub, WORLD_FRAME, 0.0, 0.0, 1.0)

    try:
        while(1):
            key = getKey()
            if key in moveBindings.keys():
            	cmdPosition.x = position.x + (moveBindings[key][0]*dist_travel)
            	cmdPosition.y = position.y + (moveBindings[key][1]*dist_travel)
            	cmdPosition.z = position.z + (moveBindings[key][2]*dist_travel)
                isMoving = True
                count = 0
            elif key in speedBindings.keys():
            	dist_travel = dist_travel + speedBindings[key][0]
                if dist_travel < 1:
                    dist_travel = 1
                elif dist_travel > 10:
                    dist_travel = 10
            	turn = turn * speedBindings[key][1]

            	print vels(dist_travel,turn)
            	if (status == 14):
            		print msg
            	status = (status + 1) % 15
            else:
            	cmdPosition.x = 0
            	cmdPosition.y = 0
            	cmdPosition.z = 0
            	if (key == '\x03'):
            		break

            poseStamped = publishPoseCommand(pub, WORLD_FRAME,
                                cmdPosition.x, cmdPosition.y, cmdPosition.z)
            # dataFile.write('xxxxx CURRENT POSITION xxxxx\n')
            dataFile.write('%d.%d\t%.2f\t%.2f\t%.2f\n'
                            % (poseStamped.header.stamp.secs,
                            poseStamped.header.stamp.nsecs/10000000,
                            position.x, position.y, position.z))
            # dataFile.write('----- POSITION COMMAND -----\n')
            dataFile.write('%d.%d\t%.2f\t%.2f\t%.2f\n'
                            % (poseStamped.header.stamp.secs,
                            poseStamped.header.stamp.nsecs/10000000,
                            cmdPosition.x, cmdPosition.y, cmdPosition.z))
            # time.sleep(0.5) # delay to avoid double clicks

    except:
        print 'An error occured'

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    dataFile.close()
    disableMotors()

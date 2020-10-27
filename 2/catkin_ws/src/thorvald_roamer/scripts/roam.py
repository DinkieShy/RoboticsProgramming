import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import tf
import math
import sys

class RoamerMover:
    """
    Simple class to move based on messages publishes to /thorvald_001/clear
    "R" indicates "clear to the right", "F" -> "clear to the front" etc.
    """
    def __init__(self, id): #takes id as a string. "thorvald_001" has id "001"
        self.sub = rospy.Subscriber("/thorvald_" + id + "/scan", LaserScan, callback=self.watch)
        self.minRange = 2.5
        self.twistPub = rospy.Publisher("/thorvald_" + id + "/twist_mux/cmd_vel", Twist, queue_size=0)
        self.closestPointPub = rospy.Publisher("/thorvald_" + id + "/closestPoint", PoseStamped, queue_size=0)
        self.forwardRate = (2)
        self.turnRate = (math.pi/2)

    def watch(self, msg):
        sweep = msg.ranges
        # sweep[0] is rightmost, sweep[len] is leftmost
        stringToPublish = ""

        if self.checkAreaClear(sweep, 0, len(sweep)/4): # Check first 4th of sweep
            stringToPublish += "R"

        if self.checkAreaClear(sweep, (3/4)*len(sweep), len(sweep)): # Check last 4th of sweep
            stringToPublish += "L"

        if self.checkAreaClear(sweep, len(sweep)/4, (3/4)*len(sweep)): # Check centre 3/4ths of sweep
            stringToPublish += "F"

        # Something was close enough to cause turning
        minIndex = sweep.index(min(sweep))
        angle2D = minIndex*msg.angle_increment-(math.pi/2)
        angleTF = PoseStamped()
        angleTF.header.frame_id = msg.header.frame_id
        angleTF.pose.orientation.z = math.sin(angle2D/2)
        angleTF.pose.orientation.w = math.cos(angle2D/2)
        self.closestPointPub.publish(angleTF)

        self.move(stringToPublish)

    def checkAreaClear(self, sweep, start, stop): # Returns false if any value between start and stop are below self.minRange, else true
        for i in range(start, stop):
            if sweep[i] < self.minRange:
                return False
        return True

    def move(self, direction):
        forwardSpeed = 0
        turnSpeed = 0

        if "F" in direction: # clear ahead
            forwardSpeed = self.forwardRate
        if "L" in direction: # clear left
            turnSpeed += self.turnRate
        if "R" in direction: # clear right
            turnSpeed -= self.turnRate

        #if clear both left and right, turnspeed == 0 but forwardspeed == 2

        if turnSpeed == 0 and forwardSpeed == 0: # If not clear in any direction, turn on the spot
            turnSpeed = 1.57

        twist = Twist()
        twist.linear.x = forwardSpeed
        twist.angular.z = turnSpeed

        self.twistPub.publish(twist)

thorvaldID = str(sys.argv[1]) if len(sys.argv) > 1 else "001"
rospy.init_node("roamer_" + thorvaldID)
mover = RoamerMover(thorvaldID)

rospy.spin()

# Mover is based on Watcher, and Watcher is based on messages from /thorvald_ID/scan, so no while loop needed

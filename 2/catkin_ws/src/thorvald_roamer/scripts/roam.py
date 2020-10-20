import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
import time

class RoamerMover:
    """
    Simple class to move based on messages publishes to /thorvald_001/clear
    "R" indicates "clear to the right", "F" -> "clear to the front" etc.
    """
    def __init__(self):
        self.sub = rospy.Subscriber("/thorvald_001/clear", String, callback=self.move)
        self.pub = rospy.Publisher("/thorvald_001/twist_mux/cmd_vel", Twist)

    def move(self, msg):
        direction = msg.data
        forwardSpeed = 0
        turnSpeed = 0

        if "F" in direction: # clear ahead
            forwardSpeed = 2
        if "L" in direction: # clear left
            turnSpeed += 1.57
        if "R" in direction: # clear right
            turnSpeed -= 1.57

        #if clear both left and right, turnspeed == 0 but forwardspeed == 2

        if turnSpeed == 0 and forwardSpeed == 0: # If not clear in any direction, turn on the spot
            turnSpeed = 1.57

        twist = Twist()
        twist.linear.x = forwardSpeed
        twist.angular.z = turnSpeed

        self.pub.publish(twist)

class RoamerWatcher:
    """
    Publishes the aforementioned messages to the /thorvald_001/clear topic
    """
    def __init__(self):
        self.sub = rospy.Subscriber("/thorvald_001/scan", LaserScan, callback=self.watch)
        self.pub = rospy.Publisher("/thorvald_001/clear", String)
        self.minRange = 2.5

    def checkAreaClear(self, sweep, start, stop): # Returns false if any value between start and stop are below self.minRange, else true
        for i in range(start, stop):
            if sweep[i] < self.minRange:
                return False
        return True

    def watch(self, msg):
        sweep = msg.ranges
        # sweep[0] is rightmost, sweep[len] is leftmost
        stringToPublish = ""

        if self.checkAreaClear(sweep, 0, len(sweep)/5): # Check first 5th of sweep
            stringToPublish += "R"

        if self.checkAreaClear(sweep, (4/5)*len(sweep), len(sweep)): # Check last 5th of sweep
            stringToPublish += "L"

        if self.checkAreaClear(sweep, len(sweep)/5, (4/5)*len(sweep)): # Check centre 3/5ths of sweep
            stringToPublish += "F"

        self.pub.publish(stringToPublish)

rospy.init_node("roamer")
mover = RoamerMover()
watcher = RoamerWatcher()

rospy.spin()

# Mover is based on Watcher, and Watcher is based on messages from /thorvald_001/scan, so no while loop needed

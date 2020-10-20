import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
import time

class RoamerMover:
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
    def __init__(self):
        self.sub = rospy.Subscriber("/thorvald_001/scan", LaserScan, callback=self.watch)
        self.pub = rospy.Publisher("/thorvald_001/clear", String)
        self.minRange = 2.5

    def checkAreaClear(self, sweep, start, stop):
        for i in range(start, stop):
            if sweep[i] < self.minRange:
                return False
        return True

    def watch(self, msg): # Watch for objects, call RoamerMover with clear/notclear
        sweep = msg.ranges
        stringToPublish = ""
        # sweep[0] is rightmost, sweep[len] is leftmost

        if self.checkAreaClear(sweep, 0, len(sweep)/5):
            stringToPublish += "R"

        if self.checkAreaClear(sweep, (4/5)*len(sweep), len(sweep)):
            stringToPublish += "L"

        if self.checkAreaClear(sweep, len(sweep)/5, (4/5)*len(sweep)):
            stringToPublish += "F"

        self.pub.publish(stringToPublish)

rospy.init_node("roamer")
mover = RoamerMover()
watcher = RoamerWatcher()

rospy.spin()

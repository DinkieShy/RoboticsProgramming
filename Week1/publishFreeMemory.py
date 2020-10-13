# Uses psutil from https://github.com/giampaolo/psutil

import psutil
import rospy
import std_msgs
import time

class Talker:
    def __init__(self):
        rospy.init_node("FreeMemTalker", anonymous=True) # Create node
        self.pub = rospy.Publisher('FreeMem', std_msgs.msg.String, queue_size=0) # Create publisher to *use* the node
        
    def publish(self):
        freeMem = psutil.virtual_memory().free # psutil class to get current free memory
        self.pub.publish(std_msgs.msg.String(str(freeMem))) # Publish the amount of free memory (as int64)

talker = Talker()

while not rospy.is_shutdown():
    talker.publish()
    time.sleep(1)

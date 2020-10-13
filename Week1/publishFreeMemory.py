# Uses psutil from https://github.com/giampaolo/psutil

import psutil
import rospy
import std_msgs
import time

class Talker:
    def __init__(self):
        rospy.init_node("FreeMemTalker", anonymous=True)
        self.pub = rospy.Publisher('FreeMem', std_msgs.msg.UInt64, queue_size=0)
        self.mem = psutil.virtual_memory()
        self.lastPublished = 0
        
    def publish(self):
        freeMem = self.mem.free
        self.pub.publish(std_msgs.msg.UInt64(freeMem))
        self.lastPublished = freeMem

talker = Talker()

while not rospy.is_shutdown():
    talker.publish()
    time.sleep(1)

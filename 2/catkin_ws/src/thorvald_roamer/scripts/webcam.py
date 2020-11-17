import rospy
import sys
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class Watcher():
    """Class to implement watching a webcam, and publishing a stop message for all thorvalds
    when a green object is shown in the image"""
    def __init__(self):
        self.cameraSub = rospy.Subscriber('camera/image_raw', Image, self.watch)
        self.camPub = rospy.Publisher('camera/image_filtered', Image, queue_size=0)
        self.stopPubs = []
        self.stopPubs.append(rospy.Publisher("camera/STOP", Bool, queue_size=0))
        self.bridge = CvBridge()
        
    def publish(self, data):
        for i in range(len(self.stopPubs)):
            self.stopPubs[i].publish(data)

    def watch(self, msg):
        #Watch webcam feed
        
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8") # Convert msg to actual cv2 image class
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # Convert to HSV, easier to separate green

        lowerGreen = np.array([50, 48, 64]) # Lower bounds of detectable green
        upperGreen = np.array([70, 255, 255]) # Upper bounds
        mask = cv2.inRange(image, lowerGreen, upperGreen) # binary mask, same size as image, 1 indicates a pixel between the bounds
        image = cv2.bitwise_and(image, image, mask=mask) # bitwise and to only keep pixels that fit in the filter

        kernel = np.ones((75, 75), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel) # Opening morphological operation to remove small areas caused by entropy of the camera

        self.camPub.publish(self.bridge.cv2_to_imgmsg(cv2.cvtColor(image, cv2.COLOR_HSV2BGR), 'bgr8')) # Convert image back to bgr8 image msg, then publish

        if np.sum(mask) > 100000: # If enough pixels fit in the filter
            self.publish(True) # Publish "STOP" variable
            # rospy.wait_for_service('thorvald_' + self.id + '/spray', Empty)
        else:
            self.publish(False) # else don't stop

rospy.init_node("webcam_watcher")

watcher = Watcher()
rospy.spin()
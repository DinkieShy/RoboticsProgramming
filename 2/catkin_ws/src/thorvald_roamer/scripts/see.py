import rospy
import sys
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

#rospy.wait_for_service('thorvald_001/spray', Empty)

class Looker():
    """Class to implement looking for green, then stopping"""
    def __init__(self, thorvaldID):
        self.id = thorvaldID
        self.camSub = rospy.Subscriber("/thorvald_" + self.id + "/kinect2_camera/hd/image_color_rect", Image, self.camCallback)
        self.camPub = rospy.Publisher("/thorvald_" + self.id + "/kinect2_camera/hd/image_color_rect_filtered", Image, queue_size=0)
        # Publishes the filtered image on /kinect2_camera/hd/image_color_rect_filtered
        self.stopPub = rospy.Publisher("/thorvald_" + self.id + "/STOP", Bool, queue_size=0)
        self.bridge = CvBridge() # Bridge used for converting msgs to cv2 image class

    def camCallback(self, msg):
        # format reply
        reply = Image()
        reply.header = msg.header

        image = self.bridge.imgmsg_to_cv2(msg, "bgr8") # Convert msg to actual cv2 image class
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # Convert to HSV, easier to separate green

        lowerGreen = np.array([50, 0, 20]) # Lower bounds of detectable green
        upperGreen = np.array([70, 128, 230]) # Upper bounds
        mask = cv2.inRange(image, lowerGreen, upperGreen) # binary mask, same size as image, 1 indicates a pixel between the bounds
        image = cv2.bitwise_and(image, image, mask=mask) # bitwise and to only keep pixels that fit in the filter

        # image = cv2.resize(image, (msg.width/3, msg.height/3)) # resize for sake of fitting on one screen
        # cv2.namedWindow("Image_" + self.id)
        # cv2.imshow("Image_" + self.id, cv2.cvtColor(image, cv2.COLOR_HSV2BGR))

        # cv2.waitKey(1)

        self.camPub.publish(self.bridge.cv2_to_imgmsg(cv2.cvtColor(image, cv2.COLOR_HSV2BGR), 'bgr8')) # Convert image back to bgr8 image msg, then publish

        # print(np.sum(mask)) # Print number of matched pixels for sanity check
        if np.sum(mask) > 100000: # If enough pixels fit in the filter
            self.stopPub.publish(True) # Publish "STOP" variable
            # rospy.wait_for_service('thorvald_' + self.id + '/spray', Empty)
        else:
            self.stopPub.publish(False) # else don't stop
        
thorvaldID = str(sys.argv[1]) if len(sys.argv) > 1 else "001"
rospy.init_node("looker_" + thorvaldID)
mover = Looker(thorvaldID)

print("Loaded!")

rospy.spin()

cv2.destroyAllWindows()
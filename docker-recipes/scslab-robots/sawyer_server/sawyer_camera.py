from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import intera_interface
import intera_external_devices
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time as t
import numpy as np

import time as t

class camera(object):
    
    def __init__(self):
        print("Initialized camera ")
        self.bridge = CvBridge()
        self.camera = intera_interface.Cameras()

    def see(self,view, show = True, size = [80,80]):
        if(view =="hand"):
            self.camera.stop_streaming("head_camera")
            self.camera.start_streaming("right_hand_camera")
            msg = rospy.wait_for_message("/io/internal_camera/right_hand_camera/image_rect", Image)
            #print("Got message ",msg)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
            print("cv_image shape ",cv_image.shape)
            self.cur_image_hand=cv2.resize(cv_image, (size[0], size[1]))
            if show:
                cv2.imshow("hand", self.cur_image_hand)
                cv2.waitKey(1)
            return self.cur_image_hand

        if(view =="head"):
            self.camera.stop_streaming("right_hand_camera")
            self.camera.start_streaming("head_camera")
            msg = rospy.wait_for_message("/io/internal_camera/head_camera/image_rect_color", Image)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            print("cv_image shape ",cv_image.shape)
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            self.cur_image_head=cv2.resize(gray_image, (size[0], size[1]))
            if show:
                cv2.imshow("head", self.cur_image_head)
                cv2.waitKey(1)
            return self.cur_image_head
    

'''
#USAGE
def main():
    rospy.init_node('camera_display', anonymous=True)
    c=camera()
    while(True):
        c.see("head")
        c.see("hand")

if __name__ == '__main__':
    main()
'''

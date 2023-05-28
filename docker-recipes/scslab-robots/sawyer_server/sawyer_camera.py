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

import imutils
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
    def detect_box(self,image, view = "hand", show_detection = True):
        blurred = cv2.GaussianBlur(image, (1, 1), 0)
        #canny
        '''
        get_edge = cv2.Canny(blurred, 10, 100)
        cv_image = np.hstack([get_edge])
        '''
        #Binary thresholding
        cv_image = cv2.threshold(blurred, 100, 200, cv2.THRESH_BINARY)[1]

        #find contours
        cnts = cv2.findContours(cv_image.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        cX = 0.0
        cY = 0.0
        ratio = 1.0 #No resizing is done afterwards
        # loop over the contours
        #create an ignore list to avoid detecting constant features in the environment
        ignore_list = []
        if view=="hand":
            ignore_list = [[950,797],[730,795],[753,794],[754,794],[18,577],[1223,388]] #ignore list for hand canera
        if view=="head":
            ignore_list = [[506,731],[862,349]] #ignore list for head camera
        for c in cnts:
            print("In cnts")
            # compute the center of the contour, then detect the name of the
            # shape using only the contour
            M = cv2.moments(c)
            try:
                cX = int((M["m10"] / M["m00"]) * ratio)
                cY = int((M["m01"] / M["m00"]) * ratio)
            except:
                print("Division by 0 occured")
            
            if [cX,cY] not in ignore_list:
                print("rectangle found at ",cX,cY)
                cv2.putText(cv_image, "cube"+repr(cX)+"_"+repr(cY), (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 255, 255), 2)
                break
        # show the output image
        if show_detection:
            cv2.imshow(view, cv_image)
            cv2.waitKey(1)
        #t.sleep(20)


        return cX, cY

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

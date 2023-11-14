#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Pose, PoseArray, Point, Quaternion
#import pyrealsense2 as rs
moments_array = []
cv_image=np.ndarray(shape=(480, 640, 3),dtype=np.uint8)
mask=np.ndarray(shape=(480, 640),dtype=np.uint8)
depth_image=np.ndarray(shape=(480, 640, 3),dtype=np.uint32)

class TomatoDetector(object):

    def __init__(self):
    
        self.image_sub = rospy.Subscriber("/camera/color/image_raw2", Image, self.camera_callback)
        self.depth_image_sub = rospy.Subscriber("/camera/depth/image_raw2", Image, self.convert_depth_image)

        self.bridge_object = CvBridge()
        self.moments_pub = rospy.Publisher ("/moments", PoseArray, queue_size=1)
        

    def camera_callback(self,data):
        global cv_image
        global mask
        global moments_array
        try:
            # We select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            #print(type(cv_image))
        except CvBridgeError as e:
            print(e)        

        height, width, channels = cv_image.shape
        #print(height,width,channels)
        descentre = 160
        rows_to_watch = 60
        crop_img = cv_image[int((height)/2+descentre):int((height)/2+(descentre+rows_to_watch))][1:width]
        #print(crop_img)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        upper_red = np.array([11,255,255])
        lower_red = np.array([0,60,98])

        # Threshold the HSV image to get only red colors
        mask = cv2.inRange(hsv, lower_red, upper_red)

        
        contours,_ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        #cv2.drawContours(cv_image, contours, -1, (0,255,0), 3)
        moments_array = []
        polygon_list = []
        for contour in contours:
            approx = cv2.approxPolyDP(contour,0.001*cv2.arcLength(contour,True),True)
            polygon_list.append(approx)
        for polygon in polygon_list:
            cv2.drawContours(cv_image, polygon_list , -1, (255,0,0), 1)
        for contour in contours:
            moment = cv2.moments(contour)
            try:
                if (moment['m10']/(moment['m00'] + 1e-5)!=0 and moment['m01']/(moment['m00'] + 1e-5))!=0:
                    moments_array.append((moment['m10']/(moment['m00'] + 1e-5), moment['m01']/(moment['m00'] + 1e-5)))
            except ZeroDivisionError as e:
                print(e)
        # print(len(moments_array))
        for (index, (x, y)) in enumerate(moments_array):
            cv2.circle(cv_image, (int(x), int(y)), radius=2, color=(255, 255, 255), thickness=-1)
            cv2.putText(cv_image, "tomato_{}".format(index), (int(x)-20, int(y)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, color=(255, 255, 255), thickness=1)

            

    def convert_depth_image(self,data):
        global depth_image
        global moments_array
        bridge = CvBridge()
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            # Convert the depth image using the default passthrough encoding
            depth_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            depth_array = np.array(depth_image, dtype=np.float32)
            center_idx = np.array(depth_array.shape) / 2
            pose_array = PoseArray()
            for (x, y) in moments_array:
                # print('x:',x,'y:',y,'depth:', depth_array[int(y), int(x)]) # Changed Over here
                depth = depth_array[int(y), int(x)] # image shape is 480 x 640 ie y coordinate is first index
                
                # Publishing (x, y, depth) data packaged in PoseArray message type to topic '/moments'
                pose = Pose(Point(x, y, depth), Quaternion())
                pose_array.poses.append(pose)
            #print("Length of message published:", len(pose_array.poses))
            self.moments_pub.publish(pose_array)

        except CvBridgeError as e:
            print(e)
            print('Error!')
        # Convert the depth image to a Numpy array
    


def main():

    rospy.init_node('tomato_detector', anonymous=True)
    tomato_detector = TomatoDetector()
    
    try:
        while not rospy.is_shutdown():
            cv2.imshow("window",cv_image)
            k=cv2.waitKey(30) & 0xff
            if k==27:
                cv2.imwrite('filename.png', cv_image)
                break
        cv2.destroyAllWindows()
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()

#!/usr/bin/env python3	

import rospy
from geometry_msgs.msg import PoseArray, Transform, TransformStamped
import tf
from tf import transformations
import tf2_ros
import numpy as np
from std_msgs.msg import Int32

depth_data_array = []

class TransformPublisher():

    def __init__(self):
        self.tomato_moments_sub = rospy.Subscriber('/moments', PoseArray, callback=self.moments_callback)
        self.total_tomato_pub = rospy.Publisher('/total',Int32,queue_size=1)
        # Camera parameters
        self.focal_length_x = 554.387
        self.focal_length_y = 554.387
        self.center_x = 320.5
        self.center_y = 240.5
        
        self.transform_broadcaster = tf2_ros.TransformBroadcaster()

    def message_from_transform(self, T):
    # Returns a message of type geometry_msgs.msg.Transform 
    # from the function parameter transform matrix 'T'

        msg = Transform()
        q = transformations.quaternion_from_matrix(T)
        translation = transformations.translation_from_matrix(T)
        msg.translation.x = translation[0]
        msg.translation.y = translation[1]
        msg.translation.z = translation[2]
        msg.rotation.x = q[0]
        msg.rotation.y = q[1]
        msg.rotation.z = q[2]
        msg.rotation.w = q[3]
        return msg
    
    def moments_callback(self, data: PoseArray):
        global depth_data_array
        depth_data_array = []
        for pose in data.poses:
            z = pose.position.z
            x = z * (pose.position.x - self.center_x)/self.focal_length_x
            y = z * (pose.position.y - self.center_y)/self.focal_length_y
            depth_data_array.append((float(x), float(y), float(z)))
        self.total_tomato_pub.publish(len(depth_data_array))
        # for index in len(depth_data_array):
        #     self.publish_transform(index)

    def publish_transform(self):
        # index = 0

        for index in range(len(depth_data_array)):
            #print("index", index)
            if(index < len(depth_data_array)):
                (x, y, z) = depth_data_array[index]
                #print("x y z", x, y ,z)

                camera_to_tomato_tf = transformations.concatenate_matrices(
                    transformations.translation_matrix((x, y, z)),
                    transformations.quaternion_matrix(
                        transformations.quaternion_from_euler(0.0, 0.0, 0.0)
                        )
                )


                camera_to_tomato_tf_stamped = TransformStamped()
                camera_to_tomato_tf_stamped.header.stamp = rospy.Time.now()
                camera_to_tomato_tf_stamped.header.frame_id = "/camera_depth_frame2"
                camera_to_tomato_tf_stamped.child_frame_id = "tomato_{}".format(index)
                camera_to_tomato_tf_stamped.transform = self.message_from_transform(camera_to_tomato_tf)
                # Broadcasts stamped transform
                self.transform_broadcaster.sendTransform(camera_to_tomato_tf_stamped)  
                
        self.total_tomato_pub.publish(len(depth_data_array))

if __name__ == '__main__':
    rospy.init_node('tf_publisher_tomato')
    tf_publisher = TransformPublisher()
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        tf_publisher.publish_transform()
        rate.sleep()
    print("Shutting down node")

            
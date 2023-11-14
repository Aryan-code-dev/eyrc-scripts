#!/usr/bin/env python3
import rospy 
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
regions={}
pose=[]
yaw=1.0


def odom_callback(data):
    global pose
    global yaw
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    yaw=euler_from_quaternion([x,y,z,w])[2]
    if yaw<0:
        yaw=6.28+yaw
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]
def laser_callback(msg):
   global regions
   regions = {        
        'fright':min(min(msg.ranges[575:719]), msg.range_max)   ,
        'ffright':min(min(msg.ranges[431:575]), msg.range_max)   ,
        'front':min(min(msg.ranges[287:431]), msg.range_max)    ,
        'fleft':min(min(msg.ranges[0:143]), msg.range_max)    ,
        'ffleft':min(min(msg.ranges[143:287]), msg.range_max)   ,
        }
    

class navigate:
    @staticmethod
    def turn1(direction):
        rate = rospy.Rate(10)
        print(yaw)
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        velocity_msg = Twist()
        pub.publish(velocity_msg)
        velocity_msg.angular.z = 0.8*direction
        rate.sleep()
        start_degree = (math.degrees(yaw))
        current_degree = (math.degrees(yaw))
        degree_rotated = 0.0
    
        while not rospy.is_shutdown():
             if((round(degree_rotated) <90 )):
                    pub.publish(velocity_msg)
                    rate.sleep()
                    current_degree = (math.degrees(yaw))
                    degree_rotated = abs(current_degree - start_degree)
                    print('Degree Rotated: {}'.format(degree_rotated))
             else:
                    velocity_msg.angular.z = 0.0
                    pub.publish(velocity_msg)
                    rate.sleep()
                    break
    
    @staticmethod                
    def move1(direction,dist):
        global pose
        global regions
        print("straight called")
        rate = rospy.Rate(10)
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        velocity_msg = Twist()
        target_dist=dist
        print(regions['front'])
        while not rospy.is_shutdown():
            if regions['front']-target_dist>=0.05 :
                velocity_msg.linear.x = 0.5
                pub.publish(velocity_msg)
                print("Current pose :{} ".format(regions['front']))       
            else:
                velocity_msg.linear.x = 0.0
                pub.publish(velocity_msg)
                break
      
       
        print("Controller message pushed at {}".format(rospy.get_time()))
    
    @staticmethod
    def testdist1(distance1,direction1,align_dis1,align_dis2,region,direction2):
        global pose
        global regions
        global yaw
        rate = rospy.Rate(10)
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        print(pose)
        velocity_msg = Twist()
        velocity_msg.linear.x = 0.5
        pub.publish(velocity_msg)
        dir1=direction1
        dir2=direction2
        start_x = pose[dir1]
        dist1=distance1
        align1=align_dis1
        align2=align_dis2
        start_degree = (math.degrees(yaw))
        current_degree = (math.degrees(yaw))
        degree_rotated1 = 0.0
        degree_rotated2 = 0.0
        dis_moved=0.0
        while not rospy.is_shutdown():
            if dis_moved<dist1:
                if regions[region]>=align1 and dis_moved>1:
                    if ((round(degree_rotated1) <1 )):
                        velocity_msg.angular.z = 0.1*dir2
                        pub.publish(velocity_msg)
                        rate.sleep()
                        current_degree = (math.degrees(yaw))
                        degree_rotated = abs(current_degree - start_degree)
                        print('Degree Rotated: {}'.format(degree_rotated))  
                        print(regions)
                    else:
                        velocity_msg.angular.z = 0.0
                        break  
                elif regions[region]<=align2 and dis_moved>0.01:
                     if ((round(degree_rotated1) <1 )):
                        velocity_msg.angular.z = -0.1*dir2
                        pub.publish(velocity_msg)
                        rate.sleep()
                        current_degree = (math.degrees(yaw))
                        degree_rotated = abs(current_degree - start_degree)
                        print('Degree Rotated: {}'.format(degree_rotated))  
                        print(regions)
                     else:
                        velocity_msg.angular.z = 0.0
                        break  
                     
                else:
                    velocity_msg.angular.z = 0.0
                    pub.publish(velocity_msg)
                    print(regions)     
                cur_x=pose[dir1]
                dis_moved = abs(cur_x - start_x)
                print('Distance Moved: {}'.format(dis_moved))
            else:
                velocity_msg.linear.x = 0.0
                pub.publish(velocity_msg)
                break
    
    @staticmethod
    def turn2(direction):
        rate = rospy.Rate(10)
        print(yaw)
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        velocity_msg = Twist()
        pub.publish(velocity_msg)
        velocity_msg.angular.z = 1*direction
        velocity_msg.linear.x = 0.4
        rate.sleep()
        start_degree = (math.degrees(yaw))
        current_degree = (math.degrees(yaw))
        degree_rotated = 0.0
    
        while not rospy.is_shutdown():
            if((round(degree_rotated) <87 )):
                pub.publish(velocity_msg)
                rate.sleep()
                current_degree = (math.degrees(yaw))
                degree_rotated = abs(current_degree - start_degree)
                print('Degree Rotated: {}'.format(degree_rotated))
            else:
                velocity_msg.angular.z = 0.0
                velocity_msg.linear.x = 0.0
                degree_rotated = 0.0
                pub.publish(velocity_msg)
                rate.sleep()
                break  
            
                    
def control_loop():
    global regions
    global yaw
    
    rospy.init_node('ebot_controller')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
    velocity_msg = Twist() 
    rate = rospy.Rate(10) 
    for i in range(50):										# delaying for 5 seconds 
        rate.sleep()
    object_nav=navigate()
    print("turn called")
    object_nav.turn1(1)
    print("turned")
    object_nav.move1('x',1.5)
    print("moved")
    print(yaw)
    object_nav.turn1(-1)
    object_nav.testdist1(8.7,1,1.887,1.885,'ffright',1)
    object_nav.turn2(-1)
    object_nav.turn2(-1)
    object_nav.testdist1(8,1,0.88,0.85,'ffright',1)
    object_nav.turn2(1)
    object_nav.turn2(1)
    object_nav.testdist1(9,1,2.08,1.8,'ffleft',-1)
    print(yaw)
    print(regions)
    print("Controller message pushed at {}".format(rospy.get_time()))
    rate.sleep()           

if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass




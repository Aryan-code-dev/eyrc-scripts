#! /usr/bin/env python3
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
import actionlib
from std_msgs.msg import Int32
import roslib
import tf
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
roslib.load_manifest('ebot_cam')

tomatoes=0
class Ur5Moveit:

    # Constructor
    def __init__(self,group):
        self.total_tomato_sub = rospy.Subscriber('/total',Int32, callback=self.tomato_callback)
        rospy.init_node('node_eg2_predefined_pose', anonymous=True)
        if group==0:        
            self._planning_group = "arm_controller"
        else:
            self._planning_group = "gripper_controller"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()


        # rospy.loginfo(
        #     '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        # rospy.loginfo(
        #     '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        # rospy.loginfo(
        #     '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        # rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')
    
    def transform_pose(self,arg_pose_name, from_frame, to_frame):
    # **Assuming /tf2 topic is being broadcasted
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        pose_stamped= tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = arg_pose_name
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time.now()

        try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
            return output_pose_stamped.pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise


    def go_to_predefined_pose(self, arg_pose_name):
        self._group.set_planner_id("RRTConnectkConfigDefault")
        #self._group.set_end_effector_link("gripper_finger1_joint")
        self._group.set_pose_reference_frame("/camera_depth_frame2")
        self._group.allow_replanning(True)
        self._group.set_goal_joint_tolerance(0.0001)
        self._group.set_goal_position_tolerance(0.0001)
        self._group.set_goal_orientation_tolerance(0.001)
        self._group.set_planning_time(10.0)
        self._group.allow_replanning(True)
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_pose_target(arg_pose_name)
        #self._group.set_goal_tolerance(0.01)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        flag_plan = self._group.go(wait=True)
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        # rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
    
    def tomato_callback(self,msg):
        global tomatoes
        tomatoes=msg.data
        
    # Destructor
    def __del__(self):

        moveit_commander.roscpp_shutdown()
        # rospy.loginfo('\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():
    ur5 = Ur5Moveit(0)
    # eg_pose = geometry_msgs.msg.PoseStamped()
    # eg_pose.pose.orientation.w=1.0
    # eg_pose.pose.position.x=-0.270000
    # eg_pose.pose.position.y=-0.146000
    # eg_pose.pose.position.z=0.885000
    # eg_pose.header.frame_id = "base"
    listener = tf.TransformListener()


    while not rospy.is_shutdown():
        try:
            for i in range(int(tomatoes)):
                (trans,rot) = listener.lookupTransform('/camera_depth_frame2',"tomato_{}".format(i),rospy.Time(0))
                temp_pose = geometry_msgs.msg.PoseStamped()
                transformed_pose = geometry_msgs.msg.PoseStamped()
                temp_pose.pose.orientation.x=rot[0]
                temp_pose.pose.orientation.y=rot[1]
                temp_pose.pose.orientation.z=rot[2]
                temp_pose.pose.orientation.w=rot[3]
                temp_pose.pose.position.x=float("{0:.7f}".format(trans[0]))
                temp_pose.pose.position.y=float("{0:.7f}".format(trans[1]))
                temp_pose.pose.position.z=float("{0:.7f}".format(trans[2]))
                transformed_pose = ur5.transform_pose(temp_pose, "camera_depth_frame2", "ebot_base")
                ur5.go_to_predefined_pose(transformed_pose)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    del ur5


if __name__ == '__main__':
    main()

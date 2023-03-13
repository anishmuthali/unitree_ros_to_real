import numpy as np
import time
import pdb

import ood_gpssm_msgs.msg
import unitree_legged_msgs.msg # Located at /home/ubuntu/mounted_home/work/code_projects_WIP/catkin_real_robot_ws/devel/lib/python3/dist-packages (this path is added automatically to the PYTHONPATH after doing 'source devel/setup.bash')
import geometry_msgs.msg
import rospy

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class RobotStateCreationNode():

    def __init__(self,topic_high_state,topic_vicon_data):
        """
        
        Collect all the relevant data from the network and re-publish only the ones we really need
        using a new msg. Specifically:
        1) Collect Vicon data, differentiate the signals and re-publish them
        2) Collect Uniteee's HighState relevant signals and re-publish them
        """

        
        # Keep track of the low-level and high-level state:
        self.topic_high_state = topic_high_state
        self.msg_high_state = unitree_legged_msgs.msg.HighState()
        rospy.Subscriber(self.topic_high_state, unitree_legged_msgs.msg.HighState, self._callback_robot_high_state)

        # Collect Vicon data:
        self.topic_vicon_data = topic_vicon_data
        self.msg_vicon_data = geometry_msgs.msg.TransformStamped()
        rospy.Subscriber(self.topic_vicon_data, geometry_msgs.msg.TransformStamped, self._callback_vicon_data)

        self.robot_position = np.zeros(3)
        self.robot_orientation_quat = np.zeros(4)
        self.robot_orientation_euler = np.zeros(3)

        self.robot_linear_velocity = np.zeros(self.robot_position.shape)
        self.robot_angular_velocity = np.zeros(self.robot_orientation_euler.shape)

        self.robot_position_prev = None
        self.robot_orientation_prev = None
        self.time_vicon_prev = None
        self.time_start = None

        topic_robot_state = "/experiments_gpssm_ood/robot_state"
        self.msg_robot_state = ood_gpssm_msgs.msg.Go1State()
        self.pub_robot_state = rospy.Publisher(topic_robot_state, ood_gpssm_msgs.msg.Go1State, queue_size=2)


    def collect_and_publish(self,rate_main_loop):
        """

        rate_main_loop: scalar, frequency in Hz
        """

        rospy.init_node("robot_state_creation_node", anonymous=False)
        ros_loop = rospy.Rate(rate_main_loop) # Hz

        time_sleep = 2.0
        rospy.loginfo("Sleeping for {0:f} sec ...".format(time_sleep))
        rospy.loginfo("Publishing Go1State.msg indefinitely ... ")
        time.sleep(time_sleep)

        # Consider having this on a separate thread using import threading
        self.time_start = time.time()
        while True:
            self._construct_robot_state() # This updates self.msg_robot_state
            self.pub_robot_state.publish(self.msg_robot_state)
            ros_loop.sleep()


    def _callback_robot_high_state(self,msg_in):
        self.msg_high_state = msg_in

    def _callback_vicon_data(self,msg_in):
        """
        messages type: http://docs.ros.org/en/api/geometry_msgs/html/msg/TransformStamped.html
        """

        # Robot position:
        self.robot_position[0] = msg_in.transform.translation.x
        self.robot_position[1] = msg_in.transform.translation.y
        self.robot_position[2] = msg_in.transform.translation.z

        # Robot orientation:
        # vicon_bridge quaternion convention: (x,y,z,w)
        self.robot_orientation_quat[0] = msg_in.transform.rotation.x
        self.robot_orientation_quat[1] = msg_in.transform.rotation.y
        self.robot_orientation_quat[2] = msg_in.transform.rotation.z
        self.robot_orientation_quat[3] = msg_in.transform.rotation.w
        self.robot_orientation_euler = euler_from_quaternion(self.robot_orientation_quat)

        # First time we enter this callback:
        if self.time_vicon_prev is None:
            self.time_vicon_prev = time.time()
            self.robot_velocity = np.zeros(self.robot_position.shape)
            self.robot_yaw_velocity = 0.0
            return

        time_vicon_curr = time.time()
        deltaT_dynamic = time_vicon_curr - self.time_vicon_prev
        self.time_vicon_prev = time_vicon_curr

        # Differentiate:
        self.robot_linear_velocity = self._differentiate_vicon_position(self.robot_position,deltaT_dynamic)
        self.robot_angular_velocity = self._differentiate_vicon_orientation(self.robot_orientation_euler,deltaT_dynamic)


    def _differentiate_vicon_position(self,robot_position_new,deltaT_dynamic):

        # First time; update and return
        if self.robot_position_prev is None:
            self.robot_position_prev = robot_position_new
            return np.zeros(robot_position_new.shape)

        # Compute velocity:
        robot_velocity = (robot_position_new - self.robot_position_prev) / deltaT_dynamic

        # Update:
        self.robot_position_prev = robot_position_new

        return robot_velocity


    def _differentiate_vicon_orientation(self,robot_orientation_new,deltaT_dynamic):
        
        # First time; update and return
        if self.robot_orientation_prev is None:
            self.robot_orientation_prev = robot_orientation_new
            return np.zeros(robot_position_new.shape)

        # Compute velocity:
        robot_yaw_vel = (robot_orientation_new - self.robot_orientation_prev) / deltaT_dynamic

        # Update:
        self.robot_orientation_prev = robot_orientation_new

        return robot_yaw_vel

    def _construct_robot_state(self):

        # Position and orientation:
        self.msg_robot_state.position.x = self.robot_position[0]
        self.msg_robot_state.position.y = self.robot_position[1]
        self.msg_robot_state.position.z = self.robot_position[2]
        self.msg_robot_state.orientation.x = self.robot_orientation_euler[0]
        self.msg_robot_state.orientation.y = self.robot_orientation_euler[1]
        self.msg_robot_state.orientation.z = self.robot_orientation_euler[2]

        # Velocities:
        self.msg_robot_state.twist.linear.x = self.robot_linear_velocity[0]
        self.msg_robot_state.twist.linear.y = self.robot_linear_velocity[1]
        self.msg_robot_state.twist.linear.z = self.robot_linear_velocity[2]
        self.msg_robot_state.twist.angular.x = self.robot_angular_velocity[0]
        self.msg_robot_state.twist.angular.y = self.robot_angular_velocity[1]
        self.msg_robot_state.twist.angular.z = self.robot_angular_velocity[2]
        self.msg_robot_state.time_stamp = time.time() - self.time_start



if __name__ == "__main__":

    topic_high_state = "/high_state_from_robot"
    topic_vicon_data = "/vicon/go1/go1"
    
    robot_state_creation_node = RobotStateCreationNode(topic_high_state,topic_vicon_data)
    robot_state_creation_node.collect_and_publish(rate_main_loop=500)















    
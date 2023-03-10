import numpy as np
import time
import pdb

import unitree_legged_msgs.msg # Located at /home/ubuntu/mounted_home/work/code_projects_WIP/catkin_real_robot_ws/devel/lib/python3/dist-packages (this path is added automatically to the PYTHONPATH after doing 'source devel/setup.bash')
import rospy

class RobotDataCollection():

    def __init__(self,topic_high_state,topic_vicon_data):
        """
        Callback will be called for each message published on the topic; we can't set the frequency
        """

        
        # Keep track of the low-level and high-level state:
        self.topic_high_state = topic_high_state
        self.msg_high_state = unitree_legged_msgs.msg.HighState()
        rospy.Subscriber(self.topic_high_state, unitree_legged_msgs.msg.HighState, self.callback_robot_high_state)

        # Collect Vicon data:
        self.topic_vicon_data = topic_vicon_data
        # self.msg_vicon_data = unitree_legged_msgs.msg.HighState()
        # rospy.Subscriber(self.topic_vicon_data, unitree_legged_msgs.msg.HighState, self.callback_vicon_data)

    def callback_robot_high_state(self,msg):
        self.msg_high_state = msg

    def callback_vicon_data(self,msg):
        pass

    def differentiate_vicon_position(self):
        pass

    def differentiate_vicon_orientation(self):
        pass

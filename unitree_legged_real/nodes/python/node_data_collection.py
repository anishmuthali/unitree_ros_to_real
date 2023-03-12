import os
import numpy as np
import pdb
import time
from datetime import datetime
import pickle
import math
import rospy

import unitree_legged_msgs.msg # Located at /home/ubuntu/mounted_home/work/code_projects_WIP/catkin_real_robot_ws/devel/lib/python3/dist-packages (this path is added automatically to the PYTHONPATH after doing 'source devel/setup.bash')
import std_msgs.msg
import ood_gpssm_msgs.msg

import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib import cm
import matplotlib

from utils.generate_vel_profile import get_training_data_from_waypoints
from utils.robot_data_collection import RobotDataCollection

markersize_x0 = 10
markersize_trajs = 0.4
fontsize_labels = 25
matplotlib.rc('xtick', labelsize=fontsize_labels)
matplotlib.rc('ytick', labelsize=fontsize_labels)
matplotlib.rc('text', usetex=False)
# matplotlib.rc('font',**{'family':'serif','serif':['Computer Modern Roman']})
plt.rc('legend',fontsize=fontsize_labels+2)


HIGHLEVEL = 0x00


trigger_start_data_collection = False
trigger_stop_data_collection = False

msg_data_collection = ood_gpssm_msgs.msg.DataCollection()
# msg_go1_control = ood_gpssm_msgs.msg.Go1Control()
msg_go1_state = ood_gpssm_msgs.msg.Go1State()
msg_high_cmd = unitree_legged_msgs.msg.HighCmd()

def callback_data_collection(self,msg_in):
    msg_data_collection = msg_in

    if msg_data_collection.start:
        rospy.loginfo("Trigger for starting data collection activated!")
    
    if msg_data_collection.stop:
        rospy.loginfo("Trigger for stopping data collection activated!")

def callback_cmd_high(self,msg_in):
    # msg_go1_control = msg_in
    msg_high_cmd = msg_in

def callback_go1_state(self,msg_in):
    msg_go1_state = msg_in


def main():

    np.random.seed(1)

    rospy.init_node("run_go2experiment", anonymous=False)
    rate_main_loop = 500 # Hz
    ros_loop = rospy.Rate(rate_main_loop) # Hz

    time_max_data_collection = 300 # sec
    Nsteps_total = int(rate_main_loop*time_max_data_collection)

    print("Saving data at ~{0:d} Hz ...".format(rate_main_loop))
    
    # path2save = "/Users/alonrot/work/code_projects_WIP/catkin_real_robot_ws/src/unitree_ros_to_real_forked/unitree_legged_real/nodes/python/experiments_go2"
    path2save = "/home/amarco/catkin_real_robot_ws/src/unitree_ros_to_real/unitree_legged_real/nodes/python/experiments_go2"


    topic_data_collection_triggers = "/experiments_gpssm_ood/data_collection_triggers"
    rospy.Subscriber(topic_data_collection_triggers, ood_gpssm_msgs.msg.DataCollection, callback_data_collection)

    # topic_control_input = "/experiments_gpssm_ood/control_input"
    # rospy.Subscriber(topic_control_input, ood_gpssm_msgs.msg.Go1Control, callback_control_input)

    topic_high_cmd = "/high_cmd_to_robot"
    rospy.Subscriber(topic_high_cmd, unitree_legged_msgs.msg.HighCmd, callback_cmd_high)

    topic_robot_state = "/experiments_gpssm_ood/robot_state"
    rospy.Subscriber(topic_robot_state, ood_gpssm_msgs.msg.Go1State, callback_go1_state)


    data2save = dict(   robot_pos=np.zeros((Nsteps_total,3)),\
                        robot_vel=np.zeros((Nsteps_total,3)),\
                        robot_orientation=np.zeros((Nsteps_total,3)),\
                        robot_angular_velocity=np.zeros((Nsteps_total,3)),\
                        vel_forward_des=np.zeros((Nsteps_total,1)),\
                        vel_yaw_des=np.zeros((Nsteps_total,1)),\
                        time_stamp=np.zeros((Nsteps_total,1)))


    collect_data_on = False
    save_data = False

    while True:


        """
        =========================
        <<<< Data collection >>>>
        =========================
        """

        if trigger_start_data_collection and collect_data_on: # trigger_start_data_collection is global, written inside its callback
            rospy.loginfo("Data collection was already requested and dat is currently being collected...")
        elif trigger_start_data_collection:
            rospy.loginfo("Starting data collection!")
            collect_data_on = True
            time_start = time.time()
            robot_data_collection.update_time_init(time_start)
            ii = 0

        if trigger_stop_data_collection and not collect_data_on: # trigger_stop_data_collection is global, written inside its callback
            rospy.loginfo("Data collection hasn't started yet...")

        # Data collection stuff:
        if collect_data_on:

            data2save["robot_pos"][ii,:] = np.array([msg_go1_state.position.x,msg_go1_state.position.y,msg_go1_state.position.z])
            data2save["robot_vel"][ii,:] = np.array([msg_go1_state.twist.linear.x,msg_go1_state.twist.linear.y,msg_go1_state.twist.linear.z])
            data2save["robot_orientation"][ii,:] = np.array([msg_go1_state.orientation.x,msg_go1_state.orientation.y,msg_go1_state.orientation.z])
            data2save["robot_angular_velocity"][ii,:] = np.array([msg_go1_state.twist.angular.x,msg_go1_state.twist.angular.y,msg_go1_state.twist.angular.z])
            data2save["time_stamp"][ii,0] = time.time() - time_start
            # data2save["vel_forward_des"][ii,0] = msg_go1_control.velocity_linear_forward_desired
            # data2save["vel_yaw_des"][ii,0] = msg_go1_control.yaw_speed_desired
            data2save["vel_forward_des"][ii,0] = unitree_legged_msgs.msg.HighCmd.velocity[0]
            data2save["vel_yaw_des"][ii,0] = unitree_legged_msgs.msg.HighCmd.yawSpeed
            ii += 1


        # Check termination criterion:
        if collect_data_on and (ii > Nsteps_total or trigger_stop_data_collection):
            if ii > Nsteps_total: rospy.loginfo("Data collection terminated! Reason: timed out")
            if trigger_stop_data_collection: rospy.loginfo("Data collection terminated! Reason: external trigger")
            collect_data_on = False
            save_data = True


        # Save data:
        if save_data:

            name_file_data = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
            name_file = "{0:s}_experiments_go2trajs.pickle".format(name_file_data)
            path2save_full = "{0:s}/{1:s}".format(path2save,name_file)
            rospy.loginfo("Saving data at {0:s} ...".format(path2save_full))
            file = open(path2save_full, 'wb')
            pickle.dump(data2save,file)
            file.close()
            rospy.loginfo("Done!")
            save_data = False


        # Reset the triggers to their default value:
        trigger_start_data_collection = False
        trigger_stop_data_collection = False

        ros_loop.sleep()




if __name__ == "__main__":

    main()


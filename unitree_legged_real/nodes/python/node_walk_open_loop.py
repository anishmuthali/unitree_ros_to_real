import os
import numpy as np
import pdb
import time
from datetime import datetime
import pickle
import math
import rospy

import unitree_legged_msgs.msg # Located at /home/ubuntu/mounted_home/work/code_projects_WIP/catkin_real_robot_ws/devel/lib/python3/dist-packages (this path is added automatically to the PYTHONPATH after doing 'source devel/setup.bash')
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

def callback_go1_state(self,msg_in):
    msg_go1_state = msg_in


if __name__ == "__main__":

    """
    
    Generate control commands and publish them. The same control commands are published in two different formats:
    1) unitree_legged_msgs.msg.HighCmd() -> The cpp robot interface is subscribed to this one
    2) ood_gpssm_msgs.msg.Go1Control() -> The data collection node is subscribed tot his one


    """

    np.random.seed(1)

    rate_freq_send_commands = 100 # Hz
    # save_data_trajs_dict = dict(save=True,path2data="/Users/alonrot/work/code_projects_WIP/catkin_real_robot_ws/src/unitree_ros_to_real_forked/unitree_legged_real/nodes/python/trajs_generated/trajs.pickle")
    save_data_trajs_dict = None
    deltaT = 1./rate_freq_send_commands

    time_tot = 5.0 # sec
    pos_waypoints = np.array(   [0.0,0.0],
                                [1.5,1.5],
                                [-1.5,2.5],
                                [0.0,4.0])
    state_tot, vel_tot = get_velocity_profile_given_waypoints(pos_waypoints,deltaT,time_tot,block_plot=False,plotting=True) # state_tot: [Nsteps_tot,2] || vel_tot: [Nsteps_tot,2]

    rospy.init_node("node_walk_open_loop", anonymous=False)
    rate_freq_read_state = 500 # Hz
    rate_read_state = rospy.Rate(rate_freq_read_state) # Hz

    # Subscribe to RobotState:
    topic_robot_state = "/experiments_gpssm_ood/robot_state"
    rospy.Subscriber(topic_robot_state, ood_gpssm_msgs.msg.Go1State, callback_go1_state)

    # Publish control command:
    topic_high_cmd = "/high_cmd_to_robot"
    pub2high_cmd = rospy.Publisher(topic_high_cmd, unitree_legged_msgs.msg.HighCmd, queue_size=10)


    # Data collection triggers:
    topic_data_collection_triggers = "/experiments_gpssm_ood/data_collection_triggers"
    msg_data_collection = ood_gpssm_msgs.msg.DataCollection()
    pub_data_collection_triggers = rospy.Publisher(topic_data_collection_triggers, ood_gpssm_msgs.msg.DataCollection, queue_size=1)


    # Message containing walking mode:
    msg_high_cmd = unitree_legged_msgs.msg.HighCmd()
    msg_high_cmd.levelFlag = HIGHLEVEL
    msg_high_cmd.mode = 2
    msg_high_cmd.gaitType = 1 # 0.idle  1.trot  2.trot running  3.climb stair
    msg_high_cmd.velocity[0] = 0.00 # [-1,1] # (unit: m/s), forwardSpeed, sideSpeed in body frame
    msg_high_cmd.bodyHeight = 0.0 # # (unit: m, default: 0.28m) -> this is measured w.r.t the current height....
    msg_high_cmd.yawSpeed = 0.0


    collect_data = True

    Nsteps = vel_tot.shape[1]
    rospy.loginfo("Velocity profile will be published at rate {0:d} for {1:2.2f} seconds".format(rate_freq_send_commands,float(Nsteps/rate_freq_send_commands)))
    if collect_data: rospy.loginfo("Data will be automatically collected and saved ...")
    rospy.loginfo("Ready to start the trajectory; press any key to continue ...")
    input()

    # Activate data collection here:
    if collect_data: 
        time_pause = 2
        rospy.loginfo("Starting data collection! Pausing for {0:d} sec, in order for the message to propagate ...".format(time_pause))
        msg_data_collection.start = True
        msg_data_collection.publish(msg_data_collection)
        time.sleep(time_pause) # Wait a bit for the message to propagate

    rospy.loginfo("Starting loop now!")
    tt = 0
    while tt < Nsteps:

        msg_high_cmd.velocity[0] = vel_tot[tt,0] # desired linear velocity || vel_tot: [Ntrajs,Nsteps_tot,2]
        msg_high_cmd.yawSpeed = vel_tot[tt,1] # desired angular velocity || vel_tot: [Ntrajs,Nsteps_tot,2]
        
        pub2high_cmd.publish(msg_high_cmd)

        rate_read_state.sleep()

        tt += 1

    # Reset to mode 0:
    rospy.loginfo("Trajectory completed!")
    rospy.loginfo("Back to standing still....")
    msg_high_cmd.mode = 1 # TODO: Shouldn't this be 0?
    msg_high_cmd.gaitType = 0 # 0.idle  1.trot  2.trot running  3.climb stair
    msg_high_cmd.velocity[0] = 0.0 # [-1,1] # (unit: m/s), forwardSpeed, sideSpeed in body frame
    msg_high_cmd.bodyHeight = 0.0 # # (unit: m, default: 0.28m)
    ii = 0
    while ii < 100:
        pub2high_cmd.publish(msg_high_cmd)
        rate_read_state.sleep()
        ii += 1


    # Activate data collection here:
    if collect_data: 
        rospy.loginfo("Stopping data collection!")
        msg_data_collection.stop = True
        msg_data_collection.publish(msg_data_collection)

    rospy.loginfo("exiting; node finished!")


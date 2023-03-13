import os
import numpy as np
import pdb

import pickle
import math

import matplotlib.pyplot as plt
import matplotlib

markersize_x0 = 10
markersize_trajs = 0.4
fontsize_labels = 25
matplotlib.rc('xtick', labelsize=fontsize_labels)
matplotlib.rc('ytick', labelsize=fontsize_labels)
matplotlib.rc('text', usetex=False)
# matplotlib.rc('font',**{'family':'serif','serif':['Computer Modern Roman']})
plt.rc('legend',fontsize=fontsize_labels+2)




if __name__ == "__main__":

	# path2save = "/home/ubuntu/mounted_home/work/code_projects_WIP/catkin_real_robot_ws/src/unitree_ros_to_real_forked/unitree_legged_real/nodes/python/data_experiments_go1" # ubuntu VM
	# path2save = "/home/amarco/catkin_real_robot_ws/src/unitree_ros_to_real/unitree_legged_real/nodes/python/data_experiments_go1" # robot's laptop
	path2save = "/Users/alonrot/work/code_projects_WIP/catkin_real_robot_ws/src/unitree_ros_to_real_forked/unitree_legged_real/nodes/python/data_experiments_go1" # mac

	name_file = "2023_03_12_19_02_50_experiments_go1trajs.pickle"
	path2data = "{0:s}/{1:s}".format(path2save,name_file)

	print("Loading {0:s} ...".format(path2data))
	file = open(path2data, 'rb')
	data_dict = pickle.load(file)
	file.close()

	robot_pos = data_dict["robot_pos"]
	robot_vel = data_dict["robot_vel"]
	robot_orientation = data_dict["robot_orientation"]
	robot_angular_velocity = data_dict["robot_angular_velocity"]
	time_stamp = data_dict["time_stamp"]
	vel_forward_des = data_dict["vel_forward_des"]
	vel_yaw_des = data_dict["vel_yaw_des"]

	# Velocity profiles:
	hdl_fig_data, hdl_splots_data = plt.subplots(3,2,figsize=(12,8),sharex=True)
	hdl_fig_data.suptitle("Robot Pose (from Vicon)",fontsize=fontsize_labels)

	hdl_splots_data[0,0].plot(time_stamp,robot_pos[:,0],lw=1,alpha=0.3,color="navy")
	hdl_splots_data[1,0].plot(time_stamp,robot_pos[:,1],lw=1,alpha=0.3,color="navy")
	hdl_splots_data[2,0].plot(time_stamp,robot_pos[:,2],lw=1,alpha=0.3,color="navy")

	hdl_splots_data[0,1].plot(time_stamp,robot_orientation[:,0],lw=1,alpha=0.3,color="navy")
	hdl_splots_data[1,1].plot(time_stamp,robot_orientation[:,1],lw=1,alpha=0.3,color="navy")
	hdl_splots_data[2,1].plot(time_stamp,robot_orientation[:,2],lw=1,alpha=0.3,color="navy")

	hdl_splots_data[-1,0].set_xlabel("time [sec]",fontsize=fontsize_labels)
	hdl_splots_data[-1,1].set_xlabel("time [sec]",fontsize=fontsize_labels)

	# Velocity profiles:
	hdl_fig_data, hdl_splots_data = plt.subplots(3,2,figsize=(12,8),sharex=True)
	hdl_fig_data.suptitle("Robot Velocities (Differentiated Vicon signals)",fontsize=fontsize_labels)

	hdl_splots_data[0,0].plot(time_stamp,robot_vel[:,0],lw=1,alpha=0.3,color="navy")
	hdl_splots_data[1,0].plot(time_stamp,robot_vel[:,1],lw=1,alpha=0.3,color="navy")
	hdl_splots_data[2,0].plot(time_stamp,robot_vel[:,2],lw=1,alpha=0.3,color="navy")

	hdl_splots_data[0,1].plot(time_stamp,robot_angular_velocity[:,0],lw=1,alpha=0.3,color="navy")
	hdl_splots_data[1,1].plot(time_stamp,robot_angular_velocity[:,1],lw=1,alpha=0.3,color="navy")
	hdl_splots_data[2,1].plot(time_stamp,robot_angular_velocity[:,2],lw=1,alpha=0.3,color="navy")

	hdl_splots_data[-1,0].set_xlabel("time [sec]",fontsize=fontsize_labels)
	hdl_splots_data[-1,1].set_xlabel("time [sec]",fontsize=fontsize_labels)


	# Velocity profiles:
	hdl_fig_data, hdl_splots_data = plt.subplots(2,1,figsize=(12,8),sharex=True)
	hdl_fig_data.suptitle("Robot Control",fontsize=fontsize_labels)

	robot_vel_curr = np.sqrt(robot_vel[:,0]**2 + robot_vel[:,1]**2)
	hdl_splots_data[0].plot(time_stamp,robot_vel_curr,lw=1,alpha=0.8,color="navy")
	hdl_splots_data[0].plot(time_stamp,vel_forward_des,lw=1,alpha=0.3,color="navy")

	hdl_splots_data[1].plot(time_stamp,robot_angular_velocity[:,2],lw=1,alpha=0.8,color="navy")
	hdl_splots_data[1].plot(time_stamp,vel_yaw_des,lw=1,alpha=0.3,color="navy")

	hdl_splots_data[-1].set_xlabel("time [sec]",fontsize=fontsize_labels)


	plt.show(block=True)




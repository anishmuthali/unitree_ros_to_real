import numpy as np
import pdb
import pickle

from utils.data_parsing import plot_all, join_data


if __name__ == "__main__":

	# path2load_root = "/home/ubuntu/mounted_home/work/code_projects_WIP/catkin_real_robot_ws/src/unitree_ros_to_real_forked/unitree_legged_real/nodes/python/data_experiments_go1" # ubuntu VM
	# path2load_root = "/home/amarco/catkin_real_robot_ws/src/unitree_ros_to_real/unitree_legged_real/nodes/python/data_experiments_go1" # robot's laptop
	path2load_root = "/Users/alonrot/work/code_projects_WIP/catkin_real_robot_ws/src/unitree_ros_to_real_forked/unitree_legged_real/nodes/python/data_experiments_go1" # mac
	folder_name_experiments = "experiments_2023_03_13"
	path2load = "{0:s}/{1:s}".format(path2load_root,folder_name_experiments)

	data = dict()
	data.update(Shape_traj_1=dict(name_file_list=[],time_tot=15.0,pos_waypoints=np.array([[0.0,0.0],[1.0,1.0],[-1.0,2.0],[0.0,3.0]])))
	data["Shape_traj_1"]["name_file_list"] += ["2023_03_13_14_05_20_experiments_go1trajs.pickle"] # S-traj-1, 1
	data["Shape_traj_1"]["name_file_list"] += ["2023_03_13_14_06_04_experiments_go1trajs.pickle"] # S-traj-1, 2
	data["Shape_traj_1"]["name_file_list"] += ["2023_03_13_14_09_39_experiments_go1trajs.pickle"] # S-traj-1, 3
	data["Shape_traj_1"]["name_file_list"] += ["2023_03_13_14_10_44_experiments_go1trajs.pickle"] # S-traj-1, 4
	data["Shape_traj_1"]["name_file_list"] += ["2023_03_13_14_11_35_experiments_go1trajs.pickle"] # S-traj-1, 5

	data.update(Shape_traj_2=dict(name_file_list=[],time_tot=15.0,pos_waypoints=np.array([[0.0,0.0],[-1.0,1.0],[1.0,2.0],[0.0,3.0]])))
	data["Shape_traj_2"]["name_file_list"] += ["2023_03_13_14_16_30_experiments_go1trajs.pickle"] # S-traj-2, 1
	data["Shape_traj_2"]["name_file_list"] += ["2023_03_13_14_18_45_experiments_go1trajs.pickle"] # S-traj-2, 2
	data["Shape_traj_2"]["name_file_list"] += ["2023_03_13_14_20_10_experiments_go1trajs.pickle"] # S-traj-2, 3
	data["Shape_traj_2"]["name_file_list"] += ["2023_03_13_14_21_39_experiments_go1trajs.pickle"] # S-traj-2, 4
	data["Shape_traj_2"]["name_file_list"] += ["2023_03_13_14_23_12_experiments_go1trajs.pickle"] # S-traj-2, 5

	data.update(Straight_traj_left=dict(name_file_list=[],time_tot=7.5,pos_waypoints=np.array([[0.0,0.0],[-1.5,3.0]])))
	data["Straight_traj_left"]["name_file_list"] += ["2023_03_13_14_33_37_experiments_go1trajs.pickle"] # Straight-traj-left_corner, 1
	data["Straight_traj_left"]["name_file_list"] += ["2023_03_13_14_34_49_experiments_go1trajs.pickle"] # Straight-traj-left_corner, 2
	data["Straight_traj_left"]["name_file_list"] += ["2023_03_13_14_35_58_experiments_go1trajs.pickle"] # Straight-traj-left_corner, 3
	data["Straight_traj_left"]["name_file_list"] += ["2023_03_13_14_37_16_experiments_go1trajs.pickle"] # Straight-traj-left_corner, 4
	data["Straight_traj_left"]["name_file_list"] += ["2023_03_13_14_41_19_experiments_go1trajs.pickle"] # Straight-traj-left_corner, 5

	data.update(Straight_traj_right=dict(name_file_list=[],time_tot=7.5,pos_waypoints=np.array([[0.0,0.0],[1.0,3.0]])))
	data["Straight_traj_right"]["name_file_list"]  += ["2023_03_13_14_45_19_experiments_go1trajs.pickle"] # Straight-traj-right_corner, 1
	data["Straight_traj_right"]["name_file_list"]  += ["2023_03_13_14_46_19_experiments_go1trajs.pickle"] # Straight-traj-right_corner, 2
	data["Straight_traj_right"]["name_file_list"]  += ["2023_03_13_14_52_59_experiments_go1trajs.pickle"] # Straight-traj-right_corner, 3
	data["Straight_traj_right"]["name_file_list"]  += ["2023_03_13_14_54_03_experiments_go1trajs.pickle"] # Straight-traj-right_corner, 4
	data["Straight_traj_right"]["name_file_list"]  += ["2023_03_13_14_54_52_experiments_go1trajs.pickle"] # Straight-traj-right_corner, 5

	# name_file = "2023_03_13_13_01_09_experiments_go1trajs.pickle" # good, all signals coming, fixed time step
	# name_file = "2023_03_13_13_16_04_experiments_go1trajs.pickle" # good, all signals coming, fixed time step, vel profile at 120 Hz
	# name_file = "2023_03_13_13_30_35_experiments_go1trajs.pickle" # good, all signals coming, variable time step, vel profile at 120 Hz
	# name_file = "2023_03_13_13_33_40_experiments_go1trajs.pickle" # straight line
	# name_file = "2023_03_13_13_50_03_experiments_go1trajs.pickle" # straight line, corrected vel
	# name_file = "2023_03_13_13_52_41_experiments_go1trajs.pickle" # straight line, corrected vel, higher Kp





	# plot_all(data,path2load,subsample_every_nr_steps=10,ind_beg=1500,Ncut_end=4990)
	state_and_control_curr, state_next_traj = join_data(data,path2load,save_data_trajs_dict=True,subsample_every_nr_steps=10,ind_beg=1500,Ncut_end=4990,name_file2save="joined_go1trajs_trimmed.pickle")
	# pdb.set_trace()






	# Copy data to remote server:
	# scp -P 4444 -r data_experiments_go1/experiments_2023_03_13/joined_go1trajs_trimmed.pickle amarco@hybridrobotics.hopto.org:/home/amarco/code_projects/ood_project/ood/experiments/data_quadruped_experiments_03_13_2023/


	# Copy data to ood project directory:
	# cp data_experiments_go1/experiments_2023_03_13/joined_go1trajs_trimmed.pickle ~/work/code_projects_WIP/ood_project/ood/experiments/data_quadruped_experiments_03_13_2023/








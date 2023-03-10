import os
import numpy as np
import time
import pdb

import unitree_legged_msgs.msg # Located at /home/ubuntu/mounted_home/work/code_projects_WIP/catkin_real_robot_ws/devel/lib/python3/dist-packages (this path is added automatically to the PYTHONPATH after doing 'source devel/setup.bash')
# from unitree_legged_sdk_python_tools.utils.visualization_raisim  import VisualizeRaisim
import rospy

# Data logging:
import time
from datetime import datetime

import pickle
import math
from min_jerk_gen import min_jerk

import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib import cm
import matplotlib

markersize_x0 = 10
markersize_trajs = 0.4
fontsize_labels = 25
matplotlib.rc('xtick', labelsize=fontsize_labels)
matplotlib.rc('ytick', labelsize=fontsize_labels)
matplotlib.rc('text', usetex=False)
# matplotlib.rc('font',**{'family':'serif','serif':['Computer Modern Roman']})
plt.rc('legend',fontsize=fontsize_labels+2)


HIGHLEVEL = 0x00

def get_training_data_from_waypoints(deltaT,which_trajectory,save_data_trajs_dict=None,block_plot=True,plotting=True):
    """
    
    :return:
    Xtrain: [(Nsteps-3)*Ntrajs, dim_x+dim_u]
    Ytrain: [(Nsteps-3)*Ntrajs, dim_x]
    """

    # Trajectory duration:
    time_tot = 5.0 # sec

    # Number of steps:
    Nsteps = int(time_tot/deltaT) + 2 # We add 2 because numerical differentiation will suppress 2 points

    # Generate randomly the middle points of vel_lin_way_points, vel_ang_way_points, and also z0
    Ntrajs = 10
    Nwaypoints = 4
    x_lim = [0.0,5.0]; y_lim = [-5.0,5.0]; 
    # th_lim_low = [-math.pi/2., math.pi/2.]; # Why do we not need the heading? Because it's inferred from [x(t),y(t)] as th = arctan(yd(t) / xd(t)), where xd(t) = d/dt x(t)
    # s_lim = np.reshape(np.array([x_lim;y_lim;th_lim]),(2,-1))
    s_lim = np.vstack((x_lim,y_lim))
    pos_waypoints = s_lim[:,0] + (s_lim[:,1]-s_lim[:,0])*np.random.rand(Ntrajs,Nwaypoints,2)

    # Set the initial position to zero without loss of generality:
    pos_waypoints[:,0,:] = np.zeros(2)

    # Force the final position to be at a random value for x in [4,5]:
    pos_waypoints[:,-1,0] = 4. + (5.-4.)*np.random.rand(Ntrajs)

    # # Initial conditions:
    # z0_x_lim = 0.1; z0_y_lim = 0.1; z0_th_lim = math.pi;
    # z0_lim = np.reshape(np.array([z0_x_lim,z0_y_lim,z0_th_lim]),(1,-1))
    # z0_vec = -z0_lim + 2.*z0_lim*np.random.rand(Ntrajs,3)

    # Sort out the x positions in increasing order to prevent weird turns:
    pos_waypoints[:,:,0] = np.sort(pos_waypoints[:,:,0],axis=1)

    pos_profile_batch = np.zeros((Ntrajs,Nsteps,2))
    for ii in range(Ntrajs):
        if ii % 10 == 0: print(" Generating trajectory {0:d} / {1:d} ...".format(ii+1,Ntrajs))
        pos_profile_batch[ii,...],_ = min_jerk(pos=pos_waypoints[ii,...], dur=Nsteps, vel=None, acc=None, psg=None) # [Nsteps, D]


    # Velocity profiles and heading with numerical differentiation:
    vel_profile_batch = np.zeros((Ntrajs,Nsteps-1,2))
    th_profile_batch = np.zeros((Ntrajs,Nsteps-1,1))
    th_vel_profile_batch = np.zeros((Ntrajs,Nsteps-2,1))
    vel_profile_batch[...,0] = np.diff(pos_profile_batch[...,0],axis=1) / deltaT
    vel_profile_batch[...,1] = np.diff(pos_profile_batch[...,1],axis=1) / deltaT
    vel_mod_profile_batch = np.sqrt(vel_profile_batch[...,0:1]**2 + vel_profile_batch[...,1:2]**2)
    th_profile_batch[...,0] = np.arctan2(vel_profile_batch[...,1], vel_profile_batch[...,0])
    th_vel_profile_batch[...,0] = np.diff(th_profile_batch[...,0],axis=1) / deltaT

    # Subselect those very smooth ones:
    vx_is_positive = np.all(vel_profile_batch[...,0] >= -0.5,axis=1)
    th_is_within_range = np.all(abs(th_profile_batch[...,0]) <= 0.98*math.pi/2.,axis=1)
    ind_smooth = np.arange(0,Ntrajs)[vx_is_positive & th_is_within_range]

    print("Smooth trajectories: {0:d} / {1:d}".format(len(ind_smooth),Ntrajs))
    assert len(ind_smooth) > 0

    Nsteps_tot = th_vel_profile_batch.shape[1]
    state_tot = np.concatenate((pos_profile_batch[ind_smooth,0:Nsteps_tot,:],th_profile_batch[ind_smooth,0:Nsteps_tot,:]),axis=2) # [Ntrajs,Nsteps_tot,3], with Nsteps_tot=Nsteps-2 due to the integration issues
    vel_tot = np.concatenate((vel_mod_profile_batch[ind_smooth,0:Nsteps_tot,:],th_vel_profile_batch[ind_smooth,:,:]),axis=2) # [Ntrajs,Nsteps_tot,2], with Nsteps_tot=Nsteps-2 due to the integration issues

    # # Get a round number of trajectories:
    # # pdb.set_trace()
    # # if state_tot.shape[0] > 300 and vel_tot.shape[0] > 300:
    # #   state_tot = state_tot[0:300,...]
    # #   vel_tot = vel_tot[0:300,...]

    # state_and_control_tot = np.concatenate((state_tot,vel_tot),axis=2)

    # # The data needs to be reshaped in this particular way; it's incorrect to do this: Xtot = np.reshape(Ntrajs*(Nsteps-3),dim_x+dim_u); Xtrain_np = Xtot[0:-1,:]; Ytrain_np = Xtot[1::,:]
    # Xtrain_np = np.reshape(state_and_control_tot[:,0:-1,:],(state_and_control_tot.shape[0]*(state_and_control_tot.shape[1]-1),state_and_control_tot.shape[2]),order="C") # order="C" -> last axis index changing fastest
    # Ytrain_np = np.reshape(state_tot[:,1::,:],(state_tot.shape[0]*(state_tot.shape[1]-1),state_tot.shape[2]),order="C") # order="C" -> last axis index changing fastest

    # Xtrain = tf.convert_to_tensor(value=Xtrain_np,dtype=np.float32)
    # Ytrain = tf.convert_to_tensor(value=Ytrain_np,dtype=np.float32)

    if plotting:

        # Velocity profiles:
        hdl_fig_data, hdl_splots_data = plt.subplots(4,1,figsize=(12,8),sharex=True)
        time_vec = np.arange(0,Nsteps-1)*deltaT
        for ii in ind_smooth:
            hdl_splots_data[0].plot(time_vec,vel_profile_batch[ii,:,0],lw=1,alpha=0.3,color="navy")
            hdl_splots_data[1].plot(time_vec,vel_profile_batch[ii,:,1],lw=1,alpha=0.3,color="navy")
            hdl_splots_data[2].plot(time_vec,vel_mod_profile_batch[ii,:],lw=1,alpha=0.3,color="navy")
            hdl_splots_data[3].plot(time_vec[0:-1],th_vel_profile_batch[ii,:,0],lw=1,alpha=0.3,color="navy")

        # Select first trajectory:
        hdl_splots_data[2].plot(time_vec[0:-1],vel_tot[which_trajectory,:,0],alpha=0.9,color="navy",lw=3.0)
        hdl_splots_data[3].plot(time_vec[0:-1],vel_tot[which_trajectory,:,1],alpha=0.9,color="navy",lw=3.0)

        hdl_splots_data[0].set_ylabel(r"$v_x$ [m/s]",fontsize=fontsize_labels)
        hdl_splots_data[1].set_ylabel(r"$v_y$ [m/s]",fontsize=fontsize_labels)
        hdl_splots_data[2].set_ylabel(r"$v$ [m/s]",fontsize=fontsize_labels)
        hdl_splots_data[3].set_ylabel(r"$\dot{\theta}$ [rad/s]",fontsize=fontsize_labels)
        hdl_splots_data[-1].set_xlabel(r"$t$ [sec]",fontsize=fontsize_labels)
        hdl_splots_data[-1].set_xlim([time_vec[0],time_vec[-1]])
        hdl_splots_data[-1].set_xticks([time_vec[0],time_vec[-1]])


        # Trajectories:
        
        hdl_fig_data, hdl_splots_data = plt.subplots(1,1,figsize=(12,8),sharex=False)
        for ii in ind_smooth:
            hdl_splots_data.plot(pos_profile_batch[ii,:,0],pos_profile_batch[ii,:,1],alpha=0.3,color="navy")
            hdl_splots_data.plot(pos_profile_batch[ii,-1,0],pos_profile_batch[ii,-1,1],marker="x",color="black",markersize=5)
            hdl_splots_data.plot(pos_profile_batch[ii,0,0],pos_profile_batch[ii,0,1],marker=".",color="green",markersize=3)

        # Select first trajectory:
        hdl_splots_data.plot(state_tot[which_trajectory,:,0],state_tot[which_trajectory,:,1],alpha=0.9,color="navy",lw=3.0)
        hdl_splots_data.set_xlabel(r"$x$ [m]",fontsize=fontsize_labels)
        hdl_splots_data.set_ylabel(r"$y$ [m]",fontsize=fontsize_labels)



    Nsteps = state_tot.shape[1]-1
    if save_data_trajs_dict is not None:
        if save_data_trajs_dict["save"]:
            data2save = dict(state_tot=state_tot,vel_tot=vel_tot,Nsteps=Nsteps,Ntrajs=Ntrajs,deltaT=deltaT)
            file = open(save_data_trajs_dict["path2data"], 'wb')
            pickle.dump(data2save,file)
            file.close()
    elif plotting:
        plt.show(block=block_plot)

    return state_tot, vel_tot, Nsteps, Ntrajs




class RobotStatesCollection():

    def __init__(self,topic_high_state):

        self.Njoints = 12

        # Current positions (read from robot):
        self.joint_pos_curr = np.zeros(self.Njoints)
        self.com_pos_odometry = np.zeros(3)
        self.com_vel = np.zeros(3) # (unit: m/s), forwardSpeed, sideSpeed, rotateSpeed in body frame
        
        # Keep track of the low-level and high-level state:
        self.msg_high_state = unitree_legged_msgs.msg.HighState()

        """

        NOTE: We do not need to call spinOnce (in fact, there's no such a function in rospy) because
        rospy runs all the suscribers "in the background" once initialized.
        Se, we just set the flag to True for a bit and back to False, hoping that 
        self.joint_pos_curr will have been updated by then
        """

        # The callback will be called for each message published on the topic; we can't set the frequency
        self.topic_high_state = topic_high_state
        rospy.Subscriber(self.topic_high_state, unitree_legged_msgs.msg.HighState, self.callback_robot_high_state)

    def callback_robot_high_state(self,msg):

        self.msg_high_state = msg
        for ii in range(3):
            self.com_pos_odometry[ii] = msg.position[ii]
            self.com_vel[ii] = msg.velocity[ii]

        for ii in range(self.Njoints):
            self.joint_pos_curr[ii] = msg.motorState[ii].q

def main():
    """
    
    Use high-level Unitree commands to move the robot around the room using velocity profiles

    1. Create a velocity profile using a bunch of waypoints
    2. Publish the the velocity profile to the robot
    3. Collect data by subscribing to a few topics

    """

    np.random.seed(1)

    rate_freq_send_commands = 10 # Hz
    # save_data_trajs_dict = dict(save=True,path2data="/Users/alonrot/work/code_projects_WIP/catkin_real_robot_ws/src/unitree_ros_to_real_forked/unitree_legged_real/nodes/python/trajs_generated/trajs.pickle")
    save_data_trajs_dict = None
    which_trajectory = 0
    deltaT = 1./rate_freq_send_commands
    state_tot, vel_tot, Nsteps_control, Ntrajs = get_training_data_from_waypoints(deltaT=deltaT,which_trajectory=which_trajectory,
                                                                                    save_data_trajs_dict=save_data_trajs_dict,block_plot=False,plotting=False)
    # vel_tot: [Ntrajs,Nsteps_tot,2]

    rospy.init_node("walk_with_vel_profile", anonymous=False)
    rate_freq_read_state = 500 # Hz
    rate_read_state = rospy.Rate(rate_freq_read_state) # Hz
    Nsteps_read_states = int(Nsteps_control * rate_freq_read_state/rate_freq_send_commands)
    print("Nsteps_read_states: ",Nsteps_read_states)
    print("total execution time: {0:f} sec".format(Nsteps_read_states/rate_freq_read_state))

    topic_high_cmd = "high_state_from_robot"
    robot_state_collection = RobotStatesCollection(topic_high_cmd) # This starts subscribers to the current robot state

    topic_high_cmd = "high_cmd_to_robot"
    pub2high_cmd = rospy.Publisher(topic_high_cmd, unitree_legged_msgs.msg.HighCmd, queue_size=10)

    print("Reading robot states at ~{0:d} Hz".format(rate_freq_read_state))
    print("Sending velocity profile at ~{0:d} Hz".format(rate_freq_send_commands))
    print("Starting movement now!")
    input("Press any key to continue ...")
    time.sleep(1.0)
    # ii = 0
    # while(not rospy.is_shutdown() and ii < Nsteps_control):

    # Log data out:
    data_joint_fields = ["q_curr","dq_curr","ddq_curr","u_est"];
    data_joint_names = ["time_stamp","FR_0","FR_1","FR_2","FL_0","FL_1","FL_2","RR_0","RR_1","RR_2","RL_0","RL_1","RL_2"];

    data_endeffector_fields = ["time_stamp","vel_lin_forw_curr","vel_lin_side_curr","vel_ang_curr","vel_lin_forw_des","vel_lin_side_des","vel_ang_des","pos_odom_x","pos_odom_y","pos_odom_z"]

    # Message containing walking mode:
    msg_high_cmd = unitree_legged_msgs.msg.HighCmd()
    msg_high_cmd.levelFlag = HIGHLEVEL
    msg_high_cmd.mode = 2
    msg_high_cmd.gaitType = 1 # 0.idle  1.trot  2.trot running  3.climb stair
    msg_high_cmd.velocity[0] = 0.05 # [-1,1] # (unit: m/s), forwardSpeed, sideSpeed in body frame
    msg_high_cmd.bodyHeight = 0.0 # # (unit: m, default: 0.28m) -> this is measured w.r.t the current height....
    msg_high_cmd.yawSpeed = 0.0


    # Before we send the velocities, saturate them:
    fac_decrease = 0.6
    vel_tot[which_trajectory,:,0] = vel_tot[which_trajectory,:,0] / np.amax(vel_tot[which_trajectory,:,0]) * fac_decrease
    vel_tot[which_trajectory,:,1] = np.sign(vel_tot[which_trajectory,:,1])*abs(vel_tot[which_trajectory,:,1]) / np.amax(abs(vel_tot[which_trajectory,:,1]))*1.2


    # Velocity profiles:
    hdl_fig_data, hdl_splots_data = plt.subplots(2,1,figsize=(12,8),sharex=True)
    time_vec = np.arange(0,Nsteps_control)*deltaT
    hdl_splots_data[0].plot(time_vec,vel_tot[which_trajectory,0:-1,0],lw=1,alpha=0.3,color="navy")
    hdl_splots_data[1].plot(time_vec,vel_tot[which_trajectory,0:-1,1],lw=1,alpha=0.3,color="navy")
    hdl_splots_data[0].set_ylabel(r"$v$ [m/s]",fontsize=fontsize_labels)
    hdl_splots_data[1].set_ylabel(r"$\dot{\theta}$ [rad/s]",fontsize=fontsize_labels)
    hdl_splots_data[-1].set_xlabel(r"$t$ [sec]",fontsize=fontsize_labels)
    hdl_splots_data[-1].set_xlim([time_vec[0],time_vec[-1]])
    hdl_splots_data[-1].set_xticks([time_vec[0],time_vec[-1]])

    plt.show(block=False)
    plt.pause(1)


    ii = 0; cc = 0
    data_joints = np.zeros((len(data_joint_fields),Nsteps_read_states,len(data_joint_names)))
    data_endeff = np.zeros((len(data_endeffector_fields),Nsteps_read_states))
    time_start = time.time()
    save_data_from_experiment = True
    print("Starting loop!")
    print("Publishing high-level command, filled with desired trajectories at topic {0:s} ...".format(topic_high_cmd))
    # send_command = False
    send_command = True
    # Nsteps_read_states_applied = 1000
    Nsteps_read_states_applied = Nsteps_read_states
    while ii < Nsteps_read_states_applied:

        time_curr = time.time() - time_start

        # Change the commands we send every rate_freq_read_state//rate_freq_send_commands steps:
        if ii % (rate_freq_read_state//rate_freq_send_commands) == 0:
            if send_command:
                msg_high_cmd.velocity[0] = vel_tot[which_trajectory,cc,0] # desired linear velocity || vel_tot: [Ntrajs,Nsteps_tot,2]
                msg_high_cmd.yawSpeed = vel_tot[which_trajectory,cc,1] # desired angular velocity || vel_tot: [Ntrajs,Nsteps_tot,2]
            cc += 1
            print("Sending command to robot at frequency {0:d} Hz | tt = {1:d} / {2:d}".format(rate_freq_send_commands,cc,Nsteps_control))
        
        # Send commands all the time (they're changed above at a lower rate):
        pub2high_cmd.publish(msg_high_cmd)
            

        """
        Logging data
        """
        data_joints[:,ii,0] = time_curr # Time

        for jj in range(1,len(data_joint_names)):

            data_joints[0,ii,jj] = robot_state_collection.msg_high_state.motorState[jj].q
            data_joints[1,ii,jj] = robot_state_collection.msg_high_state.motorState[jj].dq
            data_joints[2,ii,jj] = robot_state_collection.msg_high_state.motorState[jj].ddq
            data_joints[3,ii,jj] = robot_state_collection.msg_high_state.motorState[jj].tauEst

        # data_endeffector_fields = ["time_stamp","vel_lin_curr","vel_ang_curr","vel_lin_des","vel_ang_des","pos_odom_x","pos_odom_y","pos_odom_z"]
        data_endeff[0,ii] = time_curr

        # Current velocity:
        data_endeff[1,ii] = robot_state_collection.msg_high_state.velocity[0]
        data_endeff[2,ii] = robot_state_collection.msg_high_state.velocity[1]
        data_endeff[3,ii] = robot_state_collection.msg_high_state.velocity[2]
        
        # Desired velocity:
        data_endeff[4,ii] = msg_high_cmd.velocity[0]
        data_endeff[5,ii] = msg_high_cmd.velocity[1]
        data_endeff[6,ii] = msg_high_cmd.yawSpeed

        # Estimated position:
        data_endeff[7,ii] = robot_state_collection.msg_high_state.position[0]
        data_endeff[8,ii] = robot_state_collection.msg_high_state.position[1]
        data_endeff[9,ii] = robot_state_collection.msg_high_state.position[2]

        rate_read_state.sleep()

        ii += 1

    # Reset to mode 0:
    print("Back to standing still....")
    msg_high_cmd.mode = 1
    msg_high_cmd.gaitType = 0 # 0.idle  1.trot  2.trot running  3.climb stair
    msg_high_cmd.velocity[0] = 0.0 # [-1,1] # (unit: m/s), forwardSpeed, sideSpeed in body frame
    msg_high_cmd.bodyHeight = 0.0 # # (unit: m, default: 0.28m)
    ii = 0
    while ii < 500:
        pub2high_cmd.publish(msg_high_cmd)
        rate_read_state.sleep()
        ii += 1


    # Save data:
    if save_data_from_experiment:
        data2save = dict(data_joint_fields=data_joint_fields,data_joint_names=data_joint_names,data_endeffector_fields=data_endeffector_fields,\
            data_joints=data_joints,data_endeff=data_endeff,deltaT=deltaT)
        # path2save = "/Users/alonrot/work/code_projects_WIP/catkin_real_robot_ws/src/unitree_ros_to_real_forked/unitree_legged_real/nodes/python/trajs_generated"
        path2save = "/home/amarco/catkin_real_robot_ws/src/unitree_ros_to_real/unitree_legged_real/nodes/python/trajs_generated"
        name_file_data = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        name_file = "{0:s}_trajs_go1_walking.pickle".format(name_file_data)
        path2save_full = "{0:s}/{1:s}".format(path2save,name_file)
        print("Saving data at {0:s} ...".format(path2save_full))
        file = open(path2save_full, 'wb')
        pickle.dump(data2save,file)
        file.close()
        print("Done!")
    else:
        plt.show(block=True)



def plot_saved_trajs():

    # path2load = "/home/amarco/catkin_real_robot_ws/src/unitree_ros_to_real/unitree_legged_real/nodes/python/trajs_generated/2023_03_09_21_33_19_trajs_go1_walking.pickle"
    # path2load = "/home/amarco/catkin_real_robot_ws/src/unitree_ros_to_real/unitree_legged_real/nodes/python/trajs_generated/2023_03_09_21_50_07_trajs_go1_walking.pickle"
    # path2load = "/home/amarco/catkin_real_robot_ws/src/unitree_ros_to_real/unitree_legged_real/nodes/python/trajs_generated/2023_03_09_21_56_06_trajs_go1_walking.pickle"
    path2load = "/home/amarco/catkin_real_robot_ws/src/unitree_ros_to_real/unitree_legged_real/nodes/python/trajs_generated/2023_03_09_21_59_56_trajs_go1_walking.pickle"
    


    file = open(path2load, "rb")
    data_dict = pickle.load(file)
    file.close()

    data_endeff = data_dict["data_endeff"]

    # Velocity profiles:
    hdl_fig_data, hdl_splots_data = plt.subplots(2,1,figsize=(12,8),sharex=True)
    hdl_splots_data[0].set_title("Linear forward velocity",fontsize=fontsize_labels)
    hdl_splots_data[0].plot(data_endeff[0,:],data_endeff[4,:],lw=1,alpha=0.3,color="navy",label="Desired")
    hdl_splots_data[0].plot(data_endeff[0,:],data_endeff[1,:],lw=1,alpha=0.8,color="navy",label="Current")

    hdl_splots_data[1].set_title("Angular velocity",fontsize=fontsize_labels)
    hdl_splots_data[1].plot(data_endeff[0,:],data_endeff[6,:],lw=1,alpha=0.3,color="navy",label="Desired")
    hdl_splots_data[1].plot(data_endeff[0,:],data_endeff[3,:],lw=1,alpha=0.8,color="navy",label="Current")
    hdl_splots_data[-1].set_xlabel(r"$t$ [sec]",fontsize=fontsize_labels)
    hdl_splots_data[-1].set_xlim([data_endeff[0,0],data_endeff[0,-1]])
    hdl_splots_data[-1].set_xticks([data_endeff[0,0],data_endeff[0,-1]])

    plt.show(block=True)




if __name__ == "__main__":

    # main()

    plot_saved_trajs()



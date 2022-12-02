import os
import numpy as np
import time
import pdb

import unitree_legged_msgs.msg # Located at /home/ubuntu/mounted_home/work/code_projects_WIP/catkin_real_robot_ws/devel/lib/python3/dist-packages (this path is added automatically to the PYTHONPATH after doing 'source devel/setup.bash')
# from unitree_legged_sdk_python_tools.utils.visualization_raisim  import VisualizeRaisim
import rospy


class GoToJointPosition():

    def __init__(self):

        self.Njoints = 12
        self.joint_pos_init = np.zeros(self.Njoints)
        self.joint_pos_target = np.zeros(self.Njoints)

        # Flags:
        self.update_initial_position = False

        # The callback will be called for each message published on the topic; we can't set the frequency
        self.topic_low_state = "low_state_from_robot"
        rospy.Subscriber(self.topic_low_state, unitree_legged_msgs.msg.LowState, self.callback_robot_state)

    def callback_robot_state(self,msg):

        if not self.update_initial_position:
            return

        for ii in range(self.Njoints):
            self.joint_pos_init[ii] = msg.motorState[ii].q
        print("Inside callback..")
        print("self.joint_pos_init: ",self.joint_pos_init)

    def update_target(self,joint_pos_target):
        self.joint_pos_target = joint_pos_target

    def interpolation_linear(self,ind,Nsteps):

        alpha = min(ind / Nsteps,1)
        joint_pos_curr = self.joint_pos_init*(1.-alpha) + self.joint_pos_target*alpha

        return joint_pos_curr

    def reset_reading_of_initial_position(self):
        """

        NOTE: We do not need to call spinOnce (in fact, there's no such a function in rospy) because
        rospy runs all the suscribers "in the background" once initialized.
        Se, we just set the flag to True for a bit and back to False, hoping that 
        self.joint_pos_init will have been updated by then
        """

        print("Reading current robot position from the topic {0:s} ...".format(self.topic_low_state))
        self.update_initial_position = True
        time.sleep(2.0)
        self.update_initial_position = False

        print("Current robot position: ",self.joint_pos_init)


def main(joint_pos_target):
    """

    1. Read the current robot's position from topic "low_state_from_robot"
    2. Create a smooth transition to a desired target and publish it

    This program assumes that position_holder is running in the network: "rosrun unitree_legged_real position_holder"

    """

    rospy.init_node("go_to", anonymous=False)
    rate_freq = 500 # Hz
    rate = rospy.Rate(rate_freq) # Hz

    go2_joint_position = GoToJointPosition() # This starts a subscriber to the current robot state

    # Read initial position from the network:
    go2_joint_position.reset_reading_of_initial_position()

    # Start raisim server:
    use_raisim_visualization = True
    # if use_raisim_visualization: visualize_raisim = VisualizeRaisim()

    go2_joint_position.update_target(joint_pos_target)

    pub2low_cmd = rospy.Publisher("low_cmd_to_robot", unitree_legged_msgs.msg.LowCmd, queue_size=10)
    msg_low_cmd = unitree_legged_msgs.msg.LowCmd()

    Nsteps_total = 4000
    Nsteps_transition = 3000
    print("Moving to target within {0:f} seconds; the program will exit after {0:f} seconds ...".format(1/rate_freq*Nsteps_transition,1/rate_freq*Nsteps_total))
    input("Press any key to continue ...")
    time.sleep(1.0)
    # ii = 0
    # while(not rospy.is_shutdown() and ii < Nsteps_total):
    for ii in range(Nsteps_total):

        joint_pos_des = go2_joint_position.interpolation_linear(ind=ii,Nsteps=Nsteps_transition)

        for ii in range(go2_joint_position.Njoints):
            msg_low_cmd.motorCmd[ii].q = joint_pos_des[ii]

        pub2low_cmd.publish(msg_low_cmd)

        print("joint_pos_des:",joint_pos_des)

        # if use_raisim_visualization: visualize_raisim.update_visualization(joint_pos_curr,joint_pos_des)

        # ii += 1
        rate.sleep()

    return;


if __name__ == "__main__":


    # Target position we want to reach: Stand-up position
    joint_pos_target = np.array([0.0136, 0.7304, -1.4505, -0.0118, 0.7317, -1.4437, 0.0105, 0.6590, -1.3903, -0.0102, 0.6563, -1.3944])

    # Target position we want to reach: Lying-down position
    # joint_pos_target = np.array([-0.303926,1.15218,-2.69135,0.346799,1.17985,-2.73951,-0.348858,1.15957,-2.75885,0.348737,1.20456,-2.79926])

    main(joint_pos_target)



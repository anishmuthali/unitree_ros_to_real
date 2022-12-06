/* --------------------------------------------------------------------------------------------------------------- */
/* ------------------------------------------ Position Holder ---------------------------------------------------- */
/* --------------------------------------------------------------------------------------------------------------- */

#ifndef _POSITION_HOLDER_H_
#define _POSITION_HOLDER_H_

// #include <string>
// #include <pthread.h>
// #include <boost/thread.hpp>
// #include <boost/thread/mutex.hpp>
// #include <unitree_legged_msgs/LowState.h>
// #include "convert.h"
#include <ros/ros.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <python_interface_go1/interface_real_robot.hpp>


class PositionHolderControlLoop : public RobotInterfaceGo1 {

public:

    PositionHolderControlLoop(  const Eigen::Ref<Vector12d>& P_gains, 
                                const Eigen::Ref<Vector12d>& D_gains,
                                int Nsteps_timeout = 20, 
                                int time_sleep_ms = 500,
                                bool verbosity = true) : RobotInterfaceGo1(){

        this->joint_pos_des_hold.setZero();

        this->set_PD_gains(P_gains,D_gains);
        // int Nsteps_timeout = 20; // Have a larget timeout
        // int time_sleep_ms = 500; // Seconds
        // bool verbosity = true;
        this->read_initial_position(Nsteps_timeout,time_sleep_ms,this->joint_pos_des_hold,verbosity);
    }

    ~PositionHolderControlLoop(){
        std::cout << "Destroying PositionHolderControlLoop class ...\n";
    }

    void update_joint_pos_des_hold();
    void send_position2hold();
    void go2target_linear_interpolation( const Eigen::Ref<Vector12d>& joint_pos_init, 
                                        const Eigen::Ref<Vector12d>& joint_pos_final,
                                        Eigen::Ref<Vector12d> joint_pos_interp,
                                        double rate);

    void lowCmdCallback(const unitree_legged_msgs::LowCmd::ConstPtr &msg);

    void read_initial_position(int Nsteps_timeout, int time_sleep_ms, Eigen::Ref<Vector12d> joint_pos_init, bool verbosity = false);

    void mode_change(void);

    // Desired position to hold:
    Vector12d joint_pos_des_hold;

    // Flags:
    bool is_desired_position_all_zeros;

};


#endif  // _POSITION_HOLDER_H_
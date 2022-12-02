/* --------------------------------------------------------------------------------------------------------------- */
/* ------------------------------------------ Position Holder ---------------------------------------------------- */
/* --------------------------------------------------------------------------------------------------------------- */

#ifndef _POSITION_HOLDER_H_
#define _POSITION_HOLDER_H_

#include <ros/ros.h>
// #include <string>
// #include <pthread.h>
// #include <boost/thread.hpp>
// #include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/LowCmd.h>
// #include <unitree_legged_msgs/LowState.h>
// #include "convert.h"

#include <python_interface_go1/interface_real_robot.hpp>

class PositionHolderControlLoop : public RobotInterfaceGo1 {

public:

    PositionHolderControlLoop() : RobotInterfaceGo1(){

        this->joint_pos_des_hold.setZero();

    }

    ~PositionHolderControlLoop(){

        std::cout << "Destroying PositionHolderControlLoop class ...\n";
    }

    void update_joint_pos_des_hold();
    void send_position2hold();
    void go2target_linear_interpolation( const Eigen::Ref<Vector12d>& joint_pos_init, 
                                        const Eigen::Ref<Vector12d>& joint_pos_final,
                                        Eigen::Ref<Vector12d> & joint_pos_interp,
                                        double rate);

    void lowCmdCallback(const unitree_legged_msgs::LowCmd::ConstPtr &msg);

    void write_current_position_into_position2hold();

    // Desired position to hold:
    Vector12d joint_pos_des_hold;

};


#endif  // _POSITION_HOLDER_H_
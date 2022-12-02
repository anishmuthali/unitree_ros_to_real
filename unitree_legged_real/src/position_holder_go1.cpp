/* --------------------------------------------------------------------------------------------------------------- */
/* ------------------------------------------ Position Holder ---------------------------------------------------- */
/* --------------------------------------------------------------------------------------------------------------- */


#include <position_holder_go1.hpp>

void PositionHolderControlLoop::go2target_linear_interpolation( const Eigen::Ref<Vector12d>& joint_pos_init, 
                                                    const Eigen::Ref<Vector12d>& joint_pos_final,
                                                    Eigen::Ref<Vector12d> & joint_pos_interp,
                                                    double rate){
    
    // double rate = std::min(std::max(rate, 0.0), 1.0);
    joint_pos_interp = joint_pos_init*(1.-rate) + joint_pos_final*rate;

    return;
}


void PositionHolderControlLoop::lowCmdCallback(const unitree_legged_msgs::LowCmd::ConstPtr &msg)
{
    
    // UNITREE_LEGGED_SDK::LowCmd low_cmd_subs = {0};
    // low_cmd_subs = ToLcm(msg,low_cmd_subs);

    // NOTE: no need to convert to LCM types because we're just receiving and copying to joint_pos_des_hold
    // Read desired position, broadcasted to the network
    // Write the desired position in the global joint_pos_des_hold:
    for(int ii=0; ii < this->Njoints; ii++){
        // joint_pos_des_hold[ii] = low_cmd_subs.motorCmd[ii].q;
        this->joint_pos_des_hold[ii] = msg->motorCmd[ii].q;
    }

    std::cout << "Receiving joint_pos_des_hold ... || joint_pos_des_hold[0] = " << this->joint_pos_des_hold[0] << "\n";

    return;
}


void PositionHolderControlLoop::write_current_position_into_position2hold(){

    for(int ii=0; ii < this->Njoints; ii++){
        this->joint_pos_des_hold[ii] = this->state.motorState[ii].q;
    }
}

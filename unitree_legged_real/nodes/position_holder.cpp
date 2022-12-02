/**********************************************************************************************
 * 
**********************************************************************************************/



#include <ros/ros.h>
#include <string>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <position_holder_go1.hpp>

#include "convert.h" // Needed for ToRos

// #include <signal.h>

using namespace UNITREE_LEGGED_SDK;


// void mySigintHandler(int sig){

//     ros::shutdown();
// }


int main(int argc, char *argv[])
{

    // Input arguments:
    for(int ii=0;ii < argc; ii++){
        std::cout << "argv[" << ii << "]: " << argv[ii] << "\n";
    }

    if(argc != (4+3)){
        std::cout << "Expected 4 input arguments, but " << argc-3 << " were passed!\n";
        // std::cout << "Use: roslaunch"
        return 0;
    }

    // argv[0]: /home/ubuntu/mounted_home/work/code_projects_WIP/catkin_real_robot_ws/devel/lib/unitree_legged_real/position_holder
    // argv[1]: low_cmd_to_robot_launch
    // argv[2]: low_state_from_robot_launch
    // argv[3]: 1000
    // argv[4]: 500
    // argv[5]: __name:=node_position_holder
    // argv[6]: __log:=/home/ubuntu/.ros/log/4d8191f8-7213-11ed-9f47-87121fcf993e/node_position_holder-1.log


    std::string topic_subscribe_to_user_commands(argv[1]);
    std::string topic_publish_robot_state(argv[2]);
    int Ndur_read_init_pos = std::stoi(argv[3]);
    int loop_frequency = std::stoi(argv[4]);
    
    ros::init(argc, argv, "node_position_holder");

    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle nh;
    ros::Rate loop_rate(loop_frequency); // amarco: Frequency in Hz

    std::cout << "Initializing Go1 interface ...\n";
    PositionHolderControlLoop position_holder;

    // typedef Eigen::Matrix<double, 12, 1> Vector12d; // Column vector by default
    Vector12d P_gains;
    Vector12d D_gains;
    P_gains << 40.0,40.0,40.0,40.0,40.0,40.0,40.0,40.0,40.0,40.0,40.0,40.0;
    D_gains << 2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0,2.0;

    position_holder.set_PD_gains(P_gains,D_gains);
    position_holder.set_deltaT(1./(double)loop_frequency); // seconds

    // Initialize global variables:
    unitree_legged_msgs::LowState state_msg;
    // ros::Subscriber sub_low = nh.subscribe("low_cmd_to_robot", 100, &PositionHolderControlLoop::lowCmdCallback, &position_holder); // Receive commands from the network
    ros::Subscriber sub_low = nh.subscribe(topic_subscribe_to_user_commands, 100, &PositionHolderControlLoop::lowCmdCallback, &position_holder); // Receive commands from the network
    // ros::Publisher pub_low = nh.advertise<unitree_legged_msgs::LowState>("low_state_from_robot", 100);
    ros::Publisher pub_low = nh.advertise<unitree_legged_msgs::LowState>(topic_publish_robot_state, 100);

    std::cout << "Receiving LCM low-level state data from the robot and publishing it ...\n";
    std::cout << "Subscribing to low-level commands from the network and sending them to the robot via LCM ...\n";

    // Read initial position, do not send anything:
    long motiontime = 0;
    // int Ndur_read_init_pos = 1000;
    std::cout << "Reading initial position for " << Ndur_read_init_pos << " seconds ...\n";
    while (motiontime < Ndur_read_init_pos){

        position_holder.CollectObservations();
        position_holder.update_all_observations();

        position_holder.write_current_position_into_position2hold(); // Just read, do not send anything

        motiontime++;
        loop_rate.sleep();

        // No need to call ros::spinOnce() through the callbacks because we're not interested in listening commands from the network yet
        // We need to read the robot's current joint position and store it in position2hold
    }

    std::cout << "Initial position:\n";
    position_holder.print_joint_info("(initial) position2hold",position_holder.joint_pos_des_hold);


    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // // This overrides the default sigint handler (must be set after the first node is created):
    // signal(SIGINT, mySigintHandler);

    // Send to the robot whatever is inside position2hold using the pre-defined PD gains
    // position2hold is only modified by the lowCmdCallback, which is listening to incoming messages from the network
    std::cout << "reading position2hold from the subscriber and sending it to the robot indefinitely ...\n";
    while (ros::ok()){

        position_holder.CollectObservations();
        position_holder.update_all_observations();

        position_holder.send_desired_position(position_holder.joint_pos_des_hold);

        // Publish the current state:
        state_msg = ToRos(position_holder.state);
        pub_low.publish(state_msg);

        ros::spinOnce(); // Go through the callbacks and fill position2hold with the commands collected from the network in position_holder.lowCmdCallback()
        loop_rate.sleep();
    }

    std::cout << "Finishing position_holder ...\n";


    return 0;
}


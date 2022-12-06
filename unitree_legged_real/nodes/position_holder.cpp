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

// #include "data_parser.hpp"

using namespace UNITREE_LEGGED_SDK;


// void mySigintHandler(int sig){

//     ros::shutdown();
// }

// typedef Eigen::Matrix<double, 12, 1> Vector12d; // Column vector by default

template<typename T>
void parse_input_argument(const ros::NodeHandle & nh, const std::string & var_name, T & var_out){

    // http://wiki.ros.org/roscpp/Overview/Names%20and%20Node%20Information#Manipulating_Names
    // http://docs.ros.org/en/melodic/api/roscpp/html/classros_1_1NodeHandle.html

    // std::string this_node;
    // this_node = ros::this_node::getName();
    // std::cout << "this_node: " << this_node << "\n";

    // std::string resolved_name;
    // resolved_name = nh.resolveName(var_name); // Resolve a name using the NodeHandle's namespace. Returns ns+var_name; ns="/" by default, can be overridden in the launch file
    // std::cout << "resolved_name: " << resolved_name << "\n";


    // if(nh.getParam(var_name,var)){
    // std::string name_param_full("/"+node_name+"/"+var_name);
    std::string name_param_full = ros::this_node::getName() + nh.resolveName(var_name);
    if(nh.getParam(name_param_full,var_out)){
        std::cout << name_param_full << ": " << var_out << "\n";
    }
    else{
        std::cout << name_param_full << " failed to load because it doesn't exist...\n";
    }

    return;
}

void parse_input_vector(const ros::NodeHandle & nh, const std::string & vec_name, Eigen::VectorXd & vec_out){

    // http://wiki.ros.org/roscpp/Overview/Names%20and%20Node%20Information#Manipulating_Names
    // http://docs.ros.org/en/melodic/api/roscpp/html/classros_1_1NodeHandle.html

    std::string name_param_full = ros::this_node::getName() + nh.resolveName(vec_name);
    Eigen::IOFormat clean_format = Eigen::IOFormat(4, 0, ", ", "\n", "[", "]");

    std::vector<double> vec_doubles;
    if(nh.getParam(name_param_full,vec_doubles)){
        vec_out = Eigen::Map<Eigen::VectorXd>(vec_doubles.data(), vec_doubles.size());
        std::cout << name_param_full << ": " << vec_out.transpose().format(clean_format) << "\n";
    }
    else{
        std::cout << name_param_full << " failed to load because it doesn't exist...\n";
    }

    return;
}


int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "node_position_holder"); // Node name is overriden with field 'name' inside th launch file

    ros::NodeHandle nh;

    // DataParser data_parser(nh);
    
    // Parsing input arguments:
    std::string topic_subscribe_to_user_commands, topic_publish_robot_state;
    parse_input_argument(nh,"topic_subscribe_to_user_commands",topic_subscribe_to_user_commands);
    parse_input_argument(nh,"topic_publish_robot_state",topic_publish_robot_state);
    
    int loop_frequency, Nsteps_timeout, time_sleep_ms;
    parse_input_argument(nh,"loop_frequency",loop_frequency);
    parse_input_argument(nh,"Nsteps_timeout",Nsteps_timeout);
    parse_input_argument(nh,"time_sleep_ms",time_sleep_ms);
    
    Eigen::VectorXd P_gains, D_gains;
    parse_input_vector(nh,"P_gains",P_gains);
    parse_input_vector(nh,"D_gains",D_gains);
    
    bool verbosity;
    parse_input_argument(nh,"verbosity",verbosity);

    std::cout << "Initializing Go1 interface ...\n";
    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::Rate loop_rate(loop_frequency); // amarco: Frequency in Hz

    PositionHolderControlLoop position_holder(P_gains,D_gains,Nsteps_timeout,time_sleep_ms,verbosity);
    position_holder.set_deltaT(1./(double)loop_frequency); // seconds

    // Initialize global variables:
    unitree_legged_msgs::LowState state_msg;
    // ros::Subscriber sub_low = nh.subscribe("low_cmd_to_robot", 100, &PositionHolderControlLoop::lowCmdCallback, &position_holder); // Receive commands from the network
    ros::Subscriber sub_low = nh.subscribe(topic_subscribe_to_user_commands, 100, &PositionHolderControlLoop::lowCmdCallback, &position_holder); // Receive commands from the network
    // ros::Publisher pub_low = nh.advertise<unitree_legged_msgs::LowState>("low_state_from_robot", 100);
    ros::Publisher pub_low = nh.advertise<unitree_legged_msgs::LowState>(topic_publish_robot_state, 100);

    std::cout << "Receiving LCM low-level state data from the robot and publishing it ...\n";
    std::cout << "Subscribing to low-level commands from the network and sending them to the robot via LCM ...\n";

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
    delete(&nh); // amarco: not sure if this is the right way...
    ros::shutdown();

    return 0;
}


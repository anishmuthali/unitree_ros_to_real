/************************************************************************
*
* @brief ROS interface: publishes robot's readings (HighState) and subscribes 
* to robot's desired commands (HighCmd)
*
* @author Alonso Marco
* Contact: amarco@berkeley.edu
************************************************************************/

#include <ros/ros.h>
#include <string>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/array.hpp>
#include <boost/range/algorithm.hpp>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/SpatialState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/unitree_joystick.h"
#include <array>
#include <math.h>
#include <eigen3/Eigen/Core>
#include <vector>
#include <array>
#include <std_msgs/Float32.h>

// #include <data_parser.hpp>

// #include "convert.h" // Needed for ToRos()

using namespace UNITREE_LEGGED_SDK;

#include <python_interface_go1/real_robot_interface_go1_highlevel.hpp>


class ControllerReceiver {
  public:
    ControllerReceiver(uint16_t localPort, uint16_t targetPort, const char* targetIP) : 
      safe(LeggedType::Go1),
      udp(localPort, targetIP, targetPort, sizeof(HighCmd), sizeof(HighState)) {
        udp.InitCmdData(cmd);
    }

    void receiveState() {
      udp.Recv();
      udp.GetRecv(state);
      std::cout << "position: " << state.position[0] << ", " << state.position[1] << ", " << state.position[2] << std::endl;
    }

    xRockerBtnDataStruct getControllerData() {
      xRockerBtnDataStruct result;
      memcpy(&result, state.wirelessRemote, 40);
      return result;
    }

    boost::array<float,3> getRobotPosition() {
      boost::array<float,3> result = {{state.position[0], state.position[1], state.position[2]}};
      return result;
    }

    boost::array<float,3> getRobotVelocity() {
      boost::array<float,3> result = {{state.velocity[0], state.velocity[1], state.velocity[2]}};
      return result;
    }

    IMU getIMU() {
      return state.imu;
    }



  // private:
  HighCmd cmd = {0};
  HighState state = {0};
  Safety safe;
  UDP udp;
};






int main(int argc, char *argv[])
{
  /*
    This node:
    1) Reads the current highlevel robot state and publishes it
    2) Listens to user commands from the network through a subscriber
    3) The class used inherits from RealRobotInterfaceGo1HighLevel, defined in another repo: 
        <catkin_ws>/src/unitree_legged_sdk_from_inside_robot/include/python_interface_go1/real_robot_interface_go1_highlevel.hpp
   */
    
    ros::init(argc, argv, "node_ros_wireless_controller"); // Node name is overriden with field 'name' inside th launch file

    ros::NodeHandle nh;

    // Parsing input arguments:
    // DataParser data_parser(nh);
    // std::cout << "Loading parameters:\n";
    // std::string topic_subscribe_to_user_commands, topic_publish_robot_state;
    // data_parser.get("topic_subscribe_to_user_commands",topic_subscribe_to_user_commands);
    // data_parser.get("topic_publish_robot_state",topic_publish_robot_state);
    // int loop_frequency;
    // data_parser.get("loop_frequency",loop_frequency);
    // int localPort;
    // int targetPort;
    // std::string targetIP;
    // data_parser.get("localPort",localPort);
    // data_parser.get("targetPort",targetPort);
    // data_parser.get("targetIP",targetIP);
    int loop_frequency = 500;
    int localPort = 8090;
    int targetPort = 8082;
    string targetIP = "192.168.12.1";
    string topic_publish_robot_state = "high_state_from_robot";
    string topic_subscribe_to_user_commands = "high_cmd_to_robot";

    ControllerReceiver cr = ControllerReceiver(localPort, targetPort, targetIP.c_str());


    // Set loop frequency:
    ros::Rate loop_rate(loop_frequency); // amarco: Frequency in Hz

    std::cout << "Initializing Go1 interface ...\n";
    std::cout << " * This node has three main functionalities:\n";
    std::cout << "   (1) Initialize the parent class RealRobotInterfaceGo1HighLevel, which \n";
    std::cout << "       communicates with the robot via Unitree's UDP protocols \n";
    std::cout << "   (2) Publish the current high level robot state to the topic " << topic_publish_robot_state << " \n";
    std::cout << "   (3) Subscribe to high-level user commands found at the topic " << topic_subscribe_to_user_commands << " \n";
    std::cout << " * The parent class RealRobotInterfaceGo1HighLevel does not use LCM";
    
    ros::Publisher pub_spatial_state = nh.advertise<unitree_legged_msgs::SpatialState>(topic_publish_robot_state, 100);


    std::cout << "[WARNING]: Control level is set to HIGHLEVEL." << std::endl
              << " (1) Make sure the robot is standing." << std::endl
              << " (2) Press Enter to start the loop..." << std::endl;
    std::cin.ignore();

    // // This overrides the default sigint handler (must be set after the first node is created):
    // signal(SIGINT, mySigintHandler);

    std::cout << "Publishing the robot state and listening to user commands indefinitely ...\n";
    while (ros::ok()){

        // Receive observations from the robot and update the .state field
        cr.receiveState();

        // Publish the current state:
        unitree_legged_msgs::SpatialState spatialState;

        spatialState.position = cr.getRobotPosition();
        spatialState.velocity = cr.getRobotVelocity();        


        pub_spatial_state.publish(spatialState);

        // DBG TODO
        // std::cout << "Could it be that ToRos() is not converting it properly? Why aren't we getting foot readings?\n";

        ros::spinOnce(); // Go through the callbacks created otuside this loop; specifically, check for new user commands inside the callback ceated with the subscriber sub_high_cmd
        loop_rate.sleep();
    }

    std::cout << "Finishing ros_robot_interface_go1_highlevel ...\n";
    delete(&nh); // amarco: not sure if this is the right way...
    ros::shutdown();

    return 0;
}


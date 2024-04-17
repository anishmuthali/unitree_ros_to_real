/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/unitree_joystick.h"
#include <math.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <exception>
#include <ros/ros.h>

using namespace UNITREE_LEGGED_SDK;

class InterruptException : public std::exception
{
public:
  InterruptException(int s) : S(s) {}
  int S;
};


void sig_to_exception(int s)
{
  throw InterruptException(s);
}

class Custom
{
public:
    Custom(uint8_t level, string fileName): 
      safe(LeggedType::Go1), 
      udp(8090, "192.168.12.1", 8082, sizeof(HighCmd), sizeof(HighState)),
      saveFile(fileName)
    {
        udp.InitCmdData(cmd);
        saveFile << "x,y,z,forward_speed,side_speed,rotate_speed,gait_type,"
        << "foot_raise_height,yaw_speed,body_height,"
        << "q_w,q_x,q_y,q_z,w_x,w_y,w_z,a_x,a_y,a_z,roll,pitch,yaw\n";
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();
    void processState(HighState state);

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
    std::ofstream saveFile;
};

void Custom::processState(HighState state)
{
    saveFile << state.position[0] << "," << state.position[1] << "," << state.position[2] << ",";
    saveFile << state.velocity[0] << "," << state.velocity[1] << "," << state.velocity[2] << ",";
    saveFile << state.gaitType << "," << state.footRaiseHeight << "," << state.yawSpeed << "," << state.bodyHeight << ",";
    IMU currIMU = state.imu;
    saveFile << currIMU.quaternion[0] << "," << currIMU.quaternion[1] << "," << currIMU.quaternion[2] << "," << currIMU.quaternion[3] << ",";
    saveFile << currIMU.gyroscope[0] << "," << currIMU.gyroscope[1] << "," << currIMU.gyroscope[2] << ",";
    saveFile << currIMU.accelerometer[0] << "," << currIMU.accelerometer[1] << "," << currIMU.accelerometer[2] << ",";
    saveFile << currIMU.rpy[0] << "," << currIMU.rpy[1] << "," << currIMU.rpy[2] << "\n";
}

void displayJoystick(HighState state)
{
    xRockerBtnDataStruct joysticks;
    memcpy(&joysticks, state.wirelessRemote, 40);
    std::cout << "lx: " << joysticks.lx << ", ly: " << joysticks.ly << "\nrx: " << joysticks.rx << ", ry:" << joysticks.ry << "\n";
}

void Custom::UDPRecv()
{
    udp.Recv();
    HighState state;
    udp.GetRecv(state);
    processState(state);
    displayJoystick(state);
    
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::RobotControl() 
{
    motiontime += 2;
    udp.GetRecv(state);
    printf("%d   %f\n", motiontime, state.imu.quaternion[2]);

    cmd.mode = 0;      // 0:idle, default stand      1:forced stand     2:walk continuously
    cmd.gaitType = 0;
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.euler[0]  = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.yawSpeed = 0.0f;
    cmd.reserve = 0;

    if(motiontime > 0 && motiontime < 1000){
        cmd.mode = 1;
        cmd.euler[0] = -0.3;
    }
    if(motiontime > 1000 && motiontime < 2000){
        cmd.mode = 1;
        cmd.euler[0] = 0.3;
    }
    if(motiontime > 2000 && motiontime < 3000){
        cmd.mode = 1;
        cmd.euler[1] = -0.2;
    }
    if(motiontime > 3000 && motiontime < 4000){
        cmd.mode = 1;
        cmd.euler[1] = 0.2;
    }
    if(motiontime > 4000 && motiontime < 5000){
        cmd.mode = 1;
        cmd.euler[2] = -0.2;
    }
    if(motiontime > 5000 && motiontime < 6000){
        cmd.mode = 1;
        cmd.euler[2] = 0.2;
    }
    if(motiontime > 6000 && motiontime < 7000){
        cmd.mode = 1;
        cmd.bodyHeight = -0.2;
    }
    if(motiontime > 7000 && motiontime < 8000){
        cmd.mode = 1;
        cmd.bodyHeight = 0.1;
    }
    if(motiontime > 8000 && motiontime < 9000){
        cmd.mode = 1;
        cmd.bodyHeight = 0.0;
    }
    if(motiontime > 9000 && motiontime < 11000){
        cmd.mode = 5;
    }
    if(motiontime > 11000 && motiontime < 13000){
        cmd.mode = 6;
    }
    if(motiontime > 13000 && motiontime < 14000){
        cmd.mode = 0;
    }
    if(motiontime > 14000 && motiontime < 18000){
        cmd.mode = 2;
        cmd.gaitType = 2;
        cmd.velocity[0] = 0.4f; // -1  ~ +1
        cmd.yawSpeed = 2;
        cmd.footRaiseHeight = 0.1;
        // printf("walk\n");
    }
    if(motiontime > 18000 && motiontime < 20000){
        cmd.mode = 0;
        cmd.velocity[0] = 0;
    }
    if(motiontime > 20000 && motiontime < 24000){
        cmd.mode = 2;
        cmd.gaitType = 1;
        cmd.velocity[0] = 0.2f; // -1  ~ +1
        cmd.bodyHeight = 0.1;
        // printf("walk\n");
    }
    if(motiontime>24000 ){
        cmd.mode = 1;
    }

    udp.SetSend(cmd);
}


int main(int argc, char *argv[]) 
{
    ros::init(argc, argv, "node_example_walk", ros::init_options::NoSigintHandler);
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom(HIGHLEVEL, "~/catkin_real_robot_ws/output.csv");
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = sig_to_exception;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    // InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    // loop_control.start();

    try {
        while(1){
            sleep(10);
        };
    } catch (InterruptException &e) {
        std::cout << "Caught keyboard interrupt, saving file" << std::endl;
        custom.saveFile.close();
        return 1;
    }
    

    return 0; 
}

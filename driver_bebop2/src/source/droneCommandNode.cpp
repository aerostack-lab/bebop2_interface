//////////////////////////////////////////////////////
//  DroneCommandNode.cpp
//
//  Created on: Nov 20, 2015
//      Author: Zorana Milosevic
//
//  Last modification on: Nov 20, 2015
//      Author: Zorana Milosevic
//
//////////////////////////////////////////////////////



//I/O stream
//std::cout
#include <iostream>


//ROS
#include "ros/ros.h"

//parrotARDrone
#include "droneInps.h"


using namespace std;




int main(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv, "droneCommand");
    ros::NodeHandle n;

    cout<<"[ROSNODE] Starting droneCommand"<<endl;

    //Vars
    DroneCommandROSModule MyDroneCommandROSModule;
    MyDroneCommandROSModule.open(n,"droneCommand");

    try
    {
        //Read messages
        ros::spin();
        return 1;

    }
    catch (std::exception &ex)
    {
        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
    }
}

//////////////////////////////////////////////////////
//  DroneStatusNode.cpp
//
//  Created on: Nov 18, 2015
//      Author: Zorana Milosevic
//
//  Last modification on: Nov 18, 2015
//      Author: Zorana Milosevic
//
//////////////////////////////////////////////////////



//I/O stream
//std::cout
#include <iostream>


//ROS
#include "ros/ros.h"

//parrotARDrone
#include "droneOuts.h"


using namespace std;




int main(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv, "droneStatusARDroneROSModule");
    ros::NodeHandle n;

    cout<<"[ROSNODE] Starting droneDroneStatus"<<endl;

    //Vars
    DroneStatusROSModule MyDroneStatusROSModule;
    MyDroneStatusROSModule.open(n,"droneStatusARDroneROSModule");

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

//////////////////////////////////////////////////////
//  frontCameraNode.cpp
//
//  Created on: Nov 18,2015
//      Author: Zorana Milosevic
//
//  Last modification on: Nov 18,2015
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

//Comunications
#include "communication_definition.h"


using namespace std;




int main(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv, "frontCameraARDroneROSModule");
    ros::NodeHandle n;

    cout<<"[ROSNODE] Starting droneFrontCamera"<<endl;

    //Vars
    FrontCameraROSModule MyFrontCameraROSModule;
    MyFrontCameraROSModule.open(n,"frontCameraARDroneROSModule");

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

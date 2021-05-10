//////////////////////////////////////////////////////
//  groundSpeedNode.cpp
//
//  Created on: Nov 19, 2015
//      Author: Zorana Milosevic
//
//  Last modification on: Nov 19, 2015
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
    ros::init(argc, argv, "droneGroundSpeed");
    ros::NodeHandle n;

    cout<<"[ROSNODE] Starting droneGroundSpeed"<<endl;

    //Vars
    GroundSpeedROSModule MyGroundSpeedROSModule;
    MyGroundSpeedROSModule.open(n,"droneGroundSpeed");

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

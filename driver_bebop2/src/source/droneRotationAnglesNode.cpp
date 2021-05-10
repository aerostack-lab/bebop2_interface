//I/O stream
//std::cout
#include <iostream>


//ROS
#include "ros/ros.h"

//bebopDriver
#include "droneOuts.h"



using namespace std;




int main(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv, "droneRotationAngles");
    ros::NodeHandle n;

    cout<<"[ROSNODE] Starting droneRotationAngles"<<endl;

    //Vars
    RotationAnglesROSModule MyRotationAnglesROSModule;
    MyRotationAnglesROSModule.open(n,"droneRotationAngles");

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

#ifndef _PARROT_ARDRONE_INPS_H
#define _PARROT_ARDRONE_INPS_H

#include "droneModuleROS.h"
#include "communication_definition.h"

#include "tf/transform_datatypes.h"


#include "ros/ros.h"
// #include <geometry_msgs/Twist.h>
#include "std_msgs/Empty.h"

//Custom commands
#include "droneMsgsROS/droneStatus.h"
// #include "droneMsgsROS/dronePitchRollCmd.h"
// #include "droneMsgsROS/droneDYawCmd.h"
// #include "droneMsgsROS/droneDAltitudeCmd.h"
#include "droneMsgsROS/droneCommand.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "aerostack_msgs/FlightActionCommand.h"
#include "aerostack_msgs/FlightState.h"


#include <iostream>
#include <cmath>

class DroneCommandROSModule : public DroneModule {

private:
    //Movement commands
    ros::Publisher CommandOutPubl;
    ros::Publisher TakeOffPub;
    ros::Publisher LandPub;
    ros::Publisher ResetPub;

    ros::Subscriber PitchRollSubs;
    // ros::Subscriber AltitudeSubs;
    // ros::Subscriber YawSubs;
    ros::Subscriber CommandSubs;
    ros::Subscriber AltitudeYawSubs;

    
    //Publish Functions
    bool publishCommandValue();
    bool publishTakeOff();
    bool publishLand();
    bool publishReset();

    //Subscribers Callbacks
    void pitchRollCallback(const geometry_msgs::PoseStamped& msg);
    // void dAltitudeCallback(const droneMsgsROS::droneDAltitudeCmd::ConstPtr& msg);
    // void dYawCallback(const droneMsgsROS::droneDYawCmd::ConstPtr& msg);
    void dAltitudeDYawCallback(const geometry_msgs::TwistStamped& msg);
    void commandCallback(const aerostack_msgs::FlightActionCommand& msg);



    //Command msgs out
    geometry_msgs::Twist CommandOutMsgs;
    std_msgs::Empty EmptyMsg;

    //State msgs

    ros::Subscriber StatusSubs;
    aerostack_msgs::FlightState DroneStatusMsgs_;
    void statusCallback(const aerostack_msgs::FlightState& msg){ DroneStatusMsgs_= msg; }


public:

    DroneCommandROSModule();
    ~DroneCommandROSModule(){};

    void init();
    void open(ros::NodeHandle & nIn, std::string moduleName);
    bool run();

    void close(){};
    bool resetValues(){return true;};
    bool startVal(){return true;};
    bool stopVal(){return true;};
};


class GimbalCommandROSModule : public DroneModule{

private:

    ros::Publisher CommandsPub;
    ros::Subscriber CommandsSubs;

    bool publishGimbalCommandValue();
    void commandCallback(const geometry_msgs::Twist::ConstPtr& msg);

    // msgs
    geometry_msgs::Twist CommandMsgs;
    std_msgs::Empty EmptyMsg;


public:

    GimbalCommandROSModule();
    ~GimbalCommandROSModule(){};

    void init();
    bool run();
    void open(ros::NodeHandle & nIn, std::string moduleName);
    
    
    void close(){};
    bool resetValues(){return true;};
    bool startVal(){return true;};
    bool stopVal(){return true;};
};





#endif

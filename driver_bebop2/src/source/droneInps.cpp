#include "droneInps.h"

using namespace std;

////// DroneCommand ////////
DroneCommandROSModule::DroneCommandROSModule() : DroneModule(droneModule::active)
{
    init();
    return;
}
 
void DroneCommandROSModule::init()
{
    CommandOutMsgs.angular.x=0.0;
    CommandOutMsgs.angular.y=0.0;
    CommandOutMsgs.angular.z=0.0;

    CommandOutMsgs.linear.x=0.0;
    CommandOutMsgs.linear.y=0.0;
    CommandOutMsgs.linear.z=0.0;

    return;
}


void DroneCommandROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
    //Node
    DroneModule::open(nIn,moduleName);

    //Publisher
    CommandOutPubl = n.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
    TakeOffPub=n.advertise<std_msgs::Empty>("takeoff",1, true);
    LandPub=n.advertise<std_msgs::Empty>("land",1, true);
    ResetPub=n.advertise<std_msgs::Empty>("reset",1, true);

    //Subscribers
    PitchRollSubs=n.subscribe("actuator_command/roll_pitch", 1, &DroneCommandROSModule::pitchRollCallback, this);
    AltitudeYawSubs=n.subscribe("actuator_command/altitude_rate_yaw_rate", 1, &DroneCommandROSModule::dAltitudeDYawCallback, this);
    CommandSubs=n.subscribe("actuator_command/flight_action", 1, &DroneCommandROSModule::commandCallback, this);

    StatusSubs = n.subscribe("self_localization/flight_state", 1, &DroneCommandROSModule::statusCallback, this);

    //Flag of module opened
    droneModuleOpened=true;

    //Auto-Start module
    moduleStarted=true;

    //End
    return;
}

//Run
bool DroneCommandROSModule::run()
{
    if(!DroneModule::run())
    {
        return false;
    }
    return true;
}


void DroneCommandROSModule::pitchRollCallback(const geometry_msgs::PoseStamped& msg)
{
    //Asynchronous module with only one callback!
    if(!run())
        return;
    
    if (DroneStatusMsgs_.state == aerostack_msgs::FlightState::LANDING || 
        DroneStatusMsgs_.state == aerostack_msgs::FlightState::LANDED || 
        DroneStatusMsgs_.state == aerostack_msgs::FlightState::TAKING_OFF) 
        return;

    tf::Quaternion q;
    tf::quaternionMsgToTF(msg.pose.orientation,q);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);


    double pitchCmd=pitch;
    if(pitchCmd>1.0)
        pitchCmd=1.0;
    else if(pitchCmd<-1.0)
        pitchCmd=-1.0;

    //Roll
    double rollCmd=roll;
    if(rollCmd>1.0)
        rollCmd=1.0;
    else if(rollCmd<-1.0)
        rollCmd=-1.0;

    CommandOutMsgs.linear.x =  -pitchCmd;
    // CommandOutMsgs.linear.y =  rollCmd;
    CommandOutMsgs.linear.y = - rollCmd;
    publishCommandValue();
    return;
}


void DroneCommandROSModule::dAltitudeDYawCallback(const geometry_msgs::TwistStamped& msg)
{
    //Asynchronous module with only one callback!
    if(!run())
        return;

    if (DroneStatusMsgs_.state == aerostack_msgs::FlightState::LANDING || 
        DroneStatusMsgs_.state == aerostack_msgs::FlightState::LANDED || 
        DroneStatusMsgs_.state == aerostack_msgs::FlightState::TAKING_OFF) 
        return;

    double dAltitudeCmd=msg.twist.linear.z;
    if(dAltitudeCmd>1.0)
        dAltitudeCmd=1.0;
    else if(dAltitudeCmd<-1.0)
        dAltitudeCmd=-1.0;
    CommandOutMsgs.linear.z=dAltitudeCmd;
    publishCommandValue();

    double dYawCmd=msg.twist.angular.z;

    if(dYawCmd>1.0)
        dYawCmd=1.0;
    else if(dYawCmd<-1.0)
        dYawCmd=-1.0;
    CommandOutMsgs.angular.z=-dYawCmd;
    publishCommandValue();
    return;
}


void DroneCommandROSModule::commandCallback(const aerostack_msgs::FlightActionCommand& msg)
{
    //Asynchronous module with only one callback!
    if(!run())
        return;
    switch(msg.action)
    {
    // case droneMsgsROS::droneCommand::TAKE_OFF:
    case aerostack_msgs::FlightActionCommand::TAKE_OFF:
        //Clear command
        CommandOutMsgs.angular.x=0.0;
        CommandOutMsgs.angular.y=0.0;
        CommandOutMsgs.angular.z=0.0;
        CommandOutMsgs.linear.x=0.0;
        CommandOutMsgs.linear.y=0.0;
        CommandOutMsgs.linear.z=0.0;
        //Publish
        publishCommandValue();
        //Take off
        publishTakeOff();
        break;
    // case droneMsgsROS::droneCommand::MOVE:
    case aerostack_msgs::FlightActionCommand::MOVE:

        //Clear
        // CommandOutMsgs.angular.x=1.0;
        // CommandOutMsgs.angular.y=1.0;
        // //Publish
        // publishCommandValue();
        break;
    // case droneMsgsROS::droneCommand::LAND:
    case aerostack_msgs::FlightActionCommand::LAND:

        //Clear command
        CommandOutMsgs.angular.x=0.0;
        CommandOutMsgs.angular.y=0.0;
        CommandOutMsgs.angular.z=0.0;
        CommandOutMsgs.linear.x=0.0;
        CommandOutMsgs.linear.y=0.0;
        CommandOutMsgs.linear.z=0.0;
        //Publish
        publishCommandValue();
        //Land
        publishLand();
        break;
    // case droneMsgsROS::droneCommand::HOVER:
    case aerostack_msgs::FlightActionCommand::HOVER:
        //Clear command
        CommandOutMsgs.angular.x=0.0;
        CommandOutMsgs.angular.y=0.0;
        CommandOutMsgs.angular.z=0.0;
        CommandOutMsgs.linear.x=0.0;
        CommandOutMsgs.linear.y=0.0;
        CommandOutMsgs.linear.z=0.0;
        //Publish
        publishCommandValue();
        break;
    // case droneMsgsROS::droneCommand::RESET:

    //     //Clear command
    //     CommandOutMsgs.angular.x=0.0;
    //     CommandOutMsgs.angular.y=0.0;
    //     CommandOutMsgs.angular.z=0.0;
    //     CommandOutMsgs.linear.x=0.0;
    //     CommandOutMsgs.linear.y=0.0;
    //     CommandOutMsgs.linear.z=0.0;
    //     //Publish
    //     publishCommandValue();
    //     //Reset
    //     publishReset();
    //     break;
    default:
        break;
    }

    return;
}


bool DroneCommandROSModule::publishCommandValue()
{
    if(droneModuleOpened==false)
        return false;
    CommandOutPubl.publish(CommandOutMsgs);
    return true;
}


//Take off
bool DroneCommandROSModule::publishTakeOff()
{
    if(droneModuleOpened==false)
        return false;
    TakeOffPub.publish(EmptyMsg);
    return true;
}

//Land
bool DroneCommandROSModule::publishLand()
{
    if(droneModuleOpened==false)
        return false;
    LandPub.publish(EmptyMsg);
    return true;
}

//Reset
bool DroneCommandROSModule::publishReset()
{
    if(droneModuleOpened==false)
        return false;
    ResetPub.publish(EmptyMsg);
    return true;
}





////// Gimble Command ////////
GimbalCommandROSModule::GimbalCommandROSModule() : DroneModule(droneModule::active)
{
    init();
    return;
}

void GimbalCommandROSModule::init()
{
    CommandMsgs.angular.y=0.0;
    CommandMsgs.angular.z=0.0;

    return;
}


void GimbalCommandROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
    //Node
    DroneModule::open(nIn,moduleName);

    //init();


    //Configuration


    //Publisher
    CommandsPub=n.advertise<geometry_msgs::Twist>("camera_control",1,true);


    //Subscribers
    
    CommandsSubs=n.subscribe("gimbal_control",1,&GimbalCommandROSModule::commandCallback, this);

    //Flag of module opened
    droneModuleOpened=true;

    //Auto-Start module
    moduleStarted=true;

    //setting the camera tilt for the first time
    CommandMsgs.angular.y=3.0;
    publishGimbalCommandValue();

    //End
    return;
}


//Run
bool GimbalCommandROSModule::run()
{
    if(!DroneModule::run())
    {
        return false;
    }

    return true;
}

void GimbalCommandROSModule::commandCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    if(!run())
        return;

    //header
    //TODO_JL

    //Tilt
    int tiltCmd=msg->angular.y;
    if (tiltCmd>100)
        tiltCmd=100;
    else if (tiltCmd<-100)
        tiltCmd=-100;

    //Pan
    int panCmd=msg->angular.z;
    if (panCmd>100)
        panCmd=100;
    else if (panCmd<-100)
        panCmd=-100;

    CommandMsgs.angular.y=tiltCmd;
    CommandMsgs.angular.z=panCmd;

    publishGimbalCommandValue();
    return;
}


bool GimbalCommandROSModule::publishGimbalCommandValue()
{
    if(droneModuleOpened==false)
        return false;

    CommandsPub.publish(CommandMsgs);
    return true;
}




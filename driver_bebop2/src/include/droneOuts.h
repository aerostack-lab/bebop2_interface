#include <cmath>
#include <iostream>

//Things related to the Aerostack
#include "droneModuleROS.h"
#include "communication_definition.h"

//ROS dependencies
#include "ros/ros.h"

//tf
#include "tf/transform_datatypes.h"


#include "sensor_msgs/BatteryState.h"
#include "geometry_msgs/Vector3Stamped.h"
//#include "droneMsgsROS/droneAltitude.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"

// #include "droneMsgsROS/vector2Stamped.h"
#include "geometry_msgs/TwistStamped.h"

//ROS messages

#include <Eigen/Dense>



// #include "droneMsgsROS/droneStatus.h"
#include "aerostack_msgs/FlightState.h"

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"

//bebop messages
#include "bebop_msgs/Ardrone3PilotingStateAttitudeChanged.h"
#include "bebop_msgs/CommonCommonStateBatteryStateChanged.h"
#include "bebop_msgs/Ardrone3PilotingStateAltitudeChanged.h"
#include "bebop_msgs/Ardrone3PilotingStateSpeedChanged.h"
#include "bebop_msgs/Ardrone3PilotingStateFlyingStateChanged.h"
#include "bebop_msgs/Ardrone3PilotingStateSpeedChanged.h"

#include "bebop_msgs/Ardrone3NetworkStateWifiAuthChannelListChanged.h"


//BottomCamera and FrontCamera
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

//Empty
#include "std_msgs/Empty.h"

#define no_slam_dunk


class RotationAnglesROSModule : public DroneModule
{
private:
    //Publisher
    ros::Publisher RotationAnglesPubl;
    ros::Publisher imu_data_pub;
    ros::Publisher bebop_odom_pub;

    bool publishRotationAnglesValue();

    //Subscriber
    ros::Subscriber RotationAnglesSubs;
    ros::Subscriber bebop_imu_subs;
    ros::Subscriber slam_dunk_rotation_angles_subs;
    ros::Subscriber slam_dunk_imu_subs;

    void slamDunkRotationAnglesCallback(const geometry_msgs::PoseStamped& msg);
    void rotationAnglesCallback(const bebop_msgs::Ardrone3PilotingStateAttitudeChanged::ConstPtr& msg);
    void slamdunkIMUCallback(const sensor_msgs::Imu& msg);
    void odomCallback(const nav_msgs::Odometry& msg);



    //msgs
    geometry_msgs::Vector3Stamped RotationAnglesMsgs;

    //Constructors and destructors
public:
    RotationAnglesROSModule();
    ~RotationAnglesROSModule(){};

    //Init and close
public:
  
    void open(ros::NodeHandle & nIn, std::string moduleName);
    bool run();

    void init(){};
    void close(){};
    bool resetValues(){return true;};
    bool startVal(){return true;};
    bool stopVal(){return true;};


};




/////////////////////////////////////////
// Class Battery
//
//   Description: Needs to be adjusted the unit!
//
/////////////////////////////////////////
class BatteryROSModule : public DroneModule
{

    //Publisher
protected:
    ros::Publisher BatteryPubl;
    bool publishBatteryValue();


    //Subscriber
protected:
    ros::Subscriber BatterySubs;
    void batteryCallback(const bebop_msgs::CommonCommonStateBatteryStateChanged::ConstPtr& msg);


    //Battery msgs
protected:
    sensor_msgs::BatteryState BatteryMsgs;

    //Constructors and destructors
public:
    BatteryROSModule();
    ~BatteryROSModule(){};


    void open(ros::NodeHandle & nIn, std::string moduleName);
    bool run();

    void init(){};
    void close(){};
    bool resetValues(){return true;};
    bool startVal(){return true;};
    bool stopVal(){return true;};


};


/////////////////////////////////////////
// Class Altitude
//
//   Description
//
/////////////////////////////////////////
class AltitudeROSModule : public DroneModule
{

    //Publisher
protected:
    ros::Publisher AltitudePubl;
    bool publishAltitudeValue();


    //Subscriber
protected:
    ros::Subscriber AltitudeSubs;//, AltitudeSpeedSubs;
    void altitudeCallback(const bebop_msgs::Ardrone3PilotingStateAltitudeChanged::ConstPtr& msg);
    // void altitudeSpeedCallback(const bebop_msgs::Ardrone3PilotingStateSpeedChanged::ConstPtr& msg);

    //Altitude msgs
protected:
    //droneMsgsROS::droneAltitude AltitudeMsgs;
    geometry_msgs::PointStamped AltitudeMsgs;


    //Constructors and destructors
public:
    AltitudeROSModule();
    ~AltitudeROSModule(){};

    void open(ros::NodeHandle & nIn, std::string moduleName);
    bool run();

    void init(){};
    void close(){};
    bool resetValues(){return true;};
    bool startVal(){return true;};
    bool stopVal(){return true;};

};




/////////////////////////////////////////
// Class FrontCamera
//
//   Description:
//
/////////////////////////////////////////
class FrontCameraROSModule : public DroneModule
{
    //Publisher
protected:

    image_transport::CameraPublisher FrontCameraPubl;
    bool publishFrontCameraValue();


    //Subscriber
protected:
    image_transport::CameraSubscriber FrontCameraSubs;
    void frontCameraCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr & info_msg);


    //FrontCamera msgs
protected:
    sensor_msgs::ImageConstPtr FrontCameraMsgs;
    sensor_msgs::CameraInfoConstPtr FrontCameraInfoMsgs;


    //Constructors and destructors
public:
    FrontCameraROSModule();
    ~FrontCameraROSModule(){};

    void open(ros::NodeHandle & nIn, std::string moduleName);
    bool run();

    void init(){};
    void close(){};
    bool resetValues(){return true;};
    bool startVal(){return true;};
    bool stopVal(){return true;};



};



/////////////////////////////////////////
// Class BottomCamera
//
//   Description:
//
/////////////////////////////////////////
class BottomCameraROSModule : public DroneModule
{
    //Publisher
protected:
    image_transport::CameraPublisher BottomCameraPubl;
    bool publishBottomCameraValue();


    //Subscriber
protected:
    image_transport::CameraSubscriber BottomCameraSubs;
    void bottomCameraCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr & info_msg);


    //BottomCamera msgs
protected:
    sensor_msgs::ImageConstPtr BottomCameraMsgs;
    sensor_msgs::CameraInfoConstPtr BottomCameraInfoMsgs;


    //Constructors and destructors
public:
    BottomCameraROSModule();
    ~BottomCameraROSModule(){};

    void open(ros::NodeHandle & nIn, std::string moduleName);
    bool run();

    void init(){};
    void close(){};
    bool resetValues(){return true;};
    bool startVal(){return true;};
    bool stopVal(){return true;};


};


/////////////////////////////////////////
// Class DroneStatus
//
//   Description: Needs to be adjusted the unit!
//
/////////////////////////////////////////
class DroneStatusROSModule : public DroneModule
{

    //Publisher
protected:
    ros::Publisher DroneStatusPubl;
    bool publishDroneStatusValue();


    //Subscriber
protected:
    ros::Subscriber DroneStatusSubs;
    void droneStatusCallback(const bebop_msgs::Ardrone3PilotingStateFlyingStateChanged::ConstPtr& msg);


    //DroneStatus msgs
protected:
    // droneMsgsROS::droneStatus DroneStatusMsgs;
    aerostack_msgs::FlightState DroneStatusMsgs;


    //Constructors and destructors
public:
    DroneStatusROSModule();
    ~DroneStatusROSModule(){};
    
    bool droneStatusLanded();

    void open(ros::NodeHandle & nIn, std::string moduleName);
    bool run();

    void init(){};
    void close(){};
    bool resetValues(){return true;};
    bool startVal(){return true;};
    bool stopVal(){return true;};




};





/////////////////////////////////////////
// Class GroundSpeed
//
//   Description:
//
/////////////////////////////////////////
class GroundSpeedROSModule : public DroneModule
{

    //Publisher
protected:
    ros::Publisher GroundSpeedPubl;
    bool publishGroundSpeedValue();


    //Subscriber
protected:
    ros::Subscriber GroundSpeedSubs;
    void groundSpeedCallback(const bebop_msgs::Ardrone3PilotingStateSpeedChanged::ConstPtr& msg);


    //GroundSpeed msgs
protected:
    // droneMsgsROS::vector2Stamped GroundSpeedMsgs;
    geometry_msgs::TwistStamped GroundSpeedMsgs;


    //Constructors and destructors
public:
    GroundSpeedROSModule();
    ~GroundSpeedROSModule(){};
    void open(ros::NodeHandle & nIn, std::string moduleName);
    bool run();

    void init(){};
    void close(){};
    bool resetValues(){return true;};
    bool startVal(){return true;};
    bool stopVal(){return true;};

};




/////////////////////////////////////////
// Class WifiChannel
//
//   Description
//
/////////////////////////////////////////
class WifiChannelROSModule : public DroneModule
{


    //Publishers
protected:

    ros::Publisher CommandPub;
    bool publishWifiChannelValue();

    //Subscribers
protected:

      ros::Subscriber CommandSub;
      void wifiChannelCallback(const bebop_msgs::Ardrone3NetworkStateWifiAuthChannelListChanged::ConstPtr& msg);

    //Command msgs out
protected:
    bebop_msgs::Ardrone3NetworkStateWifiAuthChannelListChanged CommandMsgs;

    //HL Commands
protected:
    std_msgs::Empty EmptyMsg;


    //Constructors and destructors
public:
    WifiChannelROSModule();
    ~WifiChannelROSModule(){};

    void open(ros::NodeHandle & nIn, std::string moduleName);
    bool run();

    
    void init(){};
    void close(){};
    bool resetValues(){return true;};
    bool startVal(){return true;};
    bool stopVal(){return true;};

};

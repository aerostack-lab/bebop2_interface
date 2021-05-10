#include "droneOuts.h"



////// RotationAngles ////////
RotationAnglesROSModule::RotationAnglesROSModule() : DroneModule(droneModule::active)
{
    init();
}

void RotationAnglesROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
    //Node
    DroneModule::open(nIn,moduleName);
    //Publisher
    RotationAnglesPubl = n.advertise<geometry_msgs::Vector3Stamped>("sensor_measurement/rotation_angles", 1, true);
    imu_data_pub       = n.advertise<sensor_msgs::Imu>("sensor_measurement/imu",1, true);




    //Subscriber
#ifdef no_slam_dunk
    RotationAnglesSubs = n.subscribe("states/ardrone3/PilotingState/AttitudeChanged", 1, &RotationAnglesROSModule::rotationAnglesCallback, this);
    bebop_imu_subs = n.subscribe("odom",1,&RotationAnglesROSModule::odomCallback,this);

    bebop_odom_pub = n.advertise<nav_msgs::Odometry>("sensor_measurement/odom",1,true);

#endif

#ifdef use_slam_dunk
    slam_dunk_rotation_angles_subs = n.subscribe("/pose", 1, &RotationAnglesROSModule::slamDunkRotationAnglesCallback, this);
    slam_dunk_imu_subs             = n.subscribe("sensor_measurement/imu",1, &RotationAnglesROSModule::slamdunkIMUCallback, this);
#endif

    //Flag of module opened
    droneModuleOpened=true;

    //Auto-Start module
    moduleStarted=true;

}

void RotationAnglesROSModule::odomCallback(const nav_msgs::Odometry& msg){

    static sensor_msgs::Imu bebop_imu_data;
    static nav_msgs::Odometry bebop_odom_data;
    static bool bebop_first_yaw_measurement_ = false;
    static double bebop_first_roll_, bebop_first_pitch_, bebop_first_yaw_;

    ros::Time current_time = ros::Time::now();    
    
    bebop_imu_data.header.stamp = current_time;
    bebop_imu_data.header.frame_id = "fcu";

    bebop_odom_data.header.stamp = current_time;
    bebop_odom_data.header.frame_id = "speeds_odom";
    bebop_odom_data.child_frame_id  = "fcu";


    if(!bebop_first_yaw_measurement_)
    {
        tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
        tf::Matrix3x3 m(q);

        //convert quaternion to euler angels
        m.getRPY(bebop_first_roll_, bebop_first_pitch_, bebop_first_yaw_);
        std::cout << "First yaw: " << bebop_first_yaw_ << std::endl;

        bebop_first_yaw_measurement_ = true;
        // bebop_first_yaw_= 1.0f;
    }

    //converting the bebop odom data to the one required by aerostack frame
    Eigen::Vector3f bebop_frame;
    Eigen::Vector3f aerostack_frame;
    Eigen::Matrix3f RotationMat;


    bebop_frame(0) = msg.pose.pose.position.x;
    bebop_frame(1) = msg.pose.pose.position.y;
    bebop_frame(2) = msg.pose.pose.position.z;

    RotationMat(0,0) = cos(bebop_first_yaw_);
    RotationMat(1,0) = -sin(bebop_first_yaw_);
    RotationMat(2,0) = 0;

    RotationMat(0,1) = sin(bebop_first_yaw_);
    RotationMat(1,1) = cos(bebop_first_yaw_);
    RotationMat(2,1) = 0;

    RotationMat(0,2) = 0;
    RotationMat(1,2) = 0;
    RotationMat(2,2) = 1;

    aerostack_frame = RotationMat * bebop_frame;
    // aerostack_frame = bebop_frame;

    // aerostack_frame = RotationMat * bebop_frame;

    // bebop_odom_data.pose.pose.position.x = msg.pose.pose.position.x;
    // bebop_odom_data.pose.pose.position.y = msg.pose.pose.position.y;
    // bebop_odom_data.pose.pose.position.z = msg.pose.pose.position.z;

    bebop_odom_data.pose.pose.position.x = aerostack_frame(0);
    bebop_odom_data.pose.pose.position.y = aerostack_frame(1);
    bebop_odom_data.pose.pose.position.z = aerostack_frame(2);

    bebop_odom_data.twist = msg.twist;

    /****************/

    // Eigen::Vector3f speeds_frame;

    // speeds_frame(0) = msg.twist.twist.linear.x;
    // speeds_frame(1) = msg.twist.twist.linear.y;
    // speeds_frame(2) = msg.twist.twist.linear.z;


    // Eigen::Vector3f speeds_rotated = RotationMat * speeds_frame;

    // bebop_odom_data.twist.twist.angular = msg.twist.twist.angular;
    
    // bebop_odom_data.twist.twist.linear.x = speeds_rotated(0);
    // bebop_odom_data.twist.twist.linear.y = speeds_rotated(1);
    // bebop_odom_data.twist.twist.linear.z = speeds_rotated(2);



    // bebop_imu_data.orientation = msg.pose.pose.orientation;

    //IMU data
    double roll, pitch, yaw;
    tf::Quaternion q1(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    tf::Matrix3x3 m1(q1);

    //convert quaternion to euler angels
    m1.getRPY(roll, pitch, yaw);
    yaw = yaw - bebop_first_yaw_;
    std::cout<< "yaw : " << yaw << std::endl;

    tf::Quaternion quaternion = tf::createQuaternionFromRPY(roll,pitch,yaw);
    bebop_imu_data.orientation.x = quaternion.getX();
    bebop_imu_data.orientation.y = quaternion.getY();
    bebop_imu_data.orientation.z = quaternion.getZ();
    bebop_imu_data.orientation.w = quaternion.getW();

    imu_data_pub.publish(bebop_imu_data);
    bebop_odom_pub.publish(bebop_odom_data);
}

//Run
bool RotationAnglesROSModule::run()
{
    if(!DroneModule::run())
    {
        return false;
    }
    return true;
}


#ifdef no_slam_dunk
void RotationAnglesROSModule::rotationAnglesCallback(const bebop_msgs::Ardrone3PilotingStateAttitudeChanged::ConstPtr& msg)
{
    //Asynchronous module with only one callback!
    if(!run())
        return;

    // Note, units of: msg->rotX, msg->rotY, msg->rotZ are radians.

    RotationAnglesMsgs.header=msg->header;
    // deg,   mavwork reference frame
    RotationAnglesMsgs.vector.x = (msg->roll)*180/M_PI;
    RotationAnglesMsgs.vector.y = (msg->pitch)*180/M_PI;
    RotationAnglesMsgs.vector.z = (msg->yaw)*180/M_PI;

    publishRotationAnglesValue();
    return;
}
#endif

#ifdef use_slam_dunk
void RotationAnglesROSModule::slamDunkRotationAnglesCallback(const geometry_msgs::PoseStamped &msg)
{
    if(!run())
        return;

    geometry_msgs::Vector3Stamped rotation_angles ;
    double roll, pitch, yaw;

    tf::Quaternion tf_quaternion;
    tf_quaternion.setX(msg.pose.orientation.x);
    tf_quaternion.setY(msg.pose.orientation.y);
    tf_quaternion.setZ(msg.pose.orientation.z);
    tf_quaternion.setW(msg.pose.orientation.w);

    tf::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

    RotationAnglesMsgs.header=msg.header;
    // deg,   mavwork reference frame
    RotationAnglesMsgs.vector.x = roll*180/M_PI;
    RotationAnglesMsgs.vector.y = -pitch*180/M_PI;
    RotationAnglesMsgs.vector.z = -yaw*180/M_PI;

    publishRotationAnglesValue();
    return;

}
#endif

#ifdef use_slam_dunk
void RotationAnglesROSModule::slamdunkIMUCallback(const sensor_msgs::Imu &msg)
{
    sensor_msgs::Imu imu_data;
    imu_data.header = msg.header;

    imu_data.angular_velocity.x = msg.angular_velocity.x;
    imu_data.angular_velocity.y = msg.angular_velocity.y;
    imu_data.angular_velocity.z = msg.angular_velocity.z;

    imu_data.angular_velocity_covariance = msg.angular_velocity_covariance;

    imu_data.linear_acceleration.x = msg.linear_acceleration.x;
    imu_data.linear_acceleration.y = msg.linear_acceleration.y;
    imu_data.linear_acceleration.z = msg.linear_acceleration.z;

    imu_data.linear_acceleration_covariance = msg.linear_acceleration_covariance;

    imu_data_pub.publish(imu_data);

}

#endif

bool RotationAnglesROSModule::publishRotationAnglesValue()
{
    if(droneModuleOpened==false)
        return false;

    RotationAnglesPubl.publish(RotationAnglesMsgs);
    return true;
}

////// Battery ////////
BatteryROSModule::BatteryROSModule() : DroneModule(droneModule::active)
{
    init();
    return;
}


void BatteryROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
    //Node
    DroneModule::open(nIn,moduleName);

    //Publisher
    BatteryPubl = n.advertise<sensor_msgs::BatteryState>("sensor_measurement/battery_state", 1, true);
    //Subscriber
    BatterySubs=n.subscribe("states/common/CommonState/BatteryStateChanged", 1, &BatteryROSModule::batteryCallback, this);

    //Flag of module opened
    droneModuleOpened=true;

    //Auto-Start module
    moduleStarted=true;
}

//Run
bool BatteryROSModule::run()
{
    if(!DroneModule::run())
    {
        return false;
    }

    return true;
}


void BatteryROSModule::batteryCallback(const bebop_msgs::CommonCommonStateBatteryStateChanged::ConstPtr& msg)
{
    //Asynchronous module with only one callback!
    if(!run())
        return;

    //Read Battery from navdata
    BatteryMsgs.header=msg->header;

    BatteryMsgs.percentage=msg->percent/100.0f;
   
    publishBatteryValue();
    return;
}


bool BatteryROSModule::publishBatteryValue()
{
    if(droneModuleOpened==false)
        return false;

    BatteryPubl.publish(BatteryMsgs);
    return true;
}

////// Altitude ////////
AltitudeROSModule::AltitudeROSModule() : DroneModule(droneModule::active)
{
    init();
    return;
}


void AltitudeROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
    //Node
    DroneModule::open(nIn,moduleName);

    //Publisher
    AltitudePubl = n.advertise<geometry_msgs::PointStamped>("sensor_measurement/altitude", 1, true);


    //Subscriber
    AltitudeSubs=n.subscribe("states/ardrone3/PilotingState/AltitudeChanged", 1, &AltitudeROSModule::altitudeCallback, this);

    //Flag of module opened
    droneModuleOpened=true;

    //Auto-Start module
    moduleStarted=true;
}

//Run
bool AltitudeROSModule::run()
{
    if(!DroneModule::run())
    {
        return false;
    }

    return true;
}


void AltitudeROSModule::altitudeCallback(const bebop_msgs::Ardrone3PilotingStateAltitudeChanged::ConstPtr& msg)
{
    //Asynchronous module with only one callback!
    if(!run())
        return;

    //Read Altitude from navdata
    AltitudeMsgs.header = msg->header;
    //Altitude needs to be put in [m], mavwork reference frame!!
    AltitudeMsgs.point.z = -msg->altitude;

    publishAltitudeValue();
    return;
}


// void AltitudeROSModule::altitudeSpeedCallback(const bebop_msgs::Ardrone3PilotingStateSpeedChanged::ConstPtr& msg)
// {
//     if(!run())
//         return;
//     AltitudeMsgs.altitude_speed=-msg->speedZ;

//     publishAltitudeValue();
//     return;
// }

bool AltitudeROSModule::publishAltitudeValue()
{
    if(droneModuleOpened==false)
        return false;

    AltitudePubl.publish(AltitudeMsgs);
    return true;
}

////// FrontCamera ////////
FrontCameraROSModule::FrontCameraROSModule() : DroneModule(droneModule::active)
{
    init();
    return;
}


void FrontCameraROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
    //Node
    DroneModule::open(nIn,moduleName);

    image_transport::ImageTransport it(n);

    //Publisher
    FrontCameraPubl = it.advertiseCamera("sensor_measurement/camera/front/image_raw", 1, true);

    //Subscriber
    FrontCameraSubs=it.subscribeCamera("image_raw", 1, &FrontCameraROSModule::frontCameraCallback, this);

    //Flag of module opened
    droneModuleOpened=true;

    //Auto-Start module
    moduleStarted=true;

    //End
    return;
}


//Run
bool FrontCameraROSModule::run()
{
    if(!DroneModule::run())
    {
        return false;
    }

    return true;
}


void FrontCameraROSModule::frontCameraCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr & info_msg)
{
    //Asynchronous module with only one callback!
    if(!run())
        return;

    //Read FrontCamera
    FrontCameraMsgs=image_msg;
    FrontCameraInfoMsgs=info_msg;

    publishFrontCameraValue();
    return;
}


bool FrontCameraROSModule::publishFrontCameraValue()
{
    if(droneModuleOpened==false)
        return false;

    FrontCameraPubl.publish(FrontCameraMsgs,FrontCameraInfoMsgs);
    return true;
}





////// BottomCamera ////////
BottomCameraROSModule::BottomCameraROSModule() : DroneModule(droneModule::active)
{
    init();
    return;
}



void BottomCameraROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
    //Node
    DroneModule::open(nIn,moduleName);

    image_transport::ImageTransport it(n);

    //Publisher
    BottomCameraPubl = it.advertiseCamera("sensor_measurement/camera/bottom/image_raw", 1, true);
    //Subscriber
    BottomCameraSubs=it.subscribeCamera("image_raw", 1, &BottomCameraROSModule::bottomCameraCallback, this);


    //Flag of module opened
    droneModuleOpened=true;

    //Auto-Start module
    moduleStarted=true;

    //End
    return;
}

//Run
bool BottomCameraROSModule::run()
{
    if(!DroneModule::run())
    {
        return false;
    }

    return true;
}


void BottomCameraROSModule::bottomCameraCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr & info_msg)
{
    //Asynchronous module with only one callback!
    if(!run())
        return;

    //Read BottomCamera
    BottomCameraMsgs=image_msg;
    BottomCameraInfoMsgs=info_msg;

    publishBottomCameraValue();
    return;
}


bool BottomCameraROSModule::publishBottomCameraValue()
{
    if(droneModuleOpened==false)
        return false;

    BottomCameraPubl.publish(BottomCameraMsgs,BottomCameraInfoMsgs);
    return true;
}




////// DroneStatus ////////
DroneStatusROSModule::DroneStatusROSModule() : DroneModule(droneModule::active)
{
    init();
    return;
}



void DroneStatusROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
    //Node
    DroneModule::open(nIn,moduleName);
    //Publisher
    DroneStatusPubl = n.advertise<aerostack_msgs::FlightState>("self_localization/flight_state", 1, true);
    //Subscriber
    DroneStatusSubs=n.subscribe("states/ardrone3/PilotingState/FlyingStateChanged", 1, &DroneStatusROSModule::droneStatusCallback, this);
    //Flag of module opened
    droneModuleOpened=true;
    droneStatusLanded();
    //Auto-Start module
    moduleStarted=true;
}

//Run
bool DroneStatusROSModule::run()
{
    if(!DroneModule::run())
    {
        return false;
    }

    return true;
}


void DroneStatusROSModule::droneStatusCallback(const bebop_msgs::Ardrone3PilotingStateFlyingStateChanged::ConstPtr& msg)
{
    //Asynchronous module with only one callback!
    if(!run())
        return;

    //Read DroneStatus from navdata
    DroneStatusMsgs.header=msg->header;

    switch(msg->state)
    {
    case 0:
    default:
        DroneStatusMsgs.state=aerostack_msgs::FlightState::LANDED;
        break;
    case 1:
        DroneStatusMsgs.state=aerostack_msgs::FlightState::TAKING_OFF;
        break;
    case 2:
        DroneStatusMsgs.state=aerostack_msgs::FlightState::HOVERING;
        break;
    case 3:
        DroneStatusMsgs.state=aerostack_msgs::FlightState::FLYING;
        break;
    case 4:
        DroneStatusMsgs.state=aerostack_msgs::FlightState::LANDING;
        break;
    // case 5:
    //     DroneStatusMsgs.state=aerostack_msgs::FlightState::EMERGENCY;
    //     break;
    // 
    }

    publishDroneStatusValue();
}


bool DroneStatusROSModule::publishDroneStatusValue()
{
    if(droneModuleOpened==false)
        return false;

    DroneStatusPubl.publish(DroneStatusMsgs);
    return true;
}


bool DroneStatusROSModule::droneStatusLanded()
{
    if(droneModuleOpened==false)
        return false;

    DroneStatusMsgs.state=aerostack_msgs::FlightState::LANDED;
    DroneStatusPubl.publish(DroneStatusMsgs);
    return true;
}


////// GroundSpeed ////////
GroundSpeedROSModule::GroundSpeedROSModule() : DroneModule(droneModule::active)
{
    init();
    return;
}


void GroundSpeedROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
    //Node
    DroneModule::open(nIn,moduleName);
    //Publisher
    GroundSpeedPubl = n.advertise<geometry_msgs::TwistStamped>("sensor_measurement/linear_speed", 1, true);
    //Subscriber
    GroundSpeedSubs=n.subscribe("states/ardrone3/PilotingState/SpeedChanged", 1, &GroundSpeedROSModule::groundSpeedCallback, this);
    //Flag of module opened
    droneModuleOpened=true;
    //Auto-Start module
    moduleStarted=true;
}

//Run
bool GroundSpeedROSModule::run()
{
    if(!DroneModule::run())
    {
        return false;
    }
    return true;
}



void GroundSpeedROSModule::groundSpeedCallback(const bebop_msgs::Ardrone3PilotingStateSpeedChanged::ConstPtr& msg)
{
    //Asynchronous module with only one callback!
    if(!run())
        return;

    //Read GroundSpeed from navdata
   GroundSpeedMsgs.header=msg->header;

    // [m/s], mavwork reference frame
    GroundSpeedMsgs.twist.linear.x=msg->speedX;
    GroundSpeedMsgs.twist.linear.y=-msg->speedY;
    GroundSpeedMsgs.twist.linear.z=-msg->speedZ;

    publishGroundSpeedValue();
    return;
}


bool GroundSpeedROSModule::publishGroundSpeedValue()
{
    if(droneModuleOpened==false)
        return false;

    GroundSpeedPubl.publish(GroundSpeedMsgs);
    return true;
}


////// Wifi Channel ////////
WifiChannelROSModule::WifiChannelROSModule() : DroneModule(droneModule::active)
{
    init();
    return;
}
void WifiChannelROSModule::open(ros::NodeHandle & nIn, std::string moduleName)
{
    //Node
    DroneModule::open(nIn,moduleName);
    //Publishers
    CommandPub=n.advertise<bebop_msgs::Ardrone3NetworkStateWifiAuthChannelListChanged>("wifi_channel",1,true);
    //Subscribers
    CommandSub=n.subscribe("states/ardrone3/NetworkState/WifiAuthChannelListChanged",1,&WifiChannelROSModule::wifiChannelCallback, this);
    //Flag of module opened
    droneModuleOpened=true;
    //Auto-Start module
    moduleStarted=true;
}


//Run
bool WifiChannelROSModule::run()
{
    if(!DroneModule::run())
    {
        return false;
    }

    return true;
}

void WifiChannelROSModule::wifiChannelCallback(const bebop_msgs::Ardrone3NetworkStateWifiAuthChannelListChanged::ConstPtr &msg)
{
    if(!run())
        return;

    CommandMsgs.channel=msg->channel;

    publishWifiChannelValue();

    return;
}


bool WifiChannelROSModule::publishWifiChannelValue()
{
    if(droneModuleOpened==false)
        return false;

    CommandPub.publish(CommandMsgs);
    return true;
}


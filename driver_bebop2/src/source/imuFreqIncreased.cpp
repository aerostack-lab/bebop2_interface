#include "ros/ros.h"
#include "sensor_msgs/Imu.h"


using namespace std;
sensor_msgs::Imu imu_msgs;

void imuCallback(const sensor_msgs::Imu& msg){
    imu_msgs.header.stamp=msg.header.stamp;
    imu_msgs.orientation = msg.orientation;
    imu_msgs.angular_velocity = msg.angular_velocity;
    imu_msgs.linear_acceleration = msg.linear_acceleration;

}

int main(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv, "imufreq");

    ros::NodeHandle n;
    ros::Subscriber  imu_sub = n.subscribe("sensor_measurement/imu",1,imuCallback);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("sensor_measurement/imu",1);

    ros::Rate r(100);
    while(ros::ok()){

        imu_msgs.header.stamp=ros::Time::now();
        imu_pub.publish(imu_msgs);
        ros::spinOnce();
        r.sleep();
    }
}

#include<tf/transform_broadcaster.h>
#include<ros/ros.h>
#include<dynamixel_msgs/JointState.h>
#include<sensor_msgs/Imu.h>
#include<geometry_msgs/Quaternion.h> 
#include<geometry_msgs/Vector3.h>

/* This node publishes the tf between the laser scan and the servo.  This is based on the angle published by the servo. */

//Module that applies transform to laser scan of tilting hokuyo laser
using namespace std;

//global variables
float pos;

//Recieves position values from dynamixel servo and uses angle to apply transform to laser scan
void obtainValues(const sensor_msgs::Imu::ConstPtr &msg)
{
    //gets position from message

    float ori_x = msg->orientation.x;
    float ori_y = msg->orientation.y;
    float ori_z = msg->orientation.z;

    
    //perform transform
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    tf::Quaternion q;
    q.setRPY(ori_y, ori_z, ori_x);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "imu_link", "laser"));
}

//main
int main(int argc, char **argv) 
{
    //initialize
    ros::init(argc, argv, "imu_transform");
    ros::NodeHandle nh;
  
    //subscirber to current position
    ros::Subscriber position_sub = nh.subscribe("/imu/data", 5, &obtainValues);

    //wait for updates in position
    ros::spin();
}

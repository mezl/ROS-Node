#include<tf/transform_broadcaster.h>
#include<ros/ros.h>
#include<dynamixel_msgs/JointState.h>
#include<sensor_msgs/Imu.h>
#include<geometry_msgs/Quaternion.h> 
#include<geometry_msgs/Vector3.h>
#include<angles/angles.h>

/* This node publishes the tf between the laser scan and the servo.  This is based on the angle published by the servo. */

//Module that applies transform to laser scan of tilting hokuyo laser
using namespace std;

//global variables
float pos;
float dx = 0;
float dy = 0;
float dz = 0;

//Recieves position values from dynamixel servo and uses angle to apply transform to laser scan
void obtainValues(const sensor_msgs::Imu::ConstPtr &msg)
{
    //gets position from message

    float ori_x = msg->orientation.x + dx;
    float ori_y = msg->orientation.y + dy;
    float ori_z = msg->orientation.z + dz;
    float ori_w = msg->orientation.w + dz;
    geometry_msgs::Quaternion imu_q;// = msg->orientation;
    imu_q.x = ori_w;
    imu_q.y = ori_z*-1;
    imu_q.z = ori_y;
    imu_q.w = ori_x*-1;
        

    tf::Quaternion q;

    tf::quaternionMsgToTF(imu_q, q);
    
    //perform transform
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(imu_q.z, imu_q.y, imu_q.x) );
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "imu_link", "laser"));
}
void deltaValues(const geometry_msgs::Vector3 &msg)
{
    dx = angles::from_degrees(double(msg.x));
    dy = angles::from_degrees(double(msg.y));
    dz = angles::from_degrees(double(msg.z));

    ROS_INFO("Set delta angle (%f,%f,%f)", dx,dy,dz);
}

//main
int main(int argc, char **argv) 
{
    //initialize
    ros::init(argc, argv, "imu_transform");
    ros::NodeHandle nh;
  
    //subscirber to current position
    ros::Subscriber position_sub = nh.subscribe("/imu/data", 5, &obtainValues);
    ros::Subscriber delta_sub = nh.subscribe("/imu/delta", 5, &deltaValues);

    //wait for updates in position
    ros::spin();
}

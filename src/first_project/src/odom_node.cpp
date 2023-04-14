#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <first_project/Odom.h>
#include <first_project/reset_odom.h>
#include <geometry_msgs/Quaternion.h>
#include "math.h"

const double wheelDistance = 2.8;
ros::Time lastStamp;
double x=0.0;
double y=0.0;
double theta=0.0;

ros::Publisher odometryPublisher;
ros::Publisher customOdometryPublisher;

bool resetOdom(first_project::reset_odom::Request  &req, first_project::reset_odom::Response &res) {

    x = 0.0;
    y = 0.0;
    theta = 0.0;

    res.resetted = true;

    return true;
}

void calculateOdometry(const geometry_msgs::Quaternion::ConstPtr &data){
    double r, w;

    w = data->x * (tan(data->y)/ wheelDistance);


    double timeSpan;
    ros::Time currentTime = ros::Time::now();
    
    timeSpan =  currentTime.toSec() - lastStamp.toSec();

    ROS_INFO("current time: %f last time=%f delta=%f",currentTime.toSec(),lastStamp.toSec(),timeSpan);
        
    lastStamp = currentTime;
    
    //runge-kutta integration
    x+=data->x*timeSpan*cos(theta+(w*timeSpan)/2);
    y+=data->x*timeSpan*sin(theta+(w*timeSpan)/2);
    theta+=w*timeSpan;
    ROS_INFO("odometry: x=%f , y=%f, theta=%f",x,y,theta);

    nav_msgs::Odometry odMsg;

    odMsg.pose.pose.position.x = x;
    odMsg.pose.pose.position.y = y;
    odMsg.pose.pose.orientation.w = theta;

    first_project::Odom customOdMsg;

    customOdMsg.th = theta;
    customOdMsg.x = x;
    customOdMsg.y = y;
    customOdMsg.timestamp = std::to_string(currentTime.toSec());

    odometryPublisher.publish(odMsg);
    customOdometryPublisher.publish(customOdMsg);
}

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "odom_node");
    ROS_INFO("init odom_node done!");
    ros::NodeHandle n;

    odometryPublisher = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    customOdometryPublisher = n.advertise<first_project::Odom>("custom_odometry", 1000);
    ROS_INFO("publishers started!");
    ros::ServiceServer service = n.advertiseService("reset_odom", resetOdom);
    ROS_INFO("service reset odom started!");
    lastStamp = ros::Time::now();
    ros::Subscriber bagReader = n.subscribe<geometry_msgs::Quaternion>("speed_steer", 1000, calculateOdometry);

    ros::spin();   
}
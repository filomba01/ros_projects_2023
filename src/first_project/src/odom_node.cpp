#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <first_project/Odom.h>
#include <first_project/reset_odom.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
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
    
    double timeSpan;
    double r, w;
    first_project::Odom customOdMsg;
    nav_msgs::Odometry odMsg;
    ros::Time currentTime;
    /* tf variables */
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;

    w = data->x * (tan(data->y)/ wheelDistance);
    
    currentTime = ros::Time::now();
    
    timeSpan =  currentTime.toSec() - lastStamp.toSec();

    ROS_INFO("current time: %f last time=%f delta=%f",currentTime.toSec(),lastStamp.toSec(),timeSpan);
        
    lastStamp = currentTime;
    
    //runge-kutta integration
    x += data->x * timeSpan * cos(theta+(w*timeSpan)/2);
    y += data->x * timeSpan * sin(theta+(w*timeSpan)/2);
    theta += w * timeSpan;
    ROS_INFO("odometry: x=%f , y=%f, theta=%f",x,y,theta);


    /* tf publishing */
    
    transform.setOrigin( tf::Vector3(x, y, 0.0) );
    
    q.setRPY(0, 0, theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

    /* standard odom message */    

    odMsg.pose.pose.position.x = x;
    odMsg.pose.pose.position.y = y;
    odMsg.pose.pose.orientation.w = theta;

    /* custom odom message */
    customOdMsg.th = theta;
    customOdMsg.x = x;
    customOdMsg.y = y;
    customOdMsg.timestamp = std::to_string(currentTime.toSec());

    /* publishing messages */
    odometryPublisher.publish(odMsg);
    customOdometryPublisher.publish(customOdMsg);
    ROS_INFO("messages have been published!");
}

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "odom_node");
    ROS_INFO("init odom_node done!");
    ros::NodeHandle n;

    /* gets starting parameters */
    ros::param::get("starting_x", x);
    ros::param::get("starting_y", y);
    ros::param::get("starting_th", theta);
    ROS_INFO("x: %f", x);
    ROS_INFO("y: %f", y);
    ROS_INFO("theta: %f", theta);

    odometryPublisher = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    customOdometryPublisher = n.advertise<first_project::Odom>("custom_odometry", 1000);
    ROS_INFO("publishers started!");
    ros::ServiceServer service = n.advertiseService("reset_odom", resetOdom);
    ROS_INFO("service reset odom started!");
    
    do{
        lastStamp = ros::Time::now();
    }
    while(lastStamp == ros::Time(0));
    
    ros::Subscriber bagReader = n.subscribe<geometry_msgs::Quaternion>("speed_steer", 1000, calculateOdometry);

    ros::spin();   
}
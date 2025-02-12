# include "ros/ros.h"
# include <nav_msgs/Odometry.h>
# include <first_project/Odom.h>
# include <first_project/reset_odom.h>
# include <geometry_msgs/Quaternion.h>
# include <tf/transform_broadcaster.h>
# include "math.h"

class Odom_node{
    private:
        const double wheelDistance = 2.8;
        ros::Time lastStamp;
        double x=0.0;
        double y=0.0;
        double theta=0.0;

        ros::NodeHandle n;
        ros::Publisher odometryPublisher;
        ros::Publisher customOdometryPublisher;

        /* tf variables */
        tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Quaternion q;

    
        void publish_tf(){
            /* tf publishing */
            transform.setOrigin( tf::Vector3(x, y, 0.0) );

            q.setRPY(0, 0, theta);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
        }

        void publish_custom_odom_message(ros::Time currentTime){
            first_project::Odom customOdMsg;
            /* custom odom message */
            customOdMsg.th = theta;
            customOdMsg.x = x;
            customOdMsg.y = y;
            customOdMsg.timestamp = std::to_string(currentTime.toSec());

            /* publishing messages */
            customOdometryPublisher.publish(customOdMsg);
            ROS_INFO("custom message has been published!");
        }

        void publish_odom_message(){
            nav_msgs::Odometry odMsg;

            /* standard odom message */
            geometry_msgs::Quaternion theta_quaternions = tf::createQuaternionMsgFromYaw(theta);

            odMsg.pose.pose.position.x = x;
            odMsg.pose.pose.position.y = y;
            odMsg.pose.pose.orientation = theta_quaternions;
            odMsg.header.frame_id = "odom";
            odMsg.child_frame_id = "base_link";

            odometryPublisher.publish(odMsg);
            ROS_INFO("odom message has been published!");
        }

    public:


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
            double old_theta;
            ros::Time currentTime;
            

            w = data->x * (tan(data->y)/ wheelDistance);

            currentTime = ros::Time::now();

            timeSpan =  currentTime.toSec() - lastStamp.toSec();

            ROS_INFO("current time: %f last time=%f delta=%f",currentTime.toSec(),lastStamp.toSec(),timeSpan);

            lastStamp = currentTime;
            
            if(w == 0){
                /* runge-kutta integration */
                x += data->x * timeSpan * cos(theta+(w*timeSpan)/2);
                y += data->x * timeSpan * sin(theta+(w*timeSpan)/2);
                theta += w * timeSpan;
            } else {
                old_theta = theta;
                theta += w * timeSpan;
                x+= (data->x /w)*(sin(theta)-sin(old_theta));
                y-= (data->x/w)*(cos(theta)-cos(old_theta));
            }
            ROS_INFO("odometry: x=%f , y=%f, theta=%f",x,y,theta);


            publish_tf();

            publish_odom_message();

            publish_custom_odom_message(currentTime);
        }



        void init(){

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
            ros::ServiceServer service = n.advertiseService("reset_odom", &Odom_node::resetOdom, this);
            ROS_INFO("service reset odom started!");

            do{
                lastStamp = ros::Time::now();
            }
            while(!lastStamp.isValid());

            ros::Subscriber bagReader = n.subscribe<geometry_msgs::Quaternion>("speed_steer", 1000, &Odom_node::calculateOdometry,this);
            ros::spin();
        }


};


int main(int argc, char *argv[]){

    ros::init(argc, argv, "odom_node");

    Odom_node odom_node;

    odom_node.init();
    ROS_INFO("init odom_node done!");

    return 0;
}
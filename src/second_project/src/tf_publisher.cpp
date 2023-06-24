# include "ros/ros.h"
# include <nav_msgs/Odometry.h>
# include <geometry_msgs/Quaternion.h>
# include <tf/transform_broadcaster.h>
# include  <tf/transform_datatypes.h>
# include "math.h"

class Tf_publisher{
    private: 
        tf::TransformBroadcaster br;
        tf::Transform transform;

        ros::NodeHandle n;


    public:
        void publish_tf(const nav_msgs::Odometry::ConstPtr &data){

            double x = data->pose.pose.position.x;
            double y = data->pose.pose.position.y;
            geometry_msgs::Quaternion theta = data->pose.pose.orientation;
            tf::Quaternion q;

            quaternionMsgToTF(theta, q);

            /* tf publishing */
            transform.setOrigin( tf::Vector3(x, y, 0.0) );

            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), data->header.frame_id, "base_footprint"));
        }

        void init(){
            ros::Subscriber bagReader = n.subscribe<nav_msgs::Odometry>("/t265/odom", 1000, &Tf_publisher::publish_tf,this);
            ros::spin();
        }
};

int main(int argc, char *argv[]){

    ros::init(argc, argv, "tf_publisher");

    Tf_publisher tf_publisher;

    tf_publisher.init();
    ROS_INFO("init tf_publisher done!");

    return 0;
}
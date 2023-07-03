#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>


#define X_POS 0
#define Y_POS 1
#define W_POS 2 

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Navigation{
    private:
	
	ros::NodeHandle nh;

        void move_to_goal(double x, double y, double tetha){
            
            ROS_INFO("MoveBase parameters recived to be reached: X: %f , Y: %f, W: %f",x,y,tetha);
            MoveBaseClient client("move_base", true);

            ROS_INFO("Waiting for the move_base action server...");
            while(!client.waitForServer(ros::Duration(5.0))){
		    ROS_INFO("Waiting for the move_base action server to come up");
	    }

            move_base_msgs::MoveBaseGoal goal;
   	        ROS_INFO("Server Ready!");
            goal.target_pose.header.frame_id = "base_link"; 
            goal.target_pose.header.stamp = ros::Time::now(); 
            goal.target_pose.pose.position.x = x;    
            goal.target_pose.pose.position.y = y;    
            
            geometry_msgs::Quaternion theta_quaternions = tf::createQuaternionMsgFromYaw(tetha);
            goal.target_pose.pose.orientation = theta_quaternions;

            ROS_INFO("Sending goal to move_base...");
            client.sendGoal(goal);

            client.waitForResult();

            if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Goal reached!");
            }
            else
            {
                ROS_INFO("Failed to reach the goal.");
            }
        }
    
    public:
        void init(){
            std::string waypoint_file;
            ros::param::get("waypoint_file", waypoint_file);
            ROS_INFO("Node init!");

            this->readData(waypoint_file);
            return;
        }

        void readData(std::string filename){

                    std::ifstream file(filename);
                    std::string line;
		            ROS_INFO("searching waypoints!");
                    if (!file.is_open())
                    {
                        ROS_INFO("Waypoint file is missing!");
                        return;
                    }

                    
                    while (std::getline(file, line))
                    {
                        std::vector<std::string> row;
                        std::stringstream ss(line);
                        std::string cell;
			            ROS_INFO("computing waypoints!");
                        while (std::getline(ss, cell, ','))
                        {
                            row.push_back(cell);
                        }
			            ROS_INFO("GOAL to be reached: X: %f , Y: %f, W: %f",std::stod(row.at(X_POS)), std::stod(row.at(Y_POS)), std::stod(row.at(W_POS)));
                        this->move_to_goal(std::stod(row.at(X_POS)),std::stod(row.at(Y_POS)),std::stod(row.at(W_POS)));

                    }

                    file.close();
                    return;
        }

};

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "navigation");
    Navigation navigation;

    navigation.init();


    return 0;
}

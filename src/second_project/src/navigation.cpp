#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

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
        void move_to_goal(double x, double y, double tetha){
            ros::NodeHandle nh;
            MoveBaseClient client("move_base", true);

            ROS_INFO("Waiting for the move_base action server...");
            client.waitForServer();

            move_base_msgs::MoveBaseGoal goal;

            goal.target_pose.header.frame_id = "map";  
            goal.target_pose.pose.position.x = x;    
            goal.target_pose.pose.position.y = y;    
            goal.target_pose.pose.orientation.w = tetha; 

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
            this->readData(waypoint_file);
            ros::spin();
            return;
        }

        void readData(std::string filename){

                    std::ifstream file(filename);
                    std::string line;

                    if (!file.is_open())
                    {
                        ROS_INFO("Waipoint file is missing!");
                        return;
                    }

                    
                    while (std::getline(file, line))
                    {
                        std::vector<std::string> row;
                        std::stringstream ss(line);
                        std::string cell;

                        while (std::getline(ss, cell, ','))
                        {
                            row.push_back(cell);
                        }

                        this->move_to_goal(std::stoi(row.at(X_POS)),std::stoi(row.at(Y_POS)),std::stoi(row.at(W_POS)));

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
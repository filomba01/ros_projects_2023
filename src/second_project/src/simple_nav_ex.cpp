     //inspired by markwsilliman/turtlebot repository

     #include <ros/ros.h>

     //The following line is where we import the ``MoveBaseAction`` library which is responsible for accepting goals from users and move the robot to the specified location in its world.
     #include <move_base_msgs/MoveBaseAction.h>

     #include <actionlib/client/simple_action_client.h>

     //this line is where we create the client that will communicate with actions that adhere to the base station interface
     typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

     int main(int argc, char** argv){
ros::init(argc, argv, "map_navigation");
//tell the action client that we want to spin a thread by default
MoveBaseClient ac("move_base", true);

     //wait for the action server to come up and then start the process
     while(!ac.waitForServer(ros::Duration(5.0))){
     ROS_INFO("Waiting for the move_base action server to come up");
     }

     //This is where you create the goal to send to move_base using move_base_msgs::MoveBaseGoal messages to tell the robot to move one meter forward in the coordinate frame.
     move_base_msgs::MoveBaseGoal goal;

     //we'll send a goal to the robot to move 1 meter forward
     goal.target_pose.header.frame_id = "base_link";
     goal.target_pose.header.stamp = ros::Time::now();

     goal.target_pose.pose.position.x = 0.2;
     goal.target_pose.pose.orientation.w = 0.2;

     ROS_INFO("Sending goal");

     //this command sends the goal to the move_base node to be processed
     ac.sendGoal(goal);

     //After finalizing everything you have to wait for the goal to finish processing
     ac.waitForResult();

     //here we check for the goal if it succeded or failed and send a message according to the goal status.
     if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
     ROS_INFO("Hooray, the base moved 1 meter forward");
     else
     ROS_INFO("The base failed to move forward 1 meter for some reason");

     return 0;
}

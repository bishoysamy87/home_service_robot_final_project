#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  ros::Publisher pickstatus_pub = n.advertise<std_msgs::String>("pickstatus", 1000);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal1,goal2;

  // set up the frame parameters
  goal1.target_pose.header.frame_id = "map";
  goal1.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal1.target_pose.pose.position.x = 2.0;
  goal1.target_pose.pose.position.y = 2.0;
  goal1.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending a pickup goal");
  ac.sendGoal(goal1);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
{
    ROS_INFO("reached to pickup goal ");
    std_msgs::String msg;

    std::stringstream ss;
    ss << "reached_pickup";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    pickstatus_pub.publish(msg);
    ros::Duration(5.0).sleep();
// set up the frame parameters
  goal2.target_pose.header.frame_id = "map";
  goal2.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach

  goal2.target_pose.pose.position.x = 2.0;
  goal2.target_pose.pose.position.y = -2.0;
  goal2.target_pose.pose.orientation.w = 1.0;
  ROS_INFO("Sending a dropoff goal");
  ac.sendGoal(goal2);
  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
{
           std::stringstream ss;
           ROS_INFO("reached to dropoff goal ");
	    ss << "reached_dropoff";
	    msg.data = ss.str();
           pickstatus_pub.publish(msg);
}
  else
           ROS_INFO("The base failed to reach dropoff goal for some reason");
}
  else
    ROS_INFO("The base failed to reach pickup goal for some reason");
while(1);

  return 0;
}

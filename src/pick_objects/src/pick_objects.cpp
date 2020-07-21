
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

//target locations
float pickUpTarget[3] = {6.0, 3.0, 1.0};  // 
float dropOffTarget[3] = {0.0, 0.0, 1.0}; //

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  MoveBaseClient ac("move_base", true);

  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = pickUpTarget[0];
  goal.target_pose.pose.position.y = pickUpTarget[1];
  goal.target_pose.pose.orientation.w = pickUpTarget[2];

  ROS_INFO("Sending pick up target");
  ac.sendGoal(goal);

  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("the pick up target reached");
    ros::Duration(5.0).sleep();
    // new target - drop off area
    goal.target_pose.pose.position.x = dropOffTarget[0];
    goal.target_pose.pose.position.y = dropOffTarget[1];
    goal.target_pose.pose.orientation.w = dropOffTarget[2];
    ROS_INFO("Sending drop off target");
    ac.sendGoal(goal);
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("the drop off target reached");
      ros::Duration(5.0).sleep();
    }
    else 
    {
      ROS_INFO("the home robot failed for move to drop off location for some reason");
    }
  }
  else
  {
    ROS_INFO("the home robot failed for move to pick up location for some reason");
  }

  return 0;
}

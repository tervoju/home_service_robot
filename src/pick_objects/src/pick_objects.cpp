
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

struct Position
{
  float x;
  float y;
  float w;
};

//Targets
Position pickUpTarget = {6.0, 3.0, 1.0};
Position dropOffTarget = {0.0, 0.0, 1.0};

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// set target
void setTargetArea(move_base_msgs::MoveBaseGoal *goal, Position target)
{
  goal->target_pose.pose.position.x = target.x;
  goal->target_pose.pose.position.y = target.y;
  goal->target_pose.pose.orientation.w = target.w;
}

int main(int argc, char **argv)
{
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle nPrivate("~");

  // if parameters ok for pickup, use them otherwise use defaults

  if (nPrivate.hasParam("x") && nPrivate.hasParam("y"))
  {
    ROS_INFO("x and y parameters received");
    double x, y;
    // checked in add marker - should be same arguments
    if (nPrivate.getParam("x", x) && nPrivate.getParam("y", y))
    {
        pickUpTarget.x = x;
        pickUpTarget.y = y;
        ROS_INFO("x and y parameters ok");
    }
  }
  else
  {
    ROS_INFO("no x and y parameters received");
  }
  ROS_INFO("x and y parameters ok %f %f", pickUpTarget.x, pickUpTarget.y);

  MoveBaseClient ac("move_base", true);

  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // new target - pick up area
  setTargetArea(&goal, pickUpTarget);
  ROS_INFO("Sending pick up target");
  ac.sendGoal(goal);
  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("the pick up target reached");
    ros::Duration(5.0).sleep();

    // new target - drop off area
    setTargetArea(&goal, dropOffTarget);
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

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <complex>
#include <string>

struct Position
{
    float x;
    float y;
    float w;
};

//marker positions
int idx = 0;
Position pickUp = {6.0, 3.0, 1.0};
Position dropOff = {0.0, 0.0, 1.0};
Position threshold = {0.3, 0.3, 0.01};

// areas
bool atPickUp = false;
bool atDropOff = false;
// delivery status
bool pickUpDone = false;
bool dropOffDone = false;

// if more that one pickup
void resetTargetsAndState()
{
    // areas
    atPickUp = false;
    atDropOff = false;
    // delivery status
    pickUpDone = false;
    dropOffDone = false;
}

// visualization marker setting
void createMarker(visualization_msgs::Marker *marker, Position area)
{
    marker->header.frame_id = "map";
    marker->header.stamp = ros::Time::now();
    // Any marker sent with the same namespace and id will overwrite the old one
    marker->ns = "basic_shapes";
    marker->id = 0;
    // Set the marker type as cube
    marker->type = visualization_msgs::Marker::CUBE;
    // Set the marker action.
    marker->action = visualization_msgs::Marker::ADD;
    // Set the pose of the marker.
    marker->pose.position.x = area.x;
    marker->pose.position.y = area.y;
    marker->pose.position.z = 0;
    marker->pose.orientation.x = 0.0;
    marker->pose.orientation.y = 0.0;
    marker->pose.orientation.z = 0.0;
    marker->pose.orientation.w = area.w;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker->scale.x = 0.3;
    marker->scale.y = 0.3;
    marker->scale.z = 0.3;
    // Set the color -- be sure to set alpha to something non-zero!
    marker->color.r = 1.0f;
    marker->color.g = 0.0f;
    marker->color.b = 0.0f;
    marker->color.a = 1.0;
    marker->lifetime = ros::Duration();
}
// visualization marker change
void changeMarkerPosition(visualization_msgs::Marker *marker, Position area)
{
    marker->pose.position.x = area.x;
    marker->pose.position.y = area.y;
    marker->pose.orientation.w = area.w;
    marker->action = visualization_msgs::Marker::ADD;
}
bool checkArea(Position roboPos, Position target)
{
    if (((std::abs(target.x - roboPos.x) < threshold.x)) && ((std::abs(target.y - roboPos.y) < threshold.y)) && ((std::abs(target.w - roboPos.w) < threshold.w)))
    {
        return true;
    }
    return false;
}

//callback function
void odomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg)
{
    Position roboPos;
    roboPos.x = odomMsg->pose.pose.position.x;
    roboPos.y = odomMsg->pose.pose.position.y;
    roboPos.w = odomMsg->pose.pose.orientation.w;

    // checking the robot position if in pickup area
    checkArea(roboPos, pickUp) &&!atPickUp ? atPickUp = true : atPickUp = false;

    // checking the robot if in dropoff area
    checkArea(roboPos, dropOff) &&!atDropOff ? atDropOff = true : atDropOff = false;
}

// pickUp area should be inside the house 
bool checkParam(float x, float y)
{
    if (x > 7.0 || x < -3.0) 
        return false;
    if (y < 0.0)
        return false;
    if ((x <= 7.0 && x >= 6.0) && (y >= 0.0 && y <= 5.0))
        return true;
    if ((x >= -3.5 && x <= -2.5) && (y >= 0.0 && y <= 3.0))
        return true;
    if ((x >= -3.5 && x <= 7.0) && (y >=0.0 && y <= 1.0))
        return true;

}

int main(int argc, char **argv)
{
    ROS_INFO("Main add_markers");
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n("~");
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Subscriber odom_sub = n.subscribe("odom", 1000, odomCallback);
    visualization_msgs::Marker marker;

    // add marker to param
    // if parameters ok for pickup, use them otherwise use defaults
    if (n.hasParam("x") && n.hasParam("y"))
    {
        ROS_INFO("x and y parameters received");
        double x, y;
        if (n.getParam("x", x) && n.getParam("y", y))
        {
            if (checkParam(x,y))
            {
                ROS_INFO("x and y parameters ok");
                pickUp.x = x;
                pickUp.y = y;
            }
            else
            {
                ROS_INFO("x and y parameters not ok");
            }
        }
    }
    else 
    {
          ROS_INFO("no x and y parameters received");
    }

    createMarker(&marker, pickUp);

    while (ros::ok())
    {
        // Publish the marker
        while (marker_pub.getNumSubscribers() < 1)
        {
            if (!ros::ok())
            {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }

        marker_pub.publish(marker);
        ROS_INFO("Show Pick-up marker");

        //Loop : Wait for Pick Up to happen
        while (!atPickUp)
        {
            ros::spinOnce();
        }

        // pickup place found and marker removed
        if (atPickUp && !pickUpDone)
        {
            marker.action = visualization_msgs::Marker::DELETE;
            marker_pub.publish(marker);
            ROS_INFO("Pick-up marker removed");
            pickUpDone = true;
        }

        //Loop : Wait for Drop Off area to be found
        while (!atDropOff)
        {
            ros::spinOnce();
        }

        // pick up done and drop off to ne done
        if (atDropOff && !dropOffDone)
        {
            ROS_INFO("At Drop-off ");
            changeMarkerPosition(&marker, dropOff);
            marker_pub.publish(marker);
            ROS_INFO("Show Drop-off marker");
            dropOffDone = true;
            ros::Duration(10.0).sleep();
        }
        return 0;
    }
}
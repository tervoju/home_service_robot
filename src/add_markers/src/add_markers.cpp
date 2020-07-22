#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <complex>

struct Position
{
    float x;
    float y;
    float w;
};

//marker positions
Position pickUp = {6.0, 3.0, 1.0};
Position dropOff = {0.0, 0.0, 1.0};
Position threshold = {0.3, 0.3, 0.01};

// areas & flags
bool atPickUp = false;
bool atDropOff = false;
bool pickUpDone = false;
bool dropOffDone = false;

// visualization marker

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
    marker->scale.x = 0.5;
    marker->scale.y = 0.5;
    marker->scale.z = 0.5;
    // Set the color -- be sure to set alpha to something non-zero!
    marker->color.r = 1.0f;
    marker->color.g = 0.0f;
    marker->color.b = 0.0f;
    marker->color.a = 1.0;
    marker->lifetime = ros::Duration();
}

bool checkArea(Position roboPos, Position target)
{
    if (((std::abs(target.x - roboPos.x) < threshold.x)) && ((std::abs(target.y - roboBos.y) < threshold.y)) && ((std::abs(target.w - w) < threshold.z)))
    {
        return true;
    }
    return false;
}

//callback function
void odomCallback(const nav_msgs::Odometry::ConstPrt &odomMsg)
{
    Position roboPos;
    roboPos.x = odomMsg->pose.pose.position.x;
    roboPos.y = odomMsg->pose.pose.position.y;
    roboPos.z = odomMsg->pose.pose.position.w;

    // checking the robot position if in pickup area
    checkArea(roboPos, pickUp) ? atPickUp = true : atPickUp = false;

    // checking the robot if in dropoff area
    checkArea(roboPos, dropOff) ? atDropOff = true : atDropOff = false;
}

int main(int argc, char **argv)
{
    ROS_INFO("Main");
    ros::init(argc, argv, "add_markers");

    //
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Subscriber odom_sub = n.subscribe("odom", 1000, odomCallback);
    visualization_msgs::Marker marker;

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

        //Loop : Wait for Drop Off to happen
        while (!atDropOff)
        {
            ros::spinOnce();
        }

        // pick up done and drop off to ne done
        if (atDropOff && !dropOffDone)
        {
            marker.pose.position.x = dropOff.x;
            marker.pose.position.y = dropOff.y;
            marker.pose.orientation.w = dropOff.z;
            marker.action = visualization_msgs::Marker::ADD;
            marker_pub.publish(marker);
            ROS_INFO("Show Drop-off marker");
            dropOffDone = true;
            ros::Duration(10.0).sleep();
        }
        return;
    }
}
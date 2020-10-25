#include "ros/ros.h"

#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

static ros::Publisher odomgetpub;
static ros::Publisher odomsetpub;

void odomgetCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  geometry_msgs::Pose2D pose;
  pose.x = msg->pose.pose.position.x;
  pose.y = msg->pose.pose.position.y;

  tf::Quaternion q(
     msg->pose.pose.orientation.x,
     msg->pose.pose.orientation.y,
     msg->pose.pose.orientation.z,
     msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  pose.theta = yaw;

  odomgetpub.publish(pose);
}

void odomsetCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
  nav_msgs::Odometry odom;

  odom.pose.pose.position.x = msg->x;
  odom.pose.pose.position.y = msg->y;

  tf2::Quaternion quat;
  quat.setRPY( 0, 0, msg->theta);
  odom.pose.pose.orientation.x = quat[0];
  odom.pose.pose.orientation.y = quat[1];
  odom.pose.pose.orientation.z = quat[2];
  odom.pose.pose.orientation.w = quat[3];

  odomsetpub.publish(odom);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_converter");
  ros::NodeHandle n;

  odomgetpub = n.advertise<geometry_msgs::Pose2D>("/odom/2D/get", 2);
  ros::Subscriber odomgetsub = n.subscribe("/odom/get", 2, odomgetCallback);

  odomsetpub = n.advertise<nav_msgs::Odometry>("/odom/set", 2);
  ros::Subscriber odomsetsub = n.subscribe("/odom/2D/set", 2, odomsetCallback);

  ros::spin();

  return 0;
}

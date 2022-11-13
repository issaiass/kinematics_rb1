#include "kinematics_rb1/odometry_reader.hpp"
#include "tf/LinearMath/Matrix3x3.h"

void OdometryReader::odom_subscriber_cb(const nav_msgs::Odometry::ConstPtr &msg) {
    odom_pose["x"] = msg->pose.pose.position.x;
    odom_pose["y"] = msg->pose.pose.position.y;
    tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    odom_pose["theta"] = yaw;
}

OdometryReader::OdometryReader(ros::NodeHandle &nh, const std::string &topic) : nh_(nh) {
  odom_subscriber = nh_.subscribe<nav_msgs::Odometry>(topic, 100, &OdometryReader::odom_subscriber_cb, this);
  ros::Rate rate(10);
  rate.sleep();
}

void OdometryReader::unregister(void) {
  odom_subscriber.shutdown();
}
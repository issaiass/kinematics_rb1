#include "kinematics_rb1/velocity_controller.hpp"
#include "kinematics_rb1/velocity_controller.hpp"

VelocityController::VelocityController(ros::NodeHandle &nh, const std::string &topic) : nh_(nh) {
  ros::Rate rate(0.1);
  cmd_vel = nh_.advertise<geometry_msgs::Twist>(topic, 100);
  rate.sleep();
}

void VelocityController::move(geometry_msgs::Twist tw) {
    cmd_vel.publish(tw);
}
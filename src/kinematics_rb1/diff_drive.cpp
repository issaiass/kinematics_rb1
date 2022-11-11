/*#include "ros/init.h"
#include "ros/publisher.h"*/
#include <kinematics_rb1/diff_drive.hpp>


void DiffDrive::rb1_vel_cmd_cb(const geometry_msgs::Twist::ConstPtr &msg) {
  twist_vels = *msg;
  ROS_INFO_STREAM("Subscriber velocities:"<<" linear="<<twist_vels.linear.x<<" angular="<<twist_vels.angular.z);
}

DiffDrive::DiffDrive(ros::NodeHandle &nh) : nh_(nh) {  
  rb1_vel_cmd = nh_.subscribe<geometry_msgs::Twist>("/rb1_vel_cmd", 1000, &DiffDrive::rb1_vel_cmd_cb, this);
  lt_cmd = nh_.advertise<std_msgs::Float64>("/left_wheel_controller/command", 10);
  rt_cmd = nh_.advertise<std_msgs::Float64>("/right_wheel_controller/command", 10);
}

void DiffDrive::convertVelocities(void) {
  vr.data = (2.0*twist_vels.linear.x + twist_vels.angular.z*k_L)/(2*k_R);
  vl.data = (2.0*twist_vels.linear.x - twist_vels.angular.z*k_L)/(2*k_R);
  ROS_INFO_STREAM("vr:"<< vr.data << " , vl: " << vl.data);
}

void DiffDrive::pubWheelVels(void) {
    convertVelocities();
    lt_cmd.publish(vl);
    rt_cmd.publish(vr);
}





int main(int argc, char **argv) {
  ros::init(argc, argv, "diff_drive_kinematics");
  ros::NodeHandle n;
  ros::Rate rate(100);
  DiffDrive robot(n);

  while(ros::ok()) {
    ros::spinOnce();
    robot.pubWheelVels();
    rate.sleep();
  }
}
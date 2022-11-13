#include "kinematics_rb1/kinematic_controller.hpp"

void KinematicController::goal_subscriber_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  goal_pose = *msg;
  xg = goal_pose.pose.position.x;
  yg = goal_pose.pose.position.y;
  tf::Quaternion q = tf::Quaternion(goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  thetag = yaw;
  new_pose = true;
}

KinematicController::KinematicController(ros::NodeHandle &nh, const std::string &topic) : nh_(nh), 
                                                                                          xg(-0.5), 
                                                                                          yg(-1.4), 
                                                                                          thetag(0),
                                                                                          new_pose(false) {
  goal_subscriber = nh_.subscribe<geometry_msgs::PoseStamped>(topic, 1000, &KinematicController::goal_subscriber_cb, this);
  ros::Rate rate(0.5);
  rate.sleep();
}

float KinematicController::normalize(float angle) {
  return atan2(sin(angle), cos(angle));
}

void KinematicController::go_to(OdometryReader &odometry, VelocityController &velocity, float &constant_vel) {
    ros::Rate rate(100);
    float rho = std::numeric_limits<float>::max();
    float beta = std::numeric_limits<float>::max();
    while(rho > 0.1 && new_pose && !ros::isShuttingDown()) {
      ros::spinOnce();
      float dx = xg - odometry.odom_pose["x"];    
      float dy = yg - odometry.odom_pose["y"];
      rho = sqrt(pow(dx,2) + pow(dy, 2));
      float theta = odometry.odom_pose["theta"];
      float alpha = normalize(atan2(dy, dx) - theta);
      float beta = normalize(thetag - atan2(dy, dx));
      float v = k_rho*rho;
      float w = k_alpha*alpha + k_beta*beta;
      if (constant_vel) {
        float abs_v = abs(v);
        v = v/abs_v*constant_vel;
        w = w/abs_v*constant_vel; 
      }
      geometry_msgs::Twist tw;
      tw.linear.x = v;
      tw.angular.z = w;
      velocity.move(tw);
      rate.sleep();
    }
    new_pose = false;
    geometry_msgs::Twist tw;
    tw.linear.x = 0;
    tw.angular.z = 0;
    velocity.move(tw);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "diff_drive_kinematic_controller");
  ros::NodeHandle n("~");
  ROS_INFO("Kinematic Controller Initialized!");
  VelocityController velocity(n, "/rb1_vel_cmd");
  OdometryReader odometry(n, "/rb1_odom");
  KinematicController kinematic_controller(n, "/move_base_simple/goal");
  float constant_vel = 0.15;
  while(!ros::isShuttingDown()) {
    ros::spinOnce();
    kinematic_controller.go_to(odometry, velocity, constant_vel);    
  }
  return 0;
}
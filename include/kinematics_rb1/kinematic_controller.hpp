#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <limits>
#include <math.h>
#include "../include/kinematics_rb1/velocity_controller.hpp"
#include "../include/kinematics_rb1/odometry_reader.hpp"
#include <map>
#include <tf/tf.h>

using namespace std;

class KinematicController {
  public:
    KinematicController(ros::NodeHandle &nh, const std::string &topic);
    void go_to(OdometryReader &odometry, VelocityController &velocity, float &constant_vel);


  private:
    void goal_subscriber_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    float normalize(float angle);
    float xg;
    float yg;
    float thetag;
    ros::NodeHandle &nh_;
    ros::Subscriber goal_subscriber; 
    geometry_msgs::PoseStamped goal_pose;
    const float k_rho = 0.3;
    const float k_alpha = 0.8;
    const float k_beta = -0.15;
    bool new_pose;
};
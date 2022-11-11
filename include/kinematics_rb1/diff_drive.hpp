#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>


using namespace std;

class DiffDrive {
public:
  DiffDrive(ros::NodeHandle &nh);
  void pubWheelVels(void);
  geometry_msgs::Twist twist_vels;
  
private:  
  void rb1_vel_cmd_cb(const geometry_msgs::Twist::ConstPtr &msg);
  void convertVelocities(void);
  ros::NodeHandle &nh_;
  ros::Subscriber rb1_vel_cmd;
  ros::Publisher lt_cmd;
  ros::Publisher rt_cmd;
  std_msgs::Float64 vr;
  std_msgs::Float64 vl;
  const float k_L = 0.3;
  const float k_R = 0.1;
};

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using namespace std;

class VelocityController {
  public:
    VelocityController(ros::NodeHandle &nh, const std::string &topic);
    void move(geometry_msgs::Twist tw);

  private:  
    ros::NodeHandle &nh_;
    ros::Publisher cmd_vel;
};
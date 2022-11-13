#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <map>
#include <vector>

using namespace std;

class OdometryReader {
  public:
    OdometryReader(ros::NodeHandle &nh, const std::string &topic);
    void unregister(void);
    map<string, float> odom_pose;

  private:
    void odom_subscriber_cb(const nav_msgs::Odometry::ConstPtr &msg);
    ros::NodeHandle &nh_;
    ros::Subscriber odom_subscriber;
};
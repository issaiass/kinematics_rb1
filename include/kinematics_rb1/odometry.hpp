#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include <rosgraph_msgs/Clock.h>
#include "kinematics_rb1/encoders.h"

using namespace std;

class  Odom {
public:
    Odom(ros::NodeHandle &nh);
    void rb1_getTicks(const kinematics_rb1::encoders::ConstPtr &msg);
    void rb1_clock_cb(const rosgraph_msgs::Clock::ConstPtr &msg);

private:  
    void updatePose(void);
    ros::NodeHandle &nh_;
    float currentLeftTicks;
    float currentRightTicks;
    float lastLeftTicks;
    float lastRightTicks;
    ros::Subscriber ticks_sub;
    ros::Subscriber clock;
    ros::Publisher odom_pub;
    float x, y, th;
    ros::Time current_time;
    ros::Time last_time;
    const float k_L = 0.3;
    const float k_R = 0.1;
    const float k_N = 360;
//    ros::ServiceClient client;
};
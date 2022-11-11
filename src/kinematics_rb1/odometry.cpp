#include <kinematics_rb1/odometry.hpp>

void Odom::rb1_getTicks(const kinematics_rb1::encoders::ConstPtr &msg) {
    ROS_INFO_STREAM("Subscriber left tick: " <<msg->encoderTicks[0]<<" right tick: "<<msg->encoderTicks[1]);
    currentLeftTicks = msg->encoderTicks[0];
    currentRightTicks = msg->encoderTicks[1];
}

void Odom::rb1_clock_cb(const rosgraph_msgs::Clock::ConstPtr &msg) {
  ;
}

Odom::Odom(ros::NodeHandle &nh) :  nh_(nh),
                                  currentLeftTicks(0), currentRightTicks(0),
                                  lastLeftTicks(0), lastRightTicks(0),
                                  x(0), y(0), th(0), 
                                  current_time(ros::Time::now()), last_time(ros::Time::now()) {  
    ticks_sub =  nh.subscribe<kinematics_rb1::encoders>("/encoders", 100, &Odom::rb1_getTicks, this);
    odom_pub = nh.advertise<nav_msgs::Odometry> ("/rb1_odom", 100);  
    clock = nh.subscribe<rosgraph_msgs::Clock>("/clock", 100, &Odom::rb1_clock_cb, this);
    while(clock.getNumPublishers() <= 0) {
       ROS_INFO_STREAM("WAITING FOR CLOCK");
    }
    updatePose();
}

void Odom::updatePose(void) {
    
    ros::Rate rate(1);
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    while(!ros::isShuttingDown()) {
        ros::spinOnce();

        float delta_l = currentLeftTicks - lastLeftTicks;
        float delta_r = currentRightTicks - lastRightTicks;

        float d_l = 2*M_PI*k_R*delta_l/k_N;
        float d_r = 2*M_PI*k_R*delta_r/k_N;

        lastLeftTicks = currentLeftTicks;
        lastRightTicks = currentRightTicks;
 
        current_time = ros::Time::now();

        float dt = (current_time - last_time).toSec();

        if (dt == 0) {
            continue;
        }

        float v = ((d_r + d_l)/2)/dt;
        float w = ((d_r - d_l)/k_L)/dt;

        float delta_x = v*cos(th);
        float delta_y = v*sin(th);
        float delta_th = w;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
        
        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "rb1_odom";
        odom_trans.child_frame_id = "link_chassis";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        tf::TransformBroadcaster odom_broadcaster;
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "rb1_odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "link_chassis";
        odom.twist.twist.linear.x = v;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = w;

        //publish the message
        odom_pub.publish(odom);

        last_time = current_time;

        static_transformStamped.header.stamp = ros::Time::now();
        static_transformStamped.header.frame_id = "rb1_odom";
        static_transformStamped.child_frame_id = "link_chassis";
        static_transformStamped.transform.translation.x = x;
        static_transformStamped.transform.translation.y = y;
        static_transformStamped.transform.translation.z = 0;
        static_transformStamped.transform.rotation.x = odom_quat.x;
        static_transformStamped.transform.rotation.y = odom_quat.y;
        static_transformStamped.transform.rotation.z = odom_quat.z;
        static_transformStamped.transform.rotation.w = odom_quat.w;
        static_broadcaster.sendTransform(static_transformStamped);

        rate.sleep();
    }
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "diff_drive_odom");
  ros::NodeHandle n("~");
  Odom odom(n);
  ros::spin();
  return 0;
}
#! /usr/bin/env python

import rospy, math, numpy as np
from velocity_controller import VelocityController
from odometry_reader import OdometryReader
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

class KinematicController:
    def __init__(self, topic):
        self.k_rho = 0.3      
        self.k_alpha = 0.8
        self.k_beta = -0.15
        self.xg = -0.5 
        self.yg = -1.4
        self.thetag = 0.0
        self.new_pose = False
        self.goal_subcriber = rospy.Subscriber(topic, PoseStamped, self.callback)
        rospy.sleep(2)

    def callback(self, msg):
        self.xg = msg.pose.position.x
        self.yg = msg.pose.position.y
        orientation_q = msg.pose.orientation
        (_, _, self.thetag) = euler_from_quaternion ([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.new_pose = True

    def normalize(self, angle):
        return np.arctan2(np.sin(angle),np.cos(angle))

    def go_to(self, odometry, velocity, constant_vel = None):
        rho = float("inf") 
        beta = float("inf") 
        #self.thetag = math.radians(self.thetag)
        while rho>0.1 and self.new_pose:
            dx = self.xg - odometry.odom_pose['x'] 
            dy = self.yg - odometry.odom_pose['y']
            rho = np.sqrt(dx**2 + dy**2)
            theta = odometry.odom_pose['theta']
            alpha = self.normalize(np.arctan2(dy, dx) - theta)
            beta = self.normalize(self.thetag - np.arctan2(dy, dx))
            v = self.k_rho * rho
            w = self.k_alpha * alpha + self.k_beta * beta
            if constant_vel:
                abs_v = abs(v)
                v = v / abs_v * constant_vel
                w = w / abs_v * constant_vel
            velocity.move(v, w)
            rospy.sleep(0.01)
        self.new_pose = False
        velocity.move(0,0)
        

if __name__ == "__main__":
    rospy.init_node('kinematic_controller', anonymous=True)
    rospy.loginfo("Kinematics Controller Initialized.")

    velocity = VelocityController('/rb1_vel_cmd')
    odometry = OdometryReader('/rb1_odom')
    kinematic_controller = KinematicController("/move_base_simple/goal")
    while(not rospy.is_shutdown()):
        kinematic_controller.go_to(odometry, velocity, constant_vel=0.15)
    
    #odometry.unregister()
    #xypoints = [(wp[0], wp[1])for wp in waypoints]
    #np.save('waypoints',xypoints)
    #error = math.hypot(odometry.odom_pose['x'], odometry.odom_pose['y'])
    #print('Final positioning error is %.2fm' % error)
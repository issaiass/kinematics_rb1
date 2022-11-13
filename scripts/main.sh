#!/bin/sh

# launch gazebo
xterm -e "source ~/simulation_ws/devel/setup.bash;
          roslaunch rb1_base_gazebo warehouse_world.launch" &

sleep 6

# spawn robot
xterm -e "source ~/simulation_ws/devel/setup.bash;
          roslaunch rb1_diff_drive_kinematics spawn_robot.launch" &

sleep 2

# launch all nodes
xterm -e "source ~/catkin_ws/devel/setup.bash;
          cd ~/catkin_ws;
          roslaunch kinematics_rb1 main.launch;" &
#!/bin/bash

# Launch Gazebo
gnome-terminal -- bash -c "gazebo /home/lrs-ubuntu/LRS-FEI/worlds/fei_lrs_gazebo.world"

# Launch ArduPilot simulation
gnome-terminal -- bash -c "cd ~/ardupilot/ArduCopter; sim_vehicle.py -f gazebo-iris --console -l 48.15084570555732,17.072729745416016,150,0"

sleep 10
# Launch MAVROS
gnome-terminal -- bash -c "ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://127.0.0.1:14551@14555"

#. ./launchSITL.sh /home/lrs-ubuntu/fei_lrs_gazebo/worlds/fei_lrs_gazebo.world

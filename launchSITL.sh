#!/bin/bash
echo "Starting SITL"

if [ -z "$1" ]; then
    echo "Usage: $0 <world_file_path>"
    exit 1
fi

world_file_path=$1
gazebo_info_file_path=~/gInfo.inf
ardupilot_info_file_path=~/aInfo.inf

touch $gazebo_info_file_path && touch $ardupilot_info_file_path

# Launch Gazebo with the specified world file
gazebo --verbose $world_file_path > $gazebo_info_file_path &

# Wait for gInfo.inf to be created and contain the "Publicized address" string
while ! grep -q "Publicized address" $gazebo_info_file_path; do
    echo "Waiting for Publicized address to appear in gInfo.inf..."
    sleep 1
done

echo "Publicized address found in gInfo.inf."
echo "Lauching Ardupilot"

./runArdupilot.sh &
# Launch Ardupilot
# Wait for aInfo to be created and contain the "EKF3 IMU0 origin set" string
while ! grep -q "EKF3 IMU0 origin set" $ardupilot_info_file_path; do
    echo "Waiting for EKF3 IMU0 origin set to appear in aInfo..."
    sleep 1
done

echo "EKF3 IMU0 origin set found in aInfo."
echo "Launching MAVROS"
echo "" > $ardupilot_info_file_path
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://127.0.0.1:14551@14555

# Clean up
echo "Cleaning up..."
rm $gazebo_info_file_path && rm $ardupilot_info_file_path

#!/bin/bash
# Run this under Zeus/nodes/
# Make sure this script is executable with chmod +x execute.sh
# Provide this folder with a symbolic link to your George1.bag rosbag,
# name it George1.bag ex: ln ~/catkin_ws/src/lane_detector/George1.bag George1.bag
node="$1"
roscore &
sleep 1
if [ "$node" == "lane" ]; then 
	roslaunch lane_detector/lane_detection.launch &
elif [ "$node" == "kitti" ]; then 
	roslaunch lane_detector/kitti_lane_detection.launch &
elif [ "$node" == "stop" ]; then 
	roslaunch lane_detector/stop_line_detection.launch &
elif [ "$node" == "ipm" ]; then 
	roslaunch lane_detector/ipm_lane_detection.launch &
else
	echo "Usage: ./execute.sh <lane|stop|kitti|ipm>"
	exit
fi

sleep 1
rosrun image_transport republish compressed in:=/RawStereoToRos/left/image_raw raw out:=camera/image_raw &
sleep 1
rosbag play -s 450 -u 30 George1.bag
kill $(ps aux | grep 'roscore' | awk '{print $2}')
kill $(ps aux | grep 'roslaunch' | awk '{print $2}')
kill $(ps aux | grep 'republish' | awk '{print $2}')


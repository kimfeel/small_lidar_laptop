#!/bin/bash
#trap "kill %1; kill %2; kill %3; kill %4" HUP INT QUIT EXIT
trap "kill %1; kill %2; kill %3; kill %4" INT QUIT EXIT
roscore &
sleep 3
export ROS_IP=`hostname -I`
echo $ROS_IP
/home/rical/laser/lidar_segment.py &
/home/rical/laser/bin/voxel1 &
/opt/ros/melodic/bin/rviz
#sleep 3
#/opt/ros/melodic/bin/rosbag play ~/Desktop/bags/20180622_bh_scan/data_static2.bag
#sleep 100

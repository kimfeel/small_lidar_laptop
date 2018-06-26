#!/bin/bash
#trap "kill %1; kill %2; kill %3; kill %4" HUP INT QUIT EXIT
trap "kill %1" INT QUIT EXIT
export ROS_MASTER_URI=http://143.215.111.104:11311
export ROS_IP=`hostname -I`
echo $ROS_MASTER_URI
echo $ROS_IP
/home/rical/rplidar/launch/roslaunch view_rplidar360.launch
#sleep 3
#/opt/ros/melodic/bin/rosbag play ~/Desktop/bags/20180622_bh_scan/data_static2.bag
#sleep 100

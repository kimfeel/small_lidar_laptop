#!/bin/bash
./bin/voxel1 &
python lidar_segment.py &
rviz &

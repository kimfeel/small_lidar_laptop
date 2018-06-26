#!/usr/bin/python
import sys
import time
import numpy
import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import std_msgs

def publish_cloud(cloud):
	header = std_msgs.msg.Header()
	header.stamp = rospy.Time.now()
	header.frame_id = '/map'
	msg = point_cloud2.create_cloud_xyz32(header,cloud)
	pubCloud.publish(msg)

def readPCD(filename):
	pcd = open(filename,'r')
	for l in pcd:
		if l.startswith('DATA'):
			break
	points = []
	for l in pcd:
		ll = l.split()
		x = float(ll[0])
		y = float(ll[1])
		z = float(ll[2])
		try:
			rgb = int(ll[3])
			r = (rgb >> 16) & 0xFF
			g = (rgb >> 8) & 0xFF
			b = rgb & 0xFF
		except IndexError:
			r,g,b = 255,255,255
		points.append([x,y,z,r,g,b])
	pcd.close()
	return numpy.array(points)

rospy.init_node('lidar_publish')
pcd = readPCD('/home/rical/Desktop/mycloud.pcd')[:,:3]
pubCloud = rospy.Publisher('laser_cloud_surround', PointCloud2, queue_size=1)

publish_cloud(pcd)
time.sleep(2)

while not rospy.is_shutdown():
	publish_cloud(pcd)
	print('PCD: ',pcd.shape)
	time.sleep(1000)

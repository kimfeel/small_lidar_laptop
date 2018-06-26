#!/usr/bin/python

import sys
import roslib
import rospy
import time
import numpy
import os

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Int16
import std_msgs

class_to_color_rgb = {
	'clutter': (200,200,200), #clutter
	'board': (0,100,100), #board
	'bookcase': (255,0,0), #bookcase
	'beam': (255,200,200), #beam
	'chair': (0,0,100), #chair
	'column': (0,255,255), #column
	'door': (0,100,0), #door
	'sofa': (255,0,255), #sofa
	'table': (50,50,50), #table
	'window': (0,255,0), #window
	'ceiling': (255,255,0), #ceiling
	'floor': (0,0,255), #floor
	'wall': (255,165,0), #wall
	'person': (100,0,0), #person
}

def updateMarkers(idx, pcd, theta, centroid, prediction):
	marker = Marker()
	marker.header.frame_id = "/base_link";
	marker.header.stamp = rospy.Time.now();
	marker.type = Marker.LINE_LIST;
	marker.lifetime = rospy.Duration();
	marker.color.a = 1.0;
	marker.id = idx;
	marker.action = Marker.ADD;
	marker.color.r = class_to_color_rgb[prediction][0] / 255.0;
	marker.color.g = class_to_color_rgb[prediction][1] / 255.0;
	marker.color.b = class_to_color_rgb[prediction][2] / 255.0;
	marker.scale.x = 0.05;
	marker.pose.orientation.w = 1.0;
	box = numpy.zeros((8,3))
	box[0,0] = box[4,0] = box[1,0] = box[5,0] = pcd[:,0].min()
	box[2,0] = box[6,0] = box[3,0] = box[7,0] = pcd[:,0].max()
	box[0,1] = box[4,1] = box[2,1] = box[6,1] = pcd[:,1].min()
	box[1,1] = box[5,1] = box[3,1] = box[7,1] = pcd[:,1].max()
	box[0,2] = box[1,2] = box[2,2] = box[3,2] = pcd[:,2].min()
	box[4,2] = box[5,2] = box[6,2] = box[7,2] = pcd[:,2].max()
	R = numpy.array([[numpy.cos(theta),-numpy.sin(theta)],[numpy.sin(theta),numpy.cos(theta)]])
	box[:,:2] = R.dot((box[:,:2] - centroid[:2]).transpose()).transpose() + centroid[:2]
	p1 = Point(*box[0,:])
	p2 = Point(*box[1,:])
	p3 = Point(*box[2,:])
	p4 = Point(*box[3,:])
	p5 = Point(*box[4,:])
	p6 = Point(*box[5,:])
	p7 = Point(*box[6,:])
	p8 = Point(*box[7,:])
	marker.points.append(p1);marker.points.append(p2);
	marker.points.append(p1);marker.points.append(p3);
	marker.points.append(p2);marker.points.append(p4);
	marker.points.append(p3);marker.points.append(p4);
	marker.points.append(p1);marker.points.append(p5);
	marker.points.append(p2);marker.points.append(p6);
	marker.points.append(p3);marker.points.append(p7);
	marker.points.append(p4);marker.points.append(p8);
	marker.points.append(p5);marker.points.append(p6);
	marker.points.append(p5);marker.points.append(p7);
	marker.points.append(p6);marker.points.append(p8);
	marker.points.append(p7);marker.points.append(p8);
	pubMarker.publish(marker);

def deleteMarkers(idx):
	marker = Marker()
	marker.header.frame_id = "/base_link";
	marker.header.stamp = rospy.Time.now();
	marker.id = idx;
	marker.action = Marker.DELETE;
	pubMarker.publish(marker);

def updateText(idx, pcd, theta, centroid, prediction):
	marker = Marker()
	marker.header.frame_id = "/base_link";
	marker.header.stamp = rospy.Time.now();
	marker.id = idx;
	marker.type = Marker.TEXT_VIEW_FACING;
	marker.action = Marker.ADD;
	R = numpy.array([[numpy.cos(theta),-numpy.sin(theta)],[numpy.sin(theta),numpy.cos(theta)]])
	new_pcd = R.dot((pcd[:,:2] - centroid[:2]).transpose()).transpose() + centroid[:2]
	marker.pose.position.x = new_pcd[:,0].mean();
	marker.pose.position.y = new_pcd[:,1].mean();
	marker.pose.position.z = pcd[:,2].max();
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.text = prediction;
	marker.scale.x = marker.scale.y = marker.scale.z = 0.5;
	marker.color.r = class_to_color_rgb[prediction][0] / 255.0;
	marker.color.g = class_to_color_rgb[prediction][1] / 255.0;
	marker.color.b = class_to_color_rgb[prediction][2] / 255.0;
	marker.color.a = 1.0;
	pubMarker.publish(marker)

def publish_cloud(cloud):
	header = std_msgs.msg.Header()
	header.stamp = rospy.Time.now()
	header.frame_id = '/base_link'
	msg = point_cloud2.create_cloud_xyz32(header,cloud)
	pubCloud.publish(msg)

def convexHull(P):
	n = len(P)
	k = 0
	H = [None] * (2*n)
	if n==0:
		return []
	P = sorted(P,key=lambda x:x[1])
	P = sorted(P,key=lambda x:x[0])
	for i in range(n):
		while k >= 2 and numpy.cross(H[k-1]-H[k-2], P[i]-H[k-2]) <= 0:
			k -= 1
		H[k] = P[i]
		k += 1
	t = k+1
	for i in range(n-2,-1,-1):
		while k >= t and numpy.cross(H[k-1]-H[k-2], P[i]-H[k-2]) <= 0:
			k -= 1
		H[k] = P[i]
		k += 1
	return H[:k-1]

def downsample(points, res):
	output = []
	S = set()
	for p in points:
		k = tuple((p[:3] / res).astype(int))
		if not k in S:
			S.add(k)
			output.append(p)
	print('downsample %d -> %d'%(len(points), len(output)))
	return numpy.array(output)

def get_features(points):
	cx = (points[:,0].max() + points[:,0].min()) / 2
	cy = (points[:,1].max() + points[:,1].min()) / 2
	cz = (points[:,2].max() + points[:,2].min()) / 2
	width = points[:,0].max() - points[:,0].min()
	length = points[:,1].max() - points[:,1].min()
	height = points[:,2].max() - points[:,2].min()
	top = points[points[:,2] > cz, :]
	bottom = points[points[:,2] <= cz, :]
	if len(top)==0 or len(bottom)==0:
		topRatio = 1
	else:
		topArea = (top[:,0].max() - top[:,0].min()) * (top[:,1].max() - top[:,1].min())
		bottomArea = (bottom[:,0].max() - bottom[:,0].min()) * (bottom[:,1].max() - bottom[:,1].min())
		topRatio = 1.0 * topArea/bottomArea if bottomArea > 0 else 1
	width = max(width, 0.01)
	length = max(length, 0.01)
	height = max(height, 0.01)
	train_ft1 = numpy.maximum(width, length) / numpy.minimum(width, length)
	train_ft2 = height / numpy.minimum(width, length)
	train_ft3 = topRatio
#	return [train_ft1, train_ft2, train_ft3]
	return [train_ft1, train_ft2, 0]

def get_pointnet_features(p):
	centroid = 0.5 * (p[:,:2].min(axis=0) + p[:,:2].max(axis=0))
	p[:,:2] -= centroid
	R = numpy.sum(p**2,axis=1)
	p /= numpy.sqrt(numpy.max(R))
	subset = numpy.random.choice(len(p), NUM_POINT, replace=NUM_POINT>len(p))
	q = p[subset,:]
	q = numpy.array(sorted(q, key=lambda x:x[2]))
	R = numpy.sum(q[:,:2]**2,axis=1)
	id_max = numpy.argmax(R)
	principal_vector = q[id_max,:2].copy()
	principal_vector /= numpy.linalg.norm(principal_vector)
	principal_vector_perp = numpy.array([-principal_vector[1],principal_vector[0]])
	R = numpy.vstack((principal_vector,principal_vector_perp)).transpose()
	q[:,:2] = q[:,:2].dot(R)

	q = numpy.maximum(q.dot(k0) + b0, 0).reshape((256, 4, 128)).max(axis=1)
	q = numpy.maximum(q.dot(k1) + b1, 0).reshape((64, 4, 128)).max(axis=1)
	q = numpy.maximum(q.dot(k2) + b2, 0).reshape((16, 4, 128)).max(axis=1)
	q = numpy.maximum(q.dot(k3) + b3, 0).reshape((16, 256)).max(axis=0)
	return q

numSteps = 0
targetSteps = 500
direction = 'positive'
previousMarkers = 0
def steps_callback(steps):
	global numSteps
	numSteps = int(steps.data)

def cloud_surround_callback(cloud):
	global direction, targetSteps, previousMarkers
	if direction=='positive' and numSteps < targetSteps or direction=='negative' and numSteps > targetSteps:
		return
	if direction == 'positive':
		targetSteps += 500
		if targetSteps > 2040:
			direction = 'negative'
			targetSteps = 2000
	else:
		targetSteps -= 500
		if targetSteps < 10:
			direction = 'positive'
			targetSteps = 500
	pcd = []
	for p in point_cloud2.read_points(cloud, field_names=("x","y","z"), skip_nans=True):
		pcd.append(p)
	pcd = numpy.hstack((pcd, numpy.zeros((len(pcd),3))))
	pcd = downsample(pcd, 0.1)

	#rotate to cartesian axes
	centroid = pcd[:,:3].mean(axis=1)
	points_xy = pcd[:,:2] - centroid[:2]
	hull = numpy.array(convexHull(points_xy))
	edgeAngles = []
	for i in range(len(hull) - 1):
		theta = numpy.arctan2(hull[i+1][1] - hull[i][1] , hull[i+1][0] - hull[i][0])
		edgeAngles.append(theta)
	minArea = float('inf')
	bestParams = None
	for theta in edgeAngles:
		R = numpy.array([[numpy.cos(theta),numpy.sin(theta)],[-numpy.sin(theta),numpy.cos(theta)]])
		vr = R.dot(points_xy.transpose())
		xmin = min(vr[0])
		xmax = max(vr[0])
		ymin = min(vr[1])
		ymax = max(vr[1])
		A = (xmax-xmin) * (ymax-ymin)
		if A < minArea:
			minArea = A
			bestParams = theta
	print('Applied %.1f degree rotation'%(bestParams/numpy.pi*180))
	R = numpy.array([[numpy.cos(bestParams),numpy.sin(bestParams)],[-numpy.sin(bestParams),numpy.cos(bestParams)]])
	pcd[:,:2] = R.dot((pcd[:,:2] - centroid[:2]).transpose()).transpose() + centroid[:2]
#	publish_cloud(pcd[:,:3])

	#segment major planes
	pcd_int = (pcd[:,:3] / 0.1).astype(int)
	cluster_label = numpy.zeros(len(pcd), dtype=int)
	cid = 1
	margin_left = [0.3,0.3,0.1]
	margin_right = [0.3,0.3,0.3]
	for k in range(3):
		u, c = numpy.unique(pcd_int[:,k], return_counts = True)
		for m in [u[numpy.argmax(c[:int(margin_left[k]*len(c))])], u[::-1][numpy.argmax(c[::-1][:int(margin_right[k]*len(c))])]]:
			mask = numpy.logical_or(numpy.logical_or(pcd_int[:,k]==m, pcd_int[:,k]==m-1), pcd_int[:,k]==m+1)
			cluster_label[mask] = cid
			cid += 1

	#connected components
	voxel_resolution = 0.2
	voxel_grid = {}
	for i in range(len(pcd)):
		if cluster_label[i] > 0:
			continue
		key = tuple((pcd[i,:3] / voxel_resolution).astype(int))
		if not key in voxel_grid:
			voxel_grid[key] = []
		voxel_grid[key].append(i)
	print("Created %d voxel grid"%len(voxel_grid))
	for i in range(len(pcd)):
		if cluster_label[i] > 0:
			continue
		key = tuple((pcd[i,:3] / voxel_resolution).astype(int))
		if not key in voxel_grid:
			continue
		Q = [key]
		while len(Q) > 0:
			q = Q[-1]
			del Q[-1]
			if not q in voxel_grid:
				continue
			cluster_label[voxel_grid[q]] = cid
			del voxel_grid[q]
			for dx in range(-1,2):
				for dy in range(-1,2):
					for dz in range(-1,2):
						Q.append((q[0]+dx,q[1]+dy,q[2]+dz))
		cid += 1

	print('Found %d clusters'%cluster_label.max())

	for i in range(1, cluster_label.max()+1):
		p = pcd[cluster_label==i, :]
		if len(p) < 50:
			continue
		f = get_features(p.copy())
#		f = get_pointnet_features(p.copy())
		if i in [1,2,3,4]:
			prediction = 'wall'
		elif i==5:
			prediction = 'floor'
		elif i==6:
			prediction = 'ceiling'
		else:
			dist = numpy.sum((f - train_features)**2, axis=1)
			prediction = train_labels[numpy.argmin(dist)]
			print(len(p), numpy.argmin(dist), prediction)
#			print('%.2f %.2f %.2f'%(f[0],f[1],f[2]))
			g = train_features[numpy.argmin(dist)]
#			print('%.2f %.2f %.2f'%(g[0],g[1],g[2]))
		updateMarkers(i*2, p, bestParams, centroid, prediction)
		updateText(i*2+1, p, bestParams, centroid, prediction)
	
	for i in range((cluster_label.max()+1)*2, previousMarkers):
		deleteMarkers(i)
	previousMarkers = (cluster_label.max()+1)*2
	

rospy.init_node('lidar_segment')
subscriber = rospy.Subscriber('laser_cloud_surround',PointCloud2,cloud_surround_callback)
rospy.Subscriber('steps',Int16 ,steps_callback)
#pubCloud = rospy.Publisher('laser_cloud_rotated', PointCloud2, queue_size=1)
pubMarker = rospy.Publisher('markers', Marker, queue_size=10)

NUM_POINT = 1024
train_folder = '/home/rical/laser'
#if not os.path.isdir(train_folder):
#	train_folder = '/home/jd/Desktop/cad_models'
f = open('%s/labels.txt' % train_folder, 'r')
train_labels = []
for l in f:
	train_labels.append(l.strip())
f.close()
train_features = numpy.load('%s/features.npy'%train_folder)
#k0 = numpy.load('%s/k0.npy'%train_folder)
#b0 = numpy.load('%s/b0.npy'%train_folder)
#k1 = numpy.load('%s/k1.npy'%train_folder)
#b1 = numpy.load('%s/b1.npy'%train_folder)
#k2 = numpy.load('%s/k2.npy'%train_folder)
#b2 = numpy.load('%s/b2.npy'%train_folder)
#k3 = numpy.load('%s/k3.npy'%train_folder)
#b3 = numpy.load('%s/b3.npy'%train_folder)

rospy.spin()

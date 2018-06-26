#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <sys/time.h>
#include "hpcd.h"
#include "lapack.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

int countLidar = 0;
int countPose = 0;
double leaf_size = 0.03;
ros::Publisher *pubCloud;
sensor_msgs::PointCloud2 cloudMsg;
std::vector<Point> points;
std::vector<Point> inc_points;
HPCD* hpcd = NULL;
tf::Vector3 translation = tf::Vector3(0,0,0);
//tf::Quaternion rotation = tf::Quaternion(0,0,0,1);
tf::Quaternion rotation = tf::Quaternion(0,1,0,0);
tf::Quaternion rot_correction;
tf::Quaternion translationOffset(0,0,-0.2,0);
//tf::Quaternion rotationOffset(0,0,1,0);
double previousTime=-1;
unsigned char* image_data = NULL;
int image_height,image_width;
tf::Quaternion image_translation;
tf::Quaternion image_rotation;
tf::Quaternion Twc(0,-0.1,-0.15,0);
//tf::Quaternion Rwc(sqrt(2)/2,0,0,sqrt(2)/2);
tf::Quaternion Rwc(0.6892349, 0.1579723, -0.2257698, 0.6700955 );
float focal_length = 971;

double moving_angle = 0;
double offset_x = 0;//0.25*cos(50*M_PI/180);
double offset_y = 0;//-0.25*sin(50*M_PI/180);
double initial_angle = 0;
double rad_per_step = 5.625 / 64 * M_PI / 180;

void savePCD(const char* filename, std::vector<Point> *P) {
	FILE* f = fopen(filename, "w");
	if (!f) {
		printf("Cannot write to file: %s\n", filename);
		return;
	}
	fprintf(f, "# .PCD v0.7 - Point Cloud Data file format\n"
		"VERSION 0.7\n"
		"FIELDS x y z rgb\n"
		"SIZE 4 4 4 4\n"
		"TYPE F F F I\n"
		"COUNT 1 1 1 1\n"
		"WIDTH %lu\n"
		"HEIGHT 1\n"
		"VIEWPOINT 0 0 0 1 0 0 0\n"
		"POINTS %lu\n"
		"DATA ascii\n", P->size(), P->size());
	for (size_t i = 0; i<P->size(); i++) {
		Point* h = P->data() + i;
		if (h) {
			int rgb = (h->r << 16) | (h->g << 8) | h->b;
			fprintf(f, "%f %f %f %d\n", h->x, h->y, h->z, rgb);
		}
	}
	fclose(f);
	printf("Wrote %lu points to %s\n", P->size(), filename);
}

double get_walltime(void) {
	struct timeval tp;
	gettimeofday(&tp, NULL);
	double d = (double) (tp.tv_sec + tp.tv_usec/1.e6);
	return d;
}

bool applyProjectionColor(Point* p) {
	//transform world frame to previous world frame where image was taken
	tf::Quaternion pos(p->x-translation.getX(),p->y-translation.getY(),p->z-translation.getZ(),0); 
	pos = rotation.inverse() * pos * rotation;
	//transform world frame to image frame
	pos = Rwc * pos * Rwc.inverse();
	pos += Twc;
	//check if point is behind camera
	if (pos.getZ() < 0)
		return false;
	//get image coordinates
	int u = (int)round(focal_length * pos.getX() / pos.getZ() + 0.5*(image_width-1));
	int v = (int)round(focal_length * pos.getY() / pos.getZ() + 0.5*(image_height-1));
	if (u>=0 && u<image_width && v>=0 && v<image_height) {
		unsigned char* c = image_data + (v*image_width + u)*3;
		p->r = *c++;
		p->g = *c++;
		p->b = *c++;
		return true;
	}
	return false;

}

void svd(int M,int N,double* A,double *U, double* S, double* VT) {
	int info, lwork=5*(M>N?N:M);
	double* work = new double[lwork];
	char jobu = 'A', jobvt = 'A';
	dgesvd_(&jobu, &jobvt, &M, &N, A, &M, 
		S, U, &M, VT, &N, work, &lwork, &info);
	//  printf("info: %d\n",info);
	//  printf("optimal: %f\n",work[0]);
	if (info!=0) {
		printf("Error in subroutine dgesvd_ (info=%d)\n",info);
	}
	delete[] work;
}

void applyOptimalRegistration(std::vector<Point> *P) {
	//find optimal registration
	std::vector<Point> source;
	std::vector<Point> target;
	float RMSE = 0;
	for (size_t i=0;i<P->size();i++) {
		Point currentPoint = P->at(i);
		int xv = (int)(currentPoint.x / leaf_size);
		int yv = (int)(currentPoint.y / leaf_size);
		int zv = (int)(currentPoint.z / leaf_size);
		Point* closestPoint = NULL;
		float closestDistance = 0;
		int dg = 2;
		for (int dx=-dg;dx<=dg;dx++) {
			for (int dy=-dg;dy<=dg;dy++) {
				for (int dz=-dg;dz<=dg;dz++) {
					int h = HPCD_find(hpcd, xv+dx, yv+dy, zv+dz);
					if (h >= 0) {
						Point* q = points.data() + hpcd->data[h]->handle;
						float d = 0;
						d += (currentPoint.x-q->x) * (currentPoint.x-q->x);
						d += (currentPoint.y-q->y) * (currentPoint.y-q->y);
						d += (currentPoint.z-q->z) * (currentPoint.z-q->z);
						if (closestPoint==NULL || d < closestDistance) {
							closestPoint = q;
							closestDistance = d;
						}
					}
				}
			}
		}
		if (closestPoint!=NULL) {
			RMSE += closestDistance;
			source.push_back(currentPoint);
			target.push_back(*closestPoint);
		}
	}
	RMSE = sqrt(RMSE / source.size());

	Point source_mean = {0,0,0,0,0,0};
	Point target_mean = {0,0,0,0,0,0};
	for (size_t i=0;i<source.size();i++) {
		source_mean.x += source[i].x;
		source_mean.y += source[i].y;
		source_mean.z += source[i].z;
		target_mean.x += target[i].x;
		target_mean.y += target[i].y;
		target_mean.z += target[i].z;
	}
	source_mean.x /= source.size();
	source_mean.y /= source.size();
	source_mean.z /= source.size();
	target_mean.x /= target.size();
	target_mean.y /= target.size();
	target_mean.z /= target.size();
	//column major
	double C[9];
	double U[9];
	double S[3];
	double VT[9];
	//row major
	double R[9];
	double T[3];
	for (size_t i=0;i<source.size();i++) {
		C[0] += source[i].x * target[i].x;
		C[3] += source[i].x * target[i].y;
		C[6] += source[i].x * target[i].z;
		C[1] += source[i].y * target[i].x;
		C[4] += source[i].y * target[i].y;
		C[7] += source[i].y * target[i].z;
		C[2] += source[i].z * target[i].x;
		C[5] += source[i].z * target[i].y;
		C[8] += source[i].z * target[i].z;
	}
	svd(3,3,C, U, S, VT);
	R[0] = U[0]*VT[0]+U[3]*VT[1]+U[6]*VT[2];
	R[1] = U[0]*VT[3]+U[3]*VT[4]+U[6]*VT[5];
	R[2] = U[0]*VT[6]+U[3]*VT[7]+U[6]*VT[8];
	R[3] = U[1]*VT[0]+U[4]*VT[1]+U[7]*VT[2];
	R[4] = U[1]*VT[3]+U[4]*VT[4]+U[7]*VT[5];
	R[5] = U[1]*VT[6]+U[4]*VT[7]+U[7]*VT[8];
	R[6] = U[2]*VT[0]+U[5]*VT[1]+U[8]*VT[2];
	R[7] = U[2]*VT[3]+U[5]*VT[4]+U[8]*VT[5];
	R[8] = U[2]*VT[6]+U[5]*VT[7]+U[8]*VT[8];
	T[0] = target_mean.x - (R[0]*source_mean.x + R[1]*source_mean.y + R[2]*source_mean.z);
	T[1] = target_mean.y - (R[3]*source_mean.x + R[4]*source_mean.y + R[5]*source_mean.z);
	T[2] = target_mean.z - (R[6]*source_mean.x + R[7]*source_mean.y + R[8]*source_mean.z);

	double previous_RMSE = RMSE;
	RMSE = 0;
	for (size_t i=0;i<P->size();i++) {
		Point p = P->at(i);
		Point q = {
			(float) (R[0]*p.x + R[1]*p.y + R[2]*p.z + T[0]),
			(float) (R[3]*p.x + R[4]*p.y + R[5]*p.z + T[1]),
			(float) (R[6]*p.x + R[7]*p.y + R[8]*p.z + T[2]),
			p.r,p.g,p.b
		};
		RMSE += (p.x - q.x) * (p.x - q.x);
		RMSE += (p.y - q.y) * (p.y - q.y);
		RMSE += (p.z - q.z) * (p.z - q.z);
		P->at(i).x = q.x;
		P->at(i).y = q.y;
		P->at(i).z = q.z;
	}
	RMSE = sqrt(RMSE / source.size());
	printf("Source %lu Target %lu RMSE: %.3f -> %.3f\n",source.size(), target.size(), previous_RMSE, RMSE);
}

//void moving_angle_callback(const std_msgs::Float32ConstPtr& d) {
void moving_angle_callback(const std_msgs::Int16ConstPtr& d) {
	//moving_angle = d->data;
	moving_angle = d->data * -1;
 	if ( (moving_angle == -2047) || (moving_angle == 0) ) {
		//sleep(5);
		points.clear();
	} 
}

void publish_object(ros::Publisher *pub, std::vector<Point> *P) {
	cloudMsg.header.stamp = ros::Time::now();
	cloudMsg.width = P->size();
	cloudMsg.row_step = P->size() * cloudMsg.point_step;
	cloudMsg.data.clear();
	cloudMsg.data.resize(cloudMsg.row_step);
	float* float_data = (float*) cloudMsg.data.data();
	unsigned char* byte_data = cloudMsg.data.data();
	for (size_t i=0;i<P->size();i++) {
		float_data[0] = P->at(i).x;
		float_data[1] = P->at(i).y;
		float_data[2] = P->at(i).z;
		byte_data[12] = P->at(i).r;
		byte_data[13] = P->at(i).g;
		byte_data[14] = P->at(i).b;
		float_data +=  cloudMsg.point_step / sizeof(float);
		byte_data += cloudMsg.point_step;
	}
	pub->publish(cloudMsg);
}

void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& lidarMsg) {
	if (initial_angle == 0)
		initial_angle = moving_angle;
	std::string tmp;
	double angle = lidarMsg->angle_min;
	inc_points.clear();
	double startTime = get_walltime();
	std::vector<Point> currentScan;
	int countColor = 0;

	for (size_t i=0;i<lidarMsg->ranges.size();i++) {	
		angle = lidarMsg->angle_min + i*lidarMsg->angle_increment;
		double dist = lidarMsg->ranges[i];
		Point p = {0,0,0,0,0,0};
		tmp = lidarMsg->header.frame_id;
		if (dist == 0)
			continue; 
		if (i>0&&i<lidarMsg->ranges.size()-1) {
			double leftDelta = fabs(lidarMsg->ranges[i] - lidarMsg->ranges[i-1]);
			double rightDelta = fabs(lidarMsg->ranges[i] - lidarMsg->ranges[i+1]);
			if (leftDelta + rightDelta > 1)
				continue;
		}

		p.x = (dist*sin(angle)+offset_x)*cos(moving_angle*rad_per_step);
		p.y = (dist*sin(angle)+offset_y)*sin(moving_angle*rad_per_step);
		p.z = dist * cos(angle);

		tf::Quaternion pos(p.x,p.y,p.z,0); 
		pos = rotation * pos * rotation.inverse();
		Point currentPoint = {
			(float)(pos.getX() + translation.getX()),
			(float)(pos.getY() + translation.getY()),
			(float)(pos.getZ() + translation.getZ()),
			255, 255, 255
		};
//		if (i < 370 && i%10 == 0)
//			currentScan.push_back(currentPoint);	// for dynamic scan	
			points.push_back(currentPoint);		// for static scan
	}
// comment for static scan (uncomment for dynamic scan)
/*	
	if ((moving_angle-initial_angle) > 0.5 * M_PI) { //completed quarter revolution
		applyOptimalRegistration(&currentScan);
	}

	printf("%d\n",currentScan.size());
	for (size_t i=0;i<currentScan.size();i++) {
		Point currentPoint = currentScan[i];
		int xv = (int)(currentPoint.x / leaf_size);
		int yv = (int)(currentPoint.y / leaf_size);
		int zv = (int)(currentPoint.z / leaf_size);
		if (HPCD_find(hpcd,xv,yv,zv) == -1) {
			HPoint* h1 = HPCD_add(hpcd, xv, yv, zv);
			if (applyProjectionColor(&currentPoint))
				countColor++;
//			else
//				continue;
			points.push_back(currentPoint);
			inc_points.push_back(currentPoint);
			h1->handle = points.size() - 1;
		}
	}
*/
	countLidar++;
	double endTime = get_walltime();
//	printf("scan %d: %lu points (%.4fs)\n",countLidar,points.size(),endTime-startTime);
	publish_object(pubCloud, &points);
}

int main(int argc, char* argv[]) {
	ros::init(argc,argv,"pcd");
	ros::NodeHandle n;
	ros::NodeHandle n_private("~");
	ros::Subscriber subLidar = n.subscribe("/scan",2,lidar_callback);
	ros::Subscriber subAngle = n.subscribe("/steps",2,moving_angle_callback);
	ros::Publisher pub1 = n.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround",1);
	pubCloud = &pub1;
	
	cloudMsg.header.frame_id = "/base_link";
	cloudMsg.height = 1;
	cloudMsg.is_bigendian = false;
	cloudMsg.point_step = 32;
	cloudMsg.is_dense = true;
	sensor_msgs::PointField field;
	field.name = "x";
	field.offset = 0;
	field.datatype = 7;
	field.count = 1;
	cloudMsg.fields.push_back(field);
	field.name = "y";
	field.offset = 4;
	cloudMsg.fields.push_back(field);
	field.name = "z";
	field.offset = 8;
	cloudMsg.fields.push_back(field);
	field.name = "r";
	field.offset = 12;
	field.datatype = 2;
	cloudMsg.fields.push_back(field);
	field.name = "g";
	field.offset = 13;
	cloudMsg.fields.push_back(field);
	field.name = "b";
	field.offset = 14;
	cloudMsg.fields.push_back(field);

	hpcd = new HPCD();
	hpcd->leafSize = leaf_size;
	hpcd->maxSize = 8;
	hpcd->data = new HPoint*[8]();
	hpcd->deepCopy = true;
	
	ros::spin();
	savePCD("/home/rical/rplidar_ros/mycloud.pcd", &points);
}

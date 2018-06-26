// ISARC_demo.cpp : Defines the entry point for the console application.
#define _CRT_SECURE_NO_DEPRECATE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <math.h>
#include <ctype.h>
#include "hpcd.h"
#define BASE_HASH_CONSTANT 0.618033988
#define STEP_HASH_CONSTANT 0.707106781
#define STRING_HASH_CONSTANT 5381

int labelWidth;
int labelHeight;
std::vector<char*> descriptions;
std::vector<unsigned char*> raster;

std::vector<Point> cloud;
std::vector<Point> modelVertices;
std::vector<Triangle> modelFaces;
std::vector<Point> sceneVertices;
std::vector<Triangle> sceneFaces;
std::vector< std::vector<float> > boxes;
char* currentPLY = NULL;

std::vector<char*> trainingPaths;
std::vector<char*> testPaths;
std::vector<Feature> trainingData;
std::vector<Feature> testData;
std::vector<char*> labelCategories;

int compareCluster(const void* v1, const void* v2) {
	Cluster* c1 = (Cluster*)v1;
	Cluster* c2 = (Cluster*)v2;
	return c2->size - c1->size;
}

inline int baseHash(int size, int hashKey) {
	return (int)(size*((BASE_HASH_CONSTANT*hashKey) - (int)(BASE_HASH_CONSTANT*hashKey)));
}

inline int stepHash(int size, int hashKey) {
	int res = (int)(size*((STEP_HASH_CONSTANT*hashKey) - (int)(STEP_HASH_CONSTANT*hashKey)));
	//make step size odd since table size is power of 2
	return res % 2 ? res : res + 1;
}

inline int getIntKey(int x, int y, int z) {
	int h = STRING_HASH_CONSTANT;
	h = (h << 5) + h + x;
	h = (h << 5) + h + y;
	h = (h << 5) + h + z;
	if (h < 0)
		return -h;
	else return h;
}

inline int getIntKey_xy(int x, int y) {
	int h = STRING_HASH_CONSTANT;
	h = (h << 5) + h + x;
	h = (h << 5) + h + y;
	if (h < 0)
		return -h;
	else return h;
}

int HPCD_find(HPCD* cloud, int x, int y, int z) {
	int ikey = getIntKey(x, y, z);
	int j = baseHash(cloud->maxSize, ikey);
	int step = stepHash(cloud->maxSize, ikey);
	for (int k = 0; k<cloud->maxSize; k++) {
		HPoint* h = cloud->data[j];
		if (!h) {
			return -1;
		}
		else if (h->x == x && h->y == y && h->z == z){
			return j;
		}
		else {
			j += step;
			j %= cloud->maxSize;
		}
	}
	return -1;
}

int HPCD_find_xy(HPCD* cloud, int x, int y) {
	int ikey = getIntKey_xy(x, y);
	int j = baseHash(cloud->maxSize, ikey);
	int step = stepHash(cloud->maxSize, ikey);
	for (int k = 0; k<cloud->maxSize; k++) {
		HPoint* h = cloud->data[j];
		if (!h) {
			return -1;
		}
		else if (h->x == x && h->y == y){
			return j;
		}
		else {
			j += step;
			j %= cloud->maxSize;
		}
	}
	return -1;
}

HPoint* HPCD_add(HPCD* cloud,int x,int y,int z) {
//	if (x<0 || y<0 || z<0)
//		return NULL;
	int ikey = getIntKey(x,y,z);
	int j = baseHash(cloud->maxSize,ikey);
	int step = stepHash(cloud->maxSize,ikey);
	HPoint* added = NULL;
	for (int k=0;k<cloud->maxSize;k++) {
		HPoint* h = cloud->data[j];
		if (!h) {
			HPoint* p = new HPoint();
			p->x = x;
			p->y = y;
			p->z = z;
			p->label = 0;
			cloud->numPoints++;
			cloud->data[j] = p;
			added = p;
			break;
		} else if (h->x == x && h->y == y && h->z == z){
			return h;
		} else {
			j += step;
			j %= cloud->maxSize;
		}
	}
	if (added && cloud->maxSize < cloud->numPoints * 4)
		HPCD_resize(cloud);
	return added;
}

HPoint* HPCD_add_xy(HPCD* cloud,int x,int y) {
//	if (x<0 || y<0 || z<0)
//		return NULL;
	int ikey = getIntKey_xy(x,y);
	int j = baseHash(cloud->maxSize,ikey);
	int step = stepHash(cloud->maxSize,ikey);
	HPoint* added = NULL;
	for (int k=0;k<cloud->maxSize;k++) {
		HPoint* h = cloud->data[j];
		if (!h) {
			HPoint* p = new HPoint();
			p->x = x;
			p->y = y;
			p->label = 0;
			cloud->numPoints++;
			cloud->data[j] = p;
			added = p;
			break;
		} else if (h->x == x && h->y == y){
			return h;
		} else {
			j += step;
			j %= cloud->maxSize;
		}
	}
	if (added && cloud->maxSize < cloud->numPoints * 4)
		HPCD_resize(cloud);
	return added;
}

bool loadPLY(const char* filename) {
	currentPLY = new char[strlen(filename) + 1];
	strcpy(currentPLY,filename);
	modelVertices.clear();
	modelFaces.clear();
	FILE* f = fopen(currentPLY, "r");
	if (!f) {
		printf("Error: File not found: %s %d\n", currentPLY,strlen(currentPLY));
		return false;
	}
	char buf[256];
	int numVertex, numFace;
	while (fgets(buf, 256, f)) {
		if (sscanf(buf, "element vertex %d", &numVertex) == 1) {
		}
		else if (sscanf(buf, "element face %d", &numFace) == 1) {
		}
		else if (strncmp(buf, "end_header", 10) == 0) {
			for (int i = 0; i<numVertex; i++) {
				fgets(buf, 256, f);
				Point p = {0,0,0,255,255,255};
				if (sscanf(buf, "%f %f %f", &(p.x), &(p.y), &(p.z)) == 3) {
					modelVertices.push_back(p);
				}
				else {
					printf("Error parsing %s\n", filename);
					printf("Line %d: %s\n", i, buf);
					break;
				}
			}
			for (int i = 0; i<numFace; i++) {
				fgets(buf, 256, f);
				Triangle t;
				if (sscanf(buf, "3 %lu %lu %lu", &(t.id1), &(t.id2), &(t.id3)) == 3) {
					modelFaces.push_back(t);
				}
				else {
					printf("Error parsing %s\n", filename);
					printf("Line %d: %s\n", i, buf);
					break;
				}
			}
			break;
		}
	}
	fclose(f);
	printf("Loaded %lu vertices %lu triangles from %s\n", modelVertices.size(), modelFaces.size(), filename);
	float centerX = 0;
	float centerY = 0;
	float centerZ = 0;
	float minZ = modelVertices[0].z;
	float maxZ = modelVertices[0].z;
	for (size_t i = 0; i < modelVertices.size(); i++) {
		centerX += modelVertices[i].x;
		centerY += modelVertices[i].y;
		centerZ += modelVertices[i].z;
		if (modelVertices[i].z < minZ) minZ = modelVertices[i].z;
		if (modelVertices[i].z > maxZ) maxZ = modelVertices[i].z;
	}
	centerX /= modelVertices.size();
	centerY /= modelVertices.size();
	centerZ /= modelVertices.size();
	for (size_t i = 0; i < modelVertices.size(); i++) {
		modelVertices[i].x -= centerX;
		modelVertices[i].y -= centerY;
		modelVertices[i].z -= minZ;
	}
	return true;
}

HPCD* HPCD_Init(const char* inFile, float resolution) {
	HPCD* res = new HPCD;
	FILE* f = fopen(inFile, "rb");
	if (!f) {
		printf("%s not found\n", inFile);
		return NULL;
	}
	boxes.clear();
	testData.clear();
	char buf[256];
	std::vector<float> float_data;
	std::vector<unsigned char> color_data;
	std::vector<int> label_data;
	PCD_data_storage data_storage = NONE;
	int totalPoints = 0;
	while (fgets(buf, 256, f)) {
		if (sscanf(buf, "POINTS %d", &totalPoints) == 1) {
		}
		else if (strncmp(buf, "FIELDS x y z rgb label", 22) == 0) {
			res->hasLabels = true;
			res->hasColor = true;
		}
		else if (strncmp(buf, "FIELDS x y z rgb", 16) == 0) {
			res->hasLabels = false;
			res->hasColor = true;
		}
		else if (strncmp(buf, "FIELDS x y z", 12) == 0) {
			res->hasLabels = false;
			res->hasColor = false;
		}
		else if (strncmp(buf, "DATA ascii", 10) == 0) {
			data_storage = ASCII;
			break;
		}
	}
	if (data_storage != ASCII)
		return NULL;

	long int pos = ftell(f);
	fseek(f, 0, SEEK_END);
	long int end = ftell(f);
	char* c = new char[end - pos];
	fseek(f, pos, SEEK_SET);
	fread(c, 1, end - pos, f);

	float x, y, z;
	unsigned char r, g, b;
	x = strtod(c, &c);
	y = strtod(c, &c);
	z = strtod(c, &c);
	if (res->hasColor) {
		strtol(c, &c, 10);
		if (res->hasLabels)
			strtol(c, &c, 10);
	}
	res->minX = res->maxX = x;
	res->minY = res->maxY = y;
	res->minZ = res->maxZ = z;
	
	//printf("start parsing %s\n", inFile);
	if (res->hasColor) {
		for (int i = 1; i<totalPoints; i++) {
			int rgb = 0;
			x = strtod(c, &c);
			y = strtod(c, &c);
			z = strtod(c, &c);
			rgb = strtol(c, &c, 10);
			r = (rgb >> 16) & 0xFF;
			g = (rgb >> 8) & 0xFF;
			b = rgb & 0xFF;
			if (x < res->minX) res->minX = x;
			else if (x > res->maxX) res->maxX = x;
			if (y < res->minY) res->minY = y;
			else if (y > res->maxY) res->maxY = y;
			if (z < res->minZ) res->minZ = z;
			else if (z > res->maxZ) res->maxZ = z;
			float_data.push_back(x);
			float_data.push_back(y);
			float_data.push_back(z);
			color_data.push_back(r);
			color_data.push_back(g);
			color_data.push_back(b);
			if (res->hasLabels) {
				int l = strtol(c, &c, 10);
				label_data.push_back(l);
			}
		}
	}
	else {
		for (int i = 1; i<totalPoints; i++) {
			x = strtod(c, &c);
			y = strtod(c, &c);
			z = strtod(c, &c);
			if (x < res->minX) res->minX = x;
			else if (x > res->maxX) res->maxX = x;
			if (y < res->minY) res->minY = y;
			else if (y > res->maxY) res->maxY = y;
			if (z < res->minZ) res->minZ = z;
			else if (z > res->maxZ) res->maxZ = z;
			float_data.push_back(x);
			float_data.push_back(y);
			float_data.push_back(z);
		}
	}
	fclose(f);
	//printf("end parsing %s\n", inFile);
	totalPoints = float_data.size() / 3;
	float minDist = res->maxX - res->minX;
	if (res->maxY - res->minY < minDist)
		minDist = res->maxY - res->minY;
	if (res->maxZ - res->minZ < minDist)
		minDist = res->maxZ - res->minZ;
	res->leafSize = resolution;
	res->numGrid = minDist / res->leafSize;
	//res->leafSize = minDist / res->numGrid;
	res->maxSize = 8;
	while (res->maxSize < 4 * totalPoints)
		res->maxSize *= 2;
	res->data = new HPoint*[res->maxSize]();
	res->deepCopy = true;
	int i, j = 0, k, l = 0;
	res->numPoints = 0;
	if (res->hasColor){
		for (i = 0; i<totalPoints; i++) {
			float x = float_data[j++];
			float y = float_data[j++];
			float z = float_data[j++];
			unsigned char r = color_data[l++];
			unsigned char g = color_data[l++];
			unsigned char b = color_data[l++];
			int xi = (int)((x - res->minX) / res->leafSize);
			int yi = (int)((y - res->minY) / res->leafSize);
			int zi = (int)((z - res->minZ) / res->leafSize);
			int ikey = getIntKey(xi, yi, zi);
			int key = baseHash(res->maxSize, ikey);
			int step = stepHash(res->maxSize, ikey);
			for (k = 0; k < res->maxSize; k++) {
				HPoint* h = res->data[key];
				if (!h) {
					HPoint* p = new HPoint;
					p->x = xi;
					p->y = yi;
					p->z = zi;
					p->r = r;
					p->g = g;
					p->b = b;
					if (res->hasLabels)
						p->label = label_data[i];
					res->data[key] = p;
					res->numPoints++;
					break;
				}
				else if (h->x == xi && h->y == yi && h->z == zi){
					break;
				}
				else {
					key += step;
					key %= res->maxSize;
				}
			}
		}
	}
	else {
		for (i = 0; i<totalPoints; i++) {
			float x = float_data[j++];
			float y = float_data[j++];
			float z = float_data[j++];
			int xi = (int)((x - res->minX) / res->leafSize);
			int yi = (int)((y - res->minY) / res->leafSize);
			int zi = (int)((z - res->minZ) / res->leafSize);
			int ikey = getIntKey(xi, yi, zi);
			int key = baseHash(res->maxSize, ikey);
			int step = stepHash(res->maxSize, ikey);
			for (k = 0; k < res->maxSize; k++) {
				HPoint* h = res->data[key];
				if (!h) {
					HPoint* p = new HPoint;
					p->x = xi;
					p->y = yi;
					p->z = zi;
					p->r = 255;
					p->g = 255;
					p->b = 255;
					res->data[key] = p;
					res->numPoints++;
					break;
				}
				else if (h->x == xi && h->y == yi && h->z == zi){
					break;
				}
				else {
					key += step;
					key %= res->maxSize;
				}
			}
		}
	}
	printf("Processed point cloud (numPoints:%d maxSize:%d leafSize:%f)\n", res->numPoints, res->maxSize, res->leafSize);
	printf("Bounding box: x:(%.2f %.2f) y:(%.2f %.2f) z:(%.2f %.2f)\n", res->minX, res->maxX, res->minY, res->maxY, res->minZ, res->maxZ);
	return res;
}

HPCD* HPCD_copy(HPCD* cloud, bool deep) {
	HPCD* h = new HPCD;
	memcpy(h, cloud, sizeof(HPCD));
	h->data = new HPoint*[h->maxSize];
	if (deep) {
		h->deepCopy = true;
		for (int i = 0; i < cloud->maxSize; i++) {
			HPoint* p = cloud->data[i];
			if (p) {
				h->data[i] = new HPoint;
				*(h->data[i]) = *p;
			}
			else {
				h->data[i] = NULL;
			}
		}
	}
	else {
		h->deepCopy = false;
		memcpy(h->data, cloud->data, h->maxSize * sizeof(HPoint*));
	}
	return h;
}

void HPCD_resize(HPCD* res) { //to save memory
	int newSize = 8;
	while (newSize < 4 * res->numPoints)
		newSize *= 2;
	HPoint** newdata = new HPoint*[newSize]();
	int i, k;
	for (i = 0; i<res->maxSize; i++) {
		HPoint* h = res->data[i];
		if (!h)
			continue;
		int ikey = getIntKey(h->x, h->y, h->z);
		int key = baseHash(newSize, ikey);
		int step = stepHash(newSize, ikey);
		for (k = 0; k < newSize; k++) {
			if (!newdata[key]) {
				newdata[key] = h;
				break;
			}
			else {
				key += step;
				key %= newSize;
			}
		}
	}
	delete[] res->data;
	res->data = newdata;
	res->maxSize = newSize;
//	printf("Processed point cloud (numPoints:%d maxSize:%d leafSize:%f)\n", res->numPoints, res->maxSize, res->leafSize);
}

void HPCD_delete(HPCD* cloud) {
	if (cloud->deepCopy) {
		for (int i = 0; i < cloud->maxSize; i++)
			if (cloud->data[i])
				delete cloud->data[i];
	}
	delete[] cloud->data;
	delete cloud;
}

void HPCD_write(const char* filename, HPCD* pointcloud) {
	FILE* f = fopen(filename, "w");
	if (!f) {
		printf("Cannot write to file: %s\n", filename);
		return;
	}
	if (pointcloud->hasLabels) {
		fprintf(f, "# .PCD v0.7 - Point Cloud Data file format\n"
			"VERSION 0.7\n"
			"FIELDS x y z rgb label\n"
			"SIZE 4 4 4 4 4\n"
			"TYPE F F F I I\n"
			"COUNT 1 1 1 1 1\n"
			"WIDTH %d\n"
			"HEIGHT 1\n"
			"VIEWPOINT 0 0 0 1 0 0 0\n"
			"POINTS %d\n"
			"DATA ascii\n", pointcloud->numPoints, pointcloud->numPoints);
		for (int i = 0; i<pointcloud->maxSize; i++) {
			HPoint *h = pointcloud->data[i];
			if (h) {
				int rgb = (h->r << 16) | (h->g << 8) | h->b;
				fprintf(f, "%f %f %f %d %d\n",
					pointcloud->_minX + h->x * pointcloud->leafSize,
					pointcloud->_minY + h->y * pointcloud->leafSize,
					pointcloud->_minZ + h->z * pointcloud->leafSize,
					rgb,h->label);
			}
		}
	} 
	else if (pointcloud->hasColor) {
		fprintf(f, "# .PCD v0.7 - Point Cloud Data file format\n"
			"VERSION 0.7\n"
			"FIELDS x y z rgb\n"
			"SIZE 4 4 4 4\n"
			"TYPE F F F I\n"
			"COUNT 1 1 1 1\n"
			"WIDTH %d\n"
			"HEIGHT 1\n"
			"VIEWPOINT 0 0 0 1 0 0 0\n"
			"POINTS %d\n"
			"DATA ascii\n", pointcloud->numPoints, pointcloud->numPoints);
		for (int i = 0; i<pointcloud->maxSize; i++) {
			HPoint *h = pointcloud->data[i];
			if (h) {
				int rgb = (h->r << 16) | (h->g << 8) | h->b;
				fprintf(f, "%f %f %f %d\n",
					pointcloud->_minX + h->x * pointcloud->leafSize,
					pointcloud->_minY + h->y * pointcloud->leafSize,
					pointcloud->_minZ + h->z * pointcloud->leafSize,
					rgb);
			}
		}
	}
	else {
		fprintf(f, "# .PCD v0.7 - Point Cloud Data file format\n"
			"VERSION 0.7\n"
			"FIELDS x y z\n"
			"SIZE 4 4 4\n"
			"TYPE F F F\n"
			"COUNT 1 1 1\n"
			"WIDTH %d\n"
			"HEIGHT 1\n"
			"VIEWPOINT 0 0 0 1 0 0 0\n"
			"POINTS %d\n"
			"DATA ascii\n", pointcloud->numPoints, pointcloud->numPoints);
		for (int i = 0; i<pointcloud->maxSize; i++) {
			HPoint *h = pointcloud->data[i];
			if (h) {
				fprintf(f, "%f %f %f\n",
					pointcloud->_minX + h->x * pointcloud->leafSize,
					pointcloud->_minY + h->y * pointcloud->leafSize,
					pointcloud->_minZ + h->z * pointcloud->leafSize);
			}
		}
	}
	fclose(f);
	printf("Wrote %d points to %s\n", pointcloud->numPoints, filename);
}

void HPCD_center(HPCD* cloud) {
	float cx = (cloud->minX + cloud->maxX) / 2;
	float cy = (cloud->minY + cloud->maxY) / 2;
	float cz = cloud->minZ;
	cloud->minX -= cx;
	cloud->maxX -= cx;
	cloud->minY -= cy;
	cloud->maxY -= cy;
	cloud->minZ -= cz;
	cloud->maxZ -= cz;
	cloud->_minX = cloud->minX;
	cloud->_minY = cloud->minY;
	cloud->_minZ = cloud->minZ;
}

int HPCD_filter(HPCD* cloud, float minX, float  minY, float  minZ, float  maxX, float  maxY, float  maxZ) {
	int previousNumPoints = cloud->numPoints;
	for (int j = 0; j<cloud->maxSize; j++) {
		HPoint* h = cloud->data[j];
		if (h) {
			float x = h->x * cloud->leafSize + cloud->minX;
			float y = h->y * cloud->leafSize + cloud->minY;
			float z = h->z * cloud->leafSize + cloud->minZ;
			if (x<minX || x>maxX || y<minY || y>maxY || z<minZ || z>maxZ) {
				cloud->data[j] = NULL;
				cloud->numPoints--;
			}
			/*else {
				h->x = (int)((x - minX) / cloud->leafSize);
				h->y = (int)((y - minY) / cloud->leafSize);
				h->z = (int)((z - minZ) / cloud->leafSize);
			}*/
		}
	}
	cloud->minX = minX;
	cloud->maxX = maxX;
	cloud->minY = minY;
	cloud->maxY = maxY;
	cloud->minZ = minZ;
	cloud->maxZ = maxZ;
	HPCD_resize(cloud);
	return previousNumPoints - cloud->numPoints;
}

int segmentPlane(HPCD* cloud, int iter, float segThreshold, float inlierRatio) {
	int maxInliers = 0;
	int optimumInliers = (int) (inlierRatio * cloud->numPoints);
	int distanceThreshold;
	int previousNumPoints = cloud->numPoints;
	Plane bestPlane = { 0, 0, 1, 0 };
	int i, k = 0;
	int* pointdata = new int[cloud->numPoints * 3];
	for (int j = 0; j<cloud->maxSize; j++) {
		if (cloud->data[j]) {
			pointdata[k++] = cloud->data[j]->x;
			pointdata[k++] = cloud->data[j]->y;
			pointdata[k++] = cloud->data[j]->z;
		}
	}
	for (i = 0; i<iter; i++) {
		//Pick 3 points
		int *p0 = pointdata + (rand() % cloud->numPoints * 3);
		int *p1 = pointdata + (rand() % cloud->numPoints * 3);
		int *p2 = pointdata + (rand() % cloud->numPoints * 3);
		int p0x = p0[0], p0y = p0[1], p0z = p0[2];
		int p1x = p1[0], p1y = p1[1], p1z = p1[2];
		int p2x = p2[0], p2y = p2[1], p2z = p2[2];
		Plane currentPlane = {
			(p1y - p0y)*(p2z - p0z) - (p2y - p0y)*(p1z - p0z),
			(p1z - p0z)*(p2x - p0x) - (p2z - p0z)*(p1x - p0x),
			(p1x - p0x)*(p2y - p0y) - (p2x - p0x)*(p1y - p0y),
			0
		};
		currentPlane.d = -(currentPlane.a * p0x + currentPlane.b * p0y + currentPlane.c * p0z);
		if (currentPlane.a == 0 && currentPlane.b == 0 && currentPlane.c == 0)
			continue; //picked collinear points
		distanceThreshold = sqrt((double)(
			currentPlane.a * currentPlane.a +
			currentPlane.b * currentPlane.b +
			currentPlane.c * currentPlane.c)) * segThreshold / cloud->leafSize;
		int numInliers = 0;
		for (int j = 0; j<cloud->numPoints; j++) {
			if (abs(currentPlane.a * pointdata[j * 3] +
				currentPlane.b * pointdata[j * 3 + 1] +
				currentPlane.c * pointdata[j * 3 + 2] +
				currentPlane.d)
				< distanceThreshold)
				numInliers++;
		}
		if (numInliers > maxInliers) {
			maxInliers = numInliers;
			bestPlane = currentPlane;
			if (maxInliers > optimumInliers)
				break;
		}
	}
	distanceThreshold = sqrt((double)(
		bestPlane.a * bestPlane.a +
		bestPlane.b * bestPlane.b +
		bestPlane.c * bestPlane.c)) * segThreshold / cloud->leafSize;
	double RMSE = 0;
	for (int j = 0; j<cloud->maxSize; j++) {
		HPoint* h = cloud->data[j];
		if (h && abs(bestPlane.a * h->x +
			bestPlane.b * h->y +
			bestPlane.c * h->z +
			bestPlane.d)
			< distanceThreshold) {
			double d = 1.0 * (bestPlane.a * h->x + bestPlane.b * h->y + bestPlane.c * h->z + bestPlane.d) / distanceThreshold;
			RMSE += d * d;
			//delete cloud->data[j];
			cloud->data[j] = NULL;
			cloud->numPoints--;
		}
	}
	printf("RANSAC: %d points %d iters (%d,%d,%d,%d) (RMSE %f)\n", cloud->numPoints, i, bestPlane.a, bestPlane.b, bestPlane.c, bestPlane.d, sqrt(RMSE / maxInliers));
	delete[] pointdata;
	HPCD_resize(cloud);
	return previousNumPoints - cloud->numPoints;
}

void segmentLowest(HPCD* cloud) {
	bool initialized = false;
	int minX,maxX,minY,maxY;
	for (int i=0;i<cloud->maxSize;i++) {
		HPoint* h = cloud->data[i];
		if (h) {
			if (!initialized || h->x < minX) minX = h->x;
			if (!initialized || h->x > maxX) maxX = h->x;
			if (!initialized || h->y < minY) minY = h->y;
			if (!initialized || h->y > maxY) maxY = h->y;
			initialized = true;
		}
	}
	int xrange = maxX - minX + 1;
	int yrange = maxY - minY + 1;
	int* lowest = new int[xrange * yrange];
	for (int i=0;i<xrange*yrange;i++)
		lowest[i] = -1;
	for (int i=0;i<cloud->maxSize;i++) {
		HPoint* h = cloud->data[i];
		if (h) {
			h->label = 0;
			int id = (h->x-minX) * yrange + (h->y-minY);
			if (lowest[id]==-1 || cloud->data[lowest[id]]->z > h->z)
				lowest[id] = i;
		}
	}
	for (int i=0;i<xrange*yrange;i++) {
		if (lowest[i]!=-1) {
			cloud->data[lowest[i]]->label = 1;
		}
	}
	for (int i=0;i<cloud->maxSize;i++) {
		HPoint* h = cloud->data[i];
		if (h && h->label) {
			cloud->data[i] = NULL;
			cloud->numPoints--;
		}
	}
	delete[] lowest;
}

int euclideanClustering(HPCD* cloud, int clusterThreshold, int totalClusters, int minsize) {
	testData.clear();
	boxes.clear();
	bool* visited = new bool[cloud->maxSize]();
	std::vector<Cluster> clusters;
	int numClusters = 0;
	int numCombinations = 1;
	for (int i = 0; i < clusterThreshold; i++)
		numCombinations *= 6;
	for (int i = 0; i<cloud->maxSize; i++) {
		HPoint* seed = cloud->data[i];
		if (!seed || visited[i]) continue;
		Cluster c = { numClusters, 0 };
		std::vector<int> Q;
		Q.push_back(i);
		visited[i] = true;
		while (Q.size() > 0) {
			int p = Q[Q.size() - 1];
			Q.pop_back();
			HPoint* h = cloud->data[p];
			h->label = numClusters;
			c.size++;
			
			for (int k = 0; k < numCombinations; k++) {
				int l = k;
				int xi = h->x;
				int yi = h->y;
				int zi = h->z;
				for (int m = 0; m < clusterThreshold;m++) {
					switch (l % 6) {
					case 0: if (xi >= h->x) xi += 1; break;
					case 1: if (xi <= h->x) xi -= 1; break;
					case 2: if (yi >= h->y) yi += 1; break;
					case 3: if (yi <= h->y) yi -= 1; break;
					case 4: if (zi >= h->z) zi += 1; break;
					case 5: if (zi <= h->z) zi -= 1; break;
					}
					int j = HPCD_find(cloud, xi, yi, zi);
					if (j >= 0 && !visited[j]) {
						Q.push_back(j);
						visited[j] = true;
					}
					l /= 6;
				}
			}	
		}
		//		printf("Cluster %d: %d\n",numClusters,c.size);
		clusters.push_back(c);
		numClusters++;
	}
	Cluster *sortedClusters = new Cluster[numClusters];
	memcpy(sortedClusters, clusters.data(), numClusters*sizeof(Cluster));
	qsort(sortedClusters, numClusters, sizeof(Cluster), compareCluster);
	int *idx = new int[numClusters];
	for (int i = 0; i<numClusters; i++)
		idx[i] = -1;
	int k;
	for (k = 0; k<totalClusters && k<numClusters && sortedClusters[k].size > minsize; k++)
		idx[sortedClusters[k].index] = k;
	for (int i = 0; i<cloud->maxSize; i++) {
		HPoint* h = cloud->data[i];
		if (h)
			h->label = idx[h->label];
	}
	cloud->hasLabels = true;
	delete[] idx;
	delete[] visited;
	delete[] sortedClusters;
	return k;
}

int euclideanClusteringXY(HPCD* cloud, int clusterThreshold, int totalClusters,int minsize) {
	testData.clear();
	boxes.clear();
	std::vector<Cluster> clusters;
	int numClusters = 0;
	int xrange = (cloud->maxX - cloud->minX) / cloud->leafSize + 1;
	int yrange = (cloud->maxY - cloud->minY) / cloud->leafSize + 1;
	bool* valid = new bool[xrange * yrange]();
	int* grid = new int[xrange * yrange];
	for (int i = 0; i<xrange*yrange; i++)
		grid[i] = -1;
	for (int i = 0; i < cloud->maxSize; i++) {
		HPoint* h = cloud->data[i];
		if (h)
			valid[h->x*yrange + h->y] = true;
	}
	Coordinate p;
	for (p.x = 0; p.x < xrange; p.x++) {
		for (p.y = 0; p.y < yrange; p.y++) {
			int id = p.x * yrange + p.y;
			if (!valid[id] || grid[id]>=0)
				continue;
			std::vector<Coordinate> Q;
			Q.push_back(p);
			Cluster c = { numClusters, 0 };
			while (Q.size() > 0) {
				Coordinate q = Q[Q.size() - 1];
				Q.pop_back();
				int id = q.x * yrange + q.y;
				if (!valid[id] || grid[id]>=0)
					continue;
				c.size++;
				grid[id] = numClusters;
				for (int xi = q.x - clusterThreshold; xi <= q.x + clusterThreshold; xi++) {
					for (int yi = q.y - clusterThreshold; yi <= q.y + clusterThreshold; yi++) {
						if (xi >= 0 && yi >= 0 && xi < xrange && yi < yrange) {
							Coordinate r = { xi, yi };
							Q.push_back(r);
						}
					}
				}
			}
			clusters.push_back(c);
			numClusters++;
		}
	}
	for (int i = 0; i<cloud->maxSize; i++) {
		HPoint* h = cloud->data[i];
		if (h)
			h->label = grid[h->x * yrange + h->y];
	}
	Cluster *sortedClusters = new Cluster[numClusters];
	memcpy(sortedClusters, clusters.data(), numClusters*sizeof(Cluster));
	qsort(sortedClusters, numClusters, sizeof(Cluster), compareCluster);
	int *idx = new int[numClusters];
	for (int i = 0; i<numClusters; i++)
		idx[i] = -1;
	int k;
	for (k = 0; k<totalClusters && k<numClusters && sortedClusters[k].size > minsize; k++)
		idx[sortedClusters[k].index] = k;
	for (int i = 0; i<cloud->maxSize; i++) {
		HPoint* h = cloud->data[i];
		if (h)
			h->label = idx[h->label];
	}
	cloud->hasLabels = true;
	delete[] valid;
	delete[] idx;
	delete[] sortedClusters;
	return k;
}

int mergeClusters(HPCD* cloud, int numClusters, int maxClusters) {
	std::vector<float> minX(numClusters, cloud->maxX);
	std::vector<float> maxX(numClusters, cloud->minX);
	std::vector<float> minY(numClusters, cloud->maxY);
	std::vector<float> maxY(numClusters, cloud->minY);
	for (int i = 0; i < cloud->maxSize; i++) {
		HPoint* h = cloud->data[i];
		if (h && h->label >= 0) {
			float x = cloud->_minX + h->x * cloud->leafSize;
			float y = cloud->_minY + h->y * cloud->leafSize;
			if (x < minX[h->label]) minX[h->label] = x;
			if (x > maxX[h->label]) maxX[h->label] = x;
			if (y < minY[h->label]) minY[h->label] = y;
			if (y > maxY[h->label]) maxY[h->label] = y;
		}
	}

	int offset = 0;
	int remaining = numClusters;
	std::vector<int> idx(numClusters+1, -1);
	for (int k = 0; k < numClusters; k++) {
		bool reassigned = false;
		for (int j = 0; j < k; j++) {
			if (minX[k] > minX[j] && maxX[k] < maxX[j] && minY[k] > minY[j] && maxY[k] < maxY[j]) {
				idx[k] = idx[j];
				offset++;
				remaining--;
				printf("reassigned %d %d\n", k, j);
				reassigned = true;
				break;
			}
		}
		if (!reassigned && k - offset < maxClusters)
			idx[k] = k - offset;
	}

	for (int i = 0; i<cloud->maxSize; i++) {
		HPoint* h = cloud->data[i];
		if (h && h->label >= 0)
			h->label = idx[h->label];
	}

	return remaining;
}

void showHPCD(HPCD* h) {
	cloud.clear();
	int xrange = (h->maxX - h->minX) / h->leafSize + 1;
	int yrange = (h->maxY - h->minY) / h->leafSize + 1;
	for (int i = 0; i < h->maxSize; i++) {
		HPoint* p = h->data[i];
		if (p) {
			/*Point q = { 
				h->minX + p->x * h->leafSize,
				h->minY + p->y * h->leafSize,
				h->minZ + p->z * h->leafSize,
				p->r, p->g, p->b };*/
			Point q = { (float)(p->x-xrange/2), (float)(p->y-yrange/2), (float)p->z, p->r, p->g, p->b };
			cloud.push_back(q);
		}
	}
}

Feature getPCA(std::vector<Point> *cloud, std::vector<float> *box) {
	double cov[4] = {}; //column major
	Point center = {};
	for (int i = 0; i < cloud->size(); i++) {
		center.x += cloud->at(i).x;
		center.y += cloud->at(i).y;
		center.z += cloud->at(i).z;
	}
	center.x /= cloud->size(); 
	center.y /= cloud->size();
	center.z /= cloud->size();
	for (int j = 0; j<cloud->size(); j++) {
		float deltaP[2] = {
			cloud->at(j).x- center.x,
			cloud->at(j).y- center.y,
		};
		cov[0] += deltaP[0] * deltaP[0];
		cov[1] += deltaP[1] * deltaP[0];
		cov[2] += deltaP[0] * deltaP[1];
		cov[3] += deltaP[1] * deltaP[1];
	}
	cov[0] /= cloud->size() * cloud->size();
	cov[1] /= cloud->size() * cloud->size();
	cov[2] /= cloud->size() * cloud->size();
	cov[3] /= cloud->size() * cloud->size();
	float trace = cov[0] + cov[3];
	float det = cov[0] * cov[3] - cov[1] * cov[2];
	float L1 = trace / 2 + sqrt(trace*trace / 4 - det);
	float L2 = trace / 2 - sqrt(trace*trace / 4 - det);
	float minScale[3], maxScale[3];
	float v[9] = {
		0,0,0,
		0,0,0,
		0,0,1
	};
	if (cov[2] != 0) {
		v[0] = L1 - cov[3];
		v[1] = L2 - cov[3];
		v[3] = v[4] = cov[2];
	}
	else if (cov[1] != 0) {
		v[0] = v[1] = cov[1];
		v[3] = L1 - cov[0];
		v[4] = L2 - cov[0];
	}
	else {
		v[0] = v[4] = 1;
	}
	float m1 = sqrt(v[0] * v[0] + v[3] * v[3]);
	float m2 = sqrt(v[1] * v[1] + v[4] * v[4]);
	v[0] /= m1;
	v[3] /= m1;
	v[1] /= m2;
	v[4] /= m2;
	for (int j = 0; j<cloud->size(); j++) {
		for (int i = 0; i<3; i++) {
			float dotProduct =
				cloud->at(j).x * v[i * 3] +
				cloud->at(j).y * v[i * 3 + 1] +
				cloud->at(j).z * v[i * 3 + 2];
			if (j == 0 || dotProduct < minScale[i])
				minScale[i] = dotProduct;
			if (j == 0 || dotProduct > maxScale[i])
				maxScale[i] = dotProduct;
		}
	}
	float bbCenter[3] = {0,0,0};
	for (int i = 0; i<3; i++) {
		bbCenter[0] += (minScale[i] + maxScale[i]) / 2 * v[i * 3];
		bbCenter[1] += (minScale[i] + maxScale[i]) / 2 * v[i * 3 + 1];
		bbCenter[2] += (minScale[i] + maxScale[i]) / 2 * v[i * 3 + 2];
	}
//	for (int i = 0; i<8; i++) {
//		float coords[3];
//		for (int j = 0; j<3; j++) {
//			coords[j] = bbCenter[j];
//			for (int axis = 0; axis<3; axis++) {
//				float sign = (i & 1 << axis) ? 1 : -1;
//				coords[j] += sign * (maxScale[axis]-minScale[axis]) / 2 * v[axis * 3 + j];
//			}
//			box->push_back(coords[j]);
//		}
//	}
	box->push_back(bbCenter[0]);
	box->push_back(bbCenter[1]);
	box->push_back(bbCenter[2]);
	box->push_back(maxScale[0] - minScale[0]);
	box->push_back(maxScale[1] - minScale[1]);
	box->push_back(maxScale[2] - minScale[2]);
	float theta = atan2(v[1], v[0]);
	theta /= 2;
	box->push_back(cos(theta));
	box->push_back(0);
	box->push_back(0);
	box->push_back(sin(theta));
	//top-to-bottom ratio
	int countTop = 0;
	int countBottom = 0;
	for (size_t i = 0; i < cloud->size(); i++) {
		Point p = cloud->at(i);
		if (p.z > center.z)
			countTop++;
		else
			countBottom++;
	}

	Feature f;
	f.features[0] = (maxScale[0] - minScale[0]) / (maxScale[1] - minScale[1]);
	f.features[1] = (maxScale[2] - minScale[2]) / (maxScale[1] - minScale[1]);
	f.features[2] = 1.0 * countTop / countBottom;
	return f;
}

void getBoundingBox(HPCD* h) {
	testData.clear();
	boxes.clear();
	descriptions.clear();
	int xrange = (h->maxX - h->minX) / h->leafSize + 1;
	int yrange = (h->maxY - h->minY) / h->leafSize + 1;
	std::vector< std::vector<Point> > points;
	int numClusters = 0;
	for (int i = 0; i < h->maxSize; i++) {
		HPoint* p = h->data[i];
		if (p && p->label >= numClusters)
			numClusters = p->label + 1;
	}
	boxes.resize(numClusters);
	points.resize(numClusters);
	for (int i = 0; i < h->maxSize; i++) {
		HPoint* p = h->data[i];
		if (p && p->label >= 0) {
			Point q = { (float)(p->x * h->leafSize), (float)(p->y * h->leafSize), (float)(p->z * h->leafSize), 0, 0, 0 };
			points[p->label].push_back(q);
		}
	}
	for (int i = 0; i < numClusters; i++) {
		Feature f = getPCA(&points[i], &boxes[i]);
//		printf("test feature %d: %f %f\n", i,f.features[0], f.features[1]);
		testData.push_back(f);
	}
}

unsigned char* render_text(int id, int width, int height, char **text) {
	if (id < 0) {
		for (size_t i = 0; i<raster.size(); i++)
			delete[] raster[i];
		raster.clear();
		return NULL;
	}
	if (id >= labelCategories.size())
		return NULL;
	*text = labelCategories[id];
	labelWidth = width;
	labelHeight = height;
	unsigned char* c = new unsigned char[width * height * 3];
	raster.push_back(c);
	return c;
}

void renderCAD() {
	sceneVertices.clear();
	sceneFaces.clear();
	for (size_t i = 0; i < testData.size(); i++) {
		if (loadPLY(testPaths[i])) {
			float cx = 0;
			float cy = 0;
			float cz = 0;
			for (size_t j = 0; j < 8; j++) {
				cx += boxes[i][j * 3];
				cy += boxes[i][j * 3 + 1];
				cz += boxes[i][j * 3 + 2];
			}
			cx /= 8;
			cy /= 8;
			cz /= 8;
			float xl = sqrt((boxes[i][3] - boxes[i][0])*(boxes[i][3] - boxes[i][0]) + (boxes[i][4] - boxes[i][1])*(boxes[i][4] - boxes[i][1]));
			float yl = sqrt((boxes[i][6] - boxes[i][0])*(boxes[i][6] - boxes[i][0]) + (boxes[i][7] - boxes[i][1])*(boxes[i][7] - boxes[i][1]));
			float zl = boxes[i][14] - boxes[i][2];
			//printf("%f %f %f %f %f %f\n", cx, cy, cz, xl, yl, zl);
			float minZ = modelVertices[0].z;
			float maxZ = modelVertices[1].z;
			for (size_t j = 0; j < modelVertices.size(); j++) {
				if (modelVertices[j].z < minZ)
					minZ = modelVertices[j].z;
				if (modelVertices[j].z > maxZ)
					maxZ = modelVertices[j].z;
			}
			float scale = zl / (maxZ - minZ);
			for (size_t j = 0; j < modelFaces.size(); j++) {
				Triangle t = modelFaces[j];
				t.id1 += sceneVertices.size();
				t.id2 += sceneVertices.size();
				t.id3 += sceneVertices.size();
				sceneFaces.push_back(t);
			}
			for (size_t j = 0; j < modelVertices.size(); j++) {
				Point p = modelVertices[j];
				p.x = p.x * scale + cx;
				p.y = p.y * scale + cy;
				p.z = p.z * scale + boxes[i][2];
				sceneVertices.push_back(p);
			}
		}
	}
}

int calculateFeatures(const char* classname) {
	int label = -1;
	for (size_t i = 0; i < labelCategories.size(); i++) {
		if (strcmp(labelCategories[i], classname) == 0) {
			label = i;
			break;
		}
	}
	if (label < 0) {
		char* c = new char[strlen(classname)+1];
		strcpy(c, classname);
		labelCategories.push_back(c);
		label = labelCategories.size() - 1;
	}
	std::vector<float> box;
	Feature f = getPCA(&cloud, &box);
	f.label = label;
	f.origin = trainingData.size();
	//printf("features: %s %f %f\n", labelCategories[f.label], f.features[0], f.features[1]);
	trainingPaths.push_back(currentPLY);
	trainingData.push_back(f);
	return trainingData.size();
}

int compareFeatures(const void* v1, const void* v2) {
	Feature* f1 = (Feature*)v1;
	Feature* f2 = (Feature*)v2;
	if (f1->error < f2->error)
		return -1;
	if (f1->error > f2->error)
		return 1;
	return 0;
}

int classify() {
	printf("training: %lu testing: %lu\n", trainingData.size(), testData.size());
	if (trainingData.size() == 0 || testData.size() == 0)
		return 0;
	descriptions.clear();
	testPaths.clear();
	Feature *trainFeatures = new Feature[trainingData.size()];
	int *counts = new int[labelCategories.size()];
	memcpy(trainFeatures, trainingData.data(), trainingData.size() * sizeof(Feature));
	for (size_t i = 0; i < testData.size(); i++) {
		Feature* f1 = &testData[i];
		for (size_t j = 0; j < trainingData.size(); j++) {
			Feature* f2 = trainFeatures + j;
			f2->error = 0;
			for (int k = 0; k < NUM_FEATURES;k++)
				f2->error += (f1->features[k] - f2->features[k])*(f1->features[k] - f2->features[k]);
		}
		qsort(trainFeatures, trainingData.size(), sizeof(Feature), compareFeatures);
		memset(counts, 0, labelCategories.size()*sizeof(int));
		int maxCount = -1;
		for (int k = 0; k < 3; k++) {
			int classLabel = trainFeatures[k].label;
			counts[classLabel] ++ ;
			if (maxCount < 0 || counts[classLabel] > counts[maxCount])
				maxCount = classLabel;
		}
		f1->label = maxCount;
		descriptions.push_back(labelCategories[f1->label]);
		for (int k = 0; k < 3; k++) {
			if (trainFeatures[k].label == f1->label) {
				testPaths.push_back(trainingPaths[trainFeatures[k].origin]);
				break;
			}
		}
		printf("%s %d %f\n", labelCategories[f1->label],counts[maxCount], trainFeatures[0].error);
	}
	delete[] trainFeatures;
	delete[] counts;
	return descriptions.size();
}

void resetFeatures() {
	trainingData.clear();
	labelCategories.clear();
}

int loadFeatures(const char* filename) {
	trainingData.clear();
	trainingPaths.clear();
	labelCategories.clear();
	FILE* f = fopen(filename, "r");
	char buf[256];
	char classname[128];
	while (fgets(buf, 256, f)) {
		Feature ft;
		char* c = strtok(buf, " ");
		strcpy(classname, c);
		for (int i = 0; i < NUM_FEATURES; i++) {
			c = strtok(NULL, " ");
			ft.features[i] = strtod(c, NULL);
		}
		c = strtok(NULL, " ");
		int n=0;
		while (!isspace(c[n]))
			n++;
		char* path = new char[n + 1];
		strncpy(path, c, n);
		path[n] = '\0';
		trainingPaths.push_back(path);
		ft.label = -1;
		for (size_t i = 0; i < labelCategories.size(); i++) {
			if (strcmp(labelCategories[i], classname) == 0) {
				ft.label = i;
				break;
			}
		}
		if (ft.label < 0) {
			char* c = new char[strlen(classname) + 1];
			strcpy(c, classname);
			labelCategories.push_back(c);
			ft.label = labelCategories.size() - 1;
		}
		ft.origin = trainingData.size();
		trainingData.push_back(ft);
	}
	fclose(f);
	return trainingData.size();
}

int saveFeatures(const char* filename) {
	FILE* f = fopen(filename,"w");
	for (size_t i = 0; i < trainingData.size(); i++) {
		fprintf(f, "%s", labelCategories[trainingData[i].label]);
		for (int j = 0; j < NUM_FEATURES; j++)
			fprintf(f, " %f", trainingData[i].features[j]);
		fprintf(f, " %s", trainingPaths[i]);
		fprintf(f, "\n");
	}
	fclose(f);
	return trainingData.size();
}


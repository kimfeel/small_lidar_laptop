#ifdef _WIN32
#include <Windows.h>
#endif
#include <vector>
#define NUM_FEATURES 3
#define DEBUG_DEPTH 0

struct Point {
	float x, y, z;
	unsigned char r, g, b;
};

struct HPoint {
	int x, y, z;
	unsigned char r, g, b;
	int label;
	int handle;
};

struct Coordinate {
	int x, y;
};

struct HPCD {
	int numGrid;
	int numPoints;
	int maxSize;
	float leafSize;
	float minX, minY, minZ, maxX, maxY, maxZ;
	float _minX, _minY, _minZ;
	HPoint** data;
	bool hasColor;
	bool hasLabels;
	bool deepCopy;
};

struct Box {
	float minX, minY, minZ, maxX, maxY, maxZ;
};

struct Plane {
	int a, b, c, d;
};

struct Triangle {
	size_t id1, id2, id3;
};

struct Feature {
	float features[NUM_FEATURES];
	float error;
	int label;
	int origin;
};

struct Cluster {
	int index, size;
};

enum PCD_data_storage {
	ASCII,
	BINARY,
	NONE
};

extern std::vector< std::vector<float> > boxes;
int compareCluster(const void* v1, const void* v2);

HPCD* HPCD_Init(const char* inFile, float resolution);
HPCD* HPCD_copy(HPCD* cloud,bool deep);
void HPCD_resize(HPCD* res);
void HPCD_delete(HPCD* cloud);
void HPCD_write(const char* filename, HPCD* pointcloud);
int HPCD_filter(HPCD* cloud, float minX, float  minY, float  minZ, float  maxX, float  maxY, float  maxZ);
void HPCD_center(HPCD* cloud);
int HPCD_find(HPCD* cloud, int x, int y, int z);
HPoint* HPCD_add(HPCD* cloud,int x,int y,int z);
int HPCD_find_xy(HPCD* cloud, int x, int y);
HPoint* HPCD_add_xy(HPCD* cloud,int x,int y);

void segmentLowest(HPCD* cloud);
int segmentPlane(HPCD* cloud, int iter, float segThreshold, float inlierRatio);
int euclideanClustering(HPCD* cloud, int clusterThreshold, int totalClusters, int minsize);
int euclideanClusteringXY(HPCD* cloud, int clusterThreshold, int totalClusters, int minsize);
int mergeClusters(HPCD* cloud, int numClusters, int maxClusters);
Feature getPCA(std::vector<Point> *cloud, std::vector<float> *box);

void getBoundingBox(HPCD* cloud);
void drawCloud(float cameraX, float cameraY, float cameraZ, float centerX, float centerY, float centerZ, float upX, float upY, float upZ);
void showHPCD(HPCD* h);
unsigned char* render_text(int id, int width, int height, char **text);

bool loadPLY(const char* filename);
void drawPLY(float cameraX, float cameraY, float cameraZ, float centerX, float centerY, float centerZ, float upX, float upY, float upZ);
int rayTracing(float cameraX, float cameraY, float cameraZ, int numViews, int resolution);
void renderCAD();
void drawMixed(float cameraX, float cameraY, float cameraZ, float centerX, float centerY, float centerZ, float upX, float upY, float upZ);

int calculateFeatures(const char* classname);
void resetFeatures();
int loadFeatures(const char* filename);
int saveFeatures(const char* filename);
int classify();

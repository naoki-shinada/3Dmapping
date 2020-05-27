#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <iostream>
#include <string>
#include <cstdlib> 
#include <Eigen/Dense>
#include <math.h>
#include <pcl/visualization/cloud_viewer.h> 

typedef pcl::PointXYZ PointT;
using namespace std;

int main(int argc, char** argv)
{

	double r = 0.494921;
	double threshould = 0.07;

	// All the objects needed
	pcl::PCDReader reader;

	// Datasets
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

	int j;

	reader.read("stright.pcd", *cloud);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>);
	colored->points.resize(cloud->points.size());
	for (j = 0; j < cloud->points.size(); j++) {
		colored->points[j].x = cloud->points[j].x;
		colored->points[j].y = cloud->points[j].y;
		colored->points[j].z = cloud->points[j].z;
		colored->points[j].r = 0;
		colored->points[j].g = 0;
		colored->points[j].b = 255;
	}

	vector<double> point1 = { 0.0931,8.16,-0.150 };
	vector<double> jiku = { 0.002446,0.999976,0.00207 };
	for (j = 0; j < cloud->points.size(); j++) {
		vector<double> x = { cloud->points[j].x, cloud->points[j].y,  cloud->points[j].z };
		//é≤Ç∆ÇÃãóó£ÇãÅÇﬂÇÈ
		vector<double> v = { x[0] - point1[0],x[1] - point1[1],x[2] - point1[2] };
		double vsize = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
		double d = sqrt(pow(vsize, 2) - pow((v[0] * jiku[0] + v[1] * jiku[1] + v[2] * jiku[2]), 2));
		if (d > r - threshould && r + threshould > d) {
			colored->points[j].r = 255;
			colored->points[j].b = 0;
		}
	}
	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	viewer.showCloud(colored);

	while (!viewer.wasStopped())
	{

	}
}
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


int
main(int argc, char** argv)
{
	//RANSACパラメータ
	int  maxitteration = 10000;
	double r = 0.494921;
	double threshould = 0.03;

	// All the objects needed
	pcl::PCDReader reader;
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::PCDWriter writer;
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

	// Datasets
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);


	int i, j;
	string n;
	ofstream ofs("test.csv");
	for (i = 0; i < 1; i++) {
		n = to_string(i + 1);
		// Read in the cloud data
		//reader.read(n + "cylinder2m.pcd", *cloud);
		reader.read("stright.pcd", *cloud);

		// Estimate point normals
		ne.setViewPoint(0.0, 0.0, 1000.0);
		ne.setSearchMethod(tree);
		ne.setInputCloud(cloud);
		ne.setKSearch(50);
		ne.compute(*cloud_normals);


		//RANSAC
		int maxinliner = 0;
		int itteration = 0;
		vector<double> bestP(3);
		vector<double> bestJIKU(3);

		while (itteration < maxitteration) {
			int a = rand() % cloud->points.size();
			int b = rand() % cloud->points.size();

			vector<double> p1 = { cloud->points[a].x, cloud->points[a].y,  cloud->points[a].z };
			vector<double> p2 = { cloud->points[b].x, cloud->points[b].y,  cloud->points[b].z };
			vector<double> n1 = { cloud_normals->points[a].normal_x, cloud_normals->points[a].normal_y,  cloud_normals->points[a].normal_z };
			vector<double> n2 = { cloud_normals->points[b].normal_x, cloud_normals->points[b].normal_y,  cloud_normals->points[b].normal_z };
			vector<double> point1 = { p1[0] - r * n1[0],p1[1] - r * n1[1],p1[2] - r * n1[2] };
			vector<double> point2 = { p2[0] - r * n2[0],p2[1] - r * n2[1],p2[2] - r * n2[2] };

			vector<double> jiku = { point1[0] - point2[0],point1[1] - point2[1],point1[2] - point2[2] };

			double jikusize = sqrt(jiku[0] * jiku[0] + jiku[1] * jiku[1] + jiku[2] * jiku[2]);
			if (jikusize <= 0.1) {
				continue;
			}
			jiku = { jiku[0] / jikusize,jiku[1] / jikusize,jiku[2] / jikusize };


			//一つ一つの点を見る
			int count = 0;
			for (j = 0; j < cloud->points.size(); j++) {
				vector<double> x = { cloud->points[j].x, cloud->points[j].y,  cloud->points[j].z };
				//軸との距離を求める
				vector<double> v = { x[0] - point1[0],x[1] - point1[1],x[2] - point1[2] };
				double vsize = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
				double d = sqrt(pow(vsize, 2) - pow((v[0] * jiku[0] + v[1] * jiku[1] + v[2] * jiku[2]), 2));
				if (d > r - threshould && r + threshould > d) {
					count++;
				}
				if (maxinliner < count) {
					maxinliner = count;
					bestP = point1;
					bestJIKU = jiku;
				}
			}
			itteration++;
		}
		cout << i << endl;


		for (j = 0; j < 3; j++) {
			ofs << bestP[j] << ",";
		}
		for (j = 0; j < 3; j++) {
			ofs << bestJIKU[j] << ",";
		}
		ofs << maxinliner << endl;

		//モデルの正しさを可視化
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

		//vector<double> point1 = { 0.0931,8.16,-0.150 };
		//vector<double> jiku = { 0.002446,0.999976,0.00207 };
		vector<double> point1 = bestP;
		vector<double> jiku = bestJIKU;
		for (j = 0; j < cloud->points.size(); j++) {
			vector<double> x = { cloud->points[j].x, cloud->points[j].y,  cloud->points[j].z };
			//軸との距離を求める
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

	return (0);
}
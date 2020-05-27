#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <iostream>
#include <string>
#include <cstdlib> 
#include <Eigen/Dense>
#include <math.h>

typedef pcl::PointXYZ PointT;
using namespace std;


int
main(int argc, char** argv)
{
	//RANSACパラメータ
	int  maxitteration = 500;
	double threshould = 0.03;
	double Rmin = 0.45;
	double Rmax = 0.55;

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


	int i,j;
	string n;
	ofstream ofs("cpp_realstright_RANSAC.csv");
	for (i = 0; i < 100; i++) {
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
		double bestR;
		
		while (itteration < maxitteration) {
			int a = rand() % cloud->points.size() ;
			int b = rand() % cloud->points.size() ;

			vector<double> p1 = { cloud->points[a].x, cloud->points[a].y,  cloud->points[a].z };
			vector<double> p2 = { cloud->points[b].x, cloud->points[b].y,  cloud->points[b].z };
			vector<double> n1 = { cloud_normals->points[a].normal_x, cloud_normals->points[a].normal_y,  cloud_normals->points[a].normal_z };
			vector<double> n2 = { cloud_normals->points[b].normal_x, cloud_normals->points[b].normal_y,  cloud_normals->points[b].normal_z };

			vector<double> p1p2 = { p1[0] - p2[0],p1[1] - p2[1],p1[2] - p1[2] };
			double n1n2 = n1[0] * n2[0]+n1[1] * n2[1]+n1[2] * n2[2] ;
			double p1p2n1 = p1p2[0] * n1[0] + p1p2[1] * n1[1] + p1p2[2] * n1[2];
			double p1p2n2 = p1p2[0] * n2[0] + p1p2[1] * n2[1] + p1p2[2] * n2[2];

			double r1 = (p1p2n1 - n1n2 * p1p2n2) / (1 - n1n2 * n1n2);
			double r2 = (n1n2*p1p2n1 - p1p2n2) / (1 - n1n2 * n1n2);

			vector<double> point1 = { p1[0] - r1 * n1[0],p1[1] - r1 * n1[1],p1[2] - r1 * n1[2] };
			vector<double> point2 = { p2[0] - r2 * n2[0],p2[1] - r2 * n2[1],p2[2] - r2 * n2[2] };

			vector<double> jiku = { point1[0] - point2[0],point1[1] - point2[1],point1[2] - point2[2] };

			vector<double> p1q1 = { p1[0] - point1[0],p1[1] - point1[1],p1[2] - point1[2] };

			double r = sqrt(p1q1[0] * p1q1[0] + p1q1[1] * p1q1[1] + p1q1[2] * p1q1[2]);

			//範囲外の円柱は除外
			if (r < Rmin || Rmax < r) {
				continue;
			}
			//軸の正負を合わせる
			if (jiku[1] < 0) {
				jiku[0] = -1 * jiku[0];
				jiku[1] = -1 * jiku[1];
				jiku[2] = -1 * jiku[2];
			}

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
				double d = sqrt(pow(vsize,2) - pow((v[0] * jiku[0] + v[1] *jiku[1] + v[2] * jiku[2]), 2));
				if (d > r - threshould && r + threshould > d) {
					count++;
				}
				if (maxinliner < count) {
					maxinliner = count;
					bestP = point1;
					bestJIKU = jiku;
					bestR = r;
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
		ofs <<bestR<<","<<maxinliner<< endl;
	}

	return (0);
}
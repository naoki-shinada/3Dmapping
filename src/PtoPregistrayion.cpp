#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <cmath>
#include <assert.h>


typedef pcl::PointXYZ PointT;
using namespace std;

void print4x4Matrix(const Eigen::Matrix4d & matrix)
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}



int
main(int argc, char** argv)
{
	vector<vector<double>> data(100, vector<double>(7));
	// All the objects needed
	pcl::PCDReader reader;
	pcl::PassThrough<PointT> pass;
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	pcl::PCDWriter writer;
	pcl::ExtractIndices<PointT> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

	// Datasets
	pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);


	int i,j;
	string n1;
	string n2;
	ofstream ofs("registration.csv");
	for (i = 0; i < 99; i++) {
		n1 = to_string(i+1);
		n2 = to_string(i+2);
	
		// Read in the cloud data
		reader.read(n1+"cylinder2m.pcd", *cloud1);
		reader.read(n2 + "cylinder2m.pcd", *cloud2);
		

		//cloud_inとcloud_outをICPアルゴズムにより変換matrixを求める。
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setInputCloud(cloud1);
		icp.setInputTarget(cloud2);

		pcl::PointCloud<pcl::PointXYZ> Final;
		icp.align(Final);

		//変換matrixを表示する
		Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
		transformation_matrix = icp.getFinalTransformation().cast<double>();
		//print4x4Matrix(transformation_matrix);
		
		double rms=0;
		for (j = 0; j < 3; j++) {
				ofs << transformation_matrix(j,3) << ",";
				rms += transformation_matrix(j, 3)* transformation_matrix(j, 3);
			}
		rms = sqrt(rms);
		ofs <<rms<<"," <<endl;

		cout << i << endl;





		//// Obtain the cylinder inliers and coefficients
		//seg.segment(*inliers_cylinder, *coefficients_cylinder);
		////data.at(i) = *coefficients_cylinder;
		//int k;
		//for (k = 0; k < 7; k++) {
		//	ofs << coefficients_cylinder->values[k] << ",";
		//}
		//ofs << endl;
		////ofs << coefficients_cylinder->values[] << ","<< endl;
		////ofs << data.at(i).at(0) << "," << data.at(i).at(1) << "," << data.at(i).at(2) << "," << data.at(i).at(3) << "," << data.at(i).at(4) << "," << data.at(i).at(5) << "," << data.at(i).at(6) << "," << endl;
	}



	//cerr  << *coefficients_cylinder << std::endl;//ここが欲しいデータ

	cout << "finish" << endl;

	
	return (0);
}
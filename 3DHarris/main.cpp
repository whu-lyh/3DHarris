#include <iostream>
#include <pcl\io\pcd_io.h>
#include <pcl/point_cloud.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/keypoints/harris_3D.h>//harris特征点估计类头文件声明
#include <cstdlib>
#include <vector>
#include <pcl/console/parse.h>

#include "PointCloudIO.h"

using namespace std;

int main ( int argc, char *argv [] )
{
	std::string pointfilepath = "./181013_030701-11-25-35-538.las";
	std::string featurepointpath = "./harris-festure-point.las";

	//pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud ( new pcl::PointCloud<pcl::PointXYZ> );
	//pcl::io::loadPCDFile ( "./181013_030701-11-25-35-538 - Cloud.pcd", *input_cloud );
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>> ();
	Utility::Offset las_offset;
	if ( PointIO::loadSingleLAS<pcl::PointXYZ> (pointfilepath, input_cloud , las_offset ))
	{
		std::cout << "las file load successfully" << std::endl;
	}

	//pcl::PCDWriter writer;
	float r_normal;
	float r_keypoint;

	//r_normal = std::stof ( 0.5 );
	//r_keypoint = std::stof ( 0.5 );
	r_normal = 0.2;
	r_keypoint = 0.2;

	//typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> ColorHandlerT3;

	pcl::PointCloud<pcl::PointXYZI>::Ptr Harris_keypoints ( new pcl::PointCloud<pcl::PointXYZI> () );
	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal>* harris_detector = new pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal>;

	harris_detector->setNonMaxSupression(true);
	harris_detector->setRadius ( r_normal );
	harris_detector->setRadiusSearch ( r_keypoint );
	harris_detector->setInputCloud ( input_cloud );
	harris_detector->setNumberOfThreads ( 8 );
	//harris_detector->setNormals(normal_source);
	//harris_detector->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>::LOWE);
	harris_detector->compute ( *Harris_keypoints );
	cout << "Harris_keypoints的大小是" << Harris_keypoints->size () << endl;
	//writer.write<pcl::PointXYZI> ( "Harris_keypoints.pcd", *Harris_keypoints, false );

	//save as las
	PointIO::saveLAS2<pcl::PointXYZI> ( featurepointpath, Harris_keypoints, las_offset );

	/*pcl::visualization::PCLVisualizer visu3 ( "clouds" );
	visu3.setBackgroundColor ( 255, 255, 255 );
	visu3.addPointCloud ( Harris_keypoints, ColorHandlerT3 ( Harris_keypoints, 0.0, 0.0, 255.0 ), "Harris_keypoints" );
	visu3.setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "Harris_keypoints" );
	visu3.addPointCloud ( input_cloud, "input_cloud" );
	visu3.setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "input_cloud" );
	visu3.spin ();*/

	system ( "pause" );
	return 1;
}
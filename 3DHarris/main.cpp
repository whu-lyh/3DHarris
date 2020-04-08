#include <iostream>
#include <pcl\io\pcd_io.h>
#include <pcl/point_cloud.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/keypoints/harris_3D.h> //harris特征点估计类头文件声明
#include <pcl/keypoints/sift_keypoint.h> //3D sift 特征点检测
#include <pcl/filters/voxel_grid.h>
#include <cstdlib>
#include <vector>
#include <pcl/console/parse.h>

#include "PointCloudIO.h"

using namespace std;

#define PCLSIFT 
namespace pcl
{
	template<>
	struct SIFTKeypointFieldSelector<PointXYZ>
	{
		inline float
			operator () ( const PointXYZ &p ) const
		{
			return p.z;
		}
	};
}

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

	//octree voxel
#ifdef PCLSIFT
	const float min_scale = 0.1;
	const int n_octaves = 6;
	const int n_scales_per_octave = 4;
	const float min_contrast = 0.01;

	pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift; //创建sift关键点检测对象
	pcl::PointCloud<pcl::PointWithScale> result;
	sift.setInputCloud ( input_cloud ); //设置输入点云
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree ( new pcl::search::KdTree<pcl::PointXYZ> () );
	sift.setSearchMethod ( tree ); //创建一个空的kd树对象tree，并把它传递给sift检测对象
	 //指定搜索关键点的尺度范围，在sift内部计算中，使用了一个默认的下采样方法，该方法采用voxel filter的方式用体素中心点近似代替原始点云进行下采样，
	//也就是说采样之后的点不是原来的原始点云中的点了
	sift.setScales ( min_scale, n_octaves, n_scales_per_octave ); 
	sift.setMinimumContrast ( min_contrast ); //设置限制关键点检测的阈值
	sift.compute ( result ); //执行sift关键点检测，保存结果在result

	pcl::PointCloud<pcl::PointXYZ>::Ptr Sift_keypoint ( new pcl::PointCloud<pcl::PointXYZ> );
	copyPointCloud ( result, *Sift_keypoint );//将点类型pcl::PointWithScale的数据转换为点类型pcl::PointXYZ的数据

	//save as las
	PointIO::saveLAS2<pcl::PointXYZ> ( featurepointpath, Sift_keypoint, las_offset );
#else
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

	harris_detector->setNonMaxSupression ( true );
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
#endif

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
#include <iostream>
#include <pcl\io\pcd_io.h>
#include <pcl/point_cloud.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/keypoints/harris_3D.h> //harris特征点估计类头文件声明
#include <pcl/keypoints/sift_keypoint.h> //3D sift 特征点检测
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <cstdlib>
#include <vector>
#include <pcl/console/parse.h>
//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

//parallel
#include <tbb/tbb.h>

#include "PointCloudIO.h"
#include "dataStructure.h"
#include "groundExtraction.h"
#include "tmpcode.hpp"

#include <vld.h>

using namespace std;

//#define PCLSIFT 
//#define 3DHARRIS
#define VOXEL_FILTER
#define APPROXIMATE_VOXEL_FILTER
//#define SOR_FILTER
//#define CONNECT_ANALYSIS_TEST

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

//获取边界
void getCloudBound ( const pcl::PointCloud<pcl::PointXYZ> & cloud, pmProcessUrban::Bounds_ym & bound )
{
	double min_x = cloud [0].x;
	double min_y = cloud [0].y;
	double min_z = cloud [0].z;
	double max_x = cloud [0].x;
	double max_y = cloud [0].y;
	double max_z = cloud [0].z;

	for ( int i = 0; i < cloud.size (); i++ )
	{
		//获取边界
		if ( min_x > cloud.points [i].x )
			min_x = cloud.points [i].x;
		if ( min_y > cloud.points [i].y )
			min_y = cloud.points [i].y;
		if ( min_z > cloud.points [i].z )
			min_z = cloud.points [i].z;
		if ( max_x < cloud.points [i].x )
			max_x = cloud.points [i].x;
		if ( max_y < cloud.points [i].y )
			max_y = cloud.points [i].y;
		if ( max_z < cloud.points [i].z )
			max_z = cloud.points [i].z;
	}
	bound.min_x = min_x;
	bound.max_x = max_x;
	bound.min_y = min_y;
	bound.max_y = max_y;
	bound.min_z = min_z;
	bound.max_z = max_z;
}

//获取边界-重载
void getCloudBound ( const pcl::PointCloud<PointXYZINTF> & cloud, pmProcessUrban::Bounds_ym & bound )
{
	double min_x = cloud [0].x;
	double min_y = cloud [0].y;
	double min_z = cloud [0].z;
	double max_x = cloud [0].x;
	double max_y = cloud [0].y;
	double max_z = cloud [0].z;

	for ( int i = 0; i < cloud.size (); i++ )
	{
		//获取边界
		if ( min_x > cloud.points [i].x )
			min_x = cloud.points [i].x;
		if ( min_y > cloud.points [i].y )
			min_y = cloud.points [i].y;
		if ( min_z > cloud.points [i].z )
			min_z = cloud.points [i].z;
		if ( max_x < cloud.points [i].x )
			max_x = cloud.points [i].x;
		if ( max_y < cloud.points [i].y )
			max_y = cloud.points [i].y;
		if ( max_z < cloud.points [i].z )
			max_z = cloud.points [i].z;
	}
	bound.min_x = min_x;
	bound.max_x = max_x;
	bound.min_y = min_y;
	bound.max_y = max_y;
	bound.min_z = min_z;
	bound.max_z = max_z;
}

//获取中心和边界
void getBoundAndCenter ( const pcl::PointCloud<pcl::PointXYZ> & cloud, pmProcessUrban::Bounds_ym & bound, pmProcessUrban::CenterPoint_ym& centerPoint )
{
	double min_x = cloud [0].x;
	double min_y = cloud [0].y;
	double min_z = cloud [0].z;
	double max_x = cloud [0].x;
	double max_y = cloud [0].y;
	double max_z = cloud [0].z;

	double cx = 0, cy = 0, cz = 0;

	for ( int i = 0; i < cloud.size (); i++ )
	{
		//获取边界
		if ( min_x > cloud.points [i].x )
			min_x = cloud.points [i].x;
		if ( min_y > cloud.points [i].y )
			min_y = cloud.points [i].y;
		if ( min_z > cloud.points [i].z )
			min_z = cloud.points [i].z;
		if ( max_x < cloud.points [i].x )
			max_x = cloud.points [i].x;
		if ( max_y < cloud.points [i].y )
			max_y = cloud.points [i].y;
		if ( max_z < cloud.points [i].z )
			max_z = cloud.points [i].z;


		cx += cloud.points [i].x / cloud.size ();
		cy += cloud.points [i].y / cloud.size ();
		cz += cloud.points [i].z / cloud.size ();
	}
	bound.min_x = min_x;
	bound.max_x = max_x;
	bound.min_y = min_y;
	bound.max_y = max_y;
	bound.min_z = min_z;
	bound.max_z = max_z;


	centerPoint.x = cx;
	centerPoint.y = cy;
	centerPoint.z = cz;
}

void GetSubsetBoundary ( pcl::PointCloud<pcl::PointXYZ>::Ptr & plane_wall_cloud,
						 vector<int> & index, pmProcessUrban::Bounds_ym & bound )
{
	//构建点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud ( new pcl::PointCloud<pcl::PointXYZ> );
	for ( int i = 0; i < index.size (); i++ )
	{
		temp_cloud->push_back ( plane_wall_cloud->points [index [i]] );
	}
	getCloudBound ( *temp_cloud, bound );
}

int main ( int argc, char *argv [] )
{
	//tbb parallel
	tbb::parallel_for ( 0, 10, [] ( int num ) {std::cout << num << " : hello tbb " << std::endl; } );

	//std::string pointfilepath = "./181013_030701-11-25-35-538.las";
	std::string pointfilepath = "./0429-ground-xyziZNN.las";
	std::string featurepointpath = "./harris-festure-point.las";

	//pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud ( new pcl::PointCloud<pcl::PointXYZ> );
	//pcl::io::loadPCDFile ( "./181013_030701-11-25-35-538 - Cloud.pcd", *input_cloud );
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>> ();
	//pcl::PointCloud<PointXYZINTF>::Ptr input_cloud = boost::make_shared<pcl::PointCloud<PointXYZINTF>> ();
	Utility::Offset las_offset;
	if ( PointIO::loadSingleLAS<pcl::PointXYZ> (pointfilepath, input_cloud , las_offset ))
	{
		std::cout << "las file load successfully" << std::endl;
	}

	//octree voxel
#ifdef VOXEL_FILTER
	pcl::PointCloud<pcl::PointXYZ>::Ptr spl_cloud ( new pcl::PointCloud<pcl::PointXYZ> );
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud ( input_cloud );
	sor.setLeafSize ( 0.2, 0.2, 0.02 );
	sor.filter ( *spl_cloud );
	//save as las
	PointIO::saveLAS2<pcl::PointXYZ> ( featurepointpath, spl_cloud, las_offset );
#endif

	//approximate voxel filter
	//using a hash table to mappint the points into the limited number of container, if the hash conflict occurs, clean the point that are already
	// existed point as a out put point cloud? still from the origin  point cloud?
#ifdef APPROXIMATE_VOXEL_FILTER
	pcl::PointCloud<pcl::PointXYZ>::Ptr spl_cloud ( new pcl::PointCloud<pcl::PointXYZ> );
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> avg;
	avg.setInputCloud ( input_cloud );
	avg.setLeafSize ( 0.2, 0.2, 0.02 );
	avg.filter ( *spl_cloud );
	//save as las
	PointIO::saveLAS2<pcl::PointXYZ> ( featurepointpath, spl_cloud, las_offset );
#endif

	//sor
#ifdef SOR_FILTER
	for ( int i = 0; i < 5; ++i ) {
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sorfilter ( true ); // Initializing with true will allow us to extract the removed indices
		sorfilter.setInputCloud ( input_cloud );
		sorfilter.setMeanK ( 1-i*0.15 );
		sorfilter.setStddevMulThresh ( 1.0 );
		sorfilter.filter ( *input_cloud );
	}

	//save as las
	PointIO::saveLAS2<pcl::PointXYZ> ( "sor_filter.las", input_cloud, las_offset );
#endif
	
	//pcl 3d sift
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
#endif

	//pcl 3d harris
#ifdef HARRIS
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

	//conncected analysis
#ifdef CONNECT_ANALYSIS_TEST
	pmProcessUrban::Bounds_ym  bound;
	getCloudBound ( *input_cloud, bound );
	int imagerows = bound.max_x - bound.min_x + 1;
	int imagecols = bound.max_y - bound.min_y + 1;
	cv::Mat pData = cv::Mat ( imagerows, imagecols, 3, CV_32FC1 );

	for ( size_t i = 0; i < input_cloud->points.size (); ++i )
	{
		int row_x = input_cloud->points [i].x - bound.min_x;
		int col_y = input_cloud->points [i].y - bound.min_y;
		pData.at<uchar> ( row_x, col_y ) = input_cloud->points [i].intensity;
	}

	cv::imshow ( "show image", pData );
	cv::imwrite ( "./point.jpg", pData );
#endif


	//visualization
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
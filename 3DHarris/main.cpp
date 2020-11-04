#include <iostream>
#include <pcl\io\pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/keypoints/harris_3D.h> //harris特征点估计类头文件声明
#include <pcl/keypoints/sift_keypoint.h> //3D sift 特征点检测
#include <pcl/keypoints/iss_3d.h> //ISS KEY POINT
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <cstdlib>
#include <vector>
#include <ctime>
#include <chrono>
#include <pcl/console/parse.h>
//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

//parallel library test
#include <tbb/tbb.h>

#include "PointCloudIO.h"
//#include "iss_keypoint.h"

#include "CCConst.h"
#include "Neighbourhood.h"

//memory leaky
//#include <vld.h>

//portable file dialog
#include "portable-file-dialogs.h"

using namespace std;

#define REGISTRATION
//#define PDF
//#define PCLSIFT 
//#define 3DHARRIS
//#define VOXEL_FILTER
//#define APPROXIMATE_VOXEL_FILTER
#define SOR_FILTER
//#define CONNECT_ANALYSIS_TEST
//#define CPLUSPLUS

#if _WIN32
#define DEFAULT_PATH "C:\\"
#else
#define DEFAULT_PATH "/tmp"
#endif

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

// Unused function that just tests the whole API
void api ()
{
	// pfd::settings
	pfd::settings::verbose ( true );
	pfd::settings::rescan ();

	// pfd::notify
	pfd::notify ( "", "" );
	pfd::notify ( "", "", pfd::icon::info );
	pfd::notify ( "", "", pfd::icon::warning );
	pfd::notify ( "", "", pfd::icon::error );
	pfd::notify ( "", "", pfd::icon::question );

	pfd::notify a ( "", "" );
	(void) a.ready ();
	(void) a.ready ( 42 );

	// pfd::message
	pfd::message ( "", "" );
	pfd::message ( "", "", pfd::choice::ok );
	pfd::message ( "", "", pfd::choice::ok_cancel );
	pfd::message ( "", "", pfd::choice::yes_no );
	pfd::message ( "", "", pfd::choice::yes_no_cancel );
	pfd::message ( "", "", pfd::choice::retry_cancel );
	pfd::message ( "", "", pfd::choice::abort_retry_ignore );
	pfd::message ( "", "", pfd::choice::ok, pfd::icon::info );
	pfd::message ( "", "", pfd::choice::ok, pfd::icon::warning );
	pfd::message ( "", "", pfd::choice::ok, pfd::icon::error );
	pfd::message ( "", "", pfd::choice::ok, pfd::icon::question );

	pfd::message b ( "", "" );
	(void) b.ready ();
	(void) b.ready ( 42 );
	(void) b.result ();
}

void setScatterMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, const int& current_index, Eigen::Matrix3d &cov_m)
{
	const pcl::PointXYZ& current_point = input_cloud->points[current_index];

	double central_point[3];
	memset(central_point, 0, sizeof(double) * 3);

	central_point[0] = current_point.x;
	central_point[1] = current_point.y;
	central_point[2] = current_point.z;

	cov_m = Eigen::Matrix3d::Zero();

	std::vector<int> nn_indices;
	std::vector<float> nn_distances;
	int n_neighbors;

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(input_cloud);
	kdtree.radiusSearch(current_index, 5, nn_indices, nn_distances);

	n_neighbors = static_cast<int> (nn_indices.size());

	double cov[9];
	memset(cov, 0, sizeof(double) * 9);

	for ( int n_idx = 0; n_idx < n_neighbors; n_idx++ )
	{
		const pcl::PointXYZ& n_point = input_cloud->points[nn_indices[n_idx]];

		double neigh_point[3];
		memset(neigh_point, 0, sizeof(double) * 3);

		neigh_point[0] = n_point.x;
		neigh_point[1] = n_point.y;
		neigh_point[2] = n_point.z;

		for ( int i = 0; i < 3; i++ )
			for ( int j = 0; j < 3; j++ )
				cov[i * 3 + j] += (neigh_point[i] - central_point[i]) * (neigh_point[j] - central_point[j]);
	}

	cov_m << cov[0], cov[1], cov[2],
		cov[3], cov[4], cov[5],
		cov[6], cov[7], cov[8];
}

int main(int argc, char *argv[])
{

	using PointT = pcl::PointXYZ;

#ifdef REGISTRATION

	std::string pointfilepath = "F:/Data/wuhan/dataaroundGaoJia/20191211150218-df/iScan-Pcd-1_part18.las";
	//std::string pointfilepath = "../x64/Release/line.las";

	std::string keypointpath = Utility::get_parent(pointfilepath) + "/iss";
	Utility::ensure_dir(keypointpath);

	pcl::PointCloud<PointT>::Ptr input_cloud = boost::make_shared<pcl::PointCloud<PointT>>();
	Utility::Offset las_offset;

	if (PointIO::loadSingleLAS<PointT>(pointfilepath, input_cloud, las_offset))
	{
		std::cout << "las file load successfully" << std::endl;
		std::cout << input_cloud->points.size() << std::endl;
	}

	//sor
#ifdef SOR_FILTER

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sorfilter(true); // Initializing with true will allow us to extract the removed indices
	sorfilter.setInputCloud(input_cloud);
	sorfilter.setMeanK(6);
	sorfilter.setStddevMulThresh(5.0);
	sorfilter.filter(*input_cloud);

	//save as las
	std::string tmppath = keypointpath + "/" + Utility::get_name_without_ext(pointfilepath) + "_sor.las";
	PointIO::saveLAS2<pcl::PointXYZ>(tmppath, input_cloud, las_offset);
#endif

	std::chrono::high_resolution_clock::time_point t1report = std::chrono::high_resolution_clock::now();
	{
		//iss key point detected
		pcl::PointCloud<PointT>::Ptr  cloud_src_iss(new pcl::PointCloud<PointT>);
		pcl::ISSKeypoint3D<PointT, PointT> iss_det;
		pcl::search::KdTree<PointT>::Ptr tree_1(new pcl::search::KdTree<PointT>());

		double model_solution = 0.0;
		//calculate the resolution but might be more time consuming
		{
			//double model_solution = PointIO::computeCloudResolution<pcl::PointXYZ>(input_cloud);//参数小，采取的关键点多，论文中为500左右
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
			model_solution = PointIO::computeCloudResolution<pcl::PointXYZ, 50>(input_cloud, cloud_colored);

			tmppath = keypointpath + "/" + Utility::get_name_without_ext(pointfilepath) + "_distance_colord.las";
			PointIO::saveLAS2<pcl::PointXYZRGB>(tmppath, cloud_colored, las_offset);

			//double model_solution = 0.05;//basically the resolution is 0.03
			std::cout << "model_solution:\t" << model_solution << std::endl;
		}

		//参数设置
		iss_det.setInputCloud(input_cloud);
		iss_det.setSearchMethod(tree_1);
		iss_det.setSalientRadius(6 * model_solution); //the radius to collecte the neighbor points while building the scatter matrix. The block size is 5-15m
		iss_det.setNonMaxRadius(4 * model_solution); //
		//gamma32 larger ,gamma21 larger==>the plane feature
		//gamma32 larger ,gamma21 smaller==>the line feature
		//the threshold should not be too tight in case of the noisy
		iss_det.setThreshold21(0.975); //lambda2/lambda1>gamma21,lambdai is calculated by the EVD (eigen value decomposition) matrix
		//iss_det.setThreshold21_line(0.1);
		iss_det.setThreshold32(0.975); //lambda3/lambda2>gamma32
		//iss_det.setThreshold32_line(0.05);
		 //if this points lambda3 > all the other points' lambda3, this is a final points
		iss_det.setMinNeighbors(15); //Set the minimum number of neighbors that has to be found while applying the non maxima suppression algorithm.
		iss_det.setNumberOfThreads(8); //default thread is the current machine's cpu kernel
		//cull key points that are lying on the border
		//iss_det.setBorderRadius(3 * model_solution);
		//iss_det.setNormalRadius(3 * model_solution);
		iss_det.compute(*cloud_src_iss);

		tmppath = keypointpath + "/" + Utility::get_name_without_ext(pointfilepath) + "_keypoints.las";
		PointIO::saveLAS2<pcl::PointXYZ>(tmppath, cloud_src_iss, las_offset);
	}

	{
		//CCLib::GenericIndexedCloudPersist neighboursCloud(&nNSS.pointsInNeighbourhood);
		//CCLib::Neighbourhood Z(&neighboursCloud);
		//ScalarType value = NAN_VALUE;
		//value = static_cast<ScalarType>(Z.computeFeature(static_cast<CCLib::Neighbourhood::GeomFeature>(5)));
	}

	std::chrono::high_resolution_clock::time_point t2report = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> t12report = std::chrono::duration_cast<std::chrono::duration<double>>(t2report - t1report);
	std::cout << "Out put the key point extracted file, time cost: " << t12report.count() << "s" << std::endl;

#endif

#ifdef PDF  
	// Set verbosity to true
	pfd::settings::verbose ( true );

	// Directory selection
	auto dir = pfd::select_folder ( "请选择点云文件夹", DEFAULT_PATH ).result ();
	std::cout << "Selected dir: " << dir << "\n";

	std::vector<std::string> files;
	Utility::get_files ( dir, ".lin", files );
	int trajnum = files.size ();
	std::string trajname;
	for ( int i = 0; i < trajnum - 1; ++i )
	{
		//std::cout << "trajectories: " << std::endl;
		trajname += files [i] + ",";
	}
	trajname += files [trajnum - 1];
	std::cout << trajname;

	// File open
	auto f = pfd::open_file ( "请选择轨迹文件", DEFAULT_PATH,
	{ "车载点云轨迹 (.lin .Post .txtEst)", "*.lin *.Post *.txtEst",
							  "全部文件", "*.*" },
							  pfd::opt::multiselect );
	std::cout << "Selected files:";
	for ( auto const &name : f.result () )
		std::cout << name << ",";
	std::cout << "\n";
#endif // PDF

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
#endif // PCLSIFT

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
#endif // HARRIS

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
#endif // CONNECT_ANALYSIS_TEST


#ifdef CPLUSPLUS
	//tbb parallel
	tbb::parallel_for(0, 10, [](int num) {std::cout << num << " : hello tbb " << std::endl; });

	std::cout << sizeof(short int) << std::endl;
	std::cout << sizeof(char) << std::endl;
	std::cout << sizeof(bool) << std::endl;

	//float 6 valid value will be keeped(the 1 will be saved)
	float slamm = 0.000111111;
	std::cout << slamm << std::endl;

	vector<short int> occupation_grid;
	occupation_grid.reserve(40000000);
	occupation_grid.resize(40000000, false);
	//for (auto singlegrid:occupation_grid)
	//{
	//	std::cout << singlegrid << std::endl;
	//}

#endif // CPLUSPLUS

	system ( "pause" );
	return 1;
}
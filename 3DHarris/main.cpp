#include <iostream>
#include <fstream>
#include <pcl\io\pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/keypoints/harris_3D.h> //harris特征点估计类头文件声明
#include <pcl/keypoints/sift_keypoint.h> //3D sift 特征点检测
#include <pcl/keypoints/iss_3d.h> //ISS KEY POINT
#include <pcl/common/centroid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/common/transforms.h>
#include <cstdlib>
#include <vector>
#include <ctime>
#include <chrono>
#include <pcl/console/parse.h>
//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include "Utility.h"
#include "PointCloudIO.h"
#include "kmeans.hpp"
#include "dbscan.h"
#include "../Submodules/mba/mba/mba.hpp"
//#include "iss_keypoint.h"

#include "nanoflann_pcl.h"

//memory leaky
//#include <vld.h>

//portable file dialog
#include "portable-file-dialogs.h"
// CCLIB
#include <CCCoreLib/CCTypes.h>
#include <CCCoreLib/ReferenceCloud.h>
using namespace std;

#define MBA_
//#define DBSCAN_
//#define COLORSETTING
//#define ICP_REGISTRATION_
//#define ISSMODIFY_
//#define KMEANS_
//#define NORMAL_
//#define PDF
//#define PCLSIFT 
//#define 3DHARRIS
//#define VOXEL_FILTER
//#define APPROXIMATE_VOXEL_FILTER
//#define SOR_FILTER
//#define ROR_FILTER
//#define CONNECT_ANALYSIS_TEST
//#define CCLIB

#ifdef _DEBUG
#pragma comment(lib, "libboost_thread-vc141-mt-gd-1_64.lib")
#pragma comment(lib, "tbb_debug.lib")
#pragma comment(lib, "CCCoreLibd.lib")
#else
#pragma comment(lib, "libboost_thread-vc141-mt-1_64.lib")
#pragma comment(lib, "tbb.lib")
#pragma comment(lib, "CCCoreLib.lib")
#endif

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

//seems to be useless might be used in print the string contains Chinese?
std::wstring StringToWString(const std::string& str)
{
	LPCSTR pszSrc = str.c_str();
	int nLen = MultiByteToWideChar(CP_ACP, 0, pszSrc, -1, NULL, 0);
	if (nLen == 0)
		return std::wstring(L"");

	wchar_t* pwszDst = new wchar_t[nLen];
	if (!pwszDst)
		return std::wstring(L"");

	MultiByteToWideChar(CP_ACP, 0, pszSrc, -1, pwszDst, nLen);
	std::wstring wstr(pwszDst);
	delete[] pwszDst;
	pwszDst = NULL;

	return wstr;
}

std::string WStringToString(const std::wstring &wstr)
{
	LPCWSTR pwszSrc = wstr.c_str();
	int nLen = WideCharToMultiByte(CP_ACP, 0, pwszSrc, -1, NULL, 0, NULL, NULL);
	if (nLen == 0)
		return std::string("");

	char* pszDst = new char[nLen];
	if (!pszDst)
		return std::string("");

	WideCharToMultiByte(CP_ACP, 0, pwszSrc, -1, pszDst, nLen, NULL, NULL);
	std::string str(pszDst);
	delete[] pszDst;
	pszDst = NULL;

	return str;
}

std::vector<std::string> stringSplit(const string& str, const string& pattern)
{//c++ standard
	std::vector<std::string> ret;
	if (pattern.empty()) return ret;
	size_t start = 0, index = str.find_first_of(pattern, 0);
	while (index != str.npos)
	{
		if (start != index)
			ret.push_back(str.substr(start, index - start));
		start = index + 1;
		index = str.find_first_of(pattern, start);
	}
	if (!str.substr(start).empty())
		ret.push_back(str.substr(start));
	return ret;
}

//compare by the specific part of the string
bool compareString(const std::string &str1, const std::string &str2)
{
	std::vector<std::string> splited_value1 = stringSplit(str1, "_");
	std::vector<std::string> splited_value2 = stringSplit(str2, "_");
	if (!splited_value1.empty() && splited_value1.size() > 7 && !splited_value2.empty() && splited_value2.size() > 7)
		return std::stoi(splited_value1[6]) < std::stoi(splited_value2[6]);
	return str1.length() < str2.length();
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
	using PointTcolor = pcl::PointXYZRGB;

#ifdef MBA_
	// load data
	std::string filename = "F:/Data/BGP_ALS/test_data.las";
	std::string out_filename = "F:/Data/BGP_ALS/test_data_ascii.txt";
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	Utility::Offset offset;
	PointIO::loadSingleLAS<pcl::PointXYZ>(filename, input_cloud, offset);
	// Coordinates of data points.
	std::vector<mba::point<2>> coo;
	// Data values.
	std::vector<double> val;
	// type transform
	for (auto pt: input_cloud->points)
	{
		coo.push_back(mba::point<2>{pt.x, pt.y});
		val.emplace_back(pt.z);
	}
	//// save as txt file
	//std::ofstream ofs(out_filename);
	//for (auto pt : input_cloud->points)
	//{
	//	ofs << pt.x << "," << pt.y << "," << pt.z << std::endl;
	//}
	//ofs.flush();
	//ofs.close();
	Utility::Bound bound = PointIO::getBoundBox<pcl::PointXYZ>(input_cloud);
	std::cout << bound << std::endl;
	// Bounding box containing the data points.
	// here the slightly larger boundary is check from 
	// https://github.com/ddemidov/mba/issues/12#issuecomment-696804621
	mba::point<2> lo = { bound.min_x - 1.0, bound.min_y - 1.0 };
	mba::point<2> hi = { bound.max_x + 1.0, bound.max_y + 1.0 };
	// Initial grid size.
	mba::index<2> grid = { 3, 3 };
	// Algorithm setup. MBA is initialized with linear approximation
	// larger level corresponding to higher precision
	mba::MBA<2> interp(lo, hi, grid, coo, val);
	// Get interpolated value at arbitrary location.
	double w = interp(mba::point<2>{187689.5 - offset.x, 4989614.5 - offset.y});
	std::cout << w + offset.z << std::endl;
#endif MBA_

#ifdef DBSCAN_
	// the density usage is unclear and parameter matters
	std::string filename = "E:/codefiles/NewVS/3DHarris/data/000002.pcd";
	std::string out_filename = "E:/codefiles/NewVS/3DHarris/data/000002_clustered.pcd";
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	PointIO::loadPCD<pcl::PointXYZ>(filename, input_cloud);
	// minimal point number should be small
	Util::DBSCAN dbscan(0.3, 2);
	dbscan.setInputCloud(*input_cloud);
	dbscan.clustering();
	//dbscan.print();
	dbscan.save2files(out_filename);
#endif // DBSCAN_

#ifdef KMEANS_
	std::string pointfilepath = "F:/Data/TS/local_9TS0.las";

	std::string keypointpath = Utility::get_parent(pointfilepath) + "/kmeans";
	Utility::ensure_dir(keypointpath);

	pcl::PointCloud<PointT>::Ptr input_cloud = boost::make_shared<pcl::PointCloud<PointT>>();
	Utility::Offset las_offset;

	if (PointIO::loadSingleLAS<PointT>(pointfilepath, input_cloud, las_offset))
	{
		std::cout << "las file load successfully" << std::endl;
		std::cout << input_cloud->points.size() << std::endl;
	}

	KmeansPlus kmean_ts(input_cloud->size(), 3);

	std::vector<std::vector<float>> pts;//, centroids;

	for (auto pt: input_cloud->points)
	{
		std::vector<float> pt_xyz = { pt.x,pt.y,pt.z };
		pts.push_back(pt_xyz);
	}

	kmean_ts.setInputData(pts);
	kmean_ts.setClusterSize(2);
	kmean_ts.kMeans();

	pcl::Kmeans::Centroids centroids = kmean_ts.get_centroids();
	//in fact the centroid is the double vector container

	std::cout << "points in total Cloud : " << input_cloud->points.size() << std::endl;
	std::cout << "centroid count: " << centroids.size() << std::endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
	Utility::Offset las_offset_cluster;

	for (int i = 0; i < centroids.size(); ++i)
	{
		std::cout << centroids[i][0] << "," << centroids[i][1] << "," << centroids[i][2]<<std::endl;
	}

	pcl::Kmeans::ClustersToPoints clusters2points = kmean_ts.getclusters2points();

	for (int i = 0; i < clusters2points.size(); ++i)
	{
		pcl::PointXYZRGB pt;
		for (auto it = clusters2points[i].begin(); it != clusters2points[i].end(); it++)
		{
			//std::cout << *it << std::endl;
			pt.x = input_cloud->points[*it].x;
			pt.y = input_cloud->points[*it].y;
			pt.z = input_cloud->points[*it].z;
			pt.r = i * 125;
			pt.g = i;
			pt.b = i * 125;
			output_cloud->push_back(pt);
		}
	}

	PointIO::saveLAS2<pcl::PointXYZRGB>(keypointpath + "/cluester.las", output_cloud, las_offset_cluster);

#endif

#ifdef COLORSETTING
	std::wcout.imbue(locale("chs"));

	std::vector<std::string> source_files, target_files;
	Utility::get_files("F:/Data/Train/Train_Roam/refine", ".las", source_files);
	Utility::get_files("F:/Data/Train/Train_Roam/refine/0401", ".las", target_files);

	for (int i = 0; i < source_files.size(); i++)
	{
		pcl::PointCloud<PointT>::Ptr source_cloud = boost::make_shared<pcl::PointCloud<PointT>>();
		pcl::PointCloud<PointT>::Ptr target_cloud = boost::make_shared<pcl::PointCloud<PointT>>();
		pcl::PointCloud<PointTcolor>::Ptr source_cloud_c = boost::make_shared<pcl::PointCloud<PointTcolor>>();
		pcl::PointCloud<PointTcolor>::Ptr target_cloud_c = boost::make_shared<pcl::PointCloud<PointTcolor>>();

		Utility::Offset las_offset, las_offset2;

		if (PointIO::loadSingleLAS<PointT>(source_files[i], source_cloud, las_offset) &&
			PointIO::loadSingleLAS<PointT>(target_files[i], target_cloud, las_offset2))
		{
			std::cout << "las file load successfully" << std::endl;
			std::cout << source_cloud->points.size() << std::endl;
			std::cout << target_cloud->points.size() << std::endl;
		}

		for (auto pt:source_cloud->points)
		{
			PointTcolor cpt;
			cpt.x = pt.x;
			cpt.y = pt.y;
			cpt.z = pt.z;
			cpt.r = 0;
			cpt.g = 255;
			cpt.b = 0;
			source_cloud_c->push_back(cpt);
		}

		for (auto pt : target_cloud->points)
		{
			PointTcolor cpt;
			cpt.x = pt.x;
			cpt.y = pt.y;
			cpt.z = pt.z;
			cpt.r = 255;
			cpt.g = 0;
			cpt.b = 0;
			target_cloud_c->push_back(cpt);
		}

		std::string source_result = Utility::get_parent(source_files[i]) + "/" + Utility::get_name_without_ext(source_files[i]) + "_c.las";
		std::string target_result = Utility::get_parent(target_files[i]) + "/" + Utility::get_name_without_ext(target_files[i]) + "_c.las";
		PointIO::saveLAS2<PointTcolor>(source_result, source_cloud_c, las_offset);
		PointIO::saveLAS2<PointTcolor>(target_result, target_cloud_c, las_offset2);
	}


#endif //COLORSETTING

#ifdef ICP_REGISTRATION_

	//std::string source_pc_path = "F:/Data/wuhan/dataaroundGaoJia/20191211150218-df/iss/iScan-Pcd-1_part18_keypoints.las";
	//std::string target_pc_path = Utility::get_parent(source_pc_path) + "/iScan-Pcd-1_part18_keypoints_target.las";
	//std::string result_pc_path = Utility::get_parent(source_pc_path) + "/iScan-Pcd-1_part18_keypoints_result.las";

	std::wcout.imbue(locale("chs"));
	//std::wcout << "1.\t" << StringToWString(source_pc_path) << std::endl;

	std::wstring source_pc_path_w = L"F:/Data/Train/pairwiseregistration/京沈线_上行_普通线路_2019-03-15_0-27-3_0_0_t_dis.las";
	std::cout << source_pc_path_w.length()<<std::endl; //each word(character) is 1 length
	std::string source_pc_path = WStringToString(source_pc_path_w);
	std::cout << "3.\t" << source_pc_path << std::endl;
	std::wstring target_pc_path_w = L"/京沈线_上行_普通线路_2019-04-01_0-20-25_0_0_t_dis.las";
	std::string target_pc_path = Utility::get_parent(source_pc_path) + WStringToString(target_pc_path_w);
	std::string result_pc_path = Utility::get_parent(source_pc_path) + "/psline_up_normal_2019_0315-0401_0.las";

	std::vector<std::string> source_files,target_files;
	//Utility::get_files("F:/Data/Train/20190315", ".las", source_files);
	Utility::get_files("F:/Data/Train/result_700_710", ".las", source_files); 
	std::sort(source_files.begin(), source_files.end(), compareString);
	Utility::get_files("F:/Data/Train/origin_0401_700_710", ".las", target_files);
	std::sort(target_files.begin(), target_files.end(), compareString);
	std::string source_refined_path = "F:/Data/Train/result_0315_icp_700_710";

	boost::shared_ptr<pcl::IterativeClosestPoint<PointT, PointT>> icp(new pcl::IterativeClosestPoint<PointT, PointT>());
	icp->setTransformationEpsilon(0.01);
	icp->setEuclideanFitnessEpsilon(0.01);
	icp->setMaximumIterations(100);
	icp->setUseReciprocalCorrespondences(true);
	icp->setMaxCorrespondenceDistance(0.25);
	icp->setRANSACIterations(50);
	icp->setRANSACOutlierRejectionThreshold(0.05);

	boost::shared_ptr<pcl::NormalDistributionsTransform<PointT, PointT>> ndt(new pcl::NormalDistributionsTransform<PointT, PointT>());
	ndt->setTransformationEpsilon(0.005);
	ndt->setMaximumIterations(20);
	ndt->setResolution(0.2);
	ndt->setStepSize(0.05);

	for (int i=0;i<source_files.size();i++)
	{
		pcl::PointCloud<PointT>::Ptr source_cloud = boost::make_shared<pcl::PointCloud<PointT>>();
		pcl::PointCloud<PointT>::Ptr target_cloud = boost::make_shared<pcl::PointCloud<PointT>>();
		pcl::PointCloud<PointT>::Ptr result_cloud = boost::make_shared<pcl::PointCloud<PointT>>();
		Utility::Offset las_offset, las_offset2;

		if (PointIO::loadSingleLAS<PointT>(source_files[i], source_cloud, las_offset) &&
			PointIO::loadSingleLAS<PointT>(target_files[i], target_cloud, las_offset2))
		{
			std::cout << "las file load successfully" << std::endl;
			std::cout << source_cloud->points.size() << std::endl;
			std::cout << target_cloud->points.size() << std::endl;
		}

		icp->setInputSource(source_cloud);
		icp->setInputTarget(target_cloud);
		icp->align(*result_cloud);

		//ndt->setInputSource(source_cloud);
		//ndt->setInputTarget(target_cloud);
		//ndt->align(*result_cloud);
		std::cout << "Score:\t" << icp->getFitnessScore(0.05) << std::endl;;

		std::string tmp_path = source_refined_path + "/" + Utility::get_name_without_ext(source_files[i]) + "_icp_refined.las";
		PointIO::saveLAS2<PointT>(tmp_path, result_cloud, las_offset);
	}

#endif //ICP_REGISTRATION_

#ifdef ISSMODIFY_

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

#ifdef ROR_FILTER
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::RadiusOutlierRemoval<pcl::PointXYZ> pcFilter;  //创建滤波器对象
		pcFilter.setInputCloud(input_cloud);             //设置待滤波的点云
		pcFilter.setRadiusSearch(0.3);               // 设置搜索半径
		pcFilter.setMinNeighborsInRadius(2);      // 设置一个内点最少的邻居数目
		pcFilter.filter(*cloud_filtered);        //滤波结果存储到cloud_filtered
		std::cerr << "Cloud after filtering: " << std::endl;
		std::cerr << cloud_filtered->points.size() << std::endl;
	}
#endif //RadiusOutlierRemoval

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

		double model_solution = 0.03;
		//calculate the resolution but might be more time consuming
		/*{
			//double model_solution = PointIO::computeCloudResolution<pcl::PointXYZ>(input_cloud);//参数小，采取的关键点多，论文中为500左右
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
			model_solution = PointIO::computeCloudResolution<pcl::PointXYZ, 50>(input_cloud, cloud_colored);

			tmppath = keypointpath + "/" + Utility::get_name_without_ext(pointfilepath) + "_distance_colord.las";
			PointIO::saveLAS2<pcl::PointXYZRGB>(tmppath, cloud_colored, las_offset);

			std::cout << "model_solution:\t" << model_solution << std::endl;
		}*/

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

		std::chrono::high_resolution_clock::time_point t2report = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> t12report = std::chrono::duration_cast<std::chrono::duration<double>>(t2report - t1report);
		std::cout << "Out put the key point extracted file, time cost: " << t12report.count() << "s" << std::endl;

		std::string tmppath = keypointpath + "/" + Utility::get_name_without_ext(pointfilepath) + "_keypoints.las";
		PointIO::saveLAS2<pcl::PointXYZ>(tmppath, cloud_src_iss, las_offset);
	}

	{
		//CCLib::GenericIndexedCloudPersist neighboursCloud(&nNSS.pointsInNeighbourhood);
		//CCLib::Neighbourhood Z(&neighboursCloud);
		//ScalarType value = NAN_VALUE;
		//value = static_cast<ScalarType>(Z.computeFeature(static_cast<CCLib::Neighbourhood::GeomFeature>(5)));
	}

#endif //ISSMODIFY_

#ifdef NORMAL_
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr normal_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//std::string pointfilepath = "F:/Data/TS/1028WHTSpointcloudfenkuai1-2-8-intensity-filt-lable3.las";
	std::string pointfilepath = "E:/codefiles/NewVS/3DHarris/data/plane_cloud.las";
	Utility::Offset las_offset;

	if (PointIO::loadSingleLAS<PointT>(pointfilepath, normal_cloud, las_offset))
	{
		std::cout << "las file load successfully" << std::endl;
		std::cout << normal_cloud->points.size() << std::endl;
	}

	Eigen::Matrix<float, 3, 3> covariance_matrix;
	Eigen::Matrix<float, 4, 1> centroid;
	//calculate the covariance matrix using the cloud without demean
	pcl::computeMeanAndCovarianceMatrix<pcl::PointXYZ,float>(*normal_cloud, covariance_matrix, centroid);
	//normal direction
	float nx, ny, nz, curvature;
	pcl::solvePlaneParameters(covariance_matrix, nx, ny, nz, curvature);
	
	//nx,ny,nz is exactly the normal of this plane-like point cloud
	std::cout << nx << "," << ny << "," << nz << std::endl;

	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*normal_cloud, pcaCentroid);
	Eigen::Matrix3f covariance;
	//calculate the covariance matrix using the cloud without demean
	pcl::computeCovarianceMatrixNormalized(*normal_cloud, pcaCentroid, covariance);

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
	std::cout << "eigenValuesPCA_before: " << std::endl << eigenValuesPCA << std::endl;
	std::cout << "eigenVectorsPCA_before: " << std::endl << eigenVectorsPCA << std::endl;

	//y axis
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(2));
	//change to x-y-z=pca.p-pca.p.cross(normal)-normal
	// default is z-y-x
	//eigenVectorsPCA.col(2).swap(eigenVectorsPCA.col(0));

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Matrix4f trans;
	{
		pcl::PointXYZ pt;
		//获得cloud_src点云;
		pt.x = 1.f;
		pt.y = 0.f;
		pt.z = 0.f;
		cloud_src->push_back(pt);

		pt.x = 0.f;
		pt.y = 1.f;
		pt.z = 0.f;
		cloud_src->push_back(pt);

		pt.x = 0.f;
		pt.y = 0.f;
		pt.z = 1.f;
		cloud_src->push_back(pt);

		//获得cloud_target点云;
		pt.x = eigenVectorsPCA.col(0)[0];
		pt.y = eigenVectorsPCA.col(0)[1];
		pt.z = eigenVectorsPCA.col(0)[2];
		cloud_target->push_back(pt);

		pt.x = eigenVectorsPCA.col(1)[0];
		pt.y = eigenVectorsPCA.col(1)[1];
		pt.z = eigenVectorsPCA.col(1)[2];
		cloud_target->push_back(pt);

		pt.x = eigenVectorsPCA.col(2)[0];
		pt.y = eigenVectorsPCA.col(2)[1];
		pt.z = eigenVectorsPCA.col(2)[2];
		cloud_target->push_back(pt);

		/*创建点对的对应关系correspondences;*/
		pcl::Correspondences  correspondences;
		pcl::Correspondence   correspondence;
		for (size_t i = 0; i < 3; i++)
		{
			correspondence.index_match = (int)i;
			correspondence.index_query = (int)i;
			correspondences.push_back(correspondence);
		}

		//根据对应关系correspondences计算旋转平移矩阵;
		pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> trans_est;
		
		trans_est.estimateRigidTransformation(*cloud_target, *cloud_src, correspondences, trans);
	}
	std::cout << "Object main direction transformed to canonical coordinate" << std::endl << trans << std::endl;

	Eigen::Matrix4f transform(Eigen::Matrix4f::Identity());
	//transpose is necessary
	transform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
	//transform.block<3, 1>(0, 3) = Eigen::Vector3f::Zero();
	transform.block<3, 1>(0, 3) = -1.f*transform.block<3, 3>(0, 0)*pcaCentroid.head<3>();
	std::cout << "transform: " << std::endl << transform << std::endl;

	Eigen::Affine3f affine_transform(Eigen::Affine3f::Identity());
	affine_transform = eigenVectorsPCA.transpose();
	Eigen::Matrix3f r_inv = eigenVectorsPCA.inverse();
	std::cout << "r_inv:\n" << r_inv << std::endl;
	std::cout << " The rotation to the canonical coordinate could be explained as the base of coordinate:" << std::endl;
	std::cout << "[1 0 0] [x1] [main direction] [x2]" << std::endl;
	std::cout << "[0 1 0]*[y1]=[midd direction]*[y2]" << std::endl;
	std::cout << "[0 0 1] [1x] [norm direction] [z2]" << std::endl;
	std::cout << "R.transporse = R.inverse" << std::endl;

	pcl::PointCloud<PointT>::Ptr transformedCloud(new pcl::PointCloud<PointT>);
	//pcl::transformPointCloud(*normal_cloud, *transformedCloud, trans);
	pcl::transformPointCloud(*normal_cloud, *transformedCloud, affine_transform);
	pcaCentroid = -1.f*pcaCentroid;
	//pcl::demeanPointCloud(*transformedCloud, pcaCentroid, *normal_cloud_demean);


	std::cout << "eigenValuesPCA after: " << std::endl << eigenValuesPCA << std::endl;
	std::cout << "eigenVectorsPCA after: " << std::endl << eigenVectorsPCA << std::endl;
	std::cout << "eigenVectorsPCA.transpose: " << std::endl << eigenVectorsPCA.transpose() << std::endl;
	std::string out_file_name = Utility::get_parent(pointfilepath) + "/" + Utility::get_name_without_ext(pointfilepath) + "_pca.las";
	PointIO::saveLAS<PointT>(out_file_name, transformedCloud, las_offset);

#endif //NORMAL_

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

#ifdef CCLIB

#endif // CCLIB

	system ( "pause" );
	return 1;
}
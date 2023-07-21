
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include "VPFBL.h"
#include "VPFBR.h"
#include "pcl_registration_test.h"

#include "../Submodules/BSC_MLS_PointCloudRegistration-MS/Utility/io/PointCloudIO.h"
#include "../Submodules/BSC_MLS_PointCloudRegistration-MS/Utility/common/Util.h"

#ifdef _DEBUG
#pragma comment(lib, "LASLibd.lib")
#pragma comment(lib, "glogd.lib")
#pragma comment(lib, "pcl_common_debug.lib")
#pragma comment(lib, "pcl_io_debug.lib")

#pragma comment(lib, "libboost_thread-vc141-mt-gd-1_64.lib")
#pragma comment(lib, "pcl_common_debug.lib")
#pragma comment(lib, "pcl_features_debug.lib")
#pragma comment(lib, "pcl_filters_debug.lib")
#pragma comment(lib, "pcl_kdtree_debug.lib")
#pragma comment(lib, "pcl_search_debug.lib")
#pragma comment(lib, "pcl_registration_debug.lib")
#pragma comment(lib, "pcl_sample_consensus_debug.lib")
#pragma comment(lib, "pcl_segmentation_debug.lib")
#pragma comment(lib, "pcl_io_debug.lib")
#pragma comment(lib, "pcl_ml_debug.lib")

#pragma comment(lib, "tbb_debug.lib")
#pragma comment(lib, "opencv_core347d.lib")
#pragma comment(lib, "opencv_highgui347d.lib")
#pragma comment(lib, "opencv_imgproc347d.lib")
#pragma comment(lib, "opencv_imgcodecs347d.lib")
#else
#pragma comment(lib, "LASLib.lib")
#pragma comment(lib, "glog.lib")

#pragma comment(lib, "libboost_thread-vc141-mt-1_64.lib")

#pragma comment(lib, "pcl_io_release.lib")
#pragma comment(lib, "pcl_common_release.lib")
#pragma comment(lib, "pcl_features_release.lib")
#pragma comment(lib, "pcl_filters_release.lib")
#pragma comment(lib, "pcl_kdtree_release.lib")
#pragma comment(lib, "pcl_octree_release.lib")
#pragma comment(lib, "pcl_search_release.lib")
#pragma comment(lib, "pcl_registration_release.lib")
#pragma comment(lib, "pcl_sample_consensus_release.lib")
#pragma comment(lib, "pcl_segmentation_release.lib")
#pragma comment(lib, "pcl_io_release.lib")
#pragma comment(lib, "pcl_ml_release.lib")

#pragma comment(lib, "ceres.lib")

#pragma comment(lib, "tbb.lib")

#pragma comment(lib, "opencv_core347.lib")
#pragma comment(lib, "opencv_highgui347.lib")
#pragma comment(lib, "opencv_imgproc347.lib")
#pragma comment(lib, "opencv_imgcodecs347.lib")
#endif

int VPFBR()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
	// demo data
	//std::string src_path = "E:\\codefiles\\Repositories\\VPFBR-L\\data/1410.pcd";
	//if (pcl::io::loadPCDFile<pcl::PointXYZ>(src_path, *source) == -1)
	//{
	//	PCL_ERROR("Couldn't read file \n");
	//	return (-1);
	//}
	//std::string tar_path = "E:\\codefiles\\Repositories\\VPFBR-L\\data/3540.pcd";
	//if (pcl::io::loadPCDFile<pcl::PointXYZ>(tar_path, *target) == -1)
	//{
	//	PCL_ERROR("Couldn't read file \n");
	//	return (-1);
	//}
	// data from wuhan
	std::string src_path = "E:\\codefiles\\PG-Code\\small_pointcloud\\pointcloud/small1.las";
	//std::string src_path = "E:\\codefiles\\NewVS\\3DHarris\\RegistrationTest\\Data\\Wuhan/Section0.las";
	//std::string src_path = "E:\\codefiles\\PG-Code\\P2M_test_data\\MLS_Pos_Correction_Data\\data12_c_reg_tls_mls/14.las";
	Util::Offset offset_src;
	Util::loadSingleLAS<pcl::PointXYZ>(src_path, source, offset_src);
	std::string tar_path = "E:\\codefiles\\PG-Code\\small_pointcloud\\pointcloud/small2.las";
	//std::string tar_path = "E:\\codefiles\\NewVS\\3DHarris\\RegistrationTest\\Data\\Wuhan/Section61.las";
	//std::string tar_path = "E:\\codefiles\\PG-Code\\P2M_test_data\\MLS_Pos_Correction_Data\\data12_c_reg_tls_mls/MLS_14.las";
	Util::Offset offset_tgt;
	Util::loadSingleLAS<pcl::PointXYZ>(tar_path, target, offset_tgt);
	Util::Offset unify_offset;
	unify_offset = offset_tgt - offset_src;
	for (auto &pt : target->points)
	{
		pt.x += unify_offset.x;
		pt.y += unify_offset.y;
		pt.z += unify_offset.z;
	}
	// VPFB registration
	Eigen::Matrix4f transformation_matrix_src2tgt = Eigen::Matrix4f::Identity();
	// Caution, the order is inversed
	vpfbr::computer_transform_guess(target, source, transformation_matrix_src2tgt);
	std::cout << "transformation_matrix:" << std::endl;
	std::cout << transformation_matrix_src2tgt << std::endl;
	pcl::transformPointCloud(*source, *source, transformation_matrix_src2tgt);
	/*std::string trans_src_path = "E:\\codefiles\\Repositories\\VPFBR-L\\data/1410_trans.pcd";
	pcl::io::savePCDFile<pcl::PointXYZ>(trans_src_path, *source);*/
	//std::string trans_src_path = "E:\\codefiles\\NewVS\\3DHarris\\RegistrationTest\\Data\\Wuhan/Section0_trans.las";
	std::string trans_src_path = "E:\\codefiles\\PG-Code\\small_pointcloud\\outpointcloud/small1_trans.las";
	/*std::string trans_src_path = "E:\\codefiles\\PG-Code\\P2M_test_data\\MLS_Pos_Correction_Data\\data12_c_reg_tls_mls/14_VPFBR.las";*/
	Util::saveLAS<pcl::PointXYZ>(trans_src_path, source, offset_src);
	return 0;
}

//int VPFBL()
//{
//	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
//
//	
//	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_source;
//	voxel_grid_source.setLeafSize(vpfb::LeafSize, vpfb::LeafSize, vpfb::LeafSize);
//	voxel_grid_source.setInputCloud(source);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter_source(new pcl::PointCloud<pcl::PointXYZ>);
//	voxel_grid_source.filter(*cloud_filter_source);
//
//	
//	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_target;
//	voxel_grid_target.setLeafSize(vpfb::LeafSize, vpfb::LeafSize, vpfb::LeafSize);
//	voxel_grid_target.setInputCloud(target);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter_target(new pcl::PointCloud<pcl::PointXYZ>);
//	voxel_grid_target.filter(*cloud_filter_target);
//
//	std::vector<vpfb::facenode> face_vecter_source;
//	vpfb::face_extrate(cloud_filter_source, face_vecter_source);
//	vpfb::describ des_source;
//	vpfb::creat_describ(cloud_filter_source, face_vecter_source, des_source);
//
//	std::vector<vpfb::facenode> face_vecter_target;
//	vpfb::face_extrate(cloud_filter_target, face_vecter_target);
//	vpfb::describ des_target;
//	vpfb::creat_describ(cloud_filter_target, face_vecter_target, des_target);
//
//	float score = compare_describ(des_source, des_target);
//	std::cout << "similarity score: " << score << std::endl;
//	return 0;
//}

int PCL_FPFH()
{
	std::string src_tgt_filepath = "E:\\codefiles\\PG-Code\\small_pointcloud\\pointcloud/";
	std::string src_file = "small1.las";
	std::string tgt_file = "small2.las";
	/*std::string src_tgt_filepath = "E:\\codefiles\\NewVS\\3DHarris\\RegistrationTest\\Data\\Wuhan\\";
	std::string src_file = "Section0_non_ground.las";
	std::string tgt_file = "Section61_non_ground.las";*/
	// Time start (main function)
	time_t start_computation, end_computation, start_total, end_total;
	time(&start_total);
	// READ SOURCE AND TARGET FILES
	std::string src_fullpath = src_tgt_filepath + src_file;
	std::string tgt_fullpath = src_tgt_filepath + tgt_file;
	pcl::PointCloud<pcl::PointXYZ>::Ptr src_original(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_original(new pcl::PointCloud<pcl::PointXYZ>);
	Util::Offset offset_src, offset_tgt, unify_offset;
	Util::loadSingleLAS<pcl::PointXYZ>(src_fullpath, src_original, offset_src);
	Util::loadSingleLAS<pcl::PointXYZ>(tgt_fullpath, tgt_original, offset_tgt);
	unify_offset = offset_tgt - offset_src;
	for (auto &pt : tgt_original->points)
	{
		pt.x += unify_offset.x;
		pt.y += unify_offset.y;
		pt.z += unify_offset.z;
	}
	std::cout << "Src points: " << src_original->points.size() << std::endl;
	std::cout << "Tgt points: " << tgt_original->points.size() << std::endl;
	// Create the filtering object
	float LEAF_SIZE = 0.05f;
	pcl::PointCloud<pcl::PointXYZ>::Ptr src_decimated(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(src_original);
	sor.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
	sor.filter(*src_decimated);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_decimated(new pcl::PointCloud<pcl::PointXYZ>);
	sor.setInputCloud(tgt_original);
	sor.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
	sor.filter(*tgt_decimated);
	std::cerr << "Src PointCloud after decimation: " << src_decimated->width * src_decimated->height
		<< " data points (" << pcl::getFieldsList(*src_decimated) << ")." << std::endl;
	std::cerr << "Tgt PointCloud after decimation: " << tgt_decimated->width * tgt_decimated->height
		<< " data points (" << pcl::getFieldsList(*tgt_decimated) << ")." << std::endl;
	// Filtered point cloud copy
	pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>);
	src = src_decimated;
	tgt = tgt_decimated;
	std::string out_path = "E:\\codefiles\\PG-Code\\small_pointcloud\\outpointcloud";
	//std::string out_path = "E:\\codefiles\\NewVS\\3DHarris\\RegistrationTest\\Data\\Wuhan\\Section0_fpfh.las";
	for (float normal_radius = 0.1; normal_radius <= 2.0; normal_radius += 0.5)
	{
		for (float feature_radius = normal_radius+0.1; feature_radius <= 3.0; feature_radius += 0.5)
		{
			//for (float inlier_dist = 0.05; inlier_dist < 0.2; inlier_dist += 0.05)
			{
				//for (int ransac_iter = 5000; ransac_iter < 8000; ransac_iter += 1000)
				{
					time(&start_computation);
					// Copying the pointwithscale to pointxyz so as visualize the cloud
					pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_src_visualize_temp(new pcl::PointCloud<pcl::PointXYZ>);
					pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_tgt_visualize_temp(new pcl::PointCloud<pcl::PointXYZ>);
					// Find correspondences between keypoints in FPFH space
					pcl::CorrespondencesPtr good_correspondences(new pcl::Correspondences);
					// Obtain the initial transformation matirx by using the key-points
					Eigen::Matrix4f transform;
					pcl_registration_test::compute_Initial_Transformation(src, tgt, transform, keypoints_src_visualize_temp, keypoints_tgt_visualize_temp,
						*good_correspondences, normal_radius, feature_radius, 0.05, 8000);
					std::cout << "Initial Transformation Matrix" << std::endl;
					std::cout << transform << std::endl;
					time(&end_computation);
					double time_elapsed_computation = difftime(end_computation, start_computation);
					std::cout << "Elasped FPFH main function time in seconds: " << time_elapsed_computation << std::endl;
					pcl::PointCloud<pcl::PointXYZ>::Ptr src_initial_output(new pcl::PointCloud<pcl::PointXYZ>);
					pcl::transformPointCloud(*src, *src_initial_output, transform);
					std::string out_path_name = "/" + std::to_string(normal_radius) + "_" + std::to_string(feature_radius) + "_" + std::to_string(0.05) + "_" + std::to_string(8000) + ".las";
					Util::saveLAS<pcl::PointXYZ>(out_path + out_path_name, src_initial_output, offset_src);
				}
			}
		}
	}
	// Time end (main function)
	time(&end_total);
	double time_elapsed_total = difftime(end_total, start_total);
	std::cout << "Elasped total main function time in seconds: " << time_elapsed_total << std::endl;
	return 1;
}

int main(int argc, char** argv)
{
	//VPFBL();
	//VPFBR();
	PCL_FPFH();
	system("pause");
	return 1;
}
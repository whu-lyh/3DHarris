#pragma once

#include <fstream>
#include <iostream>
#include <cstring>
#include <string>
#include <ctime>
#include <vector>
#include <sstream>

#include <pcl/common/angles.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/correspondence.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/pfh.h>
#include <pcl/features/shot_omp.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/default_convergence_criteria.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include "../Submodules/BSC_MLS_PointCloudRegistration-MS/Utility/io/PointCloudIO.h"
#include "../Submodules/BSC_MLS_PointCloudRegistration-MS/Utility/common/Util.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Hyper parameters (room)
/*
#define LEAF_SIZE .01
#define normal_radius 0.05
#define feature_radius 0.05
#define RANSAC_Inlier_Threshold .3
#define RANSAC_Iterations 5000
#define CorrRejDist_Maximum_Distance 2

// ICP hyper parameters
#define ICP_Iterations 5000
#define ICP_TransformationEpsilon 1e-6
#define ICP_EuclideanFitnessEpsilon  1
#define ICP_RANSAC_Inlier_Threshold 0.5
#define ICP_Max_Correspondence_Distance 2

// Parameters for sift computation
#define min_scale .05
#define nr_octaves 8
#define nr_scales_per_octave 3
#define min_contrast 0.05

string src_file = "1189_kinect_v2_scene_1.pcd";
string tgt_file = "1189_kinect_v2_scene_2_rottranslated.pcd"; //1189_kinect_v2_scene_2_rottranslated

//*/
///////////////////////////////////////////////////////////////////////////////////////////////////////
//// Hyper parameters (plate)
//*

// ICP hyper parameters
#define ICP_Iterations 10000
#define ICP_TransformationEpsilon 1e-6
#define ICP_EuclideanFitnessEpsilon  1
#define ICP_RANSAC_Inlier_Threshold 0.001
#define ICP_Max_Correspondence_Distance 0.4

// Parameters for sift computation
#define min_scale .05
#define nr_octaves 8
#define nr_scales_per_octave 4
#define min_contrast 0.05

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Hyper parameters (civic engine)
//#define LEAF_SIZE .01
//#define normal_radius 0.05
//#define feature_radius 0.05
//#define RANSAC_Inlier_Threshold 0.2
//#define RANSAC_Iterations 5000
//#define CorrRejDist_Maximum_Distance 5
//
//// ICP hyper parameters
//#define ICP_Iterations 1000
//#define ICP_TransformationEpsilon 1e-6
//#define ICP_EuclideanFitnessEpsilon  1
//#define ICP_RANSAC_Inlier_Threshold 0.7
//#define ICP_Max_Correspondence_Distance 1.25
//
//// Parameters for sift computation
//#define min_scale .05
//#define nr_octaves 4
//#define nr_scales_per_octave 5
//#define min_contrast 0.25

//string src_file = "Civic_nochange_5Million_trimmed.pcd"
//string tgt_file = "Civic_change_1Million_scaled_trimmed.pcd"

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Hyper parameters (RAV4 engine)
//#define LEAF_SIZE .01
//#define normal_radius 0.05
//#define feature_radius 0.05
//#define RANSAC_Inlier_Threshold 0.2
//#define RANSAC_Iterations 5000
//#define CorrRejDist_Maximum_Distance 5
//
//// ICP hyper parameters
//#define ICP_Iterations 1000
//#define ICP_TransformationEpsilon 1e-6
//#define ICP_EuclideanFitnessEpsilon  1
//#define ICP_RANSAC_Inlier_Threshold 0.7
//#define ICP_Max_Correspondence_Distance 1.25
//
//// Parameters for sift computation
//#define min_scale .05
//#define nr_octaves 4
//#define nr_scales_per_octave 5
//#define min_contrast 0.25

//string src_file = "RAV4_Engine_No_Change_trimmed.pcd"
//string tgt_file = "RAV4_Engine_Change_scaled_trimmed.pcd"

namespace pcl
{
	template<>
	struct SIFTKeypointFieldSelector<PointXYZ>
	{
		inline float
			operator () (const PointXYZ &p) const
		{
			return p.z;
		}
	};
}

namespace pcl_registration_test
{
	void detect_keypoints(pcl::PointCloud <pcl::PointXYZ>::Ptr &points,
		pcl::PointCloud <pcl::PointWithScale>::Ptr &keypoints_out) 
	{
		pcl::SIFTKeypoint <pcl::PointXYZ, pcl::PointWithScale> sift_detect;
		// Use a FLANN-based KdTree to perform neighbourhood searches
		pcl::search::KdTree <pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree <pcl::PointXYZ>);
		sift_detect.setSearchMethod(tree);
		// Set the detection parameters
		sift_detect.setScales(min_scale, nr_octaves, nr_scales_per_octave);
		sift_detect.setMinimumContrast(min_contrast);
		// Set the input
		sift_detect.setInputCloud(points);
		// Detect the keypoints and store them in "keypoints.out"
		sift_detect.compute(*keypoints_out);
	}

	void compute_normals(pcl::PointCloud <pcl::PointXYZ>::Ptr &points,
		pcl::PointCloud <pcl::Normal>::Ptr &normals_out, const float &normal_radius)
	{
		pcl::NormalEstimation <pcl::PointXYZ, pcl::Normal> norm_est;
		// Use a FLANN-based KdTree to perform neighbourhood searches
		norm_est.setSearchMethod(pcl::search::KdTree <pcl::PointXYZ>::Ptr(new pcl::search::KdTree <pcl::PointXYZ>));
		norm_est.setRadiusSearch(normal_radius);
		norm_est.setInputCloud(points);
		norm_est.compute(*normals_out);
	}

	void compute_FPFH_features(pcl::PointCloud <pcl::PointXYZ>::Ptr &points,
		pcl::PointCloud <pcl::Normal>::Ptr &normals,
		pcl::PointCloud <pcl::PointWithScale>::Ptr &keypoints,
		pcl::PointCloud <pcl::FPFHSignature33>::Ptr &descriptors_out, const float &feature_radius)
	{
		pcl::FPFHEstimationOMP <pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
		fpfh_est.setSearchMethod(pcl::search::KdTree <pcl::PointXYZ>::Ptr(new pcl::search::KdTree <pcl::PointXYZ>));
		fpfh_est.setRadiusSearch(feature_radius);
		// copy only XYZ data of keypoints for use in estimating features
		pcl::PointCloud <pcl::PointXYZ>::Ptr keypoints_xyz(new pcl::PointCloud <pcl::PointXYZ>);
		pcl::copyPointCloud(*keypoints, *keypoints_xyz);
		fpfh_est.setSearchSurface(points); // use all points for analyzing local cloud structure 
		fpfh_est.setInputNormals(normals);
		// But only compute features at keypoints
		fpfh_est.setInputCloud(keypoints_xyz);
		fpfh_est.compute(*descriptors_out);
	}

	void compute_PFH_features(pcl::PointCloud <pcl::PointXYZ>::Ptr &cloud,
		pcl::PointCloud <pcl::Normal>::Ptr &normals,
		pcl::PointCloud <pcl::PointWithScale>::Ptr &keypoints,
		pcl::PointCloud <pcl::PFHSignature125>::Ptr &descriptors_out, const float &feature_radius)
	{
		// copy only XYZ data of keypoints for use in estimating features
		pcl::PointCloud <pcl::PointXYZ>::Ptr keypoints_xyz(new pcl::PointCloud <pcl::PointXYZ>);
		pcl::copyPointCloud(*keypoints, *keypoints_xyz);
		// Create the PFH estimation class, and pass the input dataset+normals to it
		pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
		pfh.setInputCloud(keypoints_xyz);
		pfh.setSearchSurface(cloud); // use all points for analyzing local cloud structure 
		pfh.setInputNormals(normals);
		// alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);
		// Create an empty kdtree representation, and pass it to the PFH estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		pfh.setSearchMethod(tree);
		// Use all neighbors in a sphere of radius 5cm
		// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
		pfh.setRadiusSearch(feature_radius);
		// Compute the features
		pfh.compute(*descriptors_out);
	}

	void compute_SHOT_features(pcl::PointCloud <pcl::PointXYZ>::Ptr &cloud,
		pcl::PointCloud <pcl::Normal>::Ptr &normals,
		pcl::PointCloud <pcl::PointWithScale>::Ptr &keypoints,
		pcl::PointCloud <pcl::SHOT352>::Ptr &descriptors_out, const float &feature_radius)
	{
		// copy only XYZ data of keypoints for use in estimating features
		pcl::PointCloud <pcl::PointXYZ>::Ptr keypoints_xyz(new pcl::PointCloud <pcl::PointXYZ>);
		pcl::copyPointCloud(*keypoints, *keypoints_xyz);
		// Create the PFH estimation class, and pass the input dataset+normals to it
		pcl::SHOTEstimationOMP <pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shotEstimation;
		shotEstimation.setInputCloud(keypoints_xyz);
		shotEstimation.setSearchSurface(cloud); // use all points for analyzing local cloud structure 
		shotEstimation.setInputNormals(normals);
		// alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);
		// Create an empty kdtree representation, and pass it to the PFH estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		shotEstimation.setSearchMethod(tree);
		// Use all neighbors in a sphere of radius radius
		// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
		shotEstimation.setRadiusSearch(feature_radius);
		// Compute the features
		shotEstimation.compute(*descriptors_out);
	}

	void findCorrespondences_FPFH(const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_src,
		const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_tgt,
		pcl::Correspondences &all_correspondences) 
	{
		pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
		est.setInputSource(fpfhs_src);
		est.setInputTarget(fpfhs_tgt);
		est.determineReciprocalCorrespondences(all_correspondences, 5.0);
	}

	void findCorrespondences_PFH(const pcl::PointCloud<pcl::PFHSignature125>::Ptr &fpfhs_src,
		const pcl::PointCloud<pcl::PFHSignature125>::Ptr &fpfhs_tgt,
		pcl::Correspondences &all_correspondences) 
	{
		pcl::registration::CorrespondenceEstimation<pcl::PFHSignature125, pcl::PFHSignature125> est;
		est.setInputSource(fpfhs_src);
		est.setInputTarget(fpfhs_tgt);
		est.determineReciprocalCorrespondences(all_correspondences);
	}

	void findCorrespondences_SHOT(const pcl::PointCloud<pcl::SHOT352>::Ptr &fpfhs_src,
		const pcl::PointCloud<pcl::SHOT352>::Ptr &fpfhs_tgt,
		pcl::Correspondences &all_correspondences) 
	{
		pcl::registration::CorrespondenceEstimation<pcl::SHOT352, pcl::SHOT352> est;
		est.setInputSource(fpfhs_src);
		est.setInputTarget(fpfhs_tgt);
		est.determineReciprocalCorrespondences(all_correspondences);
	}

	void rejectBadCorrespondences(const pcl::CorrespondencesPtr &all_correspondences,
		const pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints_src,
		const pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints_tgt,
		pcl::Correspondences &remaining_correspondences, const float &RANSAC_Inlier_Threshold, const int &RANSAC_Iterations)
	{
		// Thresholding the distances bad correspondence rejector
		//pcl::registration::CorrespondenceRejectorDistance rej;
		//rej.setInputSource<pcl::PointXYZ>(keypoints_src);
		//rej.setInputTarget<pcl::PointXYZ>(keypoints_tgt);
		//rej.setMaximumDistance(CorrRejDist_Maximum_Distance);    // hyperparameter
		//rej.setInputCorrespondences(all_correspondences);
		//rej.getCorrespondences(remaining_correspondences);

		// copy only XYZRGB data of keypoints for use in estimating features
		pcl::PointCloud <pcl::PointXYZ>::Ptr keypoints_src_xyz(new pcl::PointCloud <pcl::PointXYZ>);
		pcl::PointCloud <pcl::PointXYZ>::Ptr keypoints_tgt_xyz(new pcl::PointCloud <pcl::PointXYZ>);
		pcl::copyPointCloud(*keypoints_src, *keypoints_src_xyz);
		pcl::copyPointCloud(*keypoints_tgt, *keypoints_tgt_xyz);

		// RandomSampleConsensus bad correspondence rejector
		pcl::registration::CorrespondenceRejectorSampleConsensus <pcl::PointXYZ> correspondence_rejector;
		correspondence_rejector.setInputSource(keypoints_src_xyz);
		correspondence_rejector.setInputTarget(keypoints_tgt_xyz);
		correspondence_rejector.setInlierThreshold(RANSAC_Inlier_Threshold);
		correspondence_rejector.setMaximumIterations(RANSAC_Iterations);
		correspondence_rejector.setRefineModel(true);//false
		correspondence_rejector.setInputCorrespondences(all_correspondences);
		correspondence_rejector.getCorrespondences(remaining_correspondences);
	}

	void compute_Initial_Transformation(pcl::PointCloud<pcl::PointXYZ>::Ptr &src,
		pcl::PointCloud<pcl::PointXYZ>::Ptr &tgt, Eigen::Matrix4f &transform,
		pcl::PointCloud<pcl::PointXYZ>::Ptr & keypoints_src_visualize_temp,
		pcl::PointCloud<pcl::PointXYZ>::Ptr & keypoints_tgt_visualize_temp,
		pcl::Correspondences & good_correspondences, const float &normal_radius, const float &feature_radius, const float &RANSAC_Inlier_Threshold, const int &RANSAC_Iterations)
	{
		// ESTIMATING KEY POINTS
		pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_src(new pcl::PointCloud<pcl::PointWithScale>);
		pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_tgt(new pcl::PointCloud<pcl::PointWithScale>);
		detect_keypoints(src, keypoints_src);
		std::cout << "Num of SIFT points in the src are " << keypoints_src->points.size() << std::endl;
		detect_keypoints(tgt, keypoints_tgt);
		std::cout << "Num of SIFT points in the tgt are " << keypoints_tgt->points.size() << std::endl;
		// ESTIMATING PFH FEATURE DESCRIPTORS AT KEYPOINTS AFTER COMPUTING NORMALS
		pcl::PointCloud <pcl::Normal>::Ptr src_normals(new pcl::PointCloud<pcl::Normal>);
		pcl::PointCloud <pcl::Normal>::Ptr tgt_normals(new pcl::PointCloud<pcl::Normal>);
		compute_normals(src, src_normals, normal_radius);
		compute_normals(tgt, tgt_normals, normal_radius);
		// FPFH Estimation
		pcl::PointCloud <pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>);;
		pcl::PointCloud <pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>);;
		//const float feature_radius = 0.2;		// adjust this hyperparameter
		compute_FPFH_features(src, src_normals, keypoints_src, fpfhs_src, feature_radius);
		compute_FPFH_features(tgt, tgt_normals, keypoints_tgt, fpfhs_tgt, feature_radius);

		// PFH Estimation
		//pcl::PointCloud <pcl::PFHSignature125>::Ptr fpfhs_src(new pcl::PointCloud<pcl::PFHSignature125>);;
		//pcl::PointCloud <pcl::PFHSignature125>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::PFHSignature125>);;
		//const float feature_radius = 0.2;		// adjust this hyperparameter
		//compute_PFH_features(src, src_normals, keypoints_src,  fpfhs_src);
		//compute_PFH_features(tgt, tgt_normals, keypoints_tgt,  fpfhs_tgt);

		// SHOT Estimation
		//pcl::PointCloud <pcl::SHOT352>::Ptr fpfhs_src(new pcl::PointCloud<pcl::SHOT352>);
		//pcl::PointCloud <pcl::SHOT352>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::SHOT352>);
		//const float feature_radius = 0.2;		// adjust this hyperparameter
		//compute_SHOT_features(src, src_normals, keypoints_src,  fpfhs_src);
		//compute_SHOT_features(tgt, tgt_normals, keypoints_tgt,  fpfhs_tgt);
		std::cout << "End of compute_FPFH_features! " << std::endl;
		// Copying the pointwithscale to pointxyz so as visualize the cloud
		pcl::copyPointCloud(*keypoints_src, *keypoints_src_visualize_temp);
		pcl::copyPointCloud(*keypoints_tgt, *keypoints_tgt_visualize_temp);
		std::cout << "SIFT points in the keypoints_src_visualize_temp are " << keypoints_src_visualize_temp->points.size() << std::endl;
		std::cout << "SIFT points in the keypoints_tgt_visualize_temp are " << keypoints_tgt_visualize_temp->points.size() << std::endl;
		std::string out_path = "E:\\codefiles\\PG-Code\\small_pointcloud\\outpointcloud/small1_kps.las";
		Util::Offset offset_src;
		Util::saveLAS<pcl::PointXYZ>(out_path, keypoints_src_visualize_temp, offset_src);
		out_path = "E:\\codefiles\\PG-Code\\small_pointcloud\\outpointcloud/small2_kps.las";
		Util::saveLAS<pcl::PointXYZ>(out_path, keypoints_tgt_visualize_temp, offset_src);
		// Find correspondences between keypoints in FPFH space
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences);
		findCorrespondences_FPFH(fpfhs_src, fpfhs_tgt, *all_correspondences);
		//findCorrespondences_PFH(fpfhs_src, fpfhs_tgt, *all_correspondences);
		//findCorrespondences_SHOT(fpfhs_src, fpfhs_tgt, *all_correspondences);
		std::cout << "End of findCorrespondences! " << std::endl;
		std::cout << "All correspondences size: " << all_correspondences->size() << std::endl;
		// Reject correspondences based on their XYZ distance
		rejectBadCorrespondences(all_correspondences, keypoints_src, keypoints_tgt, good_correspondences,
			RANSAC_Inlier_Threshold, RANSAC_Iterations);
		//rejectBadCorrespondences(all_correspondences, keypoints_src, keypoints_tgt, good_correspondences);
		std::cout << "End of rejectBadCorrespondences! " << std::endl;
		std::cout << "Good correspondences size: " << good_correspondences.size() << std::endl;
		pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> trans_est;
		trans_est.estimateRigidTransformation(*keypoints_src_visualize_temp, *keypoints_tgt_visualize_temp, good_correspondences, transform);
	}
}
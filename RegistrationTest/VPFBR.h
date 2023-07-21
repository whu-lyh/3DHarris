#pragma once

#include <iostream>
#include <vector>
#include <time.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Eigen/Dense>

namespace vpfbr{
	typedef struct voxelnode
	{
		float centry_x;
		float centry_y;
		float centry_z;
		float normal_x;
		float normal_y;
		float normal_z;
		int voxel_point_size;
		bool is_allocate;
		pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud_ptr;
	}voxelnode;

	typedef struct facenode
	{
		float average_centry_x;
		float average_centry_y;
		float average_centry_z;
		float average_normal_x;
		float average_normal_y;
		float average_normal_z;
		float face_point_size;
		bool is_allocate;
		std::vector<voxelnode> voxelgrothnode;
	}facenode;

	typedef struct face_base
	{
		int index1;
		int index2;
		float angel;
	}face_base;

	typedef struct face_three
	{
		int index1;
		int index2;
		int index3;
	}face_three;

	typedef struct transform_q_t
	{
		float qw;
		float qx;
		float qy;
		float qz;
		float tx;
		float ty;
		float tz;
		bool is_allocate;
	}transform_q_t;

	typedef struct transform_score
	{
		Eigen::Matrix4f transformation_matrix;
		float score;
	}transform_score;

	typedef struct pair_face
	{
		int index1;
		int index2;
		float important;
		Eigen::Vector3f point1;
		Eigen::Vector3f normal1;
		Eigen::Vector3f point2;
		Eigen::Vector3f normal2;
	}pair_face;

	typedef struct pair_point
	{
		Eigen::Vector3f point1;
		Eigen::Vector3f point2;
	}pair_point;

	inline void ceres_refine(Eigen::Matrix4f &new_transformation_matrix, std::vector<pair_face> &pair_face_vecter);

	inline void average_normal(Eigen::Vector3f &rotaionvector1, Eigen::Vector3f &rotaionvector2, std::vector<transform_q_t> &matrixvector);

	inline float compute_normal_angel(float normal_x1, float normal_y1, float normal_z1, float normal_x2, float normal_y2, float normal_z2);

	inline bool compare_normal(float normal_x1, float normal_y1, float normal_z1, float normal_x2, float normal_y2, float normal_z2, float normal_vector_threshold);

	inline bool compare_plane(float normal_x1, float normal_y1, float normal_z1, float centry_x1, float centry_y1, float centry_z1, float normal_x2, float normal_y2, float normal_z2, float centry_x2, float centry_y2, float centry_z2, float parameter_l, float parameter_k);

	inline void range_face(std::vector<facenode> &face_vecter);

	inline void select_base(std::vector<face_base> &base_vecter, std::vector<facenode> &face_vecter);

	inline void face_extrate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, std::vector<facenode> &face_vecter, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sub);

	inline float quick_verify(Eigen::Matrix4f &transformation_matrix, std::vector<facenode> &face_vecter1, std::vector<facenode> &face_vecter2);

	inline float fine_verify(Eigen::Matrix4f &transformation_matrix, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_target);

	inline void computer_transform(std::vector<Eigen::Matrix4f> &transformation_vecter, int index11, int index12, int index21, int index22, std::vector<facenode> &face_vecter1, std::vector<facenode> &face_vecter2);

	inline void range_cluster(std::vector<std::vector<transform_q_t> > &transform_q_t_vector_cluster);

	inline void transform_cluster(std::vector<transform_q_t> &transform_q_t_vecter, std::vector<transform_q_t> &fine_transform_q_t_vecter);

	inline void score_range(std::vector<transform_score> &cluster_transformation_vecter);

	void computer_transform_guess(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target, Eigen::Matrix4f &best_transformation_matrix);
}
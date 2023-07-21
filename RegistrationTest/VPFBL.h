//#pragma once
//
//#include <iostream>
//#include <vector>
//#include <time.h>
//#include <math.h>
//
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//
//
//namespace vpfb{
//
//	//point cloud down sampling
//	float LeafSize = 0.2;
//
//	//parameter for plane feature extraction and fusion
//	//l and k
//	float parameter_l1 = 1.0;
//	float parameter_l2 = 1.0;
//	float parameter_l3 = 1.0;
//	float parameter_k1 = 8.0;
//	float parameter_k2 = 4.0;
//	float parameter_k3 = 2.0;
//	//threshold of normal vector
//	float normal_vector_threshold1 = 5.0;
//	float normal_vector_threshold2 = 10.0;
//	float normal_vector_threshold3 = 15.0;
//	//voxel size
//	float face_voxel_size = 1.0;
//	//if the point in voxel is less than this value, it will not be processed
//	float voxel_point_threshold = 10;
//	//threshold of plane curvature
//	float curvature_threshold = 0.08;
//	//maximum plane distance
//	float max_plane_distance = 5.0;
//	//select the size of planes
//	float select_plane_size = 5;
//
//	//Combination plane features and augmented descriptors parameters
//	//Select main directions number
//	int miannum = 4;
//	//length of augmented descriptors
//	int alonglength = 100;
//	//interval of the block
//	float alongresolution = 0.5;
//	//parameters of angle difference weight and the distance difference weight
//	float angelthreashold = 5.0;
//	float radiuthreashold = 5.0;
//	float downto = 0.75;
//	//feature similarity weight of combination plane features
//	float weight_combination_plane_features = 0.25;
//	//feature similarity weight of augmented descriptors
//	float weight_augmented_descriptors = 0.5;
//
//	typedef struct voxelnode
//	{
//		float centry_x;
//		float centry_y;
//		float centry_z;
//		float normal_x;
//		float normal_y;
//		float normal_z;
//		int voxel_point_size;
//		bool is_allocate;
//		pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud_ptr;
//	}voxelnode;
//
//	typedef struct facenode
//	{
//		float average_centry_x;
//		float average_centry_y;
//		float average_centry_z;
//		float average_normal_x;
//		float average_normal_y;
//		float average_normal_z;
//		float face_point_size;
//		bool is_allocate;
//		std::vector<voxelnode> voxelgrothnode;
//	}facenode;
//
//	typedef struct describ
//	{
//		std::vector<float> theta;
//		std::vector<float> radiu;
//		std::vector<bool> maindir;
//		std::vector<int> size;
//		std::vector<std::vector<float> > distribute;
//	}describ;
//
//	float compute_angel(float x, float y);
//
//	float compute_normal_angel(float normal_x1, float normal_y1, float normal_z1, float normal_x2, float normal_y2, float normal_z2);
//
//	bool compare_normal(float normal_x1, float normal_y1, float normal_z1, float normal_x2, float normal_y2, float normal_z2, float normal_vector_threshold);
//
//	bool compare_plane(float normal_x1, float normal_y1, float normal_z1, float centry_x1, float centry_y1, float centry_z1, float normal_x2, float normal_y2, float normal_z2, float centry_x2, float centry_y2, float centry_z2, float parameter_l, float parameter_k);
//
//	void range_face(std::vector<facenode> &face_vecter);
//
//	void select_face(std::vector<facenode> &face_vecter);
//
//	void face_extrate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, std::vector<facenode> &face_vecter);
//
//	void creat_describ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<facenode> &face_vecter, describ &des);
//
//	float angel_subtract(float minuend, float subtractor);
//
//	float angel_distance(float angels, float angelt);
//
//	float compare_describ(describ &dess, describ &dest);
//
//} // VPFBL
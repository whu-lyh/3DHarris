// Copyright (c) 2018 WuHan University. All Rights Reserved
// @author xhzou_whu@foxmail.com
// @date 2018/10/04
// @brief point cloud io

// Copyright (c) 2020 WuHan University. All Rights Reserved
// @author yhaoli@whu.edu.cn
// @date 2020/04/02
// @brief utility

#pragma once

#include "basicLibs.h"
#include "Utility.h"

namespace PointIO
{
	template <typename T>
	double computeCloudResolution(const typename pcl::PointCloud<T>::ConstPtr& cloud);

	template <typename T>
	Utility::Bound getBoundBox (const typename pcl::PointCloud<T>::Ptr& cloud);

	template <typename T>
	 bool loadSingleLAS(const std::string& filename, const typename pcl::PointCloud<T>::Ptr& cloud, Utility::Offset& las_offset);

	template <typename T>
	 bool loadLASFromFolder(const std::string& filepath, const typename pcl::PointCloud<T>::Ptr& cloud);

	template <typename T>
	extern  bool saveLAS ( const std::string& filepath, const typename pcl::PointCloud<T>::Ptr& cloud, const Utility::Offset& offset = Offset () );

	template <typename T>
	 bool loadPCD(const std::string& filename, const typename pcl::PointCloud<T>::Ptr& cloud);

	template <typename T>
	bool savePCD ( const std::string& filename, const typename pcl::PointCloud<T>::Ptr& cloud, const Utility::Offset& offset = Offset () );

	template <typename T>
	 bool loadSPT(const std::string& filename, const typename pcl::PointCloud<T>::Ptr& cloud, Utility::Offset& offset);

	template <typename T>
	 void outputRegPointCloud(const std::string& output_filename, const typename pcl::PointCloud<T>::Ptr& src_cloud,
		const typename pcl::PointCloud<T>::Ptr& tar_cloud, const Eigen::Matrix4f& tar2src, const Utility::Offset& offset);

} // namespace Trajectory

#define TEMPLATE_POINTCLOUD_IO
#include "PointCloudIO.cpp"

//this define is used for finding the PointCloudIO.cpp,due to the function defined in namespace PointIO is a template function ,so this kind of header-file connection is necessary
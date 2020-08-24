#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

#include "basicLibs.h"
#include "PointCloudIO.h"

#pragma comment(lib, "pcl_segmentation_release.lib")

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointIndicesPtr ground(new pcl::PointIndices);

	pcl::PCDReader reader;
	//reader.read<pcl::PointXYZ>("biandianzhan.pcd", *cloud);

	Utility::Offset las_offset;
	if ( PointIO::loadSingleLAS<pcl::PointXYZ>("biandianzhan-origin-full-voxel0.4.las", cloud, las_offset) )
	{
		std::cout << cloud->points.size() << std::endl;
		std::cout << "las file load successfully" << std::endl;
	}

	std::cerr << "Cloud before filtering: " << std::endl;
	std::cerr << *cloud << std::endl;

	// 创建形态学滤波器对象
	pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
	pmf.setInputCloud(cloud);

	// 设置过滤点最大的窗口尺寸
	pmf.setMaxWindowSize(2);

	// 设置计算高度阈值的斜率值
	pmf.setSlope(1.0f);

	// 设置初始高度参数被认为是地面点
	pmf.setInitialDistance(0.1f);

	// 设置被认为是地面点的最大高度
	pmf.setMaxDistance(1.0f);
	pmf.extract(ground->indices);

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(ground);
	extract.filter(*cloud_filtered);

	std::cerr << "Ground cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_filtered_RGB->resize(cloud_filtered->size());
	cloud_filtered_RGB->is_dense = FALSE;

	for ( size_t i = 0; i < cloud_filtered->points.size(); ++i )
	{
		cloud_filtered_RGB->points[i].x = cloud_filtered->points[i].x;
		cloud_filtered_RGB->points[i].y = cloud_filtered->points[i].y;
		cloud_filtered_RGB->points[i].z = cloud_filtered->points[i].z;

		cloud_filtered_RGB->points[i].r = 0;
		cloud_filtered_RGB->points[i].g = 255;
		cloud_filtered_RGB->points[i].b = 0;
	}

	pcl::io::savePCDFileBinary("./cloud_groud.pcd", *cloud_filtered_RGB);

	// 提取非地面点
	extract.setNegative(true);
	extract.filter(*cloud_filtered);

	std::cerr << "Object cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;

	pcl::io::savePCDFileBinary("./No_groud.pcd", *cloud_filtered);

	system("pause");
	return (0);
}
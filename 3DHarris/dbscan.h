// Copyright (c) 2023 WuHan University. All Rights Reserved
// @author yhaoli@whu.edu.cn
// @date 2023/01/14
// @brief DBSCAN class

#ifndef UTILITY_DBSCAN_H
#define UTILITY_DBSCAN_H

// STL
#include <cmath>
#include <vector>
#include <string>
// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
// Local
#include "Utility.h"

namespace Util
{
	// class: vanilla version of DBSCAN, that will cost more time for large scale point clouds
	class DBSCAN
	{
		class ClusterPoint: public Utility::Point3d
		{
		public:
			int cluster_id;
			ClusterPoint()
			{
				cluster_id = -1;
			}
		};

	public:
		// cluster margin distance, minimal neighborhood points
		DBSCAN(const float &Eps, const int &MinPts)
			:m_Eps(Eps),m_MinPts(MinPts)
		{};
		~DBSCAN(){m_Points.clear();}

		void empty(){m_Points.clear();}
		// set function
		// transform the input point cloud to custom point cloud type
		void setInputCloud(pcl::PointCloud<pcl::PointXYZ> &cloud);
		// core function: traverse each point and cluster based on distance
		void clustering();
		// get function
		// get the cluster of each point
		void getIDX(std::vector<int> &idx);
		int getClusterNumber();
		int getBiggestCluster();
		// log and save
		void print();
		int save2files(const std::string &outputfile);

	private:
		// return the neighboring points located inside m_Eps^2
		std::vector<int> calculateCluster(ClusterPoint &point);
		// begin clustering from the seed point with a certain id
		bool expandCluster(ClusterPoint &point, const int &assign_id);
		inline double calculateDistance(ClusterPoint &pointCore, ClusterPoint &pointTarget);

	public:
		// inside point container member
		std::vector<ClusterPoint> m_Points;
		// minimal points number of cluster
		// usage1: judging whether current point is a noise
		int m_MinPts;
		// minimal Euclidean distance for separate cluster point cloud
		float m_Eps;
		// total cluster number
		int m_ClusterNums;
	};
}

#endif // UTILITY_DBSCAN_H

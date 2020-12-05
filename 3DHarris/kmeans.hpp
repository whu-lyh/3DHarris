
/*
	Copyright (c) 2020 WuHan University. All Rights Reserved
	First create: 2020/12/05 13:27
	Detail: Inherited from the kmeans class designed from pcl
	Author: liyuhao
	Email: yhaoli@whu.edu.cn
*/

#pragma once

#include <pcl/ml/kmeans.h>

class KmeansPlus: public pcl::Kmeans
{
public:
	KmeansPlus(unsigned int num_points, unsigned int num_dimensions):pcl::Kmeans::Kmeans(num_points, num_dimensions){};

	~KmeansPlus(){};

	ClustersToPoints getclusters2points(){ return clusters_to_points_; }

	PointsToClusters getpoints2clusters(){ return points_to_clusters_; }
};
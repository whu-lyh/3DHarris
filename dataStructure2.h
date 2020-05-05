#pragma once
//  dataStructure.h
//  para
//
//  Created by 虞敏 on 2020/4/20.
//  Copyright © 2020 Moyna. All rights reserved.
//
#pragma once
#ifndef dataStructure_h
#define dataStructure_h


#include <vector>
#include <list>
#include <string>


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


using namespace std;

namespace pmProcessUrban
{
	struct CenterPoint_ym
	{
		double x;
		double y;
		double z;
		CenterPoint_ym ( double x = 0, double y = 0, double z = 0 ) :
			x ( x ), y ( y ), z ( z )
		{
			z = 0.0;
			x = y = 0.0;
		}

	};

	struct Bounds_ym
	{
		double min_x;
		double min_y;
		double min_z;
		double max_x;
		double max_y;
		double max_z;
		Bounds_ym ()
		{
			min_x = min_y = min_z = max_x = max_y = max_z = 0.0;
		}
	};

	struct Grid
	{
		bool is_empty;
		Grid ()
		{
			is_empty = true;
		}
	};

	struct Voxel
	{
		vector<int>point_id;
		float min_z;
		float max_z;
		float dertaz;
		float min_z_x;//格网最低点的X坐标;
		float min_z_y;//格网最低点的y坐标;
		float NeighborMin_z;
		int PointsNumber;
		float mean_z;

		Voxel ()
		{
			min_z = min_z_x = min_z_y = NeighborMin_z = mean_z = 0.f;
			PointsNumber = 1;
			dertaz = 0.0;
		}
	};


	//获取边界
	void getCloudBound ( const pcl::PointCloud<pcl::PointXYZ> & cloud, Bounds_ym & bound )
	{
		double min_x = cloud [0].x;
		double min_y = cloud [0].y;
		double min_z = cloud [0].z;
		double max_x = cloud [0].x;
		double max_y = cloud [0].y;
		double max_z = cloud [0].z;

		for ( int i = 0; i < cloud.size (); i++ )
		{
			//获取边界
			if ( min_x > cloud.points [i].x )
				min_x = cloud.points [i].x;
			if ( min_y > cloud.points [i].y )
				min_y = cloud.points [i].y;
			if ( min_z > cloud.points [i].z )
				min_z = cloud.points [i].z;
			if ( max_x < cloud.points [i].x )
				max_x = cloud.points [i].x;
			if ( max_y < cloud.points [i].y )
				max_y = cloud.points [i].y;
			if ( max_z < cloud.points [i].z )
				max_z = cloud.points [i].z;
		}
		bound.min_x = min_x;
		bound.max_x = max_x;
		bound.min_y = min_y;
		bound.max_y = max_y;
		bound.min_z = min_z;
		bound.max_z = max_z;
	}

	//获取中心和边界
	void getBoundAndCenter ( const pcl::PointCloud<pcl::PointXYZ> & cloud, Bounds_ym & bound, CenterPoint_ym& centerPoint )
	{
		double min_x = cloud [0].x;
		double min_y = cloud [0].y;
		double min_z = cloud [0].z;
		double max_x = cloud [0].x;
		double max_y = cloud [0].y;
		double max_z = cloud [0].z;

		double cx = 0, cy = 0, cz = 0;

		for ( int i = 0; i < cloud.size (); i++ )
		{
			//获取边界
			if ( min_x > cloud.points [i].x )
				min_x = cloud.points [i].x;
			if ( min_y > cloud.points [i].y )
				min_y = cloud.points [i].y;
			if ( min_z > cloud.points [i].z )
				min_z = cloud.points [i].z;
			if ( max_x < cloud.points [i].x )
				max_x = cloud.points [i].x;
			if ( max_y < cloud.points [i].y )
				max_y = cloud.points [i].y;
			if ( max_z < cloud.points [i].z )
				max_z = cloud.points [i].z;


			cx += cloud.points [i].x / cloud.size ();
			cy += cloud.points [i].y / cloud.size ();
			cz += cloud.points [i].z / cloud.size ();
		}
		bound.min_x = min_x;
		bound.max_x = max_x;
		bound.min_y = min_y;
		bound.max_y = max_y;
		bound.min_z = min_z;
		bound.max_z = max_z;


		centerPoint.x = cx;
		centerPoint.y = cy;
		centerPoint.z = cz;
	}

	void GetSubsetBoundary ( pcl::PointCloud<pcl::PointXYZ>::Ptr & plane_wall_cloud,
							 vector<int> & index, Bounds_ym & bound )
	{
		//构建点云
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud ( new pcl::PointCloud<pcl::PointXYZ> );
		for ( int i = 0; i < index.size (); i++ )
		{
			temp_cloud->push_back ( plane_wall_cloud->points [index [i]] );
		}
		getCloudBound ( *temp_cloud, bound );
	}

}

#endif /* dataStructure_h */

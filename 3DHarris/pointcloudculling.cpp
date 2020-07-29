#pragma once
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <chrono>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "pointcloudculling.h"
#include "PointCloudIO.h"
#include "PointType.h"

//WUHAN DATASET
bool loadPosTLin ( const std::string& filename, Utility::PoseVec& poses )
{
	std::ifstream traj_file;
	traj_file.open ( filename );
	int i = 0;
	while ( !traj_file.eof () )
	{
		int ImNum, Yearstamp, Monthstamp, Daystamp, Hourstamp, Minutestamp, Secoundstamp, MicroSecoundstamp;
		double  GPSTime, Xcoordiate, Ycoordiate, Altitude, Roll, Pitch, Heading;
		traj_file >> ImNum >> GPSTime >> Yearstamp >> Monthstamp >> Daystamp >> Hourstamp >> Minutestamp >> Secoundstamp >> MicroSecoundstamp
			>> Xcoordiate >> Ycoordiate >> Altitude >> Heading >> Pitch >> Roll;

		if ( i > 1 )
		{//load all trajectory pose information
			if ( std::fabs ( GPSTime - poses [poses.size () - 1].GPSTime ) > 10000.0 || ( GPSTime - poses [poses.size () - 1].GPSTime ) <= 0.003 )
			{
				++i;
				continue;
			}
		}
		poses.emplace_back ( Utility::Pose ( GPSTime, Xcoordiate, Ycoordiate, Altitude, Heading, Pitch, Roll ) );

		++i;
	}
	traj_file.close ();
	std::cout << "Succeed to load trajectory file: " << filename;
	return true;
}

int main ( int argc, char* argv [] )
{
	std::chrono::high_resolution_clock::time_point t1report = std::chrono::high_resolution_clock::now ();
	//load trajectory
	std::string filename = "D:/data/wuhangi/traj/2-east-iScan-Pos-1.lin";
	std::string pointfilepath = "D:/data/wuhangi/pointcloud/2-east-iScan-Pcd-1_part0.las";
	
	Utility::PoseVec vec_pose;

	loadPosTLin ( filename, vec_pose );

	//trajectory point cloud
	boost::shared_ptr<pcl::PointCloud<pcl::PointXY>> traj_cloud ( new pcl::PointCloud<pcl::PointXY> () );
	//for ( auto pose : vec_pose ) 
	//could not use openmp
	for(int i=0;i<vec_pose.size();++i)
	{
		pcl::PointXY pt;
		pt.x = vec_pose[i].x;
		pt.y = vec_pose[i].y;
		traj_cloud->push_back ( pt );
	}

	pcl::KdTreeFLANN<pcl::PointXY>::Ptr kdtree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXY>> ();
	kdtree->setInputCloud ( traj_cloud );

	pcl::PointCloud<PointXYZINTF>::Ptr input_cloud = boost::make_shared<pcl::PointCloud<PointXYZINTF>> ();
	pcl::PointCloud<PointXYZINTF>::Ptr output_cloud = boost::make_shared<pcl::PointCloud<PointXYZINTF>> ();
	Utility::Offset las_offset;
	PointIO::loadSingleLAS<PointXYZINTF> ( pointfilepath, input_cloud, las_offset );

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	for ( auto pt : input_cloud->points ) 
	{
		pcl::PointXY search_pt;
		search_pt.x = pt.x + las_offset.x;
		search_pt.y = pt.y + las_offset.y;

		if ( kdtree->nearestKSearch ( search_pt, 1, pointIdxRadiusSearch, pointRadiusSquaredDistance ) )
		{
			if ( pointIdxRadiusSearch.empty () ) 
			{
				std::cout << "Point index extracted failed";
				continue;
			}

			if ( pointRadiusSquaredDistance.empty () ) 
			{
				std::cout << "Distance hasn't been parsed correctly";
				continue;
			}

			if ( std::sqrt ( pointRadiusSquaredDistance [0] ) < 100 ) 
			{
				output_cloud->push_back ( pt );
			}
		}
	}
	
	std::string outfilepath = Utility::get_parent ( pointfilepath ) + "/" + Utility::get_name_without_ext ( pointfilepath ) + "_df.las";
	std::cout << "origin pts:\t" << input_cloud->size () << " left pts: \t" << output_cloud->size ();
	PointIO::saveLAS<PointXYZINTF> ( outfilepath, output_cloud, las_offset );

	std::chrono::high_resolution_clock::time_point t2treport = std::chrono::high_resolution_clock::now ();
	std::chrono::duration<double> t12report = std::chrono::duration_cast<std::chrono::duration<double>>( t2treport - t1report );
	std::cout << "Out put the regist Summary file, time cost: " << t12report.count () << "s" << std::endl;
	system ( "pause" );
	return 0;
}
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

#define DISTANCE 80

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

template <typename PointT>
int selectTraject ( const boost::shared_ptr<pcl::PointCloud<PointT>> &cloud, std::vector<Utility::PoseVec>& vvec_poses )
{
	int traj_idx = -1;
	{//binary divided to detect whether the trajecetory matches the point cloud, here the time should be monotonicial in MLS point cloud
		int leftnumber = 0, rightnumber = cloud->points.size ();
		while ( leftnumber < rightnumber )
		{
			const auto& pt = cloud->points [leftnumber];
			double gps_time = pt.gps_time;

			double min_gps_time = 0, max_gps_time = 0;

			int posesize = vvec_poses.size ();
			for ( int i = 0; i < posesize; ++i )
			{
				min_gps_time = vvec_poses [i][0].GPSTime;
				max_gps_time = vvec_poses [i] [vvec_poses[i].size () - 1].GPSTime;
				if ( gps_time >= min_gps_time && gps_time < max_gps_time )
				{
					traj_idx = i;
					break;
				}
			}

			if ( traj_idx != -1 ) break;
			//still detect nothing, and here move to the 10th detect point and loop, max detect iteration is 10
			leftnumber += rightnumber / 10;
		}
	}

	//skip this function if this point cloud belongs to none of these trajectories
	if ( traj_idx == -1 )
	{
		std::cout << "this point cloud doesn't belongs to the current trajectory!";
		return -1;
	}
	return traj_idx;
}

int main ( int argc, char* argv [] )
{
	std::chrono::high_resolution_clock::time_point t1report = std::chrono::high_resolution_clock::now ();
	//load trajectory
	std::string filename = "D:/data/wuhangi/traj";
	std::string pointfileoutpath = "D:/data/wuhangi/pointcloud-df";
	std::string pointfilename = "D:/data/wuhangi/pointcloud";
	
	//读取全部轨迹，多个文件夹，并构建多个kd树
	std::vector<std::string> trajfiles;
	std::vector<pcl::KdTreeFLANN<pcl::PointXY>::Ptr> kdtree_vec ;
	Utility::get_files ( filename, ".lin", trajfiles );

	std::vector<Utility::PoseVec> vvec_pose ( trajfiles.size () );
	for (int trajid=0;trajid<trajfiles.size();++trajid)
	{
		loadPosTLin ( trajfiles[trajid], vvec_pose[trajid] );

		//trajectory point cloud
		boost::shared_ptr<pcl::PointCloud<pcl::PointXY>> traj_cloud ( new pcl::PointCloud<pcl::PointXY> () );
		//could not use openmp
		for ( int i = 0; i < vvec_pose [trajid].size (); ++i )
		{
			pcl::PointXY pt;
			pt.x = vvec_pose [trajid] [i].x;
			pt.y = vvec_pose [trajid] [i].y;
			traj_cloud->push_back ( pt );
		}

		pcl::KdTreeFLANN<pcl::PointXY>::Ptr kdtree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXY>> ();
		kdtree->setInputCloud ( traj_cloud );
		kdtree_vec.push_back ( kdtree );
	}

	//判断点云和轨迹是不是重合的，将每一份点云对应的kd树拿出来开始剔除。
	std::vector<std::string> pointfiles;
	Utility::get_files ( pointfilename, ".las", pointfiles );

	for ( auto pointfile : pointfiles ) 
	{
		pcl::PointCloud<PointXYZINTF>::Ptr input_cloud = boost::make_shared<pcl::PointCloud<PointXYZINTF>> ();
		pcl::PointCloud<PointXYZINTF>::Ptr output_cloud = boost::make_shared<pcl::PointCloud<PointXYZINTF>> ();
		Utility::Offset las_offset;
		PointIO::loadSingleLAS<PointXYZINTF> ( pointfile, input_cloud, las_offset );

		int trajid = selectTraject ( input_cloud, vvec_pose );

		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;

		for ( auto pt : input_cloud->points )
		{
			pcl::PointXY search_pt;
			search_pt.x = pt.x + las_offset.x;
			search_pt.y = pt.y + las_offset.y;

			if ( kdtree_vec[trajid]->nearestKSearch ( search_pt, 1, pointIdxRadiusSearch, pointRadiusSquaredDistance ) )
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

				if ( std::sqrt ( pointRadiusSquaredDistance [0] ) < DISTANCE )
				{
					output_cloud->push_back ( pt );
				}
			}
		}

		std::string outfilepath = pointfileoutpath + "/" + Utility::get_name ( pointfile );
		std::cout << "origin pts:\t" << input_cloud->size () << " left pts: \t" << output_cloud->size ();
		
		//release memory and save
		input_cloud->swap ( pcl::PointCloud<PointXYZINTF > () );
		PointIO::saveLAS<PointXYZINTF> ( outfilepath, output_cloud, las_offset );
	}

	std::chrono::high_resolution_clock::time_point t2treport = std::chrono::high_resolution_clock::now ();
	std::chrono::duration<double> t12report = std::chrono::duration_cast<std::chrono::duration<double>>( t2treport - t1report );
	std::cout << "Out put the regist Summary file, time cost: " << t12report.count () << "s" << std::endl;
	system ( "pause" );
	return 0;
}
#pragma once

#include <string>
#include <vector>

namespace Utility
{
	//struct: Pose
	struct Pose
	{
		double GPSTime; //unit: sec
		double x; //coordinate system: WGS84, unit: m
		double y; //coordinate system: WGS84, unit: m
		double z; //coordinate system: WGS84, unit: m
		double heading; //unit: rad
		double pitch; //unit: rad
		double roll; //unit: rad

		int block_id;
		double linear_velocity;
		double acceleration;
		double angular_velocity;
		bool changing_point;
		bool corner_point;
		bool intersection_point;
		bool isDMP_point;
		int traj_id_correspondence; //for searching DMP correspondence
		int pose_id_correspondence; //for searching DMP correspondence

		Pose ( double GPSTime0 = 0, double x0 = 0, double y0 = 0, double z0 = 0,
			   double heading0 = 0, double pitch0 = 0, double roll0 = 0 )
			: GPSTime ( GPSTime0 ), x ( x0 ), y ( y0 ), z ( z0 ), heading ( heading0 ), pitch ( pitch0 ), roll ( roll0 )
		{
			block_id = -1;
			linear_velocity = 0.0;
			acceleration = 0.0;
			angular_velocity = 0.0;
			changing_point = false;
			corner_point = false;
			intersection_point = false;
			isDMP_point = false;
			traj_id_correspondence = -1;
			pose_id_correspondence = -1;
		}

		double distanceTo ( const Pose& pose ) const
		{
			double delta_x = x - pose.x;
			double delta_y = y - pose.y;
			double delta_z = z - pose.z;
			double dist = std::sqrt ( delta_x*delta_x + delta_y*delta_y + delta_z*delta_z );
			return dist;
		}

		bool isChangingPoint () const
		{
			return changing_point;
		}

		bool isCornerPoint () const
		{
			return corner_point;
		}

		bool isIntersectionPoint () const
		{
			return intersection_point;
		}

		bool isDMP () const
		{
			return isDMP_point;
		}

		bool hasCorrespondence () const
		{
			return traj_id_correspondence != -1 && pose_id_correspondence != -1;
		}
	};
	typedef std::vector<Pose> PoseVec;
}

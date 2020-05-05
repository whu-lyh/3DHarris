#pragma once
#ifndef LAS_DATA_STRUCT
#define LAS_DATA_STRUCT

#define  UNSEGMENTATION -1

#include <vector>
#include <list>
#include <string>


#include "WinSock2.h"


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/features/shot.h>
#include <pcl/features/shot_omp.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/impl/pcl_base.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/algorithm.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Cartesian.h>
#include <CGAL/bounding_box.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/Projection_traits_xy_3.h>

using namespace std;

typedef CGAL::Exact_predicates_inexact_constructions_kernel Ke;
typedef CGAL::Projection_traits_xy_3<Ke> Gt;
typedef CGAL::Delaunay_triangulation_2<Gt> Delaunay;
typedef CGAL::Polygon_2<Ke> Polygon_2;
typedef Delaunay::Finite_faces_iterator Finite_faces_iterator;
typedef Delaunay::Face_circulator Face_circulator;
typedef Delaunay::Finite_vertices_iterator Finite_vertices_iterator;
typedef Delaunay::Finite_edges_iterator Finite_edges_iterator;
typedef Delaunay::Face_handle Face_handle;
typedef Delaunay::Vertex_handle Vertex_handle;
typedef Delaunay::Locate_type Locate_type;
typedef Ke::Vector_3 Vector_3;
typedef Ke::FT FT;
typedef Ke::Point_2  Point_2;
typedef Ke::Point_3  Point_3;
typedef Ke::Segment_2  Segment_CGAL;
typedef Ke::Segment_3 Segment_3;
typedef CGAL::Alpha_shape_vertex_base_2<Ke> Vb;
typedef CGAL::Cartesian<FT> KC;
typedef CGAL::Alpha_shape_face_base_2<Ke>  Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
typedef CGAL::Delaunay_triangulation_2<Ke, Tds> Triangulation_2;
typedef CGAL::Alpha_shape_2<Triangulation_2>  Alpha_shape_2;
typedef Alpha_shape_2::Alpha_shape_edges_iterator Alpha_shape_edges_iterator;
typedef Alpha_shape_2::Alpha_shape_vertices_iterator Alpha_shape_vertices_iterator;


//#include <gdiplus.h>
//#pragma comment(lib, "gdiplus.lib")
//using namespace Gdiplus;

typedef  pcl::PointCloud<pcl::PointXYZI>::Ptr     CloudXYZI_Ptr;
typedef  pcl::PointCloud<pcl::PointXYZI>          CloudXYZI;

typedef  pcl::PointCloud<pcl::PointXYZ>::Ptr      CloudXYZ_Ptr;
typedef  pcl::PointCloud<pcl::PointXYZ>           CloudXYZ;

typedef  pcl::PointCloud<pcl::PointXY>::Ptr       CloudXY_Ptr;
typedef  pcl::PointCloud<pcl::PointXY>            CloudXY;

typedef  pcl::PointCloud<pcl::PointXYZRGB>::Ptr   CloudXYZRGB_Ptr;
typedef  pcl::PointCloud<pcl::PointXYZRGB>        CloudXYZRGB;

typedef  pcl::PointCloud<pcl::Normal>::Ptr        Normal_Ptr;
typedef  pcl::PointCloud<pcl::Normal>             Normal;

typedef  pcl::PointCloud<pcl::PointNormal>::Ptr  Point_Normal_Ptr;
typedef  pcl::PointCloud<pcl::PointNormal>       Point_Normal;

typedef  pcl::PointCloud<pcl::PointXYZINormal>::Ptr  PointXYZ_Normal_Ptr;
typedef  pcl::PointCloud<pcl::PointXYZINormal>      PointXYZ_Normal;

typedef  vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>  VectorIntersecctionPoint;

namespace pmProcessUrban
{
	struct TXT_Point_XY
	{
		double x;
		double y;
	};

	struct TXT_Point_XYZ
	{
		double x;
		double y;
		double z;
	};

	/*struct CenterPoint
	{
		double x;
		double y;
		double z;
		CenterPoint(double x = 0, double y = 0, double z = 0) :
			x(x), y(y), z(z)
		{
			z = 0.0;
			x = y = 0.0;
		}
	};
	struct Bounds
	{
		double min_x;
		double min_y;
		double min_z;
		double max_x;
		double max_y;
		double max_z;
		Bounds()
		{
			min_x = min_y = min_z = max_x = max_y = max_z = 0.0;
		}
	};*/
	//struct Pose
	//{
	//	double GPSTime; //unit: sec
	//	double x; //coordinate system: WGS84, unit: m
	//	double y; //coordinate system: WGS84, unit: m
	//	double z; //coordinate system: WGS84, unit: m
	//	double heading; //unit: rad
	//	double pitch; //unit: rad
	//	double roll; //unit: rad



	//	Pose(double GPSTime0 = 0, double x0 = 0, double y0 = 0, double z0 = 0,
	//		double heading0 = 0, double pitch0 = 0, double roll0 = 0)
	//		: GPSTime(GPSTime0), x(x0), y(y0), z(z0), heading(heading0), pitch(pitch0), roll(roll0)
	//	{}
	//	double distanceTo(const Pose& pose) const
	//	{
	//		double delta_x = x - pose.x;
	//		double delta_y = y - pose.y;
	//		double delta_z = z - pose.z;
	//		double dist = std::sqrt(delta_x*delta_x + delta_y * delta_y + delta_z * delta_z);
	//		return dist;
	//	}
	//};
	//typedef std::vector<Pose> PoseVec;

	struct Pose
	{
		/*string image_name;
		double width;
		double height;
		double latitude;
		double longitude; 
		double altitude;
		double roll;
		double pitch;
		double heading;
		double x;
		double y;
		Pose(string image_name0, double width0 = 0, double height0 = 0, double latitude0 = 0,double longitude0=0,
			double altitude0=0,double roll0=0,double pitch0 = 0, double heading0 = 0, double x0 = 0,double y0=0)
					:image_name(image_name0),width(width0),height(height0),latitude(latitude0), longitude(longitude0),
			        altitude(altitude0),roll(roll0), pitch(pitch0),heading(heading0), x(x0),y(y0)
		{}*/
		/*double distanceTo(const Pose& pose) const
		{
			double delta_x = x - pose.x;
			double delta_y = y - pose.y;
			double delta_z = z - pose.z;
			double dist = std::sqrt(delta_x*delta_x + delta_y * delta_y + delta_z * delta_z);
			return dist;
		}*/
		string image_name;
		double pitch;
		double heading;
		double roll;
		double x, y, z;
		Pose(string image_name0, double pitch0 = 0, double heading0 = 0, double roll0 = 0, double x0 = 0, double y0 = 0,double z0=0)
			:image_name(image_name0), pitch(pitch0), heading(heading0), roll(roll0), x(x0), y(y0),z(z0)
		{}
	};
	typedef std::vector<Pose> PoseVec;

	typedef struct {
		int r;
		int g;
		int b;
	}RGBvec;

	struct PixelInfo{
		int Row;
		int Column;
		double x;
		double y;
		double range;
		unsigned int R;
		unsigned int G;
		unsigned int B;
    };

	struct ImagePixelInfo {
		int Row;
		int Column;
		unsigned int R;
		unsigned int G;
		unsigned int B;
	};


	//classify

	/*struct PointwithClass
	{
		double px;
		double py;
		double pz;
		int R;
		int G;
		int B;
	};*/

	struct PrinDirect
	{
		double px;
		double py;
		double pz;
	};

	struct NormalDirect
	{
		double nx;
		double ny;
		double nz;
	};

	struct CenterPoint
	{
		double x;
		double y;
		double z;
		CenterPoint(double x = 0, double y = 0, double z = 0) :
			x(x), y(y), z(z)
		{
			z = 0.0;
			x = y = 0.0;
		}

	};

	struct Bounds
	{
		double min_x;
		double min_y;
		double min_z;
		double max_x;
		double max_y;
		double max_z;
		Bounds()
		{
			min_x = min_y = min_z = max_x = max_y = max_z = 0.0;
		}
	};


	enum INITIALPOINTCLASS
	{
		UNKNOWN_PT_INITIAL = 0,
		POLE_INITIAL,
		PLANE_INITIAL,
		SPHERALITY_INITIAL
	};

	enum POINTCLASS
	{
		NONE_PT = -1,		          //未启用类别;
		UNKNOWN_PT = 0,	             //未知类别;
		VERTICAL_POLE,		        //竖直干;
		HORIZENTAL_POLE,           //水平杆;
		OTHER_POLE,               //其它杆;
		VERTICAL_PLANE,          //竖直面;
		HORIZENTAL_PLANE,       //水平面;
		OTHER_PLANE,           //其他面;
		SPHERALITY            //球;
	};

	enum OBJECTCLASS
	{
		NONE = -1,		//未启用类别;
		UNKNOWN = 0,	//未知类别;
		GROUND,			//地面;
		BUILDING,       //建筑物;
		UTILITYPOLE,    //电线杆;
		TRAFFICSIGN,    //交通标志牌;
		TREE,           //树;
		STREETLAMP,     //路灯;
		CAR,            //汽车;
		ENCLOSURE,      //围墙;
		CURB,           //路坎;
		ROADMARKING,   //交通标线;
		UNKNOWN_POLE,   //未识别的杆状目标;
		POWERLINE,
		ROAD,
		BUSH
	};

	enum  RelationShipWithRoad
	{
		UNKNOWN_RS = 0,	//未知类别;
		INCLUSIVE = 1,  //包含关系;
		CROSSING,       //相交;
		DISJOINT       //相离;
	};


	//LiDAR点的属性
	struct PointProperty
	{
		/*法向量;*/
		float normal_x;
		float normal_y;
		float normal_z;
		double d;
		/*主方向;*/
		float Principal_x;
		float Principal_y;
		float Principal_z;
		/*维数特征;*/
		int Dimension;
		/*区域标示;*/
		int seg_id;
		int obj_sort;
		int obj_id;
		PointProperty()
		{
			seg_id = UNSEGMENTATION;
			Dimension = NONE_PT;
			obj_id = NONE;
			obj_sort = NONE;
		}
	};


	struct Grid
	{
		bool is_empty;
		Grid()
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

		Voxel()
		{
			min_z = min_z_x = min_z_y = NeighborMin_z = mean_z = 0.f;
			PointsNumber = 1;
			dertaz = 0.0;
		}
	};

	struct SimplifiedVoxel
	{
		vector<int>point_id;
		float max_curvature;
		int max_curvature_point_id;
		bool has_keypoint;
		SimplifiedVoxel()
		{
			has_keypoint = false;
		}
	};


	struct  MathLine
	{
		CloudXYZ  points;
		pcl::ModelCoefficients::Ptr coefficients_line;
	};
	typedef  vector<MathLine, Eigen::aligned_allocator<MathLine>>  VectorMathLine;

	struct VoteScore
	{
		float angle1;//相邻两条线段的夹角;
		float angle2;//连线和该直线的夹角;
		float length;//该直线的长度;
		float pt_line_dis;//点的直线的距离（较大的值）;
		float pt_pt_dis;//相邻直线相邻端点的距离;
		int   scores;//得分;
		bool is_in_expel_zone;
		bool start_has_connected;
		bool end_has_connected;
		VoteScore()
		{
			scores = 0;//得分;
			is_in_expel_zone = false;
			start_has_connected = false;
			end_has_connected = false;
		}
		int line_id;
	};

	struct  PhysicalLine
	{
		pcl::ModelCoefficients::Ptr coefficients_line;
		pcl::PointXYZ start_pt;
		pcl::PointXYZ end_pt;
		float length;
		int line_id;
		VoteScore similarity;
		bool is_redundant;
		PhysicalLine()
		{
			is_redundant = false;
		}
	};


	struct SequencePoint
	{
		pcl::PointXYZ pt;
		bool is_intersection;
		SequencePoint()
		{
			is_intersection = false;
		}
	};

	typedef  vector<PhysicalLine, Eigen::aligned_allocator<PhysicalLine>>  VectorPhysicalLine;


	struct PolyLine
	{
		VectorPhysicalLine polyline_v;
		int polyline_id;
		CloudXYZ_Ptr sequence_points;
		double length;
		double dertax;
		double dertay;
	};
	typedef  vector<PolyLine, Eigen::aligned_allocator<PolyLine>>  VectorPolyLine;

	struct MarkingLine
	{
		float x1, y1, z1, x2, y2, z2;
	};

	struct Rect_f//和types.hpp中的Rect重复了，moyna改成了Rect_f
	{
		Eigen::Vector4f rect_pts[4];
	};


	typedef  vector<Rect_f, Eigen::aligned_allocator<Rect_f>> Vector_Rectangle;

	struct Segment
	{
		int seg_id;
		int obj_id;
		int obj_sort;
		int segment_sort;
		vector<int>point_id;
		vector<int>boundary_pt_id;
		std::vector<std::vector<Segment_CGAL>> boundarys;
		vector<int>adjacent_seg_id;
		pcl::PointXYZ center_pt;//平面的中点;
		double ending_pt1;//平面的端点1;
		pcl::PointXYZ pt1;
		double ending_pt2;//平面的端点2;
		pcl::PointXYZ pt2;
		bool type;//true 表示比较x大小,false 表示比较y的大小;
		Segment()
		{
			seg_id = UNSEGMENTATION;
			obj_id = UNSEGMENTATION;
			obj_sort = NONE;
			length = 0.0;
			width = 0.0;
			height = 0.0;
			nx = 0.0;
			ny = 0.0;
			nz = 0.0;
			dis = 0.0;
			curvature = 0.0;
			height_difference_to_ground = 0.0;
		}
		//平面方程;
		float px;
		float py;
		float pz;
		float nx;
		float ny;
		float nz;
		double dis;
		//尺寸;
		float length;
		float width;
		float height;
		Bounds seg_bounds;
		float curvature;//平面性;

		float min_z;
		pcl::PointXYZ min_z_pt;
		float height_difference_to_ground;

		VectorPolyLine polylines_v;
		int relationship_with_road;
	};
	typedef  vector<Segment, Eigen::aligned_allocator<Segment>> Vector_Segment;


	struct InterestingObject
	{
		int obj_id;
		int obj_sort;
		vector<int> point_id;
		vector<int> seg_id;
		float height;
		float length;
		float width;
		VectorPolyLine polylines_v;
		InterestingObject()
		{
			height = 0.0;
			width = 0.0;
			length = 0.0;
			obj_sort = NONE;
		}
		float min_z;
		pcl::PointXYZ min_z_pt;
		pcl::PointXYZ ground_pt;
		float max_z;
		float height_differece_to_ground;
		int relationship_with_road;

		float percentage_of_ball_points;
		float percentage_of_vertical_plane_points;
		float percentage_of_vertical_pole_points;
		float HeightDifferenceBeteewnGeometricalCenterAndBarycenter;
		float height_ground;
	};

	typedef  vector<InterestingObject, Eigen::aligned_allocator<InterestingObject>> VectorInterestingObject;

	class StructOperator
	{
	public:
		//获取中心
		void getCloudCenterPoint(const pcl::PointCloud<pcl::PointXYZ> & cloud, CenterPoint & centerPoint)
		{
			double cx = 0, cy = 0, cz = 0;

			for (int i = 0; i < cloud.size(); i++)
			{
				cx += cloud.points[i].x / cloud.size();
				cy += cloud.points[i].y / cloud.size();
				cz += cloud.points[i].z / cloud.size();
			}
			centerPoint.x = cx;
			centerPoint.y = cy;
			centerPoint.z = cz;
		}
		//获取边界
		void getCloudBound(const pcl::PointCloud<pcl::PointXYZ> & cloud, Bounds & bound)
		{
			double min_x = cloud[0].x;
			double min_y = cloud[0].y;
			double min_z = cloud[0].z;
			double max_x = cloud[0].x;
			double max_y = cloud[0].y;
			double max_z = cloud[0].z;

			for (int i = 0; i < cloud.size(); i++)
			{
				//获取边界
				if (min_x > cloud.points[i].x)
					min_x = cloud.points[i].x;
				if (min_y > cloud.points[i].y)
					min_y = cloud.points[i].y;
				if (min_z > cloud.points[i].z)
					min_z = cloud.points[i].z;
				if (max_x < cloud.points[i].x)
					max_x = cloud.points[i].x;
				if (max_y < cloud.points[i].y)
					max_y = cloud.points[i].y;
				if (max_z < cloud.points[i].z)
					max_z = cloud.points[i].z;
			}
			bound.min_x = min_x;
			bound.max_x = max_x;
			bound.min_y = min_y;
			bound.max_y = max_y;
			bound.min_z = min_z;
			bound.max_z = max_z;
		}

		//获取中心和边界
		void getBoundAndCenter(const pcl::PointCloud<pcl::PointXYZ> & cloud, Bounds & bound, CenterPoint& centerPoint)
		{
			double min_x = cloud[0].x;
			double min_y = cloud[0].y;
			double min_z = cloud[0].z;
			double max_x = cloud[0].x;
			double max_y = cloud[0].y;
			double max_z = cloud[0].z;

			double cx = 0, cy = 0, cz = 0;

			for (int i = 0; i < cloud.size(); i++)
			{
				//获取边界
				if (min_x > cloud.points[i].x)
					min_x = cloud.points[i].x;
				if (min_y > cloud.points[i].y)
					min_y = cloud.points[i].y;
				if (min_z > cloud.points[i].z)
					min_z = cloud.points[i].z;
				if (max_x < cloud.points[i].x)
					max_x = cloud.points[i].x;
				if (max_y < cloud.points[i].y)
					max_y = cloud.points[i].y;
				if (max_z < cloud.points[i].z)
					max_z = cloud.points[i].z;


				cx += cloud.points[i].x / cloud.size();
				cy += cloud.points[i].y / cloud.size();
				cz += cloud.points[i].z / cloud.size();
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

		void GetSubsetBoundary(pcl::PointCloud<pcl::PointXYZ>::Ptr & plane_wall_cloud,
			vector<int> & index, Bounds & bound)
		{
			//构建点云
			pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			for (int i = 0; i < index.size(); i++)
			{
				temp_cloud->push_back(plane_wall_cloud->points[index[i]]);
			}
			getCloudBound(*temp_cloud, bound);
		}

	};
}

#endif
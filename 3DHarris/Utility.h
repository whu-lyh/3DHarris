// Copyright (c) 2018 WuHan University. All Rights Reserved
// @author xhzou_whu@foxmail.com
// @date 2018/10/04
// @brief utility

// Copyright (c) 2020 WuHan University. All Rights Reserved
// @author yhaoli@whu.edu.cn
// @date 2020/02/01
// @brief utility

#pragma once
#pragma warning(disable: 4267)
#pragma warning(disable: 4819)
#pragma warning(disable: 4273)

#include "basicLibs.h"

namespace Utility
{
	//some usable const variable
	const float Epsilon_f = std::numeric_limits<float>::epsilon() * 10.f; //Epsilon--float
	const double Epsilon_d = std::numeric_limits<double>::epsilon() * 10.0; //Epsilon--double
	typedef std::list<std::string> filelist;

	class  Point3d
	{
	public:
		Point3d(double x0 = 0, double y0 = 0, double z0 = 0)
			: x(x0), y(y0), z(z0)
		{}

		Point3d operator +=(const Point3d& pt)
		{
			x += pt.x;
			y += pt.y;
			z += pt.z;
			return *this;
		}
	public:
		double x;
		double y;
		double z;
	};

	typedef Point3d Offset;

	class Bound
	{
	public:
		Bound (double min_x0 = (std::numeric_limits<double>::max)(), double max_x0 = (std::numeric_limits<double>::lowest)(),
			double min_y0 = (std::numeric_limits<double>::max)(), double max_y0 = (std::numeric_limits<double>::lowest)(),
			double min_z0 = (std::numeric_limits<double>::max)(), double max_z0 = (std::numeric_limits<double>::lowest)())
			: min_x (min_x0), max_x (max_x0), min_y (min_y0), max_y (max_y0), min_z (min_z0), max_z (max_z0)
		{
		}

		std::ostream& operator<< (std::ostream& os)
		{
			os << "Boundingbox scope, max_x= " << max_x << ", min_x= "
				<< min_x << "\n max_y= " << max_y << ", min_y= "
				<< min_y << "\n max_z= " << max_z << ", min_z= " << min_z;
			return os;
		}

	public:
		double min_x;
		double max_x;
		double min_y;
		double max_y;
		double min_z;
		double max_z;
	};

	extern  void ensure_dir(const std::string &dir);

	extern  void format_path(std::string &path);

	extern  std::string make_path(const std::string &dir, const std::string &file);

	extern  std::string replace_ext(const std::string &file, const std::string &ext);

	extern  std::string get_ext(const std::string &file);

	extern  std::string get_name(const std::string &file);

	extern  std::string get_name_without_ext(const std::string &file);

	extern  std::string get_parent(const std::string &path);

	extern  std::string get_parent(const std::wstring &path);

	extern  void get_files(const std::string& dir, const std::string& ext, filelist& list);

	extern  void get_files(const std::string& dir, const std::string& ext, std::vector<std::string>& files);

	extern  bool file_exist(const std::string& file);

	extern  bool is_directory(const std::string& dir);

	extern  bool copy_file(const std::string& to_dir, const std::string& file, bool overlay = false);

	extern  void rename(const std::string& new_name, const std::string& old_name);

	extern  void remove_file(const std::string& filename);

	extern  void remove_dir(const std::string& dir);

	//transform std::set to std::vector
	template<typename T>
	extern void setToVector(const std::set<T>& set_data, std::vector<T>& vector_data)
	{
		vector_data.clear();
		for (std::set<T>::const_iterator it = set_data.cbegin();it != set_data.cend();++it)
		{
			vector_data.emplace_back(*it);
		}
	}

	//Euler Angles to Rotation Matrix(angle unit: rad)
	extern  Eigen::Matrix3d getRotationMatrix(double heading, double pitch, double roll);

	//Euler Angles and translation to pose Matrix(angle unit: rad)
	extern  Eigen::Matrix4d getPoseMatrix(double heading, double pitch, double roll, double trans_x, double trans_y, double trans_z);

	//Rotation Matrix to Euler Angles(order: heading-z,pitch-y,roll-x)
	extern  Eigen::Vector3d getEulerAngles(const Eigen::Matrix3d& rot);

	//Pose Matrix to Euler Angles(order: heading-z,pitch-y,roll-x) and translations(x,y,z)
	extern  Eigen::Matrix<double, 6, 1> getEulerAndTranslations(const Eigen::Matrix4d& pose_mat);

	//unify the offset of point cloud
	template<typename T>
	extern void unifyPointCloudOffset(typename pcl::PointCloud<T>::Ptr& cloud, const Offset& old_offset, const Offset& new_offset)
	{
		if (cloud == nullptr)
		{
			std::cout << "Error: pointer 'cloud' is a nullptr!";
			return;
		}

		Offset delta_offset(old_offset.x - new_offset.x, old_offset.y - new_offset.y, old_offset.z - new_offset.z);
		for (auto& pt : cloud->points)
		{
			pt.x += delta_offset.x;
			pt.y += delta_offset.y;
			pt.z += delta_offset.z;
		}
	}

	extern  void getRandomColor(int& r, int& g, int& b);

}//namespace Utility
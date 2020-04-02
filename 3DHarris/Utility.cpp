#ifndef NOMINMAX
#define  NOMINMAX
#endif
#include <windows.h>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <boost/version.hpp>
#include <boost/filesystem.hpp>
#include "Utility.h"

namespace Utility
{

#ifdef WIN32
#define DIR_INTERVAL '\\'
#else
#define DIR_INTERVAL '/'
#endif

	//确保当前路径存在
	void ensure_dir (const std::string & dir)
	{
		boost::filesystem::path fullpath (dir);
		if (!boost::filesystem::exists (fullpath))
		{
			boost::filesystem::create_directory (fullpath);
		}
	}

	//路径字符串添加/或\结尾;
	void format_path (std::string & path)
	{
		size_t size = path.size ();
		if (size == 0 || path [size - 1] != DIR_INTERVAL)
			path.push_back (DIR_INTERVAL);
	}

	//根据路径和文件名生成完整路径;
	std::string make_path (const std::string & dir, const std::string & file)
	{
		std::string path = dir;
		format_path (path);
		return path + file;
	}

	//替换文件扩展名;
	std::string replace_ext (const std::string & file, const std::string & ext)
	{
		size_t pos = file.rfind ('.');
		return file.substr (0, pos) + ext;
	}

	//文件扩展名;
	std::string get_ext (const std::string & file)
	{
		size_t pos = file.rfind ('.');
		return (pos != std::string::npos) ? file.substr (pos) : std::string ();
	}

	//文件名;
	std::string get_name (const std::string & file)
	{
		boost::filesystem::path p (file);
#if BOOST_VERSION > 104000
		return p.filename ().string ();
#else
		return p.filename ();
#endif
	}

	std::string get_name_without_ext (const std::string &file)
	{
		std::string name = get_name (file);
		std::string ext = get_ext (file);
		return name.substr (0, name.length () - ext.length ());
	}

	//文件父目录;
	std::string get_parent (const std::string &path)
	{
		boost::filesystem::path p (path);
#if BOOST_VERSION > 104000
		return p.parent_path ().string ();
#else
		return p.parent_path ();
#endif
	}

	//获得文件夹下所有的文件;
	void get_files (const std::string& dir, const std::string& ext, filelist& list)
	{
		boost::filesystem::path fullPath (boost::filesystem::initial_path ());
		fullPath = boost::filesystem::system_complete (boost::filesystem::path (dir));

		if (!boost::filesystem::exists (fullPath) || !boost::filesystem::is_directory (fullPath))
		{
			return;
		}

		boost::filesystem::directory_iterator end_iter;
		for (boost::filesystem::directory_iterator file_itr (fullPath); file_itr != end_iter; ++file_itr)
		{
			if (!boost::filesystem::is_directory (*file_itr) && (boost::filesystem::extension (*file_itr) == ext || ext == ""))
			{
#if BOOST_VERSION > 104000
				std::string str = make_path (dir, file_itr->path ().filename ().string ());
#else
				std::string str = make_path (dir, file_itr->path ().filename ());
#endif
				list.emplace_back (str);
			}
		}
	}
	//函数重载，返回vector
	void get_files (const std::string& dir, const std::string& ext, std::vector<std::string>& files)
	{
		boost::filesystem::path fullPath (boost::filesystem::initial_path ());
		fullPath = boost::filesystem::system_complete (boost::filesystem::path (dir));

		if (!boost::filesystem::exists (fullPath) || !boost::filesystem::is_directory (fullPath))
		{
			return;
		}

		boost::filesystem::directory_iterator end_iter;
		for (boost::filesystem::directory_iterator file_itr (fullPath); file_itr != end_iter; ++file_itr)
		{
			if (!boost::filesystem::is_directory (*file_itr) && (boost::filesystem::extension (*file_itr) == ext || ext == ""))
			{
#if BOOST_VERSION > 104000
				std::string str = make_path (dir, file_itr->path ().filename ().string ());
#else
				std::string str = make_path (dir, file_itr->path ().filename ());
#endif
				files.emplace_back (str);
			}
		}
	}

	//判断文件是否存在
	bool file_exist (const std::string& file)
	{
		boost::filesystem::path fullpath (file);
		return boost::filesystem::exists (fullpath);
	}

	//判断是否为目录
	bool is_directory (const std::string &dir)
	{
		boost::filesystem::path fullPath (boost::filesystem::initial_path ());
		fullPath = boost::filesystem::system_complete (boost::filesystem::path (dir));

		if (boost::filesystem::exists (fullPath) && boost::filesystem::is_directory (fullPath))
		{
			return true;
		}
		return false;
	}

	bool copy_file (const std::string& to_dir, const std::string& file, bool overlay)
	{
		boost::filesystem::path fullpath (to_dir);
		if (!boost::filesystem::exists (fullpath))
			return false;

		if (file_exist (file))
		{
			std::string file_name = get_name (file);
			if (!overlay)
			{
				if (file_exist (to_dir + "/" + file_name))
					return false;
			}
			boost::filesystem::copy_file (boost::filesystem::path (file), boost::filesystem::path (to_dir + "/" + file_name));
			return true;
		}
		return false;
	}

	//重命名
	void rename (const std::string& new_name, const std::string& old_name)
	{
		boost::filesystem::rename (boost::filesystem::path (old_name), boost::filesystem::path (new_name));
	}

	//删除文件
	void remove_file (const std::string& filename)
	{
		if (file_exist (filename))
		{
			boost::filesystem::remove (boost::filesystem::path (filename));
			std::cout << "'" << filename << "' is removed!";
		}
		else
		{
			std::cout << filename << " doesn't exist!";
		}
	}

	//删除目录
	void remove_dir (const std::string& dir)
	{
		if (is_directory (dir))
		{
			boost::filesystem::remove_all (boost::filesystem::path (dir));
			std::cout << "Directory '" << dir << "' is removed!";
		}
		else
		{
			std::cout << dir << " is not a directory!";
		}
	}

	Eigen::Matrix3d getRotationMatrix (double heading, double pitch, double roll)
	{
		Eigen::Vector3d ea0 (heading, pitch, roll);
		Eigen::Matrix3d R;
		R = Eigen::AngleAxisd (ea0 [0], Eigen::Vector3d::UnitZ ())
			* Eigen::AngleAxisd (ea0 [1], Eigen::Vector3d::UnitY ())
			* Eigen::AngleAxisd (ea0 [2], Eigen::Vector3d::UnitX ());
		return R;
	}

	Eigen::Matrix4d getPoseMatrix (double heading, double pitch, double roll, double trans_x, double trans_y, double trans_z)
	{
		Eigen::Matrix4d pose_matrix;
		Eigen::Matrix3d R = getRotationMatrix (heading, pitch, roll);
		pose_matrix << R (0, 0), R (0, 1), R (0, 2), trans_x,
			R (1, 0), R (1, 1), R (1, 2), trans_y,
			R (2, 0), R (2, 1), R (2, 2), trans_z,
			0.0, 0.0, 0.0, 1.0;
		return pose_matrix;
	}

	Eigen::Vector3d getEulerAngles (const Eigen::Matrix3d& rot)
	{
		return rot.eulerAngles (0, 1, 2);
		//double sy = sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));

		//bool singular = sy < 1e-6; // If

		//double x, y, z;
		//if (!singular) {
		//	x = atan2(rot(2, 1), rot(2, 2));
		//	y = atan2(-rot(2, 0), sy);
		//	z = atan2(rot(1, 0), rot(0, 0));
		//}
		//else {
		//	x = atan2(-rot(1, 2), rot(1, 1));
		//	y = atan2(-rot(2, 0), sy);
		//	z = 0;
		//}
		//return Eigen::Vector3d(x, y, z);
	}

	Eigen::Matrix<double, 6, 1> getEulerAndTranslations (const Eigen::Matrix4d& pose_mat)
	{
		Eigen::Vector3d euler = pose_mat.topLeftCorner<3, 3> ().eulerAngles (0, 1, 2);
		Eigen::Vector3d trans = pose_mat.topRightCorner<3, 1> ();
		Eigen::Matrix<double, 6, 1> euler_trans;
		euler_trans << euler (0, 0), euler (1, 0), euler (2, 0), trans (0, 0), trans (1, 0), trans (2, 0);
		return euler_trans;
	}

	void getRandomColor (int& r, int& g, int& b)
	{
		std::srand (int (time (0)));
		omp_lock_t lock;
		omp_init_lock (&lock);
		omp_set_lock (&lock);
		static int last_num = 0;
		int num = 0;
		while (1)
		{
			num = rand () % 12;
			if (num != last_num)
				break;
		}
		last_num = num;
		omp_unset_lock (&lock);
		omp_destroy_lock (&lock);
		switch (num)
		{
		case 0:
		{
			r = 255;
			g = 0;
			b = 0;
			break;
		}
		case 1:
		{
			r = 0;
			g = 255;
			b = 0;
			break;
		}
		case 2:
		{
			r = 0;
			g = 0;
			b = 255;
			break;
		}
		case 3:
		{
			r = 255;
			g = 255;
			b = 0;
			break;
		}
		case 4:
		{
			r = 255;
			g = 0;
			b = 255;
			break;
		}
		case 5:
		{
			r = 0;
			g = 255;
			b = 255;
			break;
		}
		case 6:
		{
			r = 72;
			g = 61;
			b = 139;
			break;
		}
		case 7:
		{
			r = 0;
			g = 191;
			b = 255;
			break;
		}
		case 8:
		{
			r = 0;
			g = 100;
			b = 0;
			break;
		}
		case 9:
		{
			r = 85;
			g = 107;
			b = 47;
			break;
		}
		case 10:
		{
			r = 255;
			g = 215;
			b = 0;
			break;
		}
		case 11:
		{
			r = 139;
			g = 69;
			b = 19;
			break;
		}
		case 12:
		{
			r = 160;
			g = 32;
			b = 240;
			break;
		}
		default:
		{
			r = 255;
			g = 255;
			b = 255;
			break;
		}
		}
	}
}
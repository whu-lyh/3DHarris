
// STL
#include <iostream>
#include <string>
// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// Boost
#include <boost/filesystem.hpp>

// make dir existed
bool ensure_dir(const std::string &dir)
{
	boost::filesystem::path fullpath(dir);
	if (!boost::filesystem::exists(fullpath))
	{
		boost::filesystem::create_directories(fullpath);
	}
	return true;
}
// ensure the current dir exist in a recursive manner
bool ensure_dir_recursive(const std::string &dir)
{
	bool bSuccess = true;
	boost::filesystem::path fullpath(dir);
	boost::filesystem::path parent_path = fullpath.parent_path();
	if (!boost::filesystem::exists(parent_path)) {
		bSuccess |= ensure_dir(parent_path.string());
		bSuccess |= boost::filesystem::create_directory(fullpath);
	}
	else if (!boost::filesystem::exists(fullpath)) {
		bSuccess |= boost::filesystem::create_directory(fullpath);
	}
	return bSuccess;
}
// format a canonical standard path string ending with '/' or '\'
void format_path(std::string &path)
{
	size_t size = path.size();
	if (size == 0 || (path[size - 1] != '/'))
		path.push_back('/');
}
// concatenate two string as path string
std::string make_path(const std::string &dir, const std::string &file)
{
	std::string path = dir;
	format_path(path);
	return path + file;
}
// fetch the file extension suffix
std::string get_ext(const std::string &file)
{
	size_t pos = file.rfind('.');
	return (pos != std::string::npos) ? file.substr(pos) : std::string();
}
// fetch file name with extension suffix
std::string get_name(const std::string &file)
{
	boost::filesystem::path p(file);
#if BOOST_VERSION > 104000
	return p.filename().string();
#else
	return p.filename();
#endif
}
// fetch file name without extension suffix
std::string get_name_without_ext(const std::string &file)
{
	std::string name = get_name(file);
	std::string ext = get_ext(file);
	return name.substr(0, name.length() - ext.length());
}
// fetch file's path
std::string get_path(const std::string &file)
{
	boost::filesystem::path p(file);
#if BOOST_VERSION > 104000
	return p.parent_path().string();
#else
	return p.parent_path();
#endif
}
// fetch file's parent path
std::string get_parent(const std::string &path)
{
	boost::filesystem::path p(path);
#if BOOST_VERSION > 104000
	return p.parent_path().string();
#else
	return p.parent_path();
#endif
}
// fetch the specific extension suffix files under the dir and return to vector container
// note that the vector container could be accumulated by calling muli-time
void get_files(const std::string &dir, const std::string &ext, std::vector<std::string> &files)
{
	boost::filesystem::path fullPath(boost::filesystem::initial_path());
	fullPath = boost::filesystem::system_complete(boost::filesystem::path(dir));
	if (!boost::filesystem::exists(fullPath) || !boost::filesystem::is_directory(fullPath))
	{
		return;
	}
	boost::filesystem::directory_iterator end_iter;
	for (boost::filesystem::directory_iterator file_itr(fullPath); file_itr != end_iter; ++file_itr)
	{
		if (!boost::filesystem::is_directory(*file_itr) && (boost::filesystem::extension(*file_itr) == ext || ext == ""))
		{
#if BOOST_VERSION > 104000
			std::string str = make_path(dir, file_itr->path().filename().string());
#else
			std::string str = make_path(dir, file_itr->path().filename());
#endif
			files.push_back(str);
		}
	}
}
// fetch the specific extension suffix files under the dir in a recursive manner and return to vector container
bool get_files_recursive(const std::string &basedir, const std::string &ext, std::vector<std::string> &files)
{
	namespace bfs = boost::filesystem;
	bfs::path full_path(boost::filesystem::initial_path());
	full_path = bfs::system_complete(bfs::path(basedir));
	if (!bfs::exists(full_path) || !bfs::is_directory(full_path))
	{
		std::cout << full_path << " is not exist!";
		return false;
	}
	std::vector<std::string> dirs;
	bfs::recursive_directory_iterator end_iter;
	for (bfs::recursive_directory_iterator file_itr(full_path); file_itr != end_iter; file_itr++) {
		try {
			if (bfs::is_directory(*file_itr))
			{
				dirs.emplace_back(file_itr->path().string());
			}
			else { //if is a file
				std::string file = file_itr->path().string();
				if (boost::filesystem::extension(*file_itr) == ext || ext == "")
				{
					files.emplace_back(file_itr->path().string());
				}
				dirs.emplace_back(file_itr->path().string());
			}
		}
		catch (...) {
			std::cerr << "Fail to parse some dirs, please check it!";
		}
	}
	return true;
}
// check whether the file is existed already or not
bool file_exist(const std::string &file)
{
	boost::filesystem::path fullpath(file);
	return boost::filesystem::exists(fullpath);
}
// check whether the input string is a dir or not	
bool is_directory(const std::string &dir)
{
	boost::filesystem::path fullPath(boost::filesystem::initial_path());
	fullPath = boost::filesystem::system_complete(boost::filesystem::path(dir));
	return (boost::filesystem::exists(fullPath) && boost::filesystem::is_directory(fullPath)) ? true : false;
}

int main()
{
	std::vector<std::string> seqs{
		//"2013_05_28_drive_0000_sync",
		//"2013_05_28_drive_0002_sync" ,
		//"2013_05_28_drive_0003_sync" ,
		//"2013_05_28_drive_0004_sync" ,
		//"2013_05_28_drive_0005_sync" ,
		//"2013_05_28_drive_0006_sync" ,
		//"2013_05_28_drive_0007_sync" ,
		"2013_05_28_drive_0009_sync" ,
		"2013_05_28_drive_0010_sync" };
	// source image path
	std::string source_image_path = "H:/KITTI360/data_2d_pano/";
	// target image path
	std::string target_image_path = "F:/PublicDataSet/KITTI360/";
	// following the py code
	double ratio_w = (1.0 / (1024.0 / 2800.0)) / 3.0;
	double ratio_h = (1.0 / (512.0 / 1400.0)) / 3.0;

	for (int i = 0; i < seqs.size(); ++i)
	{
		std::string out_image_path_bath = target_image_path + seqs[i] + "/pano/data_rgb";
		ensure_dir_recursive(out_image_path_bath);
		std::string images_path = source_image_path + seqs[i] + "/pano/data_rgb";
		std::vector<std::string> images;
		get_files(images_path, ".png", images);
		std::cout << "Image number:\t" << images.size() << std::endl;
#pragma omp parallel for
		for (int j = 0; j < images.size(); ++j)
		{
			cv::Mat pano_r = cv::imread(images[j]);
			if (pano_r.data == nullptr)
			{
				std::cerr << "Image not existed!" << images[j] << std::endl;
				continue;
			}
			//std::cout << "width: " << pano.cols << "\t height£º" << pano.rows << "\t channels£º" << pano.channels() << std::endl;
			//cv::imshow("1",pano);
			std::string out_image = get_name(images[j]);
			std::string out_image_path = out_image_path_bath + "/" + out_image;
			cv::Mat pano(pano_r, cv::Rect(0, 100, 2800, 1200));
			cv::imwrite(out_image_path, pano);
			cv::GaussianBlur(pano, pano, cv::Size(0, 0), ratio_w, ratio_h);
			cv::Mat pano_resize;
			cv::resize(pano, pano_resize, cv::Size(1024, 512), 0, 0, cv::INTER_CUBIC);
			cv::imwrite(out_image_path, pano_resize);
		}
		std::cout << seqs[i] << " done! \n" << std::endl;
	}
	std::cout << "Done!" << std::endl;
	system("pause");
	return 1;
}
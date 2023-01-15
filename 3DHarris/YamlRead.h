#ifndef YAMLREAD_H
#define YAMLREAD_H

// STL
#include <string>
#include <vector>
// yaml
#include <yaml-cpp/yaml.h>
// Eigen
#include <Eigen/Core>
// opencv
#include <opencv2/core.hpp>

// The default in Eigen is column-major
inline Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Mat33FromYaml(std::string FilePath, std::string vName)
{
	Eigen::Matrix<double, 3, 3, Eigen::RowMajor> ret;
	YAML::Node config = YAML::LoadFile(FilePath);
	const std::vector<double> vec_double = config[vName].as<std::vector<double>>();
	Eigen::Matrix<double, 3, 3, Eigen::RowMajor> matRowMajor(vec_double.data());
	ret = matRowMajor;
	return ret;
}
inline Eigen::Matrix<double, 4, 4, Eigen::RowMajor> Mat44FromYaml(std::string FilePath, std::string vName)
{
	Eigen::Matrix<double, 4, 4, Eigen::RowMajor> ret;
	YAML::Node config = YAML::LoadFile(FilePath);
	const std::vector<double> vec_double = config[vName].as< std::vector<double> >();
	Eigen::Matrix<double, 4, 4, Eigen::RowMajor> matRowMajor(vec_double.data());
	ret = matRowMajor;
	return ret;
}
inline cv::Mat cameraMatrixFromYamlIntrinsics(std::string FilePath, std::string intrinsics = "intrinsics")
{
	double fx, fy, cx, cy;
	YAML::Node config = YAML::LoadFile(FilePath);
	const std::vector<double> vec_double = config[intrinsics].as<std::vector<double>>();
	fx = vec_double.at(0);
	fy = vec_double.at(1);
	cx = vec_double.at(2);
	cy = vec_double.at(3);
	cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
	return cameraMatrix;
}
inline cv::Mat distCoeffsFromYaml(std::string FilePath, std::string dist = "distortion_coeffs")
{
	YAML::Node config = YAML::LoadFile(FilePath);
	const std::vector<double> vec_double = config[dist].as<std::vector<double>>();
	double k1, k2, r1, r2;
	k1 = vec_double.at(0);
	k2 = vec_double.at(1);
	r1 = vec_double.at(2);
	r2 = vec_double.at(3);
	cv::Mat distCoeffs = (cv::Mat1d(4, 1) << k1, k2, r1, r2);
	return distCoeffs;
}
inline double getDoubleVariableFromYaml(std::string FilePath, std::string vName)
{
	YAML::Node config = YAML::LoadFile(FilePath);
	const double ret = config[vName].as<double>();
	return ret;
}
inline int getIntVariableFromYaml(std::string FilePath, std::string vName)
{
	YAML::Node config = YAML::LoadFile(FilePath);
	const int ret = config[vName].as<int>();
	return ret;
}
#endif // YAML_EIGEN_H

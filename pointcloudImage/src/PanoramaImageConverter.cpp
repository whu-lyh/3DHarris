
// STL
#include <fstream>
#include <vector>
#include <string>
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
// Opencv
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/core/core.hpp>
// Local
#include "numeric/numeric.h"
#include "spherical/image_resampling.hpp"
#include "spherical/cubic_image_sampler.hpp"
#include "cameras/Camera_Pinhole.hpp"

//-----------------
  //-- Create N rectilinear cameras
  //     (according the number of asked split along the X axis)
  //-- For each camera
  //   -- Forward mapping
  //   -- Save resulting images to disk
  //-----------------
int pano2rectilinear(std::vector<std::string> &vec_filenames)
{
	// the rectilinear image size
	int image_resolution = 1024;
	// the number of rectilinear image along the X axis
	int nb_split = 5;
	// the rectilinear camera FoV in degrees
	float fov = 60.0;

	//-- Simulate a camera with many rotations along the Y axis
	const int pinhole_width = image_resolution, pinhole_height = image_resolution;
	const double focal = openMVG::spherical::FocalFromPinholeHeight(pinhole_height, openMVG::D2R(fov));
	const openMVG::cameras::Pinhole_Intrinsic pinhole_camera = openMVG::spherical::ComputeCubicCameraIntrinsics(image_resolution);

	const double alpha = (M_PI * 2.0) / static_cast<double>(nb_split); // 360 / split_count
	std::vector<openMVG::Mat3> camera_rotations;
	for (int i = 0; i < nb_split; ++i)
	{
		camera_rotations.emplace_back(openMVG::RotationAroundY(alpha * i));
	}

	//-- For each input image extract multiple pinhole images
	for (const std::string & filename_it : vec_filenames)
	{
		cv::Mat spherical_image = cv::imread(filename_it);
		if (spherical_image.data = NULL)
		{
			std::cerr << "Cannot read the image: " << filename_it << std::endl;
			continue;
		}

		std::vector<cv::Mat> sampled_images(camera_rotations.size(), cv::Mat(image_resolution, image_resolution, cv::CV_8UC3));

		openMVG::spherical::SphericalToPinholes
		(
			spherical_image,
			pinhole_camera,
			sampled_images,
			camera_rotations,
			openMVG::image::Sampler2d<image::SamplerLinear>()
		);

		// Save images to disk
		for (int i_rot = 0; i_rot < camera_rotations.size(); ++i_rot)
		{

		}
	}

	std::ofstream fileFocalOut("focal.txt");
	fileFocalOut << focal;
	fileFocalOut.close();

	return 1;
}
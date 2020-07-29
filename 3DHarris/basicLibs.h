#pragma once
/*
	Copyright (c) 2020 WuHan University. All Rights Reserved
	First create: 2020/04/02 06:27
	Detail: 3D harris test
	Author: liyuhao
	Email: yhaoli@whu.edu.cn
*/

#pragma warning(disable: 4267)
#pragma warning(disable: 4819)
#pragma warning(disable: 4273)

#ifndef NOMINMAX
#define  NOMINMAX
#endif

#ifndef GLOG_NO_ABBREVIATED_SEVERITIES
#define  GLOG_NO_ABBREVIATED_SEVERITIES
#endif

#ifndef _SCL_SECURE_NO_WARNINGS
#define _SCL_SECURE_NO_WARNINGS
#endif


#ifdef _DEBUG
#pragma comment(lib, "liblas.lib")

#pragma comment(lib, "pcl_common_debug.lib")
#pragma comment(lib, "pcl_io_debug.lib")

#pragma comment(lib, "libboost_thread-vc140-mt-gd-x64-1_67.lib")
#pragma comment(lib, "pcl_common_debug.lib")
#pragma comment(lib, "pcl_features_debug.lib")
#pragma comment(lib, "pcl_filters_debug.lib")
#pragma comment(lib, "pcl_kdtree_debug.lib")
#pragma comment(lib, "pcl_search_debug.lib")
#pragma comment(lib, "pcl_registration_debug.lib")
#pragma comment(lib, "pcl_io_debug.lib")

#pragma comment(lib, "tbb_debug.lib")
#pragma comment(lib, "opencv_core340d.lib")
#pragma comment(lib, "opencv_highgui340d.lib")
#pragma comment(lib, "opencv_imgproc340d.lib")
#pragma comment(lib, "opencv_imgcodecs340d.lib")
#else
#pragma comment(lib, "liblas.lib")

#pragma comment(lib, "pcl_io_release.lib")

#pragma comment(lib, "libboost_thread-vc140-mt-x64-1_67.lib")
#pragma comment(lib, "pcl_common_release.lib")
#pragma comment(lib, "pcl_features_release.lib")
#pragma comment(lib, "pcl_filters_release.lib")
#pragma comment(lib, "pcl_kdtree_release.lib")
#pragma comment(lib, "pcl_search_release.lib")
#pragma comment(lib, "pcl_registration_release.lib")
#pragma comment(lib, "pcl_io_release.lib")

#pragma comment(lib, "tbb.lib")
#pragma comment(lib, "opencv_core340.lib")
#pragma comment(lib, "opencv_highgui340.lib")
#pragma comment(lib, "opencv_imgproc340.lib")
#pragma comment(lib, "opencv_imgcodecs340.lib")
#endif

#pragma execution_character_set("utf-8")

#include <Windows.h>
#include <glog/logging.h>
#include <list>
#include <set>
#include <omp.h>
#include <fstream>

#include <iomanip>
#include <liblas/header.hpp>
#include <liblas/reader.hpp>
#include <liblas/writer.hpp>
#include <liblas/factory.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <boost/make_shared.hpp>
#include <boost/filesystem/convenience.hpp>

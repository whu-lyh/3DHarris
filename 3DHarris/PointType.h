// Copyright (c) 2018 WuHan University. All Rights Reserved
// @author xhzou_whu@foxmail.com
// @date 2018/10/04
// @brief point type
#pragma once

#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>

//Point Type: x/y/z/intensity/number of return/GPS time/flighting edge
struct PointXYZINTF
{
	PCL_ADD_POINT4D;
	PCL_ADD_INTENSITY;
	uint8_t num_returns;
	double gps_time;
	float flighting_line_edge;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
}EIGEN_ALIGN16;

//Point Type: x/y/z/GPS time
struct PointXYZT
{
	PCL_ADD_POINT4D;
	double gps_time;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
}EIGEN_ALIGN16;

//POINT_CLOUD_REGISTER
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZINTF,
(float, x, x)
(float, y, y)
(float, z, z)
(float, intensity, intensity)
(uint8_t, num_returns, num_returns)
(double, gps_time, gps_time)
(float, flighting_line_edge, flighting_line_edge)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZT,
(float, x, x)
(float, y, y)
(float, z, z)
(double, gps_time, gps_time)
)

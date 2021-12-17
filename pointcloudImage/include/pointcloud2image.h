#pragma once

// STL
#include <iostream>
#include <cmath>
#include <cstdio>
// ceres
#include "ceres/ceres.h"
#include "ceres/rotation.h"

namespace pci
{
	// a more efficient realization
	template <typename T> inline
	void UnitQuaternionRotatePointEx(const T q[4], const T pt[3], T result[3])
	{
		T uv0 = q[2] * pt[2] - q[3] * pt[1];
		T uv1 = q[3] * pt[0] - q[1] * pt[2];
		T uv2 = q[1] * pt[1] - q[2] * pt[0];
		uv0 += uv0;
		uv1 += uv1;
		uv2 += uv2;
		result[0] = pt[0] + q[0] * uv0;
		result[1] = pt[1] + q[0] * uv1;
		result[2] = pt[2] + q[0] * uv2;
		result[0] += q[2] * uv2 - q[3] * uv1;
		result[1] += q[3] * uv0 - q[1] * uv2;
		result[2] += q[1] * uv1 - q[2] * uv0;
	}

	struct ReprojectionErrorResidual{

		ReprojectionErrorResidual(double* K, double* observed_pixel, double* point_cloud):
			_K(K), _observed_pixel(observed_pixel), _point_cloud(point_cloud){}


		template <typename T>
		bool operator()(const T* const transformation, T* residual) const {

			/* 1. Transforming point cloud with [R|t] */

			T transformed_point_cloud[3];

			T rotation[4];
			rotation[0] = transformation[6];    // qw
			rotation[1] = transformation[3];    // qx
			rotation[2] = transformation[4];    // qy
			rotation[3] = transformation[5];    // qz

			T translation[3];
			translation[0] = transformation[0]; // tx
			translation[1] = transformation[1]; // ty
			translation[2] = transformation[2]; // tz

			T tmp_point_cloud[3] = { T(_point_cloud[0]), T(_point_cloud[1]), T(_point_cloud[2]) };

			// rotate 3d point cloud
			ceres::QuaternionRotatePoint(rotation, tmp_point_cloud, transformed_point_cloud);

	//        std::cout << "point cloud: "
	//                  << _point_cloud[0] <<
	//                  "," << _point_cloud[1] <<
	//                  "," << _point_cloud[2] << "\n";
	//        std::cout << "after rotation: "
	//                  << transformed_point_cloud[0] <<
	//                  "," << transformed_point_cloud[1] <<
	//                  "," << transformed_point_cloud[2] << "\n";

			// translate 3d point cloud after rotating
			transformed_point_cloud[0] += translation[0];
			transformed_point_cloud[1] += translation[1];
			transformed_point_cloud[2] += translation[2];

	//        std::cout << "after translation: "
	//                  << transformed_point_cloud[0] <<
	//                  "," << transformed_point_cloud[1] <<
	//                  "," << transformed_point_cloud[2] << "\n";


			/* 2. Convert 3d point cloud tp 2d pixel using camera intrinsics */

			T fx = T(_K[0]);
			T fy = T(_K[1]);
			T cx = T(_K[2]);
			T cy = T(_K[3]);

			T x = transformed_point_cloud[0];
			T y = transformed_point_cloud[1];
			T z = transformed_point_cloud[2];

			T u = fx * x / z + cx;
			T v = fy * y / z + cy;

	//        std::cout << "converting to pixels: "
	//                  << u << "," << v << "," << "\n";

			residual[0] = _observed_pixel[0] - u;
			residual[1] = _observed_pixel[1] - v;

	//        std::cout << "residuals: "
	//                  << residual[0] << "," << residual[1] << "," << "\n";

			return true;
		}

		static ceres::CostFunction* Create(double* K, double* observed_pixel, double* point_cloud) {
			return (new ceres::AutoDiffCostFunction<ReprojectionErrorResidual, 2, 7>(
				new ReprojectionErrorResidual(K, observed_pixel, point_cloud)));
		}
		double* _K;
		double* _observed_pixel;
		double* _point_cloud;

	};
}
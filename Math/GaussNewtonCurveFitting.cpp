// STL
#include <stdlib.h>
#include <iostream>
#include <chrono>
#include <vector>
#include <cmath> // also with _USE_MATH_DEFINES
#include <random>
// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

unsigned int seed = 0;
double GuassianKernel(double mu, double sigma)
{
    // generation of two normalized uniform random variables
	double U1 = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);
	double U2 = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);
	// using Box-Muller transform to obtain a varaible with a standard normal distribution
	double Z0 = sqrt(-2.0 * ::log(U1)) * cos(2.0*M_PI * U2);
	// scaling
	Z0 = sigma * Z0 + mu;
	return Z0;
}

int main(int argc, char **argv)
{
	double ar = 1.0, br = 2.0, cr = 1.0;         // 真实参数值
	double ae = 2.0, be = -1.0, ce = 5.0;        // 估计参数值
	int N = 100;                                 // 数据点
	double w_sigma = 0.5;                        // 噪声Sigma值
	double inv_sigma = 1.0 / w_sigma;
	// generate data
	std::vector<double> x_data, y_data;		// 数据
	for (int i = 0; i < N; ++i)
	{
		double x = i / 100.0;
		x_data.emplace_back(x);
		y_data.emplace_back(exp(ar * x * x + br * x + cr) + GuassianKernel(0.0,w_sigma));
	}
	// 开始Gauss-Newton迭代
	int iterations = 1000;
	double cost = 0.0, lastCost = 0.0;
	// timestamp
	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
	for (int iter = 0; iter < iterations; ++iter)
	{
		// 每次迭代的意思是参数进行更新，但是别的矩阵重新进行初始化？也就是说H,b，损失等重新进行初始化，上次的计算结果并不保留
		// Hessian matrix initialized using zero
		Eigen::Matrix3d H = Eigen::Matrix3d::Zero();             // Hessian = J^T W^{-1} J in Gauss-Newton
		Eigen::Vector3d b = Eigen::Vector3d::Zero();             // bias
		cost = 0;
		for (int i = 0; i < N; i++)
		{
			double xi = x_data[i], yi = y_data[i];  // 第i个数据点
			double error = yi - exp(ae * xi * xi + be * xi + ce);
			//因为直线拟合是二维的，所以是二维向量存储jacobian matrix，partial devatitives about the parameter a,b,c
			Eigen::Vector3d J;
			J[0] = -xi * xi * exp(ae * xi * xi + be * xi + ce);  // de/da
			J[1] = -xi * exp(ae * xi * xi + be * xi + ce);  // de/db
			J[2] = -exp(ae * xi * xi + be * xi + ce);  // de/dc
			//
			H += inv_sigma * inv_sigma * J * J.transpose();
			b += -inv_sigma * inv_sigma * error * J;
			cost += error * error;
		}
		// 求解线性方程 Hx=b, using Cholesky decomposition
		Eigen::Vector3d dx = H.ldlt().solve(b);
		if (isnan(dx[0]))
		{
			std::cout << "result is nan!" << std::endl;
			break;
		}
		if (iter > 0 && cost >= lastCost) {
			std::cout << "cost: " << cost << ">= last cost: " << lastCost << ", break." << std::endl;
			break;
		}
		// for next iteration parameter update
		ae += dx[0];
		be += dx[1];
		ce += dx[2];
		lastCost = cost;
		std::cout << "total cost: " << cost << ", \t\tupdate: " << dx.transpose() <<
			"\t\testimated params: " << ae << "," << be << "," << ce << std::endl;
	}
	std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
	std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
	std::cout << "solve time cost = " << time_used.count() << " seconds. " << std::endl;
	std::cout << "estimated abc = " << ae << ", " << be << ", " << ce << std::endl;
	system("pause");
	return 0;
}

/*
 * @Author: Yuhao Li
 * @Date: 2021-12-20 10:08:48
 * @LastEditTime: 2021-12-24 16:44:19
 * @LastEditors: Yuhao Li
 * @Description: main function interface
 * @FilePath: \Math\main.cpp
 */

// STL
#include <iostream>
// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
// Local
#include "DualQuaternion.h"

int main()
{
	Eigen::Quaternionf q;
	q = Eigen::AngleAxisf(PI / 4.f, Eigen::Vector3f::UnitX())*
		Eigen::AngleAxisf(PI / 4.f, Eigen::Vector3f::UnitY())*
		Eigen::AngleAxisf(PI / 4.f, Eigen::Vector3f::UnitZ());
	std::cout << "quanternion from eigen: \n" << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << std::endl;
	Eigen::Vector3f t = Eigen::Vector3f(2.f, 1.f, 3.f);
	std::cout << "translation: \n" << t.transpose() << std::endl;
	float *f_q = new float[4];
	f_q[0] = q.x();
	f_q[1] = q.y();
	f_q[2] = q.z();
	f_q[3] = q.w();
	float *f_t = new float[3];
	f_t[0] = t[0];
	f_t[1] = t[1];
	f_t[2] = t[2];
	float(*f_dq)[4] = new float[2][4];
	//float **f_dq = new float *[2];
	math::QuatTrans2UDQ(f_q, f_t, f_dq);
	std::cout << "udq:" << std::endl;
	for (int i = 0; i < 2; ++i)
	{
		std::cout << f_dq[i][0] << "," << f_dq[i][1] << "," << f_dq[i][2] << "," << f_dq[i][3] << std::endl;
	}
	float *f_q2 = new float[4];
	float *f_t2 = new float[3];
	math::UDQ2QuatTrans(f_dq, f_q2, f_t2);
	std::cout << "udq to q: \n" << f_q2[0] << "," << f_q2[1] << "," << f_q2[2] << "," << f_q2[3] << std::endl;
	std::cout << "udq to t: \n" << f_t2[0] << "," << f_t2[1] << "," << f_t2[2] << std::endl;
	// release memory
	// FIXME
	//for (int i = 0; i < 2; ++i)
	//{
	//	delete [](f_dq[i]);
	//}
	delete[]f_dq;
	delete[]f_q;
	delete[]f_q2;
	delete[]f_t;
	delete[]f_t2;
	system("pause");
	return 1;
}
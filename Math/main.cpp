/* dqconv.c

  Conversion routines between (regular quaternion, translation) and dual quaternion.

  Version 1.0.0, February 7th, 2007

  Copyright (C) 2006-2007 University of Dublin, Trinity College, All Rights
  Reserved

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the author(s) be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
	 claim that you wrote the original software. If you use this software
	 in a product, an acknowledgment in the product documentation would be
	 appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
	 misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.

  Author: Ladislav Kavan, ladislav.kavan@gmail.com
  From: https://www.cs.utah.edu/~ladislav/dq/index.html
*/

#include <iostream>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#define PI 3.1415926f
/*
Benefits
	eliminates skin collapsing artifacts
	GPU friendly implementation
	easy rigging: uses the same model files as standard linear blend skinning
	very simple update of a 3D engine or similar software
Limitations
	slightly slower than linear blend skinning (in our implementation: 7 more vertex shader instructions)
	scale/shear is now supported (via two-phase skinning - extra 29 vertex shader instructions)
	flipping artifacts (skin always goes the shorter way round)
*/

// input: unit quaternion 'q0', translation vector 't' 
// output: unit dual quaternion 'dq'
void QuatTrans2UDQ(const float q0[4], const float t[3], float dq[2][4])
{
   // non-dual part (just copy q0):
	for (int i = 0; i < 4; i++) dq[0][i] = q0[i];
	// dual part:
	dq[1][0] = -0.5f*(t[0] * q0[1] + t[1] * q0[2] + t[2] * q0[3]);
	dq[1][1] = 0.5f*(t[0] * q0[0] + t[1] * q0[3] - t[2] * q0[2]);
	dq[1][2] = 0.5f*(-t[0] * q0[3] + t[1] * q0[0] + t[2] * q0[1]);
	dq[1][3] = 0.5f*(t[0] * q0[2] - t[1] * q0[1] + t[2] * q0[0]);
}

// input: unit dual quaternion 'dq'
// output: unit quaternion 'q0', translation vector 't'
void UDQ2QuatTrans(const float dq[2][4], float q0[4], float t[3])
{
   // regular quaternion (just copy the non-dual part):
	for (int i = 0; i < 4; i++) q0[i] = dq[0][i];
	// translation vector:
	t[0] = 2.0f*(-dq[1][0] * dq[0][1] + dq[1][1] * dq[0][0] - dq[1][2] * dq[0][3] + dq[1][3] * dq[0][2]);
	t[1] = 2.0f*(-dq[1][0] * dq[0][2] + dq[1][1] * dq[0][3] + dq[1][2] * dq[0][0] - dq[1][3] * dq[0][1]);
	t[2] = 2.0f*(-dq[1][0] * dq[0][3] - dq[1][1] * dq[0][2] + dq[1][2] * dq[0][1] + dq[1][3] * dq[0][0]);
}

// input: dual quat. 'dq' with non-zero non-dual part
// output: unit quaternion 'q0', translation vector 't'
void DQ2QuatTrans(const float dq[2][4], float q0[4], float t[3])
{
	float len = 0.0f;
	for (int i = 0; i < 4; i++) len += dq[0][i] * dq[0][i];
	len = sqrt(len);
	for (int i = 0; i < 4; i++) q0[i] = dq[0][i] / len;
	t[0] = 2.0f*(-dq[1][0] * dq[0][1] + dq[1][1] * dq[0][0] - dq[1][2] * dq[0][3] + dq[1][3] * dq[0][2]) / len;
	t[1] = 2.0f*(-dq[1][0] * dq[0][2] + dq[1][1] * dq[0][3] + dq[1][2] * dq[0][0] - dq[1][3] * dq[0][1]) / len;
	t[2] = 2.0f*(-dq[1][0] * dq[0][3] - dq[1][1] * dq[0][2] + dq[1][2] * dq[0][1] + dq[1][3] * dq[0][0]) / len;
}

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
	QuatTrans2UDQ(f_q, f_t, f_dq);
	std::cout << "udq:" << std::endl;
	for (int i = 0; i < 2; ++i)
	{
		std::cout << f_dq[i][0] << "," << f_dq[i][1] << "," << f_dq[i][2] << "," << f_dq[i][3] << std::endl;
	}
	float *f_q2 = new float[4];
	float *f_t2 = new float[3];
	UDQ2QuatTrans(f_dq, f_q2, f_t2);
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
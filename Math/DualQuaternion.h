/*
 * @Author: Yuhao Li
 * @Date: 2021-12-24 16:24:43
 * @LastEditTime: 2021-12-24 16:44:10
 * @LastEditors: Yuhao Li
 * @Description: class of dual quaternion 
 * @FilePath: \Math\DualQuaternion.h
 */
#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#define PI 3.1415926f

namespace math
{
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
	class DualQuaternion 
	{
	private:
		Eigen::Quaterniond r_;
		Eigen::Vector3d t_;
		Eigen::Quaterniond d_;
		friend DualQuaternion operator*(const DualQuaternion &_lhs,
			const DualQuaternion &_rhs);

	public:
		DualQuaternion(const Eigen::Quaterniond &_r, const Eigen::Vector3d &_t)
			: r_{ _r }, t_{ _t },
			d_{ Eigen::Quaterniond(0, _t[0] / 2, _t[1] / 2, _t[2] / 2) * _r } {}

		DualQuaternion(const Eigen::Quaterniond &_r)
			: DualQuaternion(_r, Eigen::Vector3d(0, 0, 0)) {}

		DualQuaternion(const Eigen::Vector3d &_t)
			: DualQuaternion(Eigen::Quaterniond(1, 0, 0, 0), _t) {}

		DualQuaternion(const Eigen::Quaterniond &_r, const Eigen::Quaterniond &_d)
			: r_{ _r }, d_{ _d } {
			const Eigen::Quaterniond qt = this->d_ * this->r_.inverse();
			t_ = Eigen::Vector3d(2 * qt.x(), 2 * qt.y(), 2 * qt.z());
		}

		const Eigen::Quaterniond &r() const { return r_; }

		const Eigen::Quaterniond &d() const { return d_; }

		const Eigen::Vector3d &t() const { return t_; }

		// Transform inverse.
		DualQuaternion inverse() const {
			return DualQuaternion(this->r_.inverse(), this->r_.inverse() * (-this->t_));
		}

		DualQuaternion conjugate() const {
			const Eigen::Quaterniond d_tmp(-this->d_.w(), this->d_.x(), this->d_.y(), this->d_.z());
			return DualQuaternion(this->r_.inverse(), d_tmp);
		}

		Eigen::Vector3d transformPoint(const Eigen::Vector3d &_p) const {
			const DualQuaternion dp0(Eigen::Quaterniond(1, 0, 0, 0),
				Eigen::Quaterniond(0, _p[0], _p[1], _p[2]));
			const DualQuaternion dp1 = (*this) * dp0 * (this->conjugate());
			return Eigen::Vector3d(dp1.d_.x(), dp1.d_.y(), dp1.d_.z());
		}
	};

	DualQuaternion operator*(const DualQuaternion &_lhs,
		const DualQuaternion &_rhs) {
		const Eigen::Quaterniond r = (_lhs.r_ * _rhs.r_).normalized();
		const Eigen::Quaterniond tmp0 = _lhs.r_ * _rhs.d_;
		const Eigen::Quaterniond tmp1 = _lhs.d_ * _rhs.r_;
		const Eigen::Quaterniond qd(tmp0.w() + tmp1.w(), tmp0.x() + tmp1.x(),
			tmp0.y() + tmp1.y(), tmp0.z() + tmp1.z());
		const Eigen::Quaterniond qt = qd * r.inverse();

		const Eigen::Vector3d t(2 * qt.x(), 2 * qt.y(), 2 * qt.z());

		return DualQuaternion(r, t);
	}
	// From: https://www.cs.utah.edu/~ladislav/dq/index.html
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

}
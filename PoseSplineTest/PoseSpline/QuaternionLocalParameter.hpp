/*
 * @Author: whu-lyh yhaoli@whu.edu.cn
 * @Date: 2023-02-15 14:55:35
 * @LastEditors: whu-lyh yhaoli@whu.edu.cn
 * @LastEditTime: 2023-03-04 10:50:41
 * @FilePath: \PoseSpline\include\PoseSpline\QuaternionLocalParameter.hpp
 * @Description: QuaternionLocalParameter is different to the ceres implementation, quaternion order is xyzw
 */
#ifndef QUATERNIONLOCALPARAMETER_H
#define QUATERNIONLOCALPARAMETER_H

// Eigen
#include <Eigen/Dense>
// ceres
#include <ceres/ceres.h>
// Local
#include "Quaternion.hpp"

class QuaternionLocalParameter : public ceres::LocalParameterization
{
public:
    virtual ~QuaternionLocalParameter()
    {
    }
    // GlobalSize > LocalSize
    virtual int GlobalSize() const { return 4; };
    virtual int LocalSize() const { return 3; };
    // more details in TestNotes
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const
    {
        Eigen::Map<const Quaternion> Q_(x); // Q bar
        Eigen::Map<const Eigen::Vector3d> delta_phi(delta);
        Quaternion dq;
        dq << 0.5 * delta_phi, 1.0;
        // x_plus_delta is the output result
        Eigen::Map<Quaternion> Q_plus(x_plus_delta);
        // $\delta q \times \bar{q}$
        Q_plus = quatLeftComp(dq) * Q_;
        // normilize
        Q_plus = Q_plus / Q_plus.norm();
        return true;
    }
    virtual bool ComputeJacobian(const double *x, double *jacobian) const
    {
        plusJacobian(x, jacobian);
        return true;
    }
    static bool plusJacobian(const double *x, double *jacobian)
    {
        Eigen::Map<const Quaternion> Q_(x);
        Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> J(jacobian);
        /*
        Eigen::Matrix<double, 4, 3> m;
        m.setZero();
        m.topLeftCorner(3,3).setIdentity();
        J = quatRightComp<double>(Q_)*0.5*m;
         */
        // more effient that directly quatRightComp
        // quatLeftComp (i believe), ???
        J.setZero();
        J(0, 0) = Q_(3);
        J(0, 1) = -Q_(2);
        J(0, 2) = Q_(1);
        J(1, 0) = Q_(2);
        J(1, 1) = Q_(3);
        J(1, 2) = -Q_(0);
        J(2, 0) = -Q_(1);
        J(2, 1) = Q_(0);
        J(2, 2) = Q_(3);
        J(3, 0) = -Q_(0);
        J(3, 1) = -Q_(1);
        J(3, 2) = -Q_(2);
        J *= 0.5;
        return true;
    }
    // Additional interface
    bool ComputeLiftJacobian(const double *x, double *jacobian) const
    {
        liftJacobian<double>(x, jacobian);
        return true;
    }
    template <typename T>
    static bool liftJacobian(const T *x, T *jacobian)
    {
        Eigen::Map<Eigen::Matrix<T, 3, 4, Eigen::RowMajor>> J_lift(jacobian);
        // QuaternionTemplate = Eigen style Quaternion
        QuaternionTemplate<T> q_inv(-x[0], -x[1], -x[2], x[3]);
        Eigen::Matrix<T, 3, 4> Jq_pinv;
        Jq_pinv.setZero();
        Jq_pinv.template topLeftCorner<3, 3>() = Eigen::Matrix<T, 3, 3>::Identity() * T(2.0);
        J_lift = Jq_pinv * quatRightComp(q_inv);
        return true;
    }
};

#endif // QUATERNIONLOCALPARAMETER_H
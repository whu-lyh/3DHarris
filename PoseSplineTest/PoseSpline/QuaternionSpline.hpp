/*
 * @Author: whu-lyh yhaoli@whu.edu.cn
 * @Date: 2023-02-15 14:55:35
 * @LastEditors: whu-lyh yhaoli@whu.edu.cn
 * @LastEditTime: 2023-03-05 10:26:59
 * @FilePath: \PoseSpline\include\PoseSpline\QuaternionSpline.hpp
 * @Description: QuaternionSpline class
 */
#ifndef QUATERNIONSPLINE_H
#define QUATERNIONSPLINE_H

// Local
#include "Quaternion.hpp"
#include "BSplineBase.hpp"

class QuaternionSpline : public BSplineBase<Quaternion, 4>
{
public:
    typedef BSplineBase<Quaternion, 4> base_t;
    QuaternionSpline();
    QuaternionSpline(double interval);
    virtual ~QuaternionSpline();
    // initial quaternion spline based on the quaternion samples
    void initialQuaternionSpline(std::vector<std::pair<double, Quaternion>> Meas);
    void printKnots();
    Quaternion evalQuatSpline(real_t t);
    Quaternion evalDotQuatSpline(real_t t);
    Quaternion evalDotDotQuatSpline(real_t t);
    void evalQuatSplineDerivate(real_t t,
                                double *Quat,
                                double *dot_Qaut = NULL,
                                double *dot_dot_Quat = NULL);

public:
    Eigen::Vector3d evalOmega(real_t t);
    Eigen::Vector3d evalAlpha(real_t t);
    Eigen::Vector3d evalNumRotOmega(real_t t);
};

#endif // QUATERNIONSPLINE_H

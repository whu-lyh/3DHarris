/*
 * @Author: whu-lyh yhaoli@whu.edu.cn
 * @Date: 2023-02-15 14:55:35
 * @LastEditors: whu-lyh yhaoli@whu.edu.cn
 * @LastEditTime: 2023-03-03 16:29:10
 * @FilePath: \PoseSpline\include\PoseSpline\PoseSpline.hpp
 * @Description: PoseSpline class
 */
#ifndef POSE_SPLINE_H_
#define POSE_SPLINE_H_

// Local
#include "BSplineBase.hpp"
#include "Pose.hpp"

// ElementType = Pose<double>, spline order = 4
class PoseSpline : public BSplineBase<Pose<double>, 4>
{
public:
    // constructor function
    PoseSpline();
    PoseSpline(double interval);
    virtual ~PoseSpline() {}
    // initialization, the samples are used to estimate the coefficents of control points
    void initialPoseSpline(std::vector<std::pair<double, Pose<double>>> Meas);
    // evaluation function
    Pose<double> evalPoseSpline(real_t t);
    Eigen::Vector3d evalLinearVelocity(real_t t);
    Eigen::Vector3d evalLinearAccelerator(real_t t, const Eigen::Vector3d &gravity);
    Eigen::Vector3d evalOmega(real_t t);
};

#endif // POSE_SPLINE_H_

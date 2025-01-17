/*
 * @Author: whu-lyh yhaoli@whu.edu.cn
 * @Date: 2023-02-15 14:55:35
 * @LastEditors: whu-lyh yhaoli@whu.edu.cn
 * @LastEditTime: 2023-03-04 16:11:36
 * @FilePath: \PoseSpline\include\PoseSpline\PoseSplineSampleError.hpp
 * @Description: PoseSplineSampleError class, error dimension is 6
 */
#ifndef POSESPLINESAMPLEERROR_H
#define POSESPLINESAMPLEERROR_H

// STL
#include <iostream>
// ceres
#include <ceres/ceres.h>
// Local
#include "QuaternionSpline.hpp"
#include "QuaternionLocalParameter.hpp"
#include "ErrorInterface.hpp"
#include "Pose.hpp"

class PoseSplineSampleError
    : public ceres::SizedCostFunction<6, 7, 7, 7, 7>
{
public:
    typedef Eigen::Matrix<double, 6, 6> covariance_t;
    typedef covariance_t information_t;
    PoseSplineSampleError(double t_meas, Pose<double> T_meas);
    virtual ~PoseSplineSampleError();
    // ceres will call this virtual function
    virtual bool Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const;
    // This evaluates the error term and additionally computes
    // the Jacobians in the minimal internal representation.
    bool EvaluateWithMinimalJacobians(double const *const *parameters,
                                      double *residuals,
                                      double **jacobians,
                                      double **jacobiansMinimal) const;

private:
    double t_meas_;
    Pose<double> T_Meas_;
    mutable information_t information_;           ///< The information matrix for this error term.
    mutable information_t squareRootInformation_; ///< The square root information matrix for this error term.
};

#endif // POSESPLINESAMPLEERROR_H
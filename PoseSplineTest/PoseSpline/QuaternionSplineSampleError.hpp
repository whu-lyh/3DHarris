/*
 * @Author: whu-lyh yhaoli@whu.edu.cn
 * @Date: 2023-02-15 14:55:35
 * @LastEditors: whu-lyh yhaoli@whu.edu.cn
 * @LastEditTime: 2023-03-05 10:21:27
 * @FilePath: \PoseSpline\include\PoseSpline\QuaternionSplineSampleError.hpp
 * @Description: QuaternionSplineSampleError
 */
#ifndef QUATERNIONSPLINESAMPLEERROR_H
#define QUATERNIONSPLINESAMPLEERROR_H

// STL
#include <iostream>
// ceres
#include <ceres/ceres.h>
// Local
#include "QuaternionSpline.hpp"
#include "QuaternionLocalParameter.hpp"
#include "ErrorInterface.hpp"

/*
 * QuaternionSplineSampleError
 *
 * A derived Ceres cost functor to compute the Quaternon errer between quaternion measurement and
 * the evaluation value from spline. For cubic bspline, the evaluation at time t is related to four
 * contor points, so this functor requires four paramters.
 *
 * To simplified the jacobian, we define error = /hat ominus /meas.
 *
 * @Author Pang Fumin
 */

class QuaternionSplineSampleError
    : public ceres::SizedCostFunction<3, 4, 4, 4, 4>
{
public:
    typedef Eigen::Matrix<double, 3, 3> covariance_t;
    typedef covariance_t information_t;

    QuaternionSplineSampleError(double t_meas, Quaternion Q_meas);
    virtual ~QuaternionSplineSampleError();
    // ceres virtual function api
    virtual bool Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const;
    // minimal jacobian?
    bool EvaluateWithMinimalJacobians(double const *const *parameters,
                                      double *residuals,
                                      double **jacobians,
                                      double **jacobiansMinimal) const;

    bool VerifyJacobianNumDiff(double const *const *parameters);

private:
    double t_meas_;
    Quaternion Q_Meas_;
    mutable information_t information_;           ///< The information matrix for this error term.
    mutable information_t squareRootInformation_; ///< The square root information matrix for this error term.
};

#endif // QUATERNIONSPLINESAMPLEERROR_H
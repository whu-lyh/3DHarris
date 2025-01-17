/*
 * @Author: whu-lyh yhaoli@whu.edu.cn
 * @Date: 2023-02-15 14:55:35
 * @LastEditors: whu-lyh yhaoli@whu.edu.cn
 * @LastEditTime: 2023-03-04 21:47:05
 * @FilePath: \PoseSpline\include\PoseSpline\PoseSplineUtility.hpp
 * @Description: Class: Pose spline utility
 */
#ifndef POSESPLINEUTILITY_H
#define POSESPLINEUTILITY_H

// Local
#include "QuaternionSplineUtility.hpp"
#include "Pose.hpp"

class PSUtility
{
public:
    // infact this evaluate function is the interpolation function api
    // return the pose at u timestamp based on the given 4 control points' pose
    static Pose<double> EvaluatePS(double u, const Pose<double> &P0, const Pose<double> &P1,
                                   const Pose<double> &P2, const Pose<double> &P3);
    // interpolate a linear velocity
    static Eigen::Vector3d EvaluateLinearVelocity(double u, double dt,
                                                  const Eigen::Vector3d &V0,
                                                  const Eigen::Vector3d &V1,
                                                  const Eigen::Vector3d &V2,
                                                  const Eigen::Vector3d &V3);
    // interpolate a position
    static Eigen::Vector3d EvaluatePosition(double u,
                                            const Eigen::Vector3d &V0,
                                            const Eigen::Vector3d &V1,
                                            const Eigen::Vector3d &V2,
                                            const Eigen::Vector3d &V3);
    // interpolate a linear accelerate
    static Eigen::Vector3d EvaluateLinearAccelerate(double u, double dt,
                                                    const Pose<double> &P0, const Pose<double> &P1,
                                                    const Pose<double> &P2, const Pose<double> &P3,
                                                    const Eigen::Vector3d &gravity);
};

class PoseSplineEvaluation
{
public:
    typedef Eigen::Matrix<double, 3, 3> TranslationJacobian;
    typedef Eigen::Matrix<double, 4, 3> RotationJacobian;
    // () operator override for interpolation
    Pose<double> operator()(double u,
                            const Pose<double> &P0,
                            const Pose<double> &P1,
                            const Pose<double> &P2,
                            const Pose<double> &P3);
    template <int D>
    TranslationJacobian getTranslationJacobian()
    {
        if (D == 0)
        {
            return JacobianTrans0;
        }
        else if (D == 1)
        {
            return JacobianTrans1;
        }
        else if (D == 2)
        {
            return JacobianTrans2;
        }
        else
        {
            return JacobianTrans3;
        }
    }
    template <int D>
    RotationJacobian getRotationJacobianMinimal()
    {
        if (D == 0)
        {
            return JacobianRotate0;
        }
        else if (D == 1)
        {
            return JacobianRotate1;
        }
        else if (D == 2)
        {
            return JacobianRotate2;
        }
        else
        {
            return JacobianRotate3;
        }
    }
    inline Quaternion getRotation()
    {
        return Q;
    }
    inline Eigen::Vector3d getTranslation()
    {
        return t;
    }

private:
    Quaternion Q;
    Eigen::Vector3d t;
    TranslationJacobian JacobianTrans0, JacobianTrans1, JacobianTrans2, JacobianTrans3;
    RotationJacobian JacobianRotate0, JacobianRotate1, JacobianRotate2, JacobianRotate3;
};

#endif // POSESPLINEUTILITY_H


#include <ceres/ceres.h>
// Local
#include "PoseLocalParameter.hpp"
#include "PoseSpline.hpp"
#include "PoseSplineUtility.hpp"
#include "PoseSplineSampleError.hpp"

PoseSpline::PoseSpline() : BSplineBase(1.0)
{
}

PoseSpline::PoseSpline(double interval) : BSplineBase(interval)
{
}

void PoseSpline::initialPoseSpline(std::vector<std::pair<double, Pose<double>>> Meas)
{
    // Build a least-square problem
    ceres::Problem problem;
    PoseLocalParameter *poseLocalParameter = new PoseLocalParameter;
    // std::cout<<"Meas NUM: "<<Meas.size()<<std::endl;
    // optimization incrementally?
    for (auto Mea : Meas)
    {
        // add sample(timestamp,pose) to spline
        addElemenTypeSample(Mea.first, Mea.second);
        // Returns the normalized u value and the lower-bound time index.
        std::pair<double, unsigned int> ui = computeUAndTIndex(Mea.first);
        // the u is the lower bound knot index which associates to the t(timestamp) value
        double u = ui.first;
        int bidx = ui.second - this->spline_order() + 1;
        // get the related spline knot(control point) that will be effected by the current t(timestamp) value
        // cpi is the variables that will be fed into problem and conduct optimization
        // 4 related to the order which is 4 defined while construction
        double *cp0 = getControlPoint(bidx);
        double *cp1 = getControlPoint(bidx + 1);
        double *cp2 = getControlPoint(bidx + 2);
        double *cp3 = getControlPoint(bidx + 3);
        // add variables into parameterblock, 7 is variable dimension
        problem.AddParameterBlock(cp0, 7, poseLocalParameter);
        problem.AddParameterBlock(cp1, 7, poseLocalParameter);
        problem.AddParameterBlock(cp2, 7, poseLocalParameter);
        problem.AddParameterBlock(cp3, 7, poseLocalParameter);
        // error cost functor
        PoseSplineSampleError *poseSampleFunctor = new PoseSplineSampleError(u, Mea.second);
        // no kernel function
        problem.AddResidualBlock(poseSampleFunctor, NULL, cp0, cp1, cp2, cp3);
    }
    //std::cout << "parameternum: " << problem.numparameterblocks() << std::endl;
    //std::cout << "residualnum: " << problem.numresiduals() << std::endl;
    // Run the solver!
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.max_solver_time_in_seconds = 30;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.parameter_tolerance = 1e-4;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;
}

Pose<double> PoseSpline::evalPoseSpline(real_t t)
{
    std::pair<double, unsigned int> ui = computeUAndTIndex(t);
    double u = ui.first;
    unsigned int bidx = ui.second - spline_order() + 1;
    // get knot's parameter's translation
    Eigen::Map<Eigen::Matrix<double, 3, 1>> t0(getControlPoint(bidx));
    Eigen::Map<Eigen::Matrix<double, 3, 1>> t1(getControlPoint(bidx + 1));
    Eigen::Map<Eigen::Matrix<double, 3, 1>> t2(getControlPoint(bidx + 2));
    Eigen::Map<Eigen::Matrix<double, 3, 1>> t3(getControlPoint(bidx + 3));
    // std::cout << "t0: " << t0.transpose() << std::endl;
    // std::cout << "t1: " << t1.transpose() << std::endl;
    // std::cout << "t2: " << t2.transpose() << std::endl;
    // std::cout << "t3: " << t3.transpose() << std::endl;
    Eigen::Map<Eigen::Matrix<double, 4, 1>> q0(getControlPoint(bidx) + 3);
    Eigen::Map<Eigen::Matrix<double, 4, 1>> q1(getControlPoint(bidx + 1) + 3);
    Eigen::Map<Eigen::Matrix<double, 4, 1>> q2(getControlPoint(bidx + 2) + 3);
    Eigen::Map<Eigen::Matrix<double, 4, 1>> q3(getControlPoint(bidx + 3) + 3);
    // construct poses from t and q
    return PSUtility::EvaluatePS(u,
                                 Pose<double>(t0, q0), Pose<double>(t1, q1),
                                 Pose<double>(t2, q2), Pose<double>(t3, q3));
}

Eigen::Vector3d PoseSpline::evalLinearVelocity(real_t t)
{
    std::pair<double, unsigned int> ui = computeUAndTIndex(t);
    double u = ui.first;
    unsigned int bidx = ui.second - spline_order() + 1;
    Eigen::Map<Eigen::Matrix<double, 3, 1>> t0(getControlPoint(bidx));
    Eigen::Map<Eigen::Matrix<double, 3, 1>> t1(getControlPoint(bidx + 1));
    Eigen::Map<Eigen::Matrix<double, 3, 1>> t2(getControlPoint(bidx + 2));
    Eigen::Map<Eigen::Matrix<double, 3, 1>> t3(getControlPoint(bidx + 3));

    return PSUtility::EvaluateLinearVelocity(u, getTimeInterval(), t0, t1, t2, t3);
}

Eigen::Vector3d PoseSpline::evalLinearAccelerator(real_t t, const Eigen::Vector3d &gravity)
{
    std::pair<double, unsigned int> ui = computeUAndTIndex(t);
    double u = ui.first;
    unsigned int bidx = ui.second - spline_order() + 1;
    Eigen::Map<Eigen::Matrix<double, 7, 1>> t0(getControlPoint(bidx));
    Eigen::Map<Eigen::Matrix<double, 7, 1>> t1(getControlPoint(bidx + 1));
    Eigen::Map<Eigen::Matrix<double, 7, 1>> t2(getControlPoint(bidx + 2));
    Eigen::Map<Eigen::Matrix<double, 7, 1>> t3(getControlPoint(bidx + 3));
    Pose<double> Pose0, Pose1, Pose2, Pose3;
    Pose0.setParameters(t0);
    Pose1.setParameters(t1);
    Pose2.setParameters(t2);
    Pose3.setParameters(t3);

    return PSUtility::EvaluateLinearAccelerate(u, getTimeInterval(),
                                               Pose0, Pose1,
                                               Pose2, Pose3, gravity);
}

Eigen::Vector3d PoseSpline::evalOmega(real_t t)
{
    std::pair<double, unsigned int> ui = computeUAndTIndex(t);
    double u = ui.first;
    unsigned int bidx = ui.second - spline_order() + 1;

    Quaternion Q0 = quatMap<double>(getControlPoint(bidx) + 3);
    Quaternion Q1 = quatMap<double>(getControlPoint(bidx + 1) + 3);
    Quaternion Q2 = quatMap<double>(getControlPoint(bidx + 2) + 3);
    Quaternion Q3 = quatMap<double>(getControlPoint(bidx + 3) + 3);

    double b1 = QSUtility::beta1(u);
    double b2 = QSUtility::beta2(u);
    double b3 = QSUtility::beta3(u);

    Eigen::Vector3d Phi1 = QSUtility::Phi(Q0, Q1);
    Eigen::Vector3d Phi2 = QSUtility::Phi(Q1, Q2);
    Eigen::Vector3d Phi3 = QSUtility::Phi(Q2, Q3);

    Quaternion r1 = QSUtility::r(b1, Phi1);
    Quaternion r2 = QSUtility::r(b2, Phi2);
    Quaternion r3 = QSUtility::r(b3, Phi3);

    Quaternion Q_WI = quatLeftComp(Q0) * quatLeftComp(r1) * quatLeftComp(r2) * r3;

    double dot_b1 = QSUtility::dot_beta1(getTimeInterval(), u);
    double dot_b2 = QSUtility::dot_beta2(getTimeInterval(), u);
    double dot_b3 = QSUtility::dot_beta3(getTimeInterval(), u);

    Quaternion dot_Q_WI, part1, part2, part3;
    part1 = quatLeftComp(QSUtility::dr_dt(dot_b1, b1, Phi1)) * quatLeftComp(r2) * r3;
    part2 = quatLeftComp(r1) * quatLeftComp(QSUtility::dr_dt(dot_b2, b2, Phi2)) * r3;
    part3 = quatLeftComp(r1) * quatLeftComp(r2) * QSUtility::dr_dt(dot_b3, b3, Phi3);

    dot_Q_WI = quatLeftComp(Q0) * (part1 + part2 + part3);

    // std::cout<<"Q    : "<<Q_LG.transpose()<<std::endl;
    // std::cout<<"dot_Q: "<<dot_Q_LG.transpose()<<std::endl;
    return QSUtility::w_in_body_frame<double>(Q_WI, dot_Q_WI);
}
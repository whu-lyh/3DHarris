/*
 * @Author: whu-lyh yhaoli@whu.edu.cn
 * @Date: 2023-02-15 14:55:35
 * @LastEditors: whu-lyh yhaoli@whu.edu.cn
 * @LastEditTime: 2023-03-05 12:33:53
 * @FilePath: \PoseSpline\include\PoseSpline\BSplineBase.hpp
 * @Description: BSplineBase class
 */

#ifndef _BSPLINE_BASE_H_
#define _BSPLINE_BASE_H_

// STL
#include <vector>
// Eigen
#include <Eigen/Core>
// Local
#include "TypeTraits.hpp"

using real_t = double;

template <typename ElementType, int SplineOrder>
class BSplineBase
{
public:
    BSplineBase(double interval) : mSplineOrder(SplineOrder), mTimeInterval(interval)
    {
    }
    // @Noreturn destory function and clean the control point memory
    virtual ~BSplineBase()
    {
        for (auto i : mControlPointsParameter)
            delete[] i;
        mControlPointsParameter.clear();
    };
    // @return The order of the spline.
    int spline_order() const
    {
        return SplineOrder;
    }
    // @return The degree of polynomial used by the spline, degree = order -1
    int polynomialDegree() const
    {
        return SplineOrder - 1;
    }
    // @return the minimum number of knots, required to have at least one valid time segment.
    int minimumKnotsRequired() const
    { // 1 for only one segment, the number of knots related to the spline order
        return numKnotsRequired(1);
    }
    // @return the minimum number of coefficients required to the num_time_segments
    int numCoefficientsRequired(int num_time_segments) const
    {
        return num_time_segments + SplineOrder - 1;
    }
    // @return the number of knots required to the num_time_segments
    int numKnotsRequired(int num_time_segments) const
    {
        return numCoefficientsRequired(num_time_segments) + SplineOrder;
    }
    // @return the minimum knot index-value, here is timestamp
    real_t t_min() const
    {
        //CHECK_GE((int)knots_.size(), minimumKnotsRequired()) << "The B-spline is not well initialized";
        return knots_[SplineOrder - 1];
    }
    // @return the maximum knot index-value, here is timestamp too
    real_t t_max() const
    {
        //CHECK_GE((int)knots_.size(), minimumKnotsRequired()) << "The B-spline is not well initialized";
        return knots_[knots_.size() - SplineOrder];
    }
    // @return the knot index based on time
    std::pair<real_t, int> computeTIndex(real_t t) const
    {
        //CHECK_GE(t, t_min()) << "The time is out of range by " << (t - t_min());
        // avoids numerical problems on initialisation
        if (std::abs(t_max() - t) < 1e-10)
        {
            t = t_max();
        }
        //CHECK_LE(t, t_max()) << "The time is out of range by " << (t_max() - t);
        std::vector<real_t>::const_iterator i;
        if (t == t_max())
        {
            // This is a special case to allow us to evaluate the spline at the boundary of the
            // interval. This is not stricly correct but it will be useful when we start doing
            // estimation and defining knots at our measurement times.
            i = knots_.end() - SplineOrder;
        }
        else
        { // get the knot index of t in a ascending vector container
            i = std::upper_bound(knots_.begin(), knots_.end(), t);
        }
        // CHECK_NE(i, knots_.end()) << "Something very bad has happened in computeTIndex(" << t << ")";
        //  Returns the index of the knot segment this time lies on and the width of this knot segment.
        //  the first always 1?
        return std::make_pair(*i - *(i - 1), (i - knots_.begin()) - 1);
    }
    // @return to make the pair own some unique characteristic?, u is normalized
    // t is normalized between in t_{t+1}-t_{i}, $u \in [0,1)$
    std::pair<real_t, int> computeUAndTIndex(real_t t) const
    {
        std::pair<real_t, int> ui = computeTIndex(t);
        int index = ui.second;
        real_t denom = ui.first;
        if (denom <= 0.0)
        {
            // The case of duplicate knots.
            // std::cout << "Duplicate knots\n";
            return std::make_pair(0, index);
        }
        else
        {
            real_t u = (t - knots_[index]) / denom;
            return std::make_pair(u, index);
        }
    }
    // @Noreturn set the time interval
    void setTimeInterval(double timeInterval)
    {
        mTimeInterval = timeInterval;
    }
    // @return the time interval
    double getTimeInterval()
    {
        return mTimeInterval;
    }
    // @return if ts is valid by time interval
    bool isTsEvaluable(double ts)
    {
        return ts >= t_min() && ts < t_max();
    }
    // @Noreturn initial the spline based on elementtype and construct empty knots
    void initialSplineKnot(double t)
    {
        // Initialize the spline so that it interpolates the two points
        // and moves between them with a constant velocity.
        // initialize with only 1 segments
        int K = numKnotsRequired(1);
        // the number of coefficients is same with the number of segments
        int C = numCoefficientsRequired(1);
        // Initialize a uniform knot sequence
        real_t dt = mTimeInterval;
        std::vector<real_t> knots(K);
        for (int i = 0; i < K; ++i)
        {
            knots[i] = t + (i - SplineOrder + 1) * dt;
        }
        knots_ = knots;
        // initial control point, whose number is C
        for (int i = 0; i < C; i++)
        {
            initialNewControlPoint();
        }
    }
    // @Noreturn add a new ElementType at t time as control point
    void addElemenTypeSample(double t, ElementType sample)
    {
        // if no control inserted, initial a empty one
        if (getControlPointNum() == 0)
        {
            initialSplineKnot(t);
        }
        else if (getControlPointNum() >= numCoefficientsRequired(1))
        {
            if (t < t_min())
            {
                std::cerr << "[Error] Inserted t is smaller than t_min()" << std::endl;
            }
            else if (t >= t_max())
            {
                // add new knot and control Points
                while (t >= t_max())
                {
                    knots_.push_back(knots_.back() + mTimeInterval); // append one;
                    initialNewControlPoint();
                }
            }
        }
        // Tricky: do not add point close to t_max
        if (t_max() - t > 0.0001)
        {
            mSampleValues.insert(std::pair<double, ElementType>(t, sample));
        }
        //CHECK_EQ(knots_.size() - SplineOrder, getControlPointNum());
    }
    // @Noreturn add new knots prepared for future time
    void addControlPointsUntil(double t)
    {
        if (getControlPointNum() == 0)
        {
            initialSplineKnot(t);
        }
        else if (getControlPointNum() >= numCoefficientsRequired(1))
        {
            if (t < t_min())
            {
                std::cerr << "[Error] Inserted t is smaller than t_min()" << std::endl;
            }
            else if (t >= t_max())
            {
                // add new knot and control Points
                while (t >= t_max())
                {
                    knots_.push_back(knots_.back() + mTimeInterval); // append one;
                    initialNewControlPoint();
                }
            }
        }
        //CHECK_EQ(knots_.size() - SplineOrder, getControlPointNum());
    }
    // @return the number of control points
    inline size_t getControlPointNum()
    {
        return mControlPointsParameter.size();
    }
    // @return the control point i-th
    inline double *getControlPoint(unsigned int i)
    {
        return mControlPointsParameter.at(i);
    }

private:
    // @Noreturn initial an empty control point
    // the ElementType contains Pose, Quqternion(Eigen style) and Eigen::Matrix<double, D,1>
    void initialNewControlPoint()
    {
        typename TypeTraits<ElementType>::TypeT zero_ele = TypeTraits<ElementType>::zero();
        double *data = new double[TypeTraits<ElementType>::Dim];
        memcpy(data, zero_ele.data(), sizeof(double) * TypeTraits<ElementType>::Dim);
        mControlPointsParameter.push_back(data);
    }
    // The knot sequence used by the B-spline.
    // for original knot stores uniformly distributed integer
    // in stature estimation stores timestamp
    std::vector<real_t> knots_;
    std::vector<double *> mControlPointsParameter;
    // the samples are used to estimate the control point's coefficient
    std::map<double, ElementType> mSampleValues;
    int mSplineOrder;
    double mTimeInterval;
};

#endif // _BSPLINE_BASE_H_

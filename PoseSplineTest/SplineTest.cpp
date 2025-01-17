// STL
#include <iostream>
#include <fstream>
#include <iomanip>
// Eigen
#include <Eigen/Core>
// Local
#include "PoseSpline/QuaternionSpline.hpp"
#include "PoseSpline/QuaternionSplineUtility.hpp"
#include "PoseSpline/PoseSpline.hpp"
#include "PoseSpline/VectorSpaceSpline.hpp"

#ifdef _DEBUG
#pragma comment(lib, "glogd.lib")
#pragma comment(lib, "ceres-debug.lib")
#else
#pragma comment(lib, "glog.lib")
#pragma comment(lib, "ceres.lib")
#endif


struct StampedPose
{
	double timestamp_;
    Eigen::Vector3d t_;    // position
    Eigen::Quaterniond q_; // angular
    Eigen::Vector3d v_;    // velocity
	StampedPose(double t, Eigen::Vector3d p, Eigen::Vector3d q, Eigen::Vector3d v)
		:timestamp_(t), t_(p), v_(v)
	{
		Eigen::AngleAxisd rollAngle(q.x(), Eigen::Vector3d::UnitX());
		Eigen::AngleAxisd pitchAngle(q.y(), Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd yawAngle(q.z(), Eigen::Vector3d::UnitZ());
		q_ = rollAngle * pitchAngle * yawAngle;
	}
};
typedef std::vector<StampedPose> PoseVec;

// split string, c++ 11 standard
std::vector<std::string> stringSplit(const std::string &str, const std::string &pattern)
{
	std::vector<std::string> ret;
	if (pattern.empty()) return ret;
	size_t start = 0, index = str.find_first_of(pattern, 0);
	while (index != str.npos)
	{
		if (start != index)
			ret.push_back(str.substr(start, index - start));
		start = index + 1;
		index = str.find_first_of(pattern, start);
	}
	if (!str.substr(start).empty())
		ret.push_back(str.substr(start));
	return ret;
}

bool loadGTFile(const std::string &posFilename, PoseVec &poses)
{
	std::ifstream ifs(posFilename, std::ios_base::in);
	if (!ifs)
	{
		std::cerr << "Fail to load gt pose file: " << posFilename;
		return false;
	}
	std::string line;
	line.resize(1000);
	ifs.getline((char*)line.data(), 1024);
	while (ifs.getline((char*)line.data(), 1024)) {

		std::vector<std::string> values = stringSplit(line, "\t");
		if (values.size() < 17) {
			LOG(ERROR) << "Fail to parse the gt pose record. " << posFilename;
			return false;
		}
		// ImNum Cam CamImNm Rotate Error GPSTime Index Filename East North Height Omega Phi Kappa
		std::string panoName;
		panoName = values[7];
		double gpt_time = atof(values[5].c_str());
		double x = 0.0, y = 0.0, z = 0.0;
		double heading = 0.0, pitch = 0.0, roll = 0.0;
		x = atof(values[8].c_str());
		y = atof(values[9].c_str());
		z = atof(values[10].c_str());
		pitch = atof(values[11].c_str());
		roll = atof(values[12].c_str());
		heading = atof(values[13].c_str());
		poses.emplace_back(StampedPose(gpt_time, Eigen::Vector3d(x, y, z), Eigen::Vector3d::Identity(), Eigen::Vector3d::Identity()));
	}
	ifs.close();
	std::cout << "Succeed to load pose file: " << posFilename << "\t pose numbers: " << poses.size() << std::endl;
	return true;
}

int main()
{
    std::string pose_file = "E:/Yuto_mms/SequenceB/GroundTruth_LBP_YU_campus_UTM_Zone_17N_r.txt";
    std::string imu_meas_file = "E:/Yuto_mms/SequenceB/LidarScanTimestamp.txt";
    // load data
	PoseVec testSample;
	loadGTFile(pose_file, testSample);
    // initialize a PoseSpline, with 0.1 as the time interval
    PoseSpline poseSpline(0.001);
    // vector as spline knot variables
    // 3 for position, while 6 for acceleration and gyroscope
    VectorSpaceSpline<3> vectorSpaceSpline(0.1);
    VectorSpaceSpline<6> vectorSpaceSpline6(0.1);
    // predefined data container
    // samples, positionSamples and position6Samples are samples to be feed into spline 
    // to estimate the control point coefficients
    std::vector<std::pair<double, Pose<double>>> samples, queryMeas;
    std::vector<std::pair<double, Eigen::Matrix<double, 3, 1>>> queryVelocityMeas;
    std::vector<std::pair<double, Eigen::Matrix<double, 3, 1>>> positionSamples;
    std::vector<std::pair<double, Eigen::Matrix<double, 6, 1>>> position6Samples;
    //
    int start = 0;
    int end = testSample.size() / 10;
	std::cout << std::fixed << std::setprecision(3) << std::endl;
    for (int i = start; i < end; i++)
    {
        // fetch a single pose point
        StampedPose stampedPose = testSample[i];
        // coordinate transform
        Eigen::Quaterniond QuatHamilton(stampedPose.q_);
        Eigen::Matrix3d R = QuatHamilton.toRotationMatrix();
        Quaternion QuatJPL = rotMatToQuat(R);
        // construct a pose, necessary data structure convert
        Pose<double> pose(stampedPose.t_, QuatJPL);
        queryMeas.push_back(std::pair<double, Pose<double>>(stampedPose.timestamp_, pose));
        queryVelocityMeas.push_back(std::pair<double, Eigen::Vector3d>(stampedPose.timestamp_, stampedPose.v_));
        // convert to n second
        //std::cout << stampedPose.timestamp_ << std::endl;
        // downsample the input sample measurements
        if (i % 10 == 0)
        {
            samples.push_back(std::pair<double, Pose<double>>(stampedPose.timestamp_, pose));
            positionSamples.push_back(std::pair<double, Eigen::Vector3d>(stampedPose.timestamp_, stampedPose.t_));
			Eigen::Matrix<double, 6, 1> meas(6);
            meas << stampedPose.t_, stampedPose.t_;
            position6Samples.push_back(std::pair<double, Eigen::Matrix<double, 6, 1>>(stampedPose.timestamp_, meas));
        }
    }
    // initialize the spline by samples(sequences of time,pose)
    poseSpline.initialPoseSpline(samples);
    //vectorSpaceSpline.initialSpline(positionSamples); // wired error
    //vectorSpaceSpline6.initialSpline(position6Samples); // wired error
    // Test: pose spline evalPoseSpline
    for (auto pair: queryMeas)
    {
        if (poseSpline.isTsEvaluable(pair.first))
        {// check if located in time interval
            // get a interpolated pose by timestamp and spline model
            // to check if there are artifacts caused by spline fitting
            Pose<double> query = poseSpline.evalPoseSpline(pair.first);
            // pair.second is gt
            std::cout <<"Gt:    "<<pair.second.r().transpose() << " " << pair.second.q().transpose()<<std::endl;
            std::cout <<"Query: "<<query.r().transpose()<<" "<< query.q().transpose()<< std::endl << std::endl;
            // check translation and rotation
            //GTEST_ASSERT_LT((pair.second.r() - query.r()).norm(), 5e-2);
            //GTEST_ASSERT_LT((pair.second.q() - query.q()).norm(), 5e-2);
            // check the translation vector and measure translation vector are equal with in a certain threshold
            //Eigen::Vector3d queryPosition = vectorSpaceSpline.evaluateSpline(pair.first);
            //GTEST_ASSERT_LT((pair.second.r() - queryPosition).norm(), 5e-2);
            // check 6-D vector
            //Eigen::Matrix<double, 6, 1> queryPosition6 = vectorSpaceSpline6.evaluateSpline(pair.first);
            //GTEST_ASSERT_LT((pair.second.r() - queryPosition6.head<3>()).norm(), 5e-2);
            //GTEST_ASSERT_LT((pair.second.r() - queryPosition6.tail<3>()).norm(), 5e-2);
        }
    }
	system("pause");
}
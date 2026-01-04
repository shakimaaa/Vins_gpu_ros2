#ifndef XVINS_SOLVE_5PTS_HPP_
#define XVINS_SOLVE_5PTS_HPP_

/*******************************************************
 * Copyright (C) 2025, Xion Control
 * 
 * This file is part of XVoyager.
 * 
 * Licensed under the MIT License; you may not use this file 
 * except in compliance with the License.
 * 
 * See the LICENSE file distributed with this work for details.
 * 
 * Developed by Xion Control.
 *******************************************************/

#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

#define ROS_INFO RCUTILS_LOG_INFO
#define ROS_WARN RCUTILS_LOG_WARN
#define ROS_DEBUG RCUTILS_LOG_DEBUG
#define ROS_ERROR RCUTILS_LOG_ERROR

namespace xvins {

class MotionEstimator
{
public:
    // Solve for the relative rotation and translation using correspondences
    bool solveRelativeRT(const vector<pair<Vector3d, Vector3d>> &corres, Matrix3d &R, Vector3d &T);

private:
    // Helper functions for triangulation testing and essential matrix decomposition
    double testTriangulation(const vector<cv::Point2f> &l,
                             const vector<cv::Point2f> &r,
                             cv::Mat_<double> R, cv::Mat_<double> t);
    void decomposeE(cv::Mat E,
                    cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                    cv::Mat_<double> &t1, cv::Mat_<double> &t2);
};

} // namespace xvins

#endif // XVINS_SOLVE_5PTS_HPP_

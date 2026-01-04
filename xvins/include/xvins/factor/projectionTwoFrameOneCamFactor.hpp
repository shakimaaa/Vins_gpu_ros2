#ifndef XVINS_PROJECTION_TWO_FRAME_ONE_CAM_FACTOR_HPP_
#define XVINS_PROJECTION_TWO_FRAME_ONE_CAM_FACTOR_HPP_

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

#include <rcpputils/asserts.hpp>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "xvins/utility/utility.hpp"
#include "xvins/utility/tic_toc.hpp"
#include "xvins/estimator/parameters.hpp"
#include "xvins/xvoyager_param.hpp"

namespace xvins {

class ProjectionTwoFrameOneCamFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1, 1>
{
  public:
    ProjectionTwoFrameOneCamFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j,
                                   const Eigen::Vector2d &_velocity_i, const Eigen::Vector2d &_velocity_j,
                                   const double _td_i, const double _td_j);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector3d pts_i, pts_j;
    Eigen::Vector3d velocity_i, velocity_j;
    double td_i, td_j;
    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};

} // namespace xvins

#endif // XVINS_PROJECTION_TWO_FRAME_ONE_CAM_FACTOR_HPP_

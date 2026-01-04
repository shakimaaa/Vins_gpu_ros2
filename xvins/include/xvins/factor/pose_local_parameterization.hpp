#ifndef XVINS_POSE_LOCAL_PARAMETERIZATION_HPP_
#define XVINS_POSE_LOCAL_PARAMETERIZATION_HPP_

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

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include "xvins/utility/utility.hpp"

namespace xvins {

class PoseLocalParameterization : public ceres::Manifold // 2.3.0
{
public:
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const override;
    virtual bool ComputeJacobian(const double *x, double *jacobian) const;
    virtual bool Minus(const double *y, const double *x, double *y_minus_x) const override;
    virtual bool PlusJacobian(const double *x, double *jacobian) const override;
    virtual bool MinusJacobian(const double *x, double *jacobian) const override;
    
    virtual int AmbientSize() const override { return 7; } // Global pose (3D position + quaternion)
    virtual int TangentSize() const override { return 6; } // 6D: translation (3) + rotation (3)
};

} // namespace xvins

#endif // XVINS_POSE_LOCAL_PARAMETERIZATION_HPP_

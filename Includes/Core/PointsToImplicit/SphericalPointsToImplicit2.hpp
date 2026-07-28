// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_SPHERICAL_POINTS_TO_IMPLICIT2_HPP
#define CUBBYFLOW_SPHERICAL_POINTS_TO_IMPLICIT2_HPP

#include <Core/PointsToImplicit/PointsToImplicit2.hpp>

namespace CubbyFlow
{
//!
//! \brief 2-D points-to-implicit converter based on simple sphere model.
//!
class SphericalPointsToImplicit2 final : public PointsToImplicit2
{
 public:
    //! Constructs the converter with given sphere radius.
    explicit SphericalPointsToImplicit2(double radius = 1.0,
                                        bool isOutputSdf = true);

    //! Converts the given points to implicit surface scalar field.
    void Convert(const ConstArrayView1<Vector2D>& points,
                 ScalarGrid2* output) const override;

 private:
    double m_radius = 1.0;
    bool m_isOutputSDF = true;
};

//! Shared pointer type for SphericalPointsToImplicit2.
using SphericalPointsToImplicit2Ptr =
    std::shared_ptr<SphericalPointsToImplicit2>;
}  // namespace CubbyFlow

#endif
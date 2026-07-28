// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_SPHERICAL_POINTS_TO_IMPLICIT3_HPP
#define CUBBYFLOW_SPHERICAL_POINTS_TO_IMPLICIT3_HPP

#include <Core/PointsToImplicit/PointsToImplicit3.hpp>

namespace CubbyFlow
{
//!
//! \brief 3-D points-to-implicit converter based on simple sphere model.
//!
class SphericalPointsToImplicit3 final : public PointsToImplicit3
{
 public:
    //! Constructs the converter with default parameters.
    SphericalPointsToImplicit3() : SphericalPointsToImplicit3(1.0)
    {
        // Do nothing
    }

    //! Constructs the converter with given sphere radius.
    explicit SphericalPointsToImplicit3(double radius, bool isOutputSDF = true);

    //! Converts the given points to implicit surface scalar field.
    void Convert(const ConstArrayView1<Vector3D>& points,
                 ScalarGrid3* output) const override;

 private:
    double m_radius = 1.0;
    bool m_isOutputSDF = true;
};

//! Shared pointer type for SphericalPointsToImplicit3.
using SphericalPointsToImplicit3Ptr =
    std::shared_ptr<SphericalPointsToImplicit3>;
}  // namespace CubbyFlow

#endif

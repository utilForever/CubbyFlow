// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_SPH_POINTS_TO_IMPLICIT2_HPP
#define CUBBYFLOW_SPH_POINTS_TO_IMPLICIT2_HPP

#include <Core/PointsToImplicit/PointsToImplicit2.hpp>

namespace CubbyFlow
{
//!
//! \brief 2-D points-to-implicit converter based on standard SPH kernel.
//!
//! \see Müller, Matthias, David Charypar, and Markus Gross.
//!      "Particle-based fluid simulation for interactive applications."
//!      Proceedings of the 2003 ACM SIGGRAPH/Eurographics symposium on Computer
//!      animation. Eurographics Association, 2003.
//!
class SPHPointsToImplicit2 final : public PointsToImplicit2
{
 public:
    //! Constructs the converter with default parameters.
    SPHPointsToImplicit2() : SPHPointsToImplicit2(1.0)
    {
        // Do nothing
    }

    //! Constructs the converter with given kernel radius and cut-off density.
    explicit SPHPointsToImplicit2(double kernelRadius,
                                  double cutOffDensity = 0.5,
                                  bool isOutputSDF = true);

    //! Converts the given points to implicit surface scalar field.
    void Convert(const ConstArrayView1<Vector2D>& points,
                 ScalarGrid2* output) const override;

 private:
    double m_kernelRadius = 1.0;
    double m_cutOffDensity = 0.5;
    bool m_isOutputSDF = true;
};

//! Shared pointer type for SPHPointsToImplicit2 class.
using SPHPointsToImplicit2Ptr = std::shared_ptr<SPHPointsToImplicit2>;
}  // namespace CubbyFlow

#endif

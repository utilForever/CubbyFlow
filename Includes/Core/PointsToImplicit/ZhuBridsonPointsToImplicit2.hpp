// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_ZHU_BRIDSON_POINTS_TO_IMPLICIT2_HPP
#define CUBBYFLOW_ZHU_BRIDSON_POINTS_TO_IMPLICIT2_HPP

#include <Core/PointsToImplicit/PointsToImplicit2.hpp>

namespace CubbyFlow
{
//!
//! \brief 2-D points-to-implicit converter based on Zhu and Bridson's method.
//!
//! \see Zhu, Yongning, and Robert Bridson. "Animating sand as a fluid."
//!      ACM Transactions on Graphics (TOG). Vol. 24. No. 3. ACM, 2005.
//!
class ZhuBridsonPointsToImplicit2 final : public PointsToImplicit2
{
 public:
    //! Constructs the converter with default parameters.
    ZhuBridsonPointsToImplicit2() : ZhuBridsonPointsToImplicit2(1.0)
    {
        // Do nothing
    }

    //! Constructs the converter with given kernel radius and cut-off threshold.
    explicit ZhuBridsonPointsToImplicit2(double kernelRadius,
                                         double cutOffThreshold = 0.25,
                                         bool isOutputSDF = true);

    //! Converts the given points to implicit surface scalar field.
    void Convert(const ConstArrayView1<Vector2D>& points,
                 ScalarGrid2* output) const override;

 private:
    double m_kernelRadius = 1.0;
    double m_cutOffThreshold = 0.25;
    bool m_isOutputSDF = true;
};

//! Shared pointer type for ZhuBridsonPointsToImplicit2 class
using ZhuBridsonPointsToImplicit2Ptr =
    std::shared_ptr<ZhuBridsonPointsToImplicit2>;
}  // namespace CubbyFlow

#endif

// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_ZHU_BRIDSON_POINTS_TO_IMPLICIT3_HPP
#define CUBBYFLOW_ZHU_BRIDSON_POINTS_TO_IMPLICIT3_HPP

#include <Core/PointsToImplicit/PointsToImplicit3.hpp>

namespace CubbyFlow
{
//!
//! \brief 3-D points-to-implicit converter based on Zhu and Bridson's method.
//!
//! \see Zhu, Yongning, and Robert Bridson. "Animating sand as a fluid."
//!      ACM Transactions on Graphics (TOG). Vol. 24. No. 3. ACM, 2005.
//!
class ZhuBridsonPointsToImplicit3 final : public PointsToImplicit3
{
 public:
    //! Constructs the converter with default parameters.
    ZhuBridsonPointsToImplicit3() : ZhuBridsonPointsToImplicit3(1.0)
    {
        // Do nothing
    }

    //! Constructs the converter with given kernel radius and cut-off threshold.
    explicit ZhuBridsonPointsToImplicit3(double kernelRadius,
                                         double cutOffThreshold = 0.25,
                                         bool isOutputSDF = true);

    //! Converts the given points to implicit surface scalar field.
    void Convert(const ConstArrayView1<Vector3D>& points,
                 ScalarGrid3* output) const override;

 private:
    double m_kernelRadius = 1.0;
    double m_cutOffThreshold = 0.25;
    bool m_isOutputSDF = true;
};

//! Shared pointer type for ZhuBridsonPointsToImplicit3 class
using ZhuBridsonPointsToImplicit3Ptr =
    std::shared_ptr<ZhuBridsonPointsToImplicit3>;
}  // namespace CubbyFlow

#endif

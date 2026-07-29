// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_CUSTOM_IMPLICIT_SURFACE_HPP
#define CUBBYFLOW_CUSTOM_IMPLICIT_SURFACE_HPP

#include <Core/Geometry/ImplicitSurface.hpp>

namespace CubbyFlow
{
//! Custom N-D implicit surface using arbitrary function.
template <size_t N>
class CustomImplicitSurface final : public ImplicitSurface<N>
{
 public:
    class Builder;

    //!
    //! Constructs an implicit surface using the given signed-distance function.
    //!
    //! \param func Custom SDF function object.
    //! \param domain Bounding box of the SDF if exists.
    //! \param resolution Finite differencing resolution for derivatives.
    //! \param rayMarchingResolution Ray marching resolution for ray tests.
    //! \param maxNumOfIterations Number of iterations for closest point search.
    //! \param _transform Local-to-world transform.
    //! \param _isNormalFlipped True if normal is flipped.
    //!
    explicit CustomImplicitSurface(
        std::function<double(const Vector<double, N>&)> func,
        const BoundingBox<double, N>& domain = BoundingBox<double, N>{},
        double resolution = 1e-3, double rayMarchingResolution = 1e-6,
        unsigned int maxNumOfIterations = 5,
        const Transform<N>& _transform = Transform<N>{},
        bool _isNormalFlipped = false);

    //! Default virtual destructor.
    ~CustomImplicitSurface() override = default;

    //! Default copy constructor.
    CustomImplicitSurface(const CustomImplicitSurface& other) = default;

    //! Default move constructor.
    CustomImplicitSurface(CustomImplicitSurface&& other) noexcept = default;

    //! Default copy assignment operator.
    CustomImplicitSurface& operator=(const CustomImplicitSurface& other) =
        default;

    //! Default move assignment operator.
    CustomImplicitSurface& operator=(CustomImplicitSurface&& other) noexcept =
        default;

    //! Returns builder for CustomImplicitSurface.
    static Builder GetBuilder();

 private:
    [[nodiscard]] Vector<double, N> ClosestPointLocal(
        const Vector<double, N>& otherPoint) const override;

    [[nodiscard]] bool IntersectsLocal(
        const Ray<double, N>& ray) const override;

    [[nodiscard]] BoundingBox<double, N> BoundingBoxLocal() const override;

    [[nodiscard]] Vector<double, N> ClosestNormalLocal(
        const Vector<double, N>& otherPoint) const override;

    [[nodiscard]] double SignedDistanceLocal(
        const Vector<double, N>& otherPoint) const override;

    [[nodiscard]] SurfaceRayIntersection<N> ClosestIntersectionLocal(
        const Ray<double, N>& ray) const override;

    [[nodiscard]] Vector<double, N> GradientLocal(
        const Vector<double, N>& x) const;

    std::function<double(const Vector<double, N>&)> m_func;
    BoundingBox<double, N> m_domain;
    double m_resolution = 1e-3;
    double m_rayMarchingResolution = 1e-6;
    unsigned int m_maxNumOfIterations = 5;
};

//! 2-D CustomImplicitSurface type.
using CustomImplicitSurface2 = CustomImplicitSurface<2>;

//! 3-D CustomImplicitSurface type.
using CustomImplicitSurface3 = CustomImplicitSurface<3>;

//! Shared pointer type for the CustomImplicitSurface2.
using CustomImplicitSurface2Ptr = std::shared_ptr<CustomImplicitSurface2>;

//! Shared pointer type for the CustomImplicitSurface3.
using CustomImplicitSurface3Ptr = std::shared_ptr<CustomImplicitSurface3>;

//!
//! \brief Front-end to create CustomImplicitSurface objects step by step.
//!
template <size_t N>
class CustomImplicitSurface<N>::Builder final
    : public SurfaceBuilderBase<N, typename CustomImplicitSurface<N>::Builder>
{
 public:
    //! Returns builder with custom signed-distance function
    Builder& WithSignedDistanceFunction(
        const std::function<double(const Vector<double, N>&)>& func);

    //! Returns builder with domain.
    Builder& WithDomain(const BoundingBox<double, N>& domain);

    //! Returns builder with finite differencing resolution.
    Builder& WithResolution(double resolution);

    //! Returns builder with ray marching resolution which determines the ray
    //! intersection quality.
    Builder& WithRayMarchingResolution(double rayMarchingResolution);

    //! Returns builder with number of iterations for closest point/normal
    //! searches.
    Builder& WithMaxNumberOfIterations(unsigned int numIter);

    //! Builds CustomImplicitSurface.
    CustomImplicitSurface<N> Build() const;

    //! Builds shared pointer of CustomImplicitSurface instance.
    std::shared_ptr<CustomImplicitSurface<N>> MakeShared() const;

 private:
    using Base =
        SurfaceBuilderBase<N, typename CustomImplicitSurface<N>::Builder>;
    using Base::m_isNormalFlipped;
    using Base::m_transform;

    std::function<double(const Vector<double, N>&)> m_func;
    BoundingBox<double, N> m_domain;
    double m_resolution = 1e-3;
    double m_rayMarchingResolution = 1e-6;
    unsigned int m_maxNumOfIterations = 5;
};
}  // namespace CubbyFlow

#endif
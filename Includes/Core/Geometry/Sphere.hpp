// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_SPHERE_HPP
#define CUBBYFLOW_SPHERE_HPP

#include <Core/Geometry/Surface.hpp>

namespace CubbyFlow
{
//!
//! \brief N-D sphere geometry.
//!
//! This class represents N-D sphere geometry which extends Surface2 by
//! overriding surface-related queries.
//!
template <size_t N>
class Sphere final : public Surface<N>
{
 public:
    class Builder;

    //! Constructs a sphere with center at the origin and radius of 1.
    explicit Sphere(const Transform<N>& _transform = Transform<N>{},
                    bool _isNormalFlipped = false);

    //! Constructs a sphere with \p center and \p radius.
    Sphere(const Vector<double, N>& center, double radius,
           const Transform<N>& _transform = Transform<N>{},
           bool _isNormalFlipped = false);

    //! Default virtual destructor.
    ~Sphere() override = default;

    //! Copy constructor.
    Sphere(const Sphere& other);

    //! Move constructor.
    Sphere(Sphere&& other) noexcept;

    //! Copy assignment operator.
    Sphere& operator=(const Sphere& other);

    //! Move assignment operator.
    Sphere& operator=(Sphere&& other) noexcept;

    //! Returns builder fox Sphere.
    static Builder GetBuilder();

    //! Center of the sphere.
    Vector<double, N> center;

    //! Radius of the sphere.
    double radius = 1.0;

 private:
    [[nodiscard]] Vector<double, N> ClosestPointLocal(
        const Vector<double, N>& otherPoint) const override;

    [[nodiscard]] double ClosestDistanceLocal(
        const Vector<double, N>& otherPoint) const override;

    [[nodiscard]] bool IntersectsLocal(
        const Ray<double, N>& ray) const override;

    [[nodiscard]] BoundingBox<double, N> BoundingBoxLocal() const override;

    [[nodiscard]] Vector<double, N> ClosestNormalLocal(
        const Vector<double, N>& otherPoint) const override;

    [[nodiscard]] SurfaceRayIntersection<N> ClosestIntersectionLocal(
        const Ray<double, N>& ray) const override;
};

//! 2-D Sphere type.
using Sphere2 = Sphere<2>;

//! 3-D Sphere type.
using Sphere3 = Sphere<3>;

//! Shared pointer for the Sphere2 type.
using Sphere2Ptr = std::shared_ptr<Sphere2>;

//! Shared pointer for the Sphere3 type.
using Sphere3Ptr = std::shared_ptr<Sphere3>;

//!
//! \brief Front-end to create Sphere objects step by step.
//!
template <size_t N>
class Sphere<N>::Builder final
    : public SurfaceBuilderBase<N, typename Sphere<N>::Builder>
{
 public:
    //! Returns builder with sphere center.
    Builder& WithCenter(const Vector<double, N>& _center);

    //! Returns builder with sphere radius.
    Builder& WithRadius(double _radius);

    //! Builds Sphere.
    Sphere<N> Build() const;

    //! Builds shared pointer of Sphere instance.
    std::shared_ptr<Sphere<N>> MakeShared() const;

 private:
    using Base = SurfaceBuilderBase<N, typename Sphere<N>::Builder>;
    using Base::m_isNormalFlipped;
    using Base::m_transform;

    Vector<double, N> m_center;
    double m_radius = 0.0;
};
}  // namespace CubbyFlow

#endif
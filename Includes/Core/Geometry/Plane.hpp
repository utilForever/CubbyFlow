// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_PLANE_HPP
#define CUBBYFLOW_PLANE_HPP

#include <Core/Geometry/Surface.hpp>

namespace CubbyFlow
{
//!
//! \brief N-D plane geometry.
//!
//! This class represents N-D plane geometry which extends Surface by
//! overriding surface-related queries.
//!
template <size_t N>
class Plane final : public Surface<N>
{
 public:
    class Builder;

    //! Constructs a plane that crosses (0, 0, ...) with surface normal
    //! (y-axis).
    Plane() : Plane(Transform<N>{})
    {
        // Do nothing
    }

    //! Constructs a default plane with a transform.
    explicit Plane(const Transform<N>& _transform,
                   bool _isNormalFlipped = false);

    //! Constructs a plane that cross \p point with surface normal \p normal.
    Plane(const Vector<double, N>& normal, const Vector<double, N>& point,
          const Transform<N>& _transform = Transform<N>{},
          bool _isNormalFlipped = false);

    //! Default virtual destructor.
    ~Plane() override = default;

    //! Copy constructor.
    Plane(const Plane& other);

    //! Move constructor.
    Plane(Plane&& other) noexcept;

    //! Copy assignment operator.
    Plane& operator=(const Plane& other);

    //! Move assignment operator.
    Plane& operator=(Plane&& other) noexcept;

    //! Returns true if bounding box can be defined.
    [[nodiscard]] bool IsBounded() const override;

    //! Returns builder fox Plane.
    static Builder GetBuilder();

    //! Plane normal.
    Vector<double, N> normal = Vector<double, N>::MakeUnitY();

    //! Point that lies on the plane.
    Vector<double, N> point;

 private:
    [[nodiscard]] Vector<double, N> ClosestPointLocal(
        const Vector<double, N>& otherPoint) const override;

    [[nodiscard]] bool IntersectsLocal(
        const Ray<double, N>& ray) const override;

    [[nodiscard]] BoundingBox<double, N> BoundingBoxLocal() const override;

    [[nodiscard]] Vector<double, N> ClosestNormalLocal(
        const Vector<double, N>& otherPoint) const override;

    [[nodiscard]] SurfaceRayIntersection<N> ClosestIntersectionLocal(
        const Ray<double, N>& ray) const override;
};

//! 2-D plane type.
using Plane2 = Plane<2>;

//! 3-D plane type.
using Plane3 = Plane<3>;

//! Shared pointer for the Plane2 type.
using Plane2Ptr = std::shared_ptr<Plane2>;

//! Shared pointer for the Plane3 type.
using Plane3Ptr = std::shared_ptr<Plane3>;

//!
//! \brief Front-end to create Plane objects step by step.
//!
template <size_t N>
class Plane<N>::Builder final
    : public SurfaceBuilderBase<N, typename Plane<N>::Builder>
{
 public:
    //! Returns builder with plane normal.
    Builder& WithNormal(const Vector<double, N>& _normal);

    //! Returns builder with point on the plane.
    Builder& WithPoint(const Vector<double, N>& _point);

    //! Builds Plane.
    Plane Build() const;

    //! Builds shared pointer of Plane instance.
    std::shared_ptr<Plane<N>> MakeShared() const;

 private:
    using Base = SurfaceBuilderBase<N, typename Plane<N>::Builder>;
    using Base::m_isNormalFlipped;
    using Base::m_transform;

    Vector<double, N> m_normal = Vector<double, N>::MakeUnitY();
    Vector<double, N> m_point;
};
}  // namespace CubbyFlow

#endif

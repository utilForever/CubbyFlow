// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_SURFACE_HPP
#define CUBBYFLOW_SURFACE_HPP

#include <Core/Geometry/Ray.hpp>
#include <Core/Geometry/Transform.hpp>
#include <Core/Matrix/Matrix.hpp>

#include <limits>
#include <memory>

namespace CubbyFlow
{
//! Struct that represents ray-surface intersection point.
template <size_t N>
struct SurfaceRayIntersection
{
    bool isIntersecting = false;
    double distance = std::numeric_limits<double>::max();
    Vector<double, N> point;
    Vector<double, N> normal;
};

using SurfaceRayIntersection2 = SurfaceRayIntersection<2>;
using SurfaceRayIntersection3 = SurfaceRayIntersection<3>;

//! Abstract base class for N-D surface.
template <size_t N>
class Surface
{
 public:
    //! Constructs a surface with normal direction.
    explicit Surface(const Transform<N>& transform = Transform<N>(),
                     bool isNormalFlipped = false);

    //! Default virtual destructor.
    virtual ~Surface() = default;

    //! Copy constructor.
    Surface(const Surface& other);

    //! Move constructor.
    Surface(Surface&& other) noexcept;

    //! Copy assignment operator.
    Surface& operator=(const Surface& other);

    //! Move assignment operator.
    Surface& operator=(Surface&& other) noexcept;

    //! Returns the closest point from the given point \p otherPoint to the
    //! surface.
    [[nodiscard]] Vector<double, N> ClosestPoint(
        const Vector<double, N>& otherPoint) const;

    //! Returns the bounding box of this surface object.
    [[nodiscard]] BoundingBox<double, N> GetBoundingBox() const;

    //! Returns true if the given \p ray intersects with this surface object.
    [[nodiscard]] bool Intersects(const Ray<double, N>& ray) const;

    //! Returns the closest distance from the given point \p otherPoint to the
    //! point on the surface.
    [[nodiscard]] double ClosestDistance(
        const Vector<double, N>& otherPoint) const;

    //! Returns the closest intersection point for given \p ray.
    [[nodiscard]] SurfaceRayIntersection<N> ClosestIntersection(
        const Ray<double, N>& ray) const;

    //! Returns the normal to the closest point on the surface from the given
    //! point \p otherPoint.
    [[nodiscard]] Vector<double, N> ClosestNormal(
        const Vector<double, N>& otherPoint) const;

    //! Updates internal spatial query engine.
    virtual void UpdateQueryEngine();

    //! Returns true if bounding box can be defined.
    [[nodiscard]] virtual bool IsBounded() const;

    //! Returns true if the surface is a valid geometry.
    [[nodiscard]] virtual bool IsValidGeometry() const;

    //! Returns true if \p otherPoint is inside the volume defined by the
    //! surface.
    [[nodiscard]] bool IsInside(const Vector<double, N>& otherPoint) const;

    //! Local-to-world transform.
    Transform<N> transform;

    //! Flips normal.
    bool isNormalFlipped = false;

 protected:
    //! Returns the closest point from the given point \p otherPoint to the
    //! surface in local frame.
    [[nodiscard]] virtual Vector<double, N> ClosestPointLocal(
        const Vector<double, N>& otherPoint) const = 0;

    //! Returns the bounding box of this surface object in local frame.
    [[nodiscard]] virtual BoundingBox<double, N> BoundingBoxLocal() const = 0;

    //! Returns the closest intersection point for given \p ray in local frame.
    [[nodiscard]] virtual SurfaceRayIntersection<N> ClosestIntersectionLocal(
        const Ray<double, N>& ray) const = 0;

    //! Returns the normal to the closest point on the surface from the given
    //! point \p otherPoint in local frame.
    [[nodiscard]] virtual Vector<double, N> ClosestNormalLocal(
        const Vector<double, N>& otherPoint) const = 0;

    //! Returns true if the given \p ray intersects with this surface object
    //! in local frame.
    [[nodiscard]] virtual bool IntersectsLocal(const Ray<double, N>& ray) const;

    //! Returns the closest distance from the given point \p otherPoint to the
    //! point on the surface in local frame.
    [[nodiscard]] virtual double ClosestDistanceLocal(
        const Vector<double, N>& otherPoint) const;

    //! Returns true if \p otherPoint is inside by given \p depth the volume
    //! defined by the surface in local frame.
    [[nodiscard]] virtual bool IsInsideLocal(
        const Vector<double, N>& otherPoint) const;
};

//! 2-D Surface type.
using Surface2 = Surface<2>;

//! 3-D Surface type.
using Surface3 = Surface<3>;

//! Shared pointer for the Surface2 type.
using Surface2Ptr = std::shared_ptr<Surface2>;

//! Shared pointer for the Surface3 type.
using Surface3Ptr = std::shared_ptr<Surface3>;

//!
//! \brief Base class for N-D surface builder.
//!
template <size_t N, typename DerivedBuilder>
class SurfaceBuilderBase
{
 public:
    //! Returns builder with flipped normal flag.
    DerivedBuilder& WithIsNormalFlipped(bool isNormalFlipped);

    //! Returns builder with translation.
    DerivedBuilder& WithTranslation(const Vector<double, N>& translation);

    //! Returns builder with orientation.
    DerivedBuilder& WithOrientation(const Orientation<N>& orientation);

    //! Returns builder with transform.
    DerivedBuilder& WithTransform(const Transform<N>& transform);

 protected:
    bool m_isNormalFlipped = false;
    Transform<N> m_transform;
};

template <size_t N, typename T>
T& SurfaceBuilderBase<N, T>::WithIsNormalFlipped(bool isNormalFlipped)
{
    m_isNormalFlipped = isNormalFlipped;
    return static_cast<T&>(*this);
}

template <size_t N, typename T>
T& SurfaceBuilderBase<N, T>::WithTranslation(
    const Vector<double, N>& translation)
{
    m_transform.SetTranslation(translation);
    return static_cast<T&>(*this);
}

template <size_t N, typename T>
T& SurfaceBuilderBase<N, T>::WithOrientation(const Orientation<N>& orientation)
{
    m_transform.SetOrientation(orientation);
    return static_cast<T&>(*this);
}

template <size_t N, typename T>
T& SurfaceBuilderBase<N, T>::WithTransform(const Transform<N>& transform)
{
    m_transform = transform;
    return static_cast<T&>(*this);
}

template <typename T>
using SurfaceBuilderBase2 = SurfaceBuilderBase<2, T>;

template <typename T>
using SurfaceBuilderBase3 = SurfaceBuilderBase<3, T>;
}  // namespace CubbyFlow

#endif
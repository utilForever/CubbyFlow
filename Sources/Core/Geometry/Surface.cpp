// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Geometry/Surface.hpp>

#include <utility>

namespace CubbyFlow
{
template <size_t N>
Surface<N>::Surface(const Transform<N>& _transform, bool _isNormalFlipped)
    : transform(_transform), isNormalFlipped(_isNormalFlipped)
{
    // Do nothing
}

template <size_t N>
Surface<N>::Surface(const Surface& other)
    : transform(other.transform), isNormalFlipped(other.isNormalFlipped)
{
    // Do nothing
}

template <size_t N>
Surface<N>::Surface(Surface&& other) noexcept
    : transform(std::move(other.transform)),
      isNormalFlipped(std::exchange(other.isNormalFlipped, false))
{
    // Do nothing
}

template <size_t N>
Surface<N>& Surface<N>::operator=(const Surface& other)
{
    transform = other.transform;
    isNormalFlipped = other.isNormalFlipped;
    return *this;
}

template <size_t N>
Surface<N>& Surface<N>::operator=(Surface&& other) noexcept
{
    transform = std::move(other.transform);
    isNormalFlipped = std::exchange(other.isNormalFlipped, false);
    return *this;
}

template <size_t N>
Vector<double, N> Surface<N>::ClosestPoint(
    const Vector<double, N>& otherPoint) const
{
    return transform.ToWorld(ClosestPointLocal(transform.ToLocal(otherPoint)));
}

template <size_t N>
BoundingBox<double, N> Surface<N>::GetBoundingBox() const
{
    return transform.ToWorld(BoundingBoxLocal());
}

template <size_t N>
bool Surface<N>::Intersects(const Ray<double, N>& ray) const
{
    return IntersectsLocal(transform.ToLocal(ray));
}

template <size_t N>
double Surface<N>::ClosestDistance(const Vector<double, N>& otherPoint) const
{
    return ClosestDistanceLocal(transform.ToLocal(otherPoint));
}

template <size_t N>
SurfaceRayIntersection<N> Surface<N>::ClosestIntersection(
    const Ray<double, N>& ray) const
{
    SurfaceRayIntersection<N> result =
        ClosestIntersectionLocal(transform.ToLocal(ray));
    result.point = transform.ToWorld(result.point);
    result.normal = transform.ToWorldDirection(result.normal);
    result.normal *= (isNormalFlipped) ? -1.0 : 1.0;

    return result;
}

template <size_t N>
Vector<double, N> Surface<N>::ClosestNormal(
    const Vector<double, N>& otherPoint) const
{
    Vector<double, N> result = transform.ToWorldDirection(
        ClosestNormalLocal(transform.ToLocal(otherPoint)));
    result *= (isNormalFlipped) ? -1.0 : 1.0;

    return result;
}

template <size_t N>
bool Surface<N>::IntersectsLocal(const Ray<double, N>& rayLocal) const
{
    SurfaceRayIntersection<N> result = ClosestIntersectionLocal(rayLocal);

    return result.isIntersecting;
}

template <size_t N>
void Surface<N>::UpdateQueryEngine()
{
    // Do nothing
}

template <size_t N>
bool Surface<N>::IsBounded() const
{
    return true;
}

template <size_t N>
bool Surface<N>::IsValidGeometry() const
{
    return true;
}

template <size_t N>
bool Surface<N>::IsInside(const Vector<double, N>& otherPoint) const
{
    return isNormalFlipped == !IsInsideLocal(transform.ToLocal(otherPoint));
}

template <size_t N>
double Surface<N>::ClosestDistanceLocal(
    const Vector<double, N>& otherPointLocal) const
{
    return otherPointLocal.DistanceTo(ClosestPointLocal(otherPointLocal));
}

template <size_t N>
bool Surface<N>::IsInsideLocal(const Vector<double, N>& otherPointLocal) const
{
    const Vector<double, N> cpLocal = ClosestPointLocal(otherPointLocal);
    const Vector<double, N> normalLocal = ClosestNormalLocal(otherPointLocal);

    return (otherPointLocal - cpLocal).Dot(normalLocal) < 0.0;
}

template class Surface<2>;

template class Surface<3>;
}  // namespace CubbyFlow
// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Geometry/Box.hpp>
#include <Core/Geometry/Plane.hpp>

namespace CubbyFlow
{
template <size_t N>
Box<N>::Box(const Transform<N>& _transform, bool _isNormalFlipped)
    : Surface<N>{ _transform, _isNormalFlipped }
{
    // Do nothing
}

template <size_t N>
Box<N>::Box(const Vector<double, N>& lowerCorner,
            const Vector<double, N>& upperCorner,
            const Transform<N>& _transform, bool _isNormalFlipped)
    : Box{ BoundingBox<double, N>{ lowerCorner, upperCorner }, _transform,
           _isNormalFlipped }
{
    // Do nothing
}

template <size_t N>
Box<N>::Box(const BoundingBox<double, N>& boundingBox,
            const Transform<N>& _transform, bool _isNormalFlipped)
    : Surface<N>{ _transform, _isNormalFlipped }, bound(boundingBox)
{
    // Do nothing
}

template <size_t N>
Box<N>::Box(const Box& other) : Surface<N>{ other }, bound(other.bound)
{
    // Do nothing
}

template <size_t N>
Box<N>::Box(Box&& other) noexcept
    : Surface<N>{ std::move(other) }, bound(std::move(other.bound))
{
    // Do nothing
}

template <size_t N>
Box<N>& Box<N>::operator=(const Box& other)
{
    bound = other.bound;
    Surface<N>::operator=(other);
    return *this;
}

template <size_t N>
Box<N>& Box<N>::operator=(Box&& other) noexcept
{
    bound = std::move(other.bound);
    Surface<N>::operator=(std::move(other));
    return *this;
}

template <size_t N>
Vector<double, N> Box<N>::ClosestPointLocal(
    const Vector<double, N>& otherPoint) const
{
    if (bound.Contains(otherPoint))
    {
        Plane<N> planes[2 * N];

        for (size_t i = 0; i < N; ++i)
        {
            Vector<double, N> normal;

            normal[i] = 1.0;
            planes[i] = Plane<N>{ normal, bound.upperCorner };
            planes[i + N] = Plane<N>{ -normal, bound.lowerCorner };
        }

        Vector<double, N> result = planes[0].ClosestPoint(otherPoint);
        double distanceSquared = result.DistanceSquaredTo(otherPoint);

        for (int i = 1; i < static_cast<int>(2 * N); ++i)
        {
            Vector<double, N> localResult = planes[i].ClosestPoint(otherPoint);

            if (const double localDistanceSquared =
                    localResult.DistanceSquaredTo(otherPoint);
                localDistanceSquared < distanceSquared)
            {
                result = localResult;
                distanceSquared = localDistanceSquared;
            }
        }

        return result;
    }

    return Clamp(otherPoint, bound.lowerCorner, bound.upperCorner);
}

template <size_t N>
Vector<double, N> Box<N>::ClosestNormalLocal(
    const Vector<double, N>& otherPoint) const
{
    Plane<N> planes[2 * N];

    for (size_t i = 0; i < N; ++i)
    {
        Vector<double, N> normal;

        normal[i] = 1.0;
        planes[i] = Plane<N>{ normal, bound.upperCorner };
        planes[i + N] = Plane<N>{ -normal, bound.lowerCorner };
    }

    if (bound.Contains(otherPoint))
    {
        Vector<double, N> closestNormal = planes[0].normal;
        Vector<double, N> closestPoint = planes[0].ClosestPoint(otherPoint);
        double minDistanceSquared = (closestPoint - otherPoint).LengthSquared();

        for (int i = 1; i < static_cast<int>(2 * N); ++i)
        {
            Vector<double, N> localClosestPoint =
                planes[i].ClosestPoint(otherPoint);

            if (const double localDistanceSquared =
                    (localClosestPoint - otherPoint).LengthSquared();
                localDistanceSquared < minDistanceSquared)
            {
                closestNormal = planes[i].normal;
                minDistanceSquared = localDistanceSquared;
            }
        }

        return closestNormal;
    }

    Vector<double, N> closestPoint =
        Clamp(otherPoint, bound.lowerCorner, bound.upperCorner);
    Vector<double, N> closestPointToInputPoint = otherPoint - closestPoint;
    Vector<double, N> closestNormal = planes[0].normal;
    double maxCosineAngle = closestNormal.Dot(closestPointToInputPoint);

    for (int i = 1; i < static_cast<int>(2 * N); ++i)
    {
        if (const double cosineAngle =
                planes[i].normal.Dot(closestPointToInputPoint);
            cosineAngle > maxCosineAngle)
        {
            closestNormal = planes[i].normal;
            maxCosineAngle = cosineAngle;
        }
    }

    return closestNormal;
}

template <size_t N>
bool Box<N>::IntersectsLocal(const Ray<double, N>& ray) const
{
    return bound.Intersects(ray);
}

template <size_t N>
SurfaceRayIntersection<N> Box<N>::ClosestIntersectionLocal(
    const Ray<double, N>& ray) const
{
    SurfaceRayIntersection<N> intersection;
    BoundingBoxRayIntersectionD bbRayIntersection =
        bound.ClosestIntersection(ray);

    intersection.isIntersecting = bbRayIntersection.isIntersecting;
    if (intersection.isIntersecting)
    {
        intersection.distance = bbRayIntersection.near;
        intersection.point = ray.PointAt(bbRayIntersection.near);
        intersection.normal = ClosestNormalLocal(intersection.point);
    }

    return intersection;
}

template <size_t N>
BoundingBox<double, N> Box<N>::BoundingBoxLocal() const
{
    return bound;
}

template <size_t N>
typename Box<N>::Builder Box<N>::GetBuilder()
{
    return Builder{};
}

template <size_t N>
typename Box<N>::Builder& Box<N>::Builder::WithLowerCorner(
    const Vector<double, N>& pt)
{
    m_lowerCorner = pt;
    return *this;
}

template <size_t N>
typename Box<N>::Builder& Box<N>::Builder::WithUpperCorner(
    const Vector<double, N>& pt)
{
    m_upperCorner = pt;
    return *this;
}

template <size_t N>
typename Box<N>::Builder& Box<N>::Builder::WithBoundingBox(
    const BoundingBox<double, N>& bbox)
{
    m_lowerCorner = bbox.lowerCorner;
    m_upperCorner = bbox.upperCorner;
    return *this;
}

template <size_t N>
Box<N> Box<N>::Builder::Build() const
{
    return Box<N>{ m_lowerCorner, m_upperCorner, m_transform,
                   m_isNormalFlipped };
}

template <size_t N>
std::shared_ptr<Box<N>> Box<N>::Builder::MakeShared() const
{
    return std::make_shared<Box<N>>(m_lowerCorner, m_upperCorner, m_transform,
                                    m_isNormalFlipped);
}

template class Box<2>;

template class Box<3>;
}  // namespace CubbyFlow
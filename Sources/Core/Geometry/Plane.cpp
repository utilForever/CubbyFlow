// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Geometry/Plane.hpp>

#include <limits>

namespace CubbyFlow
{
template <size_t N>
Plane<N>::Plane(const Transform<N>& _transform, bool _isNormalFlipped)
    : Surface<N>{ _transform, _isNormalFlipped }
{
    // Do nothing
}

template <size_t N>
Plane<N>::Plane(const Vector<double, N>& _normal,
                const Vector<double, N>& _point, const Transform<N>& _transform,
                bool _isNormalFlipped)
    : Surface<N>{ _transform, _isNormalFlipped }, normal(_normal), point(_point)
{
    // Do nothing
}

template <size_t N>
Plane<N>::Plane(const Plane& other) = default;

template <size_t N>
Plane<N>::Plane(Plane&& other) noexcept = default;

template <size_t N>
Plane<N>& Plane<N>::operator=(const Plane& other) = default;

template <size_t N>
Plane<N>& Plane<N>::operator=(Plane&& other) noexcept = default;

template <size_t N>
bool Plane<N>::IsBounded() const
{
    return false;
}

template <size_t N>
Vector<double, N> Plane<N>::ClosestPointLocal(
    const Vector<double, N>& otherPoint) const
{
    Vector<double, N> r = otherPoint - point;
    return r - normal.Dot(r) * normal + point;
}

template <size_t N>
Vector<double, N> Plane<N>::ClosestNormalLocal(
    const Vector<double, N>& otherPoint) const
{
    UNUSED_VARIABLE(otherPoint);
    return normal;
}

template <size_t N>
bool Plane<N>::IntersectsLocal(const Ray<double, N>& ray) const
{
    return std::fabs(ray.direction.Dot(normal)) > 0;
}

template <size_t N>
SurfaceRayIntersection<N> Plane<N>::ClosestIntersectionLocal(
    const Ray<double, N>& ray) const
{
    SurfaceRayIntersection<N> intersection;

    // Check if not parallel
    if (double dDotN = ray.direction.Dot(normal); std::fabs(dDotN) > 0)
    {
        if (double t = normal.Dot(point - ray.origin) / dDotN; t >= 0.0)
        {
            intersection.isIntersecting = true;
            intersection.distance = t;
            intersection.point = ray.PointAt(t);
            intersection.normal = normal;
        }
    }

    return intersection;
}

template <size_t N>
BoundingBox<double, N> Plane<N>::BoundingBoxLocal() const
{
    Vector<double, N> maxCorner =
        Vector<double, N>::MakeConstant(std::numeric_limits<double>::max());

    for (size_t i = 0; i < N; ++i)
    {
        Vector<double, N> n;
        Vector<double, N> corner = maxCorner;

        n[i] = 1.0;
        corner[i] = 0.0;

        // See if the plane is axis-aligned and return flat box if true.
        if (std::fabs(normal.Dot(n) - 1.0) <
            std::numeric_limits<double>::epsilon())
        {
            return BoundingBox<double, N>{ point - corner, point + corner };
        }
    }

    // Otherwise, the plane does not have bbox.
    return BoundingBox<double, N>{ maxCorner, maxCorner };
}

template <size_t N>
typename Plane<N>::Builder Plane<N>::GetBuilder()
{
    return Builder{};
}

template <size_t N>
typename Plane<N>::Builder& Plane<N>::Builder::WithNormal(
    const Vector<double, N>& _normal)
{
    m_normal = _normal;
    return *this;
}

template <size_t N>
typename Plane<N>::Builder& Plane<N>::Builder::WithPoint(
    const Vector<double, N>& _point)
{
    m_point = _point;
    return *this;
}

template <size_t N>
Plane<N> Plane<N>::Builder::Build() const
{
    return Plane<N>(m_normal, m_point, m_transform, m_isNormalFlipped);
}

template <size_t N>
std::shared_ptr<Plane<N>> Plane<N>::Builder::MakeShared() const
{
    return std::make_shared<Plane<N>>(m_normal, m_point, m_transform,
                                      m_isNormalFlipped);
}

template class Plane<2>;

template class Plane<3>;
}  // namespace CubbyFlow

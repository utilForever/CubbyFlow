// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Geometry/Sphere.hpp>

#include <utility>

namespace CubbyFlow
{
template <size_t N>
Sphere<N>::Sphere(const Transform<N>& _transform, bool _isNormalFlipped)
    : Surface<N>{ _transform, _isNormalFlipped }
{
    // Do nothing
}

template <size_t N>
Sphere<N>::Sphere(const Vector<double, N>& _center, double _radius,
                  const Transform<N>& _transform, bool _isNormalFlipped)
    : Surface<N>{ _transform, _isNormalFlipped },
      center(_center),
      radius(_radius)
{
    // Do nothing
}

template <size_t N>
Sphere<N>::Sphere(const Sphere& other)
    : Surface<N>{ other }, center(other.center), radius(other.radius)
{
    // Do nothing
}

template <size_t N>
Sphere<N>::Sphere(Sphere&& other) noexcept
    : Surface<N>{ std::move(other) },
      center(std::move(other.center)),
      radius(std::exchange(other.radius, 1.0))
{
    // Do nothing
}

template <size_t N>
Sphere<N>& Sphere<N>::operator=(const Sphere& other)
{
    center = other.center;
    radius = other.radius;
    Surface<N>::operator=(other);
    return *this;
}

template <size_t N>
Sphere<N>& Sphere<N>::operator=(Sphere&& other) noexcept
{
    center = std::move(other.center);
    radius = std::exchange(other.radius, 1.0);
    Surface<N>::operator=(std::move(other));
    return *this;
}

template <size_t N>
Vector<double, N> Sphere<N>::ClosestPointLocal(
    const Vector<double, N>& otherPoint) const
{
    return radius * ClosestNormalLocal(otherPoint) + center;
}

template <size_t N>
double Sphere<N>::ClosestDistanceLocal(
    const Vector<double, N>& otherPoint) const
{
    return std::fabs(center.DistanceTo(otherPoint) - radius);
}

template <size_t N>
Vector<double, N> Sphere<N>::ClosestNormalLocal(
    const Vector<double, N>& otherPoint) const
{
    if (center.IsSimilar(otherPoint))
    {
        return Vector<double, N>::MakeUnitX();
    }

    return (otherPoint - center).Normalized();
}

template <size_t N>
bool Sphere<N>::IntersectsLocal(const Ray<double, N>& ray) const
{
    Vector<double, N> r = ray.origin - center;
    const double b = ray.direction.Dot(r);
    const double c = r.LengthSquared() - Square(radius);

    if (double d = b * b - c; d > 0.0)
    {
        d = std::sqrt(d);
        double tMin = -b - d;
        const double tMax = -b + d;

        if (tMin < 0.0)
        {
            tMin = tMax;
        }

        if (tMin >= 0.0)
        {
            return true;
        }
    }

    return false;
}

template <size_t N>
SurfaceRayIntersection<N> Sphere<N>::ClosestIntersectionLocal(
    const Ray<double, N>& ray) const
{
    SurfaceRayIntersection<N> intersection;
    Vector<double, N> r = ray.origin - center;
    const double b = ray.direction.Dot(r);
    const double c = r.LengthSquared() - Square(radius);

    if (double d = b * b - c; d > 0.0)
    {
        d = std::sqrt(d);
        double tMin = -b - d;
        const double tMax = -b + d;

        if (tMin < 0.0)
        {
            tMin = tMax;
        }

        if (tMin < 0.0)
        {
            intersection.isIntersecting = false;
        }
        else
        {
            intersection.isIntersecting = true;
            intersection.distance = tMin;
            intersection.point = ray.origin + tMin * ray.direction;
            intersection.normal = (intersection.point - center).Normalized();
        }
    }
    else
    {
        intersection.isIntersecting = false;
    }

    return intersection;
}

template <size_t N>
BoundingBox<double, N> Sphere<N>::BoundingBoxLocal() const
{
    Vector<double, N> r = Vector<double, N>::MakeConstant(radius);
    return BoundingBox<double, N>{ center - r, center + r };
}

template <size_t N>
typename Sphere<N>::Builder Sphere<N>::GetBuilder()
{
    return Builder{};
}

template <size_t N>
typename Sphere<N>::Builder& Sphere<N>::Builder::WithCenter(
    const Vector<double, N>& _center)
{
    m_center = _center;
    return *this;
}

template <size_t N>
typename Sphere<N>::Builder& Sphere<N>::Builder::WithRadius(double _radius)
{
    m_radius = _radius;
    return *this;
}

template <size_t N>
Sphere<N> Sphere<N>::Builder::Build() const
{
    return Sphere{ m_center, m_radius, m_transform, m_isNormalFlipped };
}

template <size_t N>
std::shared_ptr<Sphere<N>> Sphere<N>::Builder::MakeShared() const
{
    return std::shared_ptr<Sphere>(
        new Sphere{ m_center, m_radius, m_transform, m_isNormalFlipped },
        [](Sphere* obj) { delete obj; });
}

template class Sphere<2>;

template class Sphere<3>;
}  // namespace CubbyFlow
// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Geometry/CustomImplicitSurface.hpp>

namespace CubbyFlow
{
template <size_t N>
CustomImplicitSurface<N>::CustomImplicitSurface(
    std::function<double(const Vector<double, N>&)> func,
    const BoundingBox<double, N>& domain, double resolution,
    double rayMarchingResolution, unsigned int maxNumOfIterations,
    const Transform<N>& _transform, bool _isNormalFlipped)
    : ImplicitSurface<N>{ _transform, _isNormalFlipped },
      m_func(std::move(func)),
      m_domain(domain),
      m_resolution(resolution),
      m_rayMarchingResolution(rayMarchingResolution),
      m_maxNumOfIterations(maxNumOfIterations)
{
    // Do nothing
}

template <size_t N>
typename CustomImplicitSurface<N>::Builder
CustomImplicitSurface<N>::GetBuilder()
{
    return Builder{};
}

template <size_t N>
Vector<double, N> CustomImplicitSurface<N>::ClosestPointLocal(
    const Vector<double, N>& otherPoint) const
{
    Vector<double, N> pt =
        Clamp(otherPoint, m_domain.lowerCorner, m_domain.upperCorner);

    for (unsigned int iter = 0; iter < m_maxNumOfIterations; ++iter)
    {
        double sdf = SignedDistanceLocal(pt);

        if (std::fabs(sdf) < std::numeric_limits<double>::epsilon())
        {
            break;
        }

        Vector<double, N> g = GradientLocal(pt);
        pt = pt - sdf * g;
    }

    return pt;
}

template <size_t N>
bool CustomImplicitSurface<N>::IntersectsLocal(const Ray<double, N>& ray) const
{
    if (const BoundingBoxRayIntersectionD intersection =
            m_domain.ClosestIntersection(ray);
        intersection.isIntersecting)
    {
        double start, end;
        if (std::abs(intersection.far - std::numeric_limits<double>::max()) <
            std::numeric_limits<double>::epsilon())
        {
            start = 0.0;
            end = intersection.near;
        }
        else
        {
            start = intersection.near;
            end = intersection.far;
        }

        double t = start;
        Vector<double, N> pt = ray.PointAt(t);
        double prevPhi = m_func(pt);

        while (t <= end)
        {
            pt = ray.PointAt(t);
            const double newPhi = m_func(pt);
            const double newPhiAbs = std::fabs(newPhi);

            if (newPhi * prevPhi < 0.0)
            {
                return true;
            }

            t += std::max(newPhiAbs, m_rayMarchingResolution);
            prevPhi = newPhi;
        }
    }

    return false;
}

template <size_t N>
BoundingBox<double, N> CustomImplicitSurface<N>::BoundingBoxLocal() const
{
    return m_domain;
}

template <size_t N>
double CustomImplicitSurface<N>::SignedDistanceLocal(
    const Vector<double, N>& otherPoint) const
{
    if (m_func)
    {
        return m_func(otherPoint);
    }

    return std::numeric_limits<double>::max();
}

template <size_t N>
Vector<double, N> CustomImplicitSurface<N>::ClosestNormalLocal(
    const Vector<double, N>& otherPoint) const
{
    Vector<double, N> pt = ClosestPointLocal(otherPoint);
    Vector<double, N> g = GradientLocal(pt);

    if (g.LengthSquared() > 0.0)
    {
        return g.Normalized();
    }

    return g;
}

template <size_t N>
SurfaceRayIntersection<N> CustomImplicitSurface<N>::ClosestIntersectionLocal(
    const Ray<double, N>& ray) const
{
    SurfaceRayIntersection<N> result;
    const BoundingBoxRayIntersectionD intersection =
        m_domain.ClosestIntersection(ray);

    if (!intersection.isIntersecting)
    {
        return result;
    }

    double start, end;
    if (std::abs(intersection.far - std::numeric_limits<double>::max()) <
        std::numeric_limits<double>::epsilon())
    {
        start = 0.0;
        end = intersection.near;
    }
    else
    {
        start = intersection.near;
        end = intersection.far;
    }

    double t = start;
    double prev = t;
    Vector<double, N> pt = ray.PointAt(t);
    double prevPhi = m_func(pt);

    while (t <= end)
    {
        pt = ray.PointAt(t);
        const double newPhi = m_func(pt);
        const double newPhiAbs = std::fabs(newPhi);

        if (newPhi * prevPhi < 0.0)
        {
            const double frac = prevPhi / (prevPhi - newPhi);
            const double sub = prev + m_rayMarchingResolution * frac;

            result.isIntersecting = true;
            result.distance = sub;
            result.point = ray.PointAt(sub);
            result.normal = GradientLocal(result.point);

            if (result.normal.Length() > 0.0)
            {
                result.normal.Normalize();
            }

            return result;
        }

        prev = t;
        t += std::max(newPhiAbs, m_rayMarchingResolution);
        prevPhi = newPhi;
    }

    return result;
}

template <size_t N>
Vector<double, N> CustomImplicitSurface<N>::GradientLocal(
    const Vector<double, N>& x) const
{
    Vector<double, N> grad;
    for (size_t i = 0; i < N; ++i)
    {
        Vector<double, N> h;
        h[i] = 0.5 * m_resolution;

        const double lowerF = m_func(x - h);
        const double upperF = m_func(x + h);

        grad[i] = (upperF - lowerF) / m_resolution;
    }

    return grad;
}

template <size_t N>
typename CustomImplicitSurface<N>::Builder&
CustomImplicitSurface<N>::Builder::WithSignedDistanceFunction(
    const std::function<double(const Vector<double, N>&)>& func)
{
    m_func = func;
    return *this;
}

template <size_t N>
typename CustomImplicitSurface<N>::Builder&
CustomImplicitSurface<N>::Builder::WithDomain(
    const BoundingBox<double, N>& domain)
{
    m_domain = domain;
    return *this;
}

template <size_t N>
typename CustomImplicitSurface<N>::Builder&
CustomImplicitSurface<N>::Builder::WithResolution(double resolution)
{
    m_resolution = resolution;
    return *this;
}

template <size_t N>
typename CustomImplicitSurface<N>::Builder&
CustomImplicitSurface<N>::Builder::WithRayMarchingResolution(double resolution)
{
    m_rayMarchingResolution = resolution;
    return *this;
}

template <size_t N>
typename CustomImplicitSurface<N>::Builder&
CustomImplicitSurface<N>::Builder::WithMaxNumberOfIterations(
    unsigned int numIter)
{
    m_maxNumOfIterations = numIter;
    return *this;
}

template <size_t N>
CustomImplicitSurface<N> CustomImplicitSurface<N>::Builder::Build() const
{
    return CustomImplicitSurface{ m_func,
                                  m_domain,
                                  m_resolution,
                                  m_rayMarchingResolution,
                                  m_maxNumOfIterations,
                                  m_transform,
                                  m_isNormalFlipped };
}

template <size_t N>
std::shared_ptr<CustomImplicitSurface<N>>
CustomImplicitSurface<N>::Builder::MakeShared() const
{
    return std::make_shared<CustomImplicitSurface>(
        m_func, m_domain, m_resolution, m_rayMarchingResolution,
        m_maxNumOfIterations, m_transform, m_isNormalFlipped);
}

template class CustomImplicitSurface<2>;

template class CustomImplicitSurface<3>;
}  // namespace CubbyFlow

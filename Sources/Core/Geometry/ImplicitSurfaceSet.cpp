// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Geometry/ImplicitSurfaceSet.hpp>
#include <Core/Geometry/SurfaceToImplicit.hpp>

#include <limits>

namespace CubbyFlow
{
template <size_t N>
ImplicitSurfaceSet<N>::ImplicitSurfaceSet(
    ConstArrayView1<std::shared_ptr<ImplicitSurface<N>>> surfaces,
    const Transform<N>& _transform, bool _isNormalFlipped)
    : ImplicitSurface<N>{ _transform, _isNormalFlipped }, m_surfaces(surfaces)
{
    for (const auto& surface : m_surfaces)
    {
        if (!surface->IsBounded())
        {
            m_unboundedSurfaces.Append(surface);
        }
    }

    InvalidateBVH();
}

template <size_t N>
ImplicitSurfaceSet<N>::ImplicitSurfaceSet(
    ConstArrayView1<std::shared_ptr<Surface<N>>> surfaces,
    const Transform<N>& _transform, bool _isNormalFlipped)
    : ImplicitSurface<N>{ _transform, _isNormalFlipped }
{
    for (const auto& surface : surfaces)
    {
        AddExplicitSurface(surface);
    }
}

template <size_t N>
ImplicitSurfaceSet<N>::ImplicitSurfaceSet(const ImplicitSurfaceSet& other)
    : ImplicitSurface<N>{ other },
      m_surfaces(other.m_surfaces),
      m_unboundedSurfaces(other.m_unboundedSurfaces)
{
    // Do nothing
}

template <size_t N>
ImplicitSurfaceSet<N>::ImplicitSurfaceSet(ImplicitSurfaceSet&& other) noexcept
    : ImplicitSurface<N>{ std::move(other) },
      m_surfaces(std::move(other.m_surfaces)),
      m_unboundedSurfaces(std::move(other.m_unboundedSurfaces))
{
    // Do nothing
}

template <size_t N>
ImplicitSurfaceSet<N>& ImplicitSurfaceSet<N>::operator=(
    const ImplicitSurfaceSet& other)
{
    m_surfaces = other.m_surfaces;
    m_unboundedSurfaces = other.m_unboundedSurfaces;
    ImplicitSurface<N>::operator=(other);
    return *this;
}

template <size_t N>
ImplicitSurfaceSet<N>& ImplicitSurfaceSet<N>::operator=(
    ImplicitSurfaceSet&& other) noexcept
{
    m_surfaces = std::move(other.m_surfaces);
    m_unboundedSurfaces = std::move(other.m_unboundedSurfaces);
    ImplicitSurface<N>::operator=(std::move(other));
    return *this;
}

template <size_t N>
void ImplicitSurfaceSet<N>::UpdateQueryEngine()
{
    InvalidateBVH();
    BuildBVH();
}

template <size_t N>
bool ImplicitSurfaceSet<N>::IsBounded() const
{
    // All surfaces should be bounded
    for (const auto& surface : m_surfaces)
    {
        if (!surface->IsBounded())
        {
            return false;
        }
    }

    // Empty set is not bounded
    return !m_surfaces.IsEmpty();
}

template <size_t N>
bool ImplicitSurfaceSet<N>::IsValidGeometry() const
{
    // All surfaces should be valid.
    for (const auto& surface : m_surfaces)
    {
        if (!surface->IsValidGeometry())
        {
            return false;
        }
    }

    // Empty set is not valid.
    return !m_surfaces.IsEmpty();
}

template <size_t N>
size_t ImplicitSurfaceSet<N>::NumberOfSurfaces() const
{
    return m_surfaces.Length();
}

template <size_t N>
const std::shared_ptr<ImplicitSurface<N>>& ImplicitSurfaceSet<N>::SurfaceAt(
    size_t i) const
{
    return m_surfaces[i];
}

template <size_t N>
void ImplicitSurfaceSet<N>::AddExplicitSurface(
    const std::shared_ptr<Surface<N>>& surface)
{
    AddSurface(std::make_shared<SurfaceToImplicit<N>>(surface));
}

template <size_t N>
void ImplicitSurfaceSet<N>::AddSurface(
    const std::shared_ptr<ImplicitSurface<N>>& surface)
{
    m_surfaces.Append(surface);

    if (!surface->IsBounded())
    {
        m_unboundedSurfaces.Append(surface);
    }

    InvalidateBVH();
}

template <size_t N>
Vector<double, N> ImplicitSurfaceSet<N>::ClosestPointLocal(
    const Vector<double, N>& otherPoint) const
{
    BuildBVH();

    const auto distanceFunc = [](const std::shared_ptr<Surface<N>>& surface,
                                 const Vector<double, N>& pt) {
        return surface->ClosestDistance(pt);
    };

    Vector<double, N> result =
        Vector<double, N>::MakeConstant(std::numeric_limits<double>::max());
    const auto queryResult = m_bvh.Nearest(otherPoint, distanceFunc);

    if (queryResult.item != nullptr)
    {
        result = (*queryResult.item)->ClosestPoint(otherPoint);
    }

    double minDist = queryResult.distance;

    for (const auto& surface : m_unboundedSurfaces)
    {
        Vector<double, N> pt = surface->ClosestPoint(otherPoint);

        if (const double dist = pt.DistanceTo(otherPoint); dist < minDist)
        {
            minDist = dist;
            result = surface->ClosestPoint(otherPoint);
        }
    }

    return result;
}

template <size_t N>
double ImplicitSurfaceSet<N>::ClosestDistanceLocal(
    const Vector<double, N>& otherPoint) const
{
    BuildBVH();

    const auto distanceFunc = [](const std::shared_ptr<Surface<N>>& surface,
                                 const Vector<double, N>& pt) {
        return surface->ClosestDistance(pt);
    };

    const auto queryResult = m_bvh.Nearest(otherPoint, distanceFunc);
    double minDist = queryResult.distance;

    for (const auto& surface : m_unboundedSurfaces)
    {
        Vector<double, N> pt = surface->ClosestPoint(otherPoint);

        if (const double dist = pt.DistanceTo(otherPoint); dist < minDist)
        {
            minDist = dist;
        }
    }

    return minDist;
}

template <size_t N>
Vector<double, N> ImplicitSurfaceSet<N>::ClosestNormalLocal(
    const Vector<double, N>& otherPoint) const
{
    BuildBVH();

    const auto distanceFunc = [](const std::shared_ptr<Surface<N>>& surface,
                                 const Vector<double, N>& pt) {
        return surface->ClosestDistance(pt);
    };

    Vector<double, N> result = Vector<double, N>::MakeUnitX();
    const auto queryResult = m_bvh.Nearest(otherPoint, distanceFunc);

    if (queryResult.item != nullptr)
    {
        result = (*queryResult.item)->ClosestNormal(otherPoint);
    }

    double minDist = queryResult.distance;

    for (const auto& surface : m_unboundedSurfaces)
    {
        Vector<double, N> pt = surface->ClosestPoint(otherPoint);

        if (const double dist = pt.DistanceTo(otherPoint); dist < minDist)
        {
            minDist = dist;
            result = surface->ClosestNormal(otherPoint);
        }
    }

    return result;
}

template <size_t N>
bool ImplicitSurfaceSet<N>::IntersectsLocal(const Ray<double, N>& ray) const
{
    BuildBVH();

    const auto testFunc = [](const std::shared_ptr<Surface<N>>& surface,
                             const Ray<double, N>& ray) {
        return surface->Intersects(ray);
    };

    bool result = m_bvh.Intersects(ray, testFunc);

    for (const auto& surface : m_unboundedSurfaces)
    {
        result |= surface->Intersects(ray);
    }

    return result;
}

template <size_t N>
SurfaceRayIntersection<N> ImplicitSurfaceSet<N>::ClosestIntersectionLocal(
    const Ray<double, N>& ray) const
{
    BuildBVH();

    const auto testFunc = [](const std::shared_ptr<Surface<N>>& surface,
                             const Ray<double, N>& ray) {
        SurfaceRayIntersection<N> result = surface->ClosestIntersection(ray);
        return result.distance;
    };

    const auto queryResult = m_bvh.ClosestIntersection(ray, testFunc);

    SurfaceRayIntersection<N> result;
    result.distance = queryResult.distance;
    result.isIntersecting = queryResult.item != nullptr;

    if (queryResult.item != nullptr)
    {
        result.point = ray.PointAt(queryResult.distance);
        result.normal = (*queryResult.item)->ClosestNormal(result.point);
    }

    for (const auto& surface : m_unboundedSurfaces)
    {
        if (SurfaceRayIntersection<N> localResult =
                surface->ClosestIntersection(ray);
            localResult.distance < result.distance)
        {
            result = localResult;
        }
    }

    return result;
}

template <size_t N>
BoundingBox<double, N> ImplicitSurfaceSet<N>::BoundingBoxLocal() const
{
    BuildBVH();

    return m_bvh.GetBoundingBox();
}

template <size_t N>
bool ImplicitSurfaceSet<N>::IsInsideLocal(
    const Vector<double, N>& otherPoint) const
{
    return std::any_of(m_surfaces.begin(), m_surfaces.end(),
                       [&](const std::shared_ptr<Surface<N>> surface) {
                           return surface->IsInside(otherPoint);
                       });
}

template <size_t N>
double ImplicitSurfaceSet<N>::SignedDistanceLocal(
    const Vector<double, N>& otherPoint) const
{
    double sdf = std::numeric_limits<double>::max();

    for (const auto& surface : m_surfaces)
    {
        sdf = std::min(sdf, surface->SignedDistance(otherPoint));
    }

    return sdf;
}

template <size_t N>
void ImplicitSurfaceSet<N>::InvalidateBVH() const
{
    m_bvhInvalidated = true;
}

template <size_t N>
void ImplicitSurfaceSet<N>::BuildBVH() const
{
    if (m_bvhInvalidated)
    {
        Array1<std::shared_ptr<ImplicitSurface<N>>> surfs;
        Array1<BoundingBox<double, N>> bounds;

        for (const auto& surface : m_surfaces)
        {
            if (surface->IsBounded())
            {
                surfs.Append(surface);
                bounds.Append(surface->GetBoundingBox());
            }
        }

        m_bvh.Build(surfs, bounds);
        m_bvhInvalidated = false;
    }
}

template <size_t N>
typename ImplicitSurfaceSet<N>::Builder ImplicitSurfaceSet<N>::GetBuilder()
{
    return Builder{};
}

template <size_t N>
typename ImplicitSurfaceSet<N>::Builder&
ImplicitSurfaceSet<N>::Builder::WithSurfaces(
    const ConstArrayView1<std::shared_ptr<ImplicitSurface<N>>>& surfaces)
{
    m_surfaces = surfaces;
    return *this;
}

template <size_t N>
typename ImplicitSurfaceSet<N>::Builder&
ImplicitSurfaceSet<N>::Builder::WithExplicitSurfaces(
    const ConstArrayView1<std::shared_ptr<Surface<N>>>& surfaces)
{
    m_surfaces.Clear();

    for (const auto& surface : surfaces)
    {
        m_surfaces.Append(std::make_shared<SurfaceToImplicit<N>>(surface));
    }

    return *this;
}

template <size_t N>
ImplicitSurfaceSet<N> ImplicitSurfaceSet<N>::Builder::Build() const
{
    return ImplicitSurfaceSet{ m_surfaces, m_transform, m_isNormalFlipped };
}

template <size_t N>
std::shared_ptr<ImplicitSurfaceSet<N>>
ImplicitSurfaceSet<N>::Builder::MakeShared() const
{
    return std::make_shared<ImplicitSurfaceSet>(m_surfaces, m_transform,
                                                m_isNormalFlipped);
}

template class ImplicitSurfaceSet<2>;

template class ImplicitSurfaceSet<3>;
}  // namespace CubbyFlow
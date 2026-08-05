// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Geometry/SurfaceToImplicit.hpp>

#include <utility>

namespace CubbyFlow
{
template <size_t N>
SurfaceToImplicit<N>::SurfaceToImplicit(std::shared_ptr<Surface<N>> surface,
                                        const Transform<N>& _transform,
                                        bool _isNormalFlipped)
    : ImplicitSurface<N>{ _transform, _isNormalFlipped },
      m_surface(std::move(surface))
{
    // Do nothing
}

template <size_t N>
SurfaceToImplicit<N>::SurfaceToImplicit(const SurfaceToImplicit& other) =
    default;

template <size_t N>
SurfaceToImplicit<N>::SurfaceToImplicit(SurfaceToImplicit&& other) noexcept =
    default;

template <size_t N>
SurfaceToImplicit<N>& SurfaceToImplicit<N>::operator=(
    const SurfaceToImplicit& other) = default;

template <size_t N>
SurfaceToImplicit<N>& SurfaceToImplicit<N>::operator=(
    SurfaceToImplicit&& other) noexcept = default;

template <size_t N>
bool SurfaceToImplicit<N>::IsBounded() const
{
    return m_surface->IsBounded();
}

template <size_t N>
void SurfaceToImplicit<N>::UpdateQueryEngine()
{
    m_surface->UpdateQueryEngine();
}

template <size_t N>
bool SurfaceToImplicit<N>::IsValidGeometry() const
{
    return m_surface->IsValidGeometry();
}

template <size_t N>
std::shared_ptr<Surface<N>> SurfaceToImplicit<N>::GetSurface() const
{
    return m_surface;
}

template <size_t N>
Vector<double, N> SurfaceToImplicit<N>::ClosestPointLocal(
    const Vector<double, N>& otherPoint) const
{
    return m_surface->ClosestPoint(otherPoint);
}

template <size_t N>
Vector<double, N> SurfaceToImplicit<N>::ClosestNormalLocal(
    const Vector<double, N>& otherPoint) const
{
    return m_surface->ClosestNormal(otherPoint);
}

template <size_t N>
double SurfaceToImplicit<N>::ClosestDistanceLocal(
    const Vector<double, N>& otherPoint) const
{
    return m_surface->ClosestDistance(otherPoint);
}

template <size_t N>
bool SurfaceToImplicit<N>::IntersectsLocal(const Ray<double, N>& ray) const
{
    return m_surface->Intersects(ray);
}

template <size_t N>
SurfaceRayIntersection<N> SurfaceToImplicit<N>::ClosestIntersectionLocal(
    const Ray<double, N>& ray) const
{
    return m_surface->ClosestIntersection(ray);
}

template <size_t N>
BoundingBox<double, N> SurfaceToImplicit<N>::BoundingBoxLocal() const
{
    return m_surface->GetBoundingBox();
}

template <size_t N>
double SurfaceToImplicit<N>::SignedDistanceLocal(
    const Vector<double, N>& otherPoint) const
{
    Vector<double, N> x = m_surface->ClosestPoint(otherPoint);
    const bool inside = m_surface->IsInside(otherPoint);
    return inside ? -x.DistanceTo(otherPoint) : x.DistanceTo(otherPoint);
}

template <size_t N>
typename SurfaceToImplicit<N>::Builder SurfaceToImplicit<N>::GetBuilder()
{
    return Builder{};
}

template <size_t N>
typename SurfaceToImplicit<N>::Builder&
SurfaceToImplicit<N>::Builder::WithSurface(
    const std::shared_ptr<Surface<N>>& surface)
{
    m_surface = surface;
    return *this;
}

template <size_t N>
SurfaceToImplicit<N> SurfaceToImplicit<N>::Builder::Build() const
{
    return SurfaceToImplicit{ m_surface, m_transform, m_isNormalFlipped };
}

template <size_t N>
std::shared_ptr<SurfaceToImplicit<N>>
SurfaceToImplicit<N>::Builder::MakeShared() const
{
    return std::make_shared<SurfaceToImplicit>(m_surface, m_transform,
                                               m_isNormalFlipped);
}

template class SurfaceToImplicit<2>;

template class SurfaceToImplicit<3>;
}  // namespace CubbyFlow

// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Geometry/ImplicitSurface.hpp>
#include <Core/Utils/LevelSetUtils.hpp>

namespace CubbyFlow
{
template <size_t N>
ImplicitSurface<N>::ImplicitSurface(const Transform<N>& _transform,
                                    bool _isNormalFlipped)
    : Surface<N>{ _transform, _isNormalFlipped }
{
    // Do nothing
}

template <size_t N>
ImplicitSurface<N>::ImplicitSurface(const ImplicitSurface& other) = default;

template <size_t N>
ImplicitSurface<N>::ImplicitSurface(ImplicitSurface&& other) noexcept = default;

template <size_t N>
ImplicitSurface<N>& ImplicitSurface<N>::operator=(
    const ImplicitSurface& other) = default;

template <size_t N>
ImplicitSurface<N>& ImplicitSurface<N>::operator=(
    ImplicitSurface&& other) noexcept = default;

template <size_t N>
double ImplicitSurface<N>::SignedDistance(
    const Vector<double, N>& otherPoint) const
{
    const double sd = SignedDistanceLocal(transform.ToLocal(otherPoint));
    return isNormalFlipped ? -sd : sd;
}

template <size_t N>
double ImplicitSurface<N>::ClosestDistanceLocal(
    const Vector<double, N>& otherPoint) const
{
    return std::fabs(SignedDistanceLocal(otherPoint));
}

template class ImplicitSurface<2>;

template class ImplicitSurface<3>;
}  // namespace CubbyFlow

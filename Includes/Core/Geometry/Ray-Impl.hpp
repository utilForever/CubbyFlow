// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_RAY_IMPL_HPP
#define CUBBYFLOW_RAY_IMPL_HPP

namespace CubbyFlow
{
template <typename T, size_t N>
Ray<T, N>::Ray() : origin(VectorType{}), direction(VectorType{})
{
    direction[0] = 1;
}

template <typename T, size_t N>
Ray<T, N>::Ray(const VectorType& newOrigin, const VectorType& newDirection)
    : origin(newOrigin), direction(newDirection.Normalized())
{
    // Do nothing
}

template <typename T, size_t N>
Ray<T, N>::Ray(const Ray& other)
    : origin(other.origin), direction(other.direction)
{
    // Do nothing
}

template <typename T, size_t N>
Ray<T, N>::Ray(Ray&& other) noexcept
    : origin(std::move(other.origin)), direction(std::move(other.direction))
{
    // Do nothing
}

template <typename T, size_t N>
Ray<T, N>& Ray<T, N>::operator=(const Ray& other) = default;

template <typename T, size_t N>
Ray<T, N>& Ray<T, N>::operator=(Ray&& other) noexcept
{
    origin = std::move(other.origin);
    direction = std::move(other.direction);
    return *this;
}

template <typename T, size_t N>
typename Ray<T, N>::VectorType Ray<T, N>::PointAt(T t) const
{
    return origin + t * direction;
}
}  // namespace CubbyFlow

#endif

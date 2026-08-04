// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_BOUNDING_BOX_IMPL_HPP
#define CUBBYFLOW_BOUNDING_BOX_IMPL_HPP

namespace CubbyFlow
{
template <typename T, size_t N>
BoundingBox<T, N>::BoundingBox()
{
    Reset();
}

template <typename T, size_t N>
BoundingBox<T, N>::BoundingBox(const VectorType& point1,
                               const VectorType& point2)
{
    lowerCorner = Min(point1, point2);
    upperCorner = Max(point1, point2);
}

template <typename T, size_t N>
BoundingBox<T, N>::BoundingBox(const BoundingBox& other) = default;

template <typename T, size_t N>
BoundingBox<T, N>::BoundingBox(BoundingBox&& other) noexcept = default;

template <typename T, size_t N>
BoundingBox<T, N>& BoundingBox<T, N>::operator=(const BoundingBox& other) =
    default;

template <typename T, size_t N>
BoundingBox<T, N>& BoundingBox<T, N>::operator=(BoundingBox&& other) noexcept =
    default;

template <typename T, size_t N>
T BoundingBox<T, N>::Width() const
{
    return upperCorner[0] - lowerCorner[0];
}

template <typename T, size_t N>
template <typename U>
std::enable_if_t<(N > 1), U> BoundingBox<T, N>::Height() const
{
    return upperCorner[1] - lowerCorner[1];
}

template <typename T, size_t N>
template <typename U>
std::enable_if_t<(N > 2), U> BoundingBox<T, N>::Depth() const
{
    return upperCorner[2] - lowerCorner[2];
}

template <typename T, size_t N>
T BoundingBox<T, N>::Length(size_t axis)
{
    return upperCorner[axis] - lowerCorner[axis];
}

template <typename T, size_t N>
bool BoundingBox<T, N>::Overlaps(const BoundingBox& other) const
{
    for (size_t i = 0; i < N; ++i)
    {
        if (upperCorner[i] < other.lowerCorner[i] ||
            lowerCorner[i] > other.upperCorner[i])
        {
            return false;
        }
    }

    return true;
}

template <typename T, size_t N>
bool BoundingBox<T, N>::Contains(const VectorType& point) const
{
    for (size_t i = 0; i < N; ++i)
    {
        if (upperCorner[i] < point[i] || lowerCorner[i] > point[i])
        {
            return false;
        }
    }

    return true;
}

template <typename T, size_t N>
bool BoundingBox<T, N>::Intersects(const RayType& ray) const
{
    T min = 0;
    T max = std::numeric_limits<T>::max();
    const VectorType& rayInvDir = T(1) / ray.direction;

    for (size_t i = 0; i < N; ++i)
    {
        T near = (lowerCorner[i] - ray.origin[i]) * rayInvDir[i];
        T far = (upperCorner[i] - ray.origin[i]) * rayInvDir[i];

        if (near > far)
        {
            std::swap(near, far);
        }

        min = near > min ? near : min;
        max = far < max ? far : max;

        if (min > max)
        {
            return false;
        }
    }

    return true;
}

template <typename T, size_t N>
BoundingBoxRayIntersection<T> BoundingBox<T, N>::ClosestIntersection(
    const RayType& ray) const
{
    BoundingBoxRayIntersection<T> intersection;

    T min = 0;
    T max = std::numeric_limits<T>::max();
    const VectorType& rayInvDir = T(1) / ray.direction;

    for (size_t i = 0; i < N; ++i)
    {
        T near = (lowerCorner[i] - ray.origin[i]) * rayInvDir[i];
        T far = (upperCorner[i] - ray.origin[i]) * rayInvDir[i];

        if (near > far)
        {
            std::swap(near, far);
        }

        min = near > min ? near : min;
        max = far < max ? far : max;

        if (min > max)
        {
            intersection.isIntersecting = false;
            return intersection;
        }
    }

    intersection.isIntersecting = true;

    if (Contains(ray.origin))
    {
        intersection.near = max;
        intersection.far = std::numeric_limits<T>::max();
    }
    else
    {
        intersection.near = min;
        intersection.far = max;
    }

    return intersection;
}

template <typename T, size_t N>
typename BoundingBox<T, N>::VectorType BoundingBox<T, N>::MidPoint() const
{
    return (upperCorner + lowerCorner) / static_cast<T>(2);
}

template <typename T, size_t N>
T BoundingBox<T, N>::DiagonalLength() const
{
    return VectorType(upperCorner - lowerCorner).Length();
}

template <typename T, size_t N>
T BoundingBox<T, N>::DiagonalLengthSquared() const
{
    return VectorType(upperCorner - lowerCorner).LengthSquared();
}

template <typename T, size_t N>
void BoundingBox<T, N>::Reset()
{
    lowerCorner = VectorType::MakeConstant(std::numeric_limits<T>::max());
    upperCorner = VectorType::MakeConstant(-std::numeric_limits<T>::max());
}

template <typename T, size_t N>
void BoundingBox<T, N>::Merge(const VectorType& point)
{
    lowerCorner = Min(lowerCorner, point);
    upperCorner = Max(upperCorner, point);
}

template <typename T, size_t N>
void BoundingBox<T, N>::Merge(const BoundingBox& other)
{
    lowerCorner = Min(lowerCorner, other.lowerCorner);
    upperCorner = Max(upperCorner, other.upperCorner);
}

template <typename T, size_t N>
void BoundingBox<T, N>::Expand(T delta)
{
    lowerCorner -= delta;
    upperCorner += delta;
}

template <typename T, size_t N>
typename BoundingBox<T, N>::VectorType BoundingBox<T, N>::Corner(
    size_t idx) const
{
    VectorType ret;
    for (size_t i = 0; i < N; ++i)
    {
        ret[i] = lowerCorner[i] + (((ONE_SIZE << i) & idx) != 0) *
                                      (upperCorner[i] - lowerCorner[i]);
    }
    return ret;
}

template <typename T, size_t N>
typename BoundingBox<T, N>::VectorType BoundingBox<T, N>::Clamp(
    const VectorType& point) const
{
    return CubbyFlow::Clamp(point, lowerCorner, upperCorner);
}

template <typename T, size_t N>
bool BoundingBox<T, N>::IsEmpty() const
{
    for (size_t i = 0; i < N; ++i)
    {
        if (lowerCorner[i] >= upperCorner[i])
        {
            return true;
        }
    }

    return false;
}

template <typename T, size_t N>
template <typename U>
BoundingBox<U, N> BoundingBox<T, N>::CastTo() const
{
    return BoundingBox<U, N>{ lowerCorner.template CastTo<U>(),
                              upperCorner.template CastTo<U>() };
}
}  // namespace CubbyFlow

#endif

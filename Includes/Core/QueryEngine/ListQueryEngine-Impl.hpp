// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_LIST_QUERY_ENGINE_IMPL_HPP
#define CUBBYFLOW_LIST_QUERY_ENGINE_IMPL_HPP

namespace CubbyFlow
{
template <typename T, size_t N>
void ListQueryEngine<T, N>::Add(const T& item)
{
    m_items.Append(item);
}

template <typename T, size_t N>
void ListQueryEngine<T, N>::Add(const ConstArrayView1<T>& items)
{
    m_items.Append(items);
}

template <typename T, size_t N>
bool ListQueryEngine<T, N>::Intersects(
    const BoundingBox<double, N>& box,
    const BoxIntersectionTestFunc<T, N>& testFunc) const
{
    return std::ranges::any_of(
        m_items,
        [&testFunc, &box](const T& item) { return testFunc(item, box); });
}

template <typename T, size_t N>
bool ListQueryEngine<T, N>::Intersects(
    const Ray<double, N>& ray,
    const RayIntersectionTestFunc<T, N>& testFunc) const
{
    return std::ranges::any_of(
        m_items,
        [&testFunc, &ray](const T& item) { return testFunc(item, ray); });
}

template <typename T, size_t N>
void ListQueryEngine<T, N>::ForEachIntersectingItem(
    const BoundingBox<double, N>& box,
    const BoxIntersectionTestFunc<T, N>& testFunc,
    const IntersectionVisitorFunc<T>& visitorFunc) const
{
    for (const auto& item : m_items)
    {
        if (testFunc(item, box))
        {
            visitorFunc(item);
        }
    }
}

template <typename T, size_t N>
void ListQueryEngine<T, N>::ForEachIntersectingItem(
    const Ray<double, N>& ray, const RayIntersectionTestFunc<T, N>& testFunc,
    const IntersectionVisitorFunc<T>& visitorFunc) const
{
    for (const auto& item : m_items)
    {
        if (testFunc(item, ray))
        {
            visitorFunc(item);
        }
    }
}

template <typename T, size_t N>
ClosestIntersectionQueryResult<T, N> ListQueryEngine<T, N>::ClosestIntersection(
    const Ray<double, N>& ray,
    const GetRayIntersectionFunc<T, N>& testFunc) const
{
    ClosestIntersectionQueryResult<T, N> best;

    for (const auto& item : m_items)
    {
        if (double dist = testFunc(item, ray); dist < best.distance)
        {
            best.distance = dist;
            best.item = &item;
        }
    }

    return best;
}

template <typename T, size_t N>
NearestNeighborQueryResult<T, N> ListQueryEngine<T, N>::Nearest(
    const Vector<double, N>& pt,
    const NearestNeighborDistanceFunc<T, N>& distanceFunc) const
{
    NearestNeighborQueryResult<T, N> best;

    for (const auto& item : m_items)
    {
        if (double dist = distanceFunc(item, pt); dist < best.distance)
        {
            best.item = &item;
            best.distance = dist;
        }
    }

    return best;
}
}  // namespace CubbyFlow

#endif

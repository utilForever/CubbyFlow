// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Grid/Grid.hpp>

namespace CubbyFlow
{
template <size_t N>
Grid<N>::Grid(const Grid& other) = default;

template <size_t N>
Grid<N>::Grid(Grid&& other) noexcept = default;

template <size_t N>
Grid<N>& Grid<N>::operator=(const Grid& other) = default;

template <size_t N>
Grid<N>& Grid<N>::operator=(Grid&& other) noexcept = default;

template <size_t N>
const Vector<size_t, N>& Grid<N>::Resolution() const
{
    return m_resolution;
}

template <size_t N>
const Vector<double, N>& Grid<N>::Origin() const
{
    return m_origin;
}

template <size_t N>
const Vector<double, N>& Grid<N>::GridSpacing() const
{
    return m_gridSpacing;
}

template <size_t N>
const BoundingBox<double, N>& Grid<N>::GetBoundingBox() const
{
    return m_boundingBox;
}

template <size_t N>
GridDataPositionFunc<N> Grid<N>::CellCenterPosition() const
{
    Vector<double, N> h = m_gridSpacing;
    Vector<double, N> o = m_origin;

    return GridDataPositionFunc<N>(
        [h, o](const Vector<size_t, N>& idx) -> Vector<double, N> {
            return o + ElemMul(h, idx.template CastTo<double>() +
                                      Vector<double, N>::MakeConstant(0.5));
        });
}

template <size_t N>
void Grid<N>::ForEachCellIndex(
    const std::function<void(const Vector<size_t, N>&)>& func) const
{
    ForEachIndex(m_resolution, [&func](auto... indices) {
        func(Vector<size_t, N>(indices...));
    });
}

template <size_t N>
void Grid<N>::ParallelForEachCellIndex(
    const std::function<void(const Vector<size_t, N>&)>& func) const
{
    ParallelForEachIndex(
        Vector<size_t, N>::MakeZero(), m_resolution,
        [&func](auto... indices) { func(Vector<size_t, N>(indices...)); });
}

template <size_t N>
bool Grid<N>::HasSameShape(const Grid& other) const
{
    return m_resolution == other.m_resolution &&
           m_gridSpacing.IsSimilar(other.m_gridSpacing) &&
           m_origin.IsSimilar(other.m_origin);
}

template <size_t N>
void Grid<N>::SetSizeParameters(const Vector<size_t, N>& resolution,
                                const Vector<double, N>& gridSpacing,
                                const Vector<double, N>& origin)
{
    m_resolution = resolution;
    m_origin = origin;
    m_gridSpacing = gridSpacing;

    Vector<double, N> resolutionD = resolution.template CastTo<double>();

    m_boundingBox = BoundingBox<double, N>(
        origin, origin + ElemMul(gridSpacing, resolutionD));
}

template <size_t N>
void Grid<N>::SwapGrid(Grid* other)
{
    std::swap(m_resolution, other->m_resolution);
    std::swap(m_gridSpacing, other->m_gridSpacing);
    std::swap(m_origin, other->m_origin);
    std::swap(m_boundingBox, other->m_boundingBox);
}

template <size_t N>
void Grid<N>::SetGrid(const Grid& other)
{
    m_resolution = other.m_resolution;
    m_gridSpacing = other.m_gridSpacing;
    m_origin = other.m_origin;
    m_boundingBox = other.m_boundingBox;
}

template class Grid<2>;

template class Grid<3>;
}  // namespace CubbyFlow

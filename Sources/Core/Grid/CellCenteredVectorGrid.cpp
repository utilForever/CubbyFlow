// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Grid/CellCenteredVectorGrid.hpp>

namespace CubbyFlow
{
template <size_t N>
CellCenteredVectorGrid<N>::CellCenteredVectorGrid(
    const Vector<size_t, N>& resolution, const Vector<double, N>& gridSpacing,
    const Vector<double, N>& origin, const Vector<double, N>& initialValue)
{
    Resize(resolution, gridSpacing, origin, initialValue);
}

template <size_t N>
CellCenteredVectorGrid<N>::CellCenteredVectorGrid(
    const CellCenteredVectorGrid& other)
{
    Set(other);
}

template <size_t N>
CellCenteredVectorGrid<N>::CellCenteredVectorGrid(
    CellCenteredVectorGrid&& other) noexcept = default;

template <size_t N>
CellCenteredVectorGrid<N>& CellCenteredVectorGrid<N>::operator=(
    const CellCenteredVectorGrid& other)
{
    Set(other);
    return *this;
}

template <size_t N>
CellCenteredVectorGrid<N>& CellCenteredVectorGrid<N>::operator=(
    CellCenteredVectorGrid&& other) noexcept = default;

template <size_t N>
Vector<size_t, N> CellCenteredVectorGrid<N>::DataSize() const
{
    return Resolution();
}

template <size_t N>
Vector<double, N> CellCenteredVectorGrid<N>::DataOrigin() const
{
    return Origin() + 0.5 * GridSpacing();
}

template <size_t N>
void CellCenteredVectorGrid<N>::Swap(Grid<N>* other)
{
    if (auto sameType = dynamic_cast<CellCenteredVectorGrid*>(other); sameType)
    {
        SwapCollocatedVectorGrid(sameType);
    }
}

template <size_t N>
void CellCenteredVectorGrid<N>::Set(const CellCenteredVectorGrid& other)
{
    SetCollocatedVectorGrid(other);
}

template <size_t N>
void CellCenteredVectorGrid<N>::Fill(const Vector<double, N>& value,
                                     ExecutionPolicy policy)
{
    Vector<size_t, N> size = DataSize();
    auto acc = DataView();

    ParallelForEachIndex(
        Vector<size_t, N>(), size,
        [&acc, &value](auto... indices) { acc(indices...) = value; }, policy);
}

template <size_t N>
void CellCenteredVectorGrid<N>::Fill(
    const std::function<Vector<double, N>(const Vector<double, N>&)>& func,
    ExecutionPolicy policy)
{
    Vector<size_t, N> size = DataSize();
    auto acc = DataView();
    auto pos = DataPosition();

    ParallelForEachIndex(
        Vector<size_t, N>::MakeZero(), size,
        [&func, &acc, &pos](auto... indices) {
            acc(indices...) = func(pos(indices...));
        },
        policy);
}

template <size_t N>
std::shared_ptr<VectorGrid<N>> CellCenteredVectorGrid<N>::Clone() const
{
    return std::make_shared<CellCenteredVectorGrid<N>>(*this);
}

template <size_t N>
typename CellCenteredVectorGrid<N>::Builder
CellCenteredVectorGrid<N>::GetBuilder()
{
    return Builder{};
}

template <size_t N>
typename CellCenteredVectorGrid<N>::Builder&
CellCenteredVectorGrid<N>::Builder::WithResolution(
    const Vector<size_t, N>& resolution)
{
    m_resolution = resolution;
    return *this;
}

template <size_t N>
typename CellCenteredVectorGrid<N>::Builder&
CellCenteredVectorGrid<N>::Builder::WithGridSpacing(
    const Vector<double, N>& gridSpacing)
{
    m_gridSpacing = gridSpacing;
    return *this;
}

template <size_t N>
typename CellCenteredVectorGrid<N>::Builder&
CellCenteredVectorGrid<N>::Builder::WithOrigin(
    const Vector<double, N>& gridOrigin)
{
    m_gridOrigin = gridOrigin;
    return *this;
}

template <size_t N>
typename CellCenteredVectorGrid<N>::Builder&
CellCenteredVectorGrid<N>::Builder::WithInitialValue(
    const Vector<double, N>& initialVal)
{
    m_initialVal = initialVal;
    return *this;
}

template <size_t N>
CellCenteredVectorGrid<N> CellCenteredVectorGrid<N>::Builder::Build() const
{
    return CellCenteredVectorGrid{ m_resolution, m_gridSpacing, m_gridOrigin,
                                   m_initialVal };
}

template <size_t N>
std::shared_ptr<VectorGrid<N>> CellCenteredVectorGrid<N>::Builder::Build(
    const Vector<size_t, N>& resolution, const Vector<double, N>& gridSpacing,
    const Vector<double, N>& gridOrigin,
    const Vector<double, N>& initialVal) const
{
    return std::make_shared<CellCenteredVectorGrid>(resolution, gridSpacing,
                                                    gridOrigin, initialVal);
}

template <size_t N>
std::shared_ptr<CellCenteredVectorGrid<N>>
CellCenteredVectorGrid<N>::Builder::MakeShared() const
{
    return std::make_shared<CellCenteredVectorGrid>(m_resolution, m_gridSpacing,
                                                    m_gridOrigin, m_initialVal);
}

template class CellCenteredVectorGrid<2>;

template class CellCenteredVectorGrid<3>;
}  // namespace CubbyFlow

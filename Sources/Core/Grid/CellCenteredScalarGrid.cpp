// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Grid/CellCenteredScalarGrid.hpp>

namespace CubbyFlow
{
template <size_t N>
CellCenteredScalarGrid<N>::CellCenteredScalarGrid(
    const Vector<size_t, N>& resolution, const Vector<double, N>& gridSpacing,
    const Vector<double, N>& origin, double initialValue)
{
    Resize(resolution, gridSpacing, origin, initialValue);
}

template <size_t N>
CellCenteredScalarGrid<N>::CellCenteredScalarGrid(
    const CellCenteredScalarGrid& other)
    : ScalarGrid<N>{ other }
{
    Set(other);
}

template <size_t N>
CellCenteredScalarGrid<N>::CellCenteredScalarGrid(
    CellCenteredScalarGrid&& other) noexcept
    : ScalarGrid<N>{ std::move(other) }
{
    // Do nothing
}

template <size_t N>
CellCenteredScalarGrid<N>& CellCenteredScalarGrid<N>::operator=(
    const CellCenteredScalarGrid& other)
{
    Set(other);
    ScalarGrid<N>::operator=(other);
    return *this;
}

template <size_t N>
CellCenteredScalarGrid<N>& CellCenteredScalarGrid<N>::operator=(
    CellCenteredScalarGrid&& other) noexcept
{
    ScalarGrid<N>::operator=(std::move(other));
    return *this;
}

template <size_t N>
Vector<size_t, N> CellCenteredScalarGrid<N>::DataSize() const
{
    // The size of the data should be the same as the grid resolution.
    return Resolution();
}

template <size_t N>
Vector<double, N> CellCenteredScalarGrid<N>::DataOrigin() const
{
    return Origin() + 0.5 * GridSpacing();
}

template <size_t N>
std::shared_ptr<ScalarGrid<N>> CellCenteredScalarGrid<N>::Clone() const
{
    return std::make_shared<CellCenteredScalarGrid<N>>(*this);
}

template <size_t N>
void CellCenteredScalarGrid<N>::Swap(Grid<N>* other)
{
    if (auto sameType = dynamic_cast<CellCenteredScalarGrid<N>*>(other);
        sameType)
    {
        SwapScalarGrid(sameType);
    }
}

template <size_t N>
void CellCenteredScalarGrid<N>::Set(const CellCenteredScalarGrid& other)
{
    SetScalarGrid(other);
}

template <size_t N>
typename CellCenteredScalarGrid<N>::Builder
CellCenteredScalarGrid<N>::GetBuilder()
{
    return Builder{};
}

template <size_t N>
typename CellCenteredScalarGrid<N>::Builder&
CellCenteredScalarGrid<N>::Builder::WithResolution(
    const Vector<size_t, N>& resolution)
{
    m_resolution = resolution;
    return *this;
}

template <size_t N>
typename CellCenteredScalarGrid<N>::Builder&
CellCenteredScalarGrid<N>::Builder::WithGridSpacing(
    const Vector<double, N>& gridSpacing)
{
    m_gridSpacing = gridSpacing;
    return *this;
}

template <size_t N>
typename CellCenteredScalarGrid<N>::Builder&
CellCenteredScalarGrid<N>::Builder::WithOrigin(
    const Vector<double, N>& gridOrigin)
{
    m_gridOrigin = gridOrigin;
    return *this;
}

template <size_t N>
typename CellCenteredScalarGrid<N>::Builder&
CellCenteredScalarGrid<N>::Builder::WithInitialValue(double initialVal)
{
    m_initialVal = initialVal;
    return *this;
}

template <size_t N>
CellCenteredScalarGrid<N> CellCenteredScalarGrid<N>::Builder::Build() const
{
    return CellCenteredScalarGrid{ m_resolution, m_gridSpacing, m_gridOrigin,
                                   m_initialVal };
}

template <size_t N>
std::shared_ptr<ScalarGrid<N>> CellCenteredScalarGrid<N>::Builder::Build(
    const Vector<size_t, N>& resolution, const Vector<double, N>& gridSpacing,
    const Vector<double, N>& gridOrigin, double initialVal) const
{
    return std::make_shared<CellCenteredScalarGrid>(resolution, gridSpacing,
                                                    gridOrigin, initialVal);
}

template <size_t N>
std::shared_ptr<CellCenteredScalarGrid<N>>
CellCenteredScalarGrid<N>::Builder::MakeShared() const
{
    return std::make_shared<CellCenteredScalarGrid>(m_resolution, m_gridSpacing,
                                                    m_gridOrigin, m_initialVal);
}

template class CellCenteredScalarGrid<2>;

template class CellCenteredScalarGrid<3>;
}  // namespace CubbyFlow
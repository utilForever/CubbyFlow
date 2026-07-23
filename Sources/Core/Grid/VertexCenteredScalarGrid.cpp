// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Grid/VertexCenteredScalarGrid.hpp>

namespace CubbyFlow
{
template <size_t N>
VertexCenteredScalarGrid<N>::VertexCenteredScalarGrid(
    const Vector<size_t, N>& resolution, const Vector<double, N>& gridSpacing,
    const Vector<double, N>& origin, double initialValue)
{
    Resize(resolution, gridSpacing, origin, initialValue);
}

template <size_t N>
VertexCenteredScalarGrid<N>::VertexCenteredScalarGrid(
    const VertexCenteredScalarGrid& other)
    : ScalarGrid<N>{ other }
{
    Set(other);
}

template <size_t N>
VertexCenteredScalarGrid<N>::VertexCenteredScalarGrid(
    VertexCenteredScalarGrid&& other) noexcept
    : ScalarGrid<N>{ std::move(other) }
{
    // Do nothing
}

template <size_t N>
VertexCenteredScalarGrid<N>& VertexCenteredScalarGrid<N>::operator=(
    const VertexCenteredScalarGrid& other)
{
    Set(other);
    ScalarGrid<N>::operator=(other);
    return *this;
}

template <size_t N>
VertexCenteredScalarGrid<N>& VertexCenteredScalarGrid<N>::operator=(
    VertexCenteredScalarGrid&& other) noexcept
{
    ScalarGrid<N>::operator=(std::move(other));
    return *this;
}

template <size_t N>
Vector<size_t, N> VertexCenteredScalarGrid<N>::DataSize() const
{
    if (Resolution() != Vector<size_t, N>{})
    {
        return Resolution() + Vector<size_t, N>::MakeConstant(1);
    }

    return Vector<size_t, N>{};
}

template <size_t N>
Vector<double, N> VertexCenteredScalarGrid<N>::DataOrigin() const
{
    return Origin();
}

template <size_t N>
std::shared_ptr<ScalarGrid<N>> VertexCenteredScalarGrid<N>::Clone() const
{
    return std::make_shared<VertexCenteredScalarGrid<N>>(*this);
}

template <size_t N>
void VertexCenteredScalarGrid<N>::Swap(Grid<N>* other)
{
    if (auto sameType = dynamic_cast<VertexCenteredScalarGrid*>(other);
        sameType)
    {
        SwapScalarGrid(sameType);
    }
}

template <size_t N>
void VertexCenteredScalarGrid<N>::Set(const VertexCenteredScalarGrid& other)
{
    SetScalarGrid(other);
}

template <size_t N>
typename VertexCenteredScalarGrid<N>::Builder
VertexCenteredScalarGrid<N>::GetBuilder()
{
    return Builder{};
}

template <size_t N>
typename VertexCenteredScalarGrid<N>::Builder&
VertexCenteredScalarGrid<N>::Builder::WithResolution(
    const Vector<size_t, N>& resolution)
{
    m_resolution = resolution;
    return *this;
}

template <size_t N>
typename VertexCenteredScalarGrid<N>::Builder&
VertexCenteredScalarGrid<N>::Builder::WithGridSpacing(
    const Vector<double, N>& gridSpacing)
{
    m_gridSpacing = gridSpacing;
    return *this;
}

template <size_t N>
typename VertexCenteredScalarGrid<N>::Builder&
VertexCenteredScalarGrid<N>::Builder::WithOrigin(
    const Vector<double, N>& gridOrigin)
{
    m_gridOrigin = gridOrigin;
    return *this;
}

template <size_t N>
typename VertexCenteredScalarGrid<N>::Builder&
VertexCenteredScalarGrid<N>::Builder::WithInitialValue(double initialVal)
{
    m_initialVal = initialVal;
    return *this;
}

template <size_t N>
VertexCenteredScalarGrid<N> VertexCenteredScalarGrid<N>::Builder::Build() const
{
    return VertexCenteredScalarGrid{ m_resolution, m_gridSpacing, m_gridOrigin,
                                     m_initialVal };
}

template <size_t N>
std::shared_ptr<VertexCenteredScalarGrid<N>>
VertexCenteredScalarGrid<N>::Builder::MakeShared() const
{
    return std::make_shared<VertexCenteredScalarGrid>(
        m_resolution, m_gridSpacing, m_gridOrigin, m_initialVal);
}

template <size_t N>
std::shared_ptr<ScalarGrid<N>> VertexCenteredScalarGrid<N>::Builder::Build(
    const Vector<size_t, N>& resolution, const Vector<double, N>& gridSpacing,
    const Vector<double, N>& gridOrigin, double initialVal) const
{
    return std::make_shared<VertexCenteredScalarGrid>(resolution, gridSpacing,
                                                      gridOrigin, initialVal);
}

template class VertexCenteredScalarGrid<2>;

template class VertexCenteredScalarGrid<3>;
}  // namespace CubbyFlow
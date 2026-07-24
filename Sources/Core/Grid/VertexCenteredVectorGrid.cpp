// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Grid/VertexCenteredVectorGrid.hpp>

namespace CubbyFlow
{
template <size_t N>
VertexCenteredVectorGrid<N>::VertexCenteredVectorGrid(
    const Vector<size_t, N>& resolution, const Vector<double, N>& gridSpacing,
    const Vector<double, N>& origin, const Vector<double, N>& initialValue)
{
    Resize(resolution, gridSpacing, origin, initialValue);
}

template <size_t N>
VertexCenteredVectorGrid<N>::VertexCenteredVectorGrid(
    const VertexCenteredVectorGrid& other)
    : CollocatedVectorGrid<N>{ other }
{
    // Do nothing
}

template <size_t N>
VertexCenteredVectorGrid<N>::VertexCenteredVectorGrid(
    VertexCenteredVectorGrid&& other) noexcept
    : CollocatedVectorGrid<N>{ std::move(other) }
{
    // Do nothing
}

template <size_t N>
VertexCenteredVectorGrid<N>& VertexCenteredVectorGrid<N>::operator=(
    const VertexCenteredVectorGrid& other)
{
    CollocatedVectorGrid<N>::operator=(other);
    return *this;
}

template <size_t N>
VertexCenteredVectorGrid<N>& VertexCenteredVectorGrid<N>::operator=(
    VertexCenteredVectorGrid&& other) noexcept
{
    CollocatedVectorGrid<N>::operator=(std::move(other));
    return *this;
}

template <size_t N>
Vector<size_t, N> VertexCenteredVectorGrid<N>::DataSize() const
{
    if (Resolution() != Vector<size_t, N>{})
    {
        return Resolution() + Vector<size_t, N>::MakeConstant(1);
    }

    return Vector<size_t, N>{};
}

template <size_t N>
Vector<double, N> VertexCenteredVectorGrid<N>::DataOrigin() const
{
    return Origin();
}

template <size_t N>
void VertexCenteredVectorGrid<N>::Swap(Grid<N>* other)
{
    if (auto sameType = dynamic_cast<VertexCenteredVectorGrid*>(other);
        sameType)
    {
        SwapCollocatedVectorGrid(sameType);
    }
}

template <size_t N>
void VertexCenteredVectorGrid<N>::Set(const VertexCenteredVectorGrid& other)
{
    SetCollocatedVectorGrid(other);
}

template <size_t N>
void VertexCenteredVectorGrid<N>::Fill(const Vector<double, N>& value,
                                       ExecutionPolicy policy)
{
    Vector<size_t, N> size = DataSize();
    auto view = DataView();

    ParallelForEachIndex(
        Vector<size_t, N>{}, size,
        [&view, &value](auto... indices) { view(indices...) = value; }, policy);
}

template <size_t N>
void VertexCenteredVectorGrid<N>::Fill(
    const std::function<Vector<double, N>(const Vector<double, N>&)>& func,
    ExecutionPolicy policy)
{
    Vector<size_t, N> size = DataSize();
    auto view = DataView();
    auto pos = DataPosition();

    ParallelForEachIndex(
        Vector<size_t, N>::MakeZero(), size,
        [&func, &view, &pos](auto... indices) {
            view(indices...) = func(pos(indices...));
        },
        policy);
}

template <size_t N>
std::shared_ptr<VectorGrid<N>> VertexCenteredVectorGrid<N>::Clone() const
{
    return std::make_shared<VertexCenteredVectorGrid<N>>(*this);
}

template <size_t N>
typename VertexCenteredVectorGrid<N>::Builder
VertexCenteredVectorGrid<N>::GetBuilder()
{
    return Builder{};
}

template <size_t N>
typename VertexCenteredVectorGrid<N>::Builder&
VertexCenteredVectorGrid<N>::Builder::WithResolution(
    const Vector<size_t, N>& resolution)
{
    m_resolution = resolution;
    return *this;
}

template <size_t N>
typename VertexCenteredVectorGrid<N>::Builder&
VertexCenteredVectorGrid<N>::Builder::WithGridSpacing(
    const Vector<double, N>& gridSpacing)
{
    m_gridSpacing = gridSpacing;
    return *this;
}

template <size_t N>
typename VertexCenteredVectorGrid<N>::Builder&
VertexCenteredVectorGrid<N>::Builder::WithOrigin(
    const Vector<double, N>& gridOrigin)
{
    m_gridOrigin = gridOrigin;
    return *this;
}

template <size_t N>
typename VertexCenteredVectorGrid<N>::Builder&
VertexCenteredVectorGrid<N>::Builder::WithInitialValue(
    const Vector<double, N>& initialVal)
{
    m_initialVal = initialVal;
    return *this;
}

template <size_t N>
VertexCenteredVectorGrid<N> VertexCenteredVectorGrid<N>::Builder::Build() const
{
    return VertexCenteredVectorGrid{ m_resolution, m_gridSpacing, m_gridOrigin,
                                     m_initialVal };
}

template <size_t N>
std::shared_ptr<VertexCenteredVectorGrid<N>>
VertexCenteredVectorGrid<N>::Builder::MakeShared() const
{
    return std::make_shared<VertexCenteredVectorGrid>(
        m_resolution, m_gridSpacing, m_gridOrigin, m_initialVal);
}

template <size_t N>
std::shared_ptr<VectorGrid<N>> VertexCenteredVectorGrid<N>::Builder::Build(
    const Vector<size_t, N>& resolution, const Vector<double, N>& gridSpacing,
    const Vector<double, N>& gridOrigin,
    const Vector<double, N>& initialVal) const
{
    return std::make_shared<VertexCenteredVectorGrid>(resolution, gridSpacing,
                                                      gridOrigin, initialVal);
}

template class VertexCenteredVectorGrid<2>;

template class VertexCenteredVectorGrid<3>;
}  // namespace CubbyFlow
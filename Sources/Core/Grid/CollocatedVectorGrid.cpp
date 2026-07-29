// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/FDM/FDMUtils.hpp>
#include <Core/Grid/CollocatedVectorGrid.hpp>

namespace CubbyFlow
{
template <size_t N>
CollocatedVectorGrid<N>::CollocatedVectorGrid()
    : m_linearSampler(m_data, Vector<double, N>::MakeConstant(1.0),
                      Vector<double, N>{})
{
    // Do nothing
}

template <size_t N>
const Vector<double, N>& CollocatedVectorGrid<N>::operator()(
    const Vector<size_t, N>& idx) const
{
    return m_data(idx);
}

template <size_t N>
Vector<double, N>& CollocatedVectorGrid<N>::operator()(
    const Vector<size_t, N>& idx)
{
    return m_data(idx);
}

template <size_t N>
double CollocatedVectorGrid<N>::DivergenceAtDataPoint(
    const Vector<size_t, N>& idx) const
{
    return GetFDMUtils<N>::Divergence(m_data, GridSpacing(), idx);
}

template <size_t N>
typename GetCurl<N>::Type CollocatedVectorGrid<N>::CurlAtDataPoint(
    const Vector<size_t, N>& idx) const
{
    return GetFDMUtils<N>::Curl(m_data, GridSpacing(), idx);
}

template <size_t N>
Vector<double, N> CollocatedVectorGrid<N>::Sample(
    const Vector<double, N>& x) const
{
    return m_sampler(x);
}

template <size_t N>
double CollocatedVectorGrid<N>::Divergence(const Vector<double, N>& x) const
{
    constexpr size_t numPoints = 1 << N;
    std::array<Vector<size_t, N>, numPoints> indices;
    std::array<double, numPoints> weights{};
    m_linearSampler.GetCoordinatesAndWeights(x, indices, weights);

    double result = 0.0;

    for (size_t i = 0; i < numPoints; ++i)
    {
        result += weights[i] * DivergenceAtDataPoint(indices[i]);
    }

    return result;
}

template <size_t N>
typename GetCurl<N>::Type CollocatedVectorGrid<N>::Curl(
    const Vector<double, N>& x) const
{
    constexpr size_t numPoints = 1 << N;
    std::array<Vector<size_t, N>, numPoints> indices;
    std::array<double, numPoints> weights{};
    m_linearSampler.GetCoordinatesAndWeights(x, indices, weights);

    typename GetCurl<N>::Type result{};

    for (size_t i = 0; i < numPoints; ++i)
    {
        result += weights[i] * CurlAtDataPoint(indices[i]);
    }

    return result;
}

template <size_t N>
std::function<Vector<double, N>(const Vector<double, N>&)>
CollocatedVectorGrid<N>::Sampler() const
{
    return m_sampler;
}

template <size_t N>
typename CollocatedVectorGrid<N>::VectorDataView
CollocatedVectorGrid<N>::DataView()
{
    return CollocatedVectorGrid<N>::VectorDataView{ m_data };
}

template <size_t N>
typename CollocatedVectorGrid<N>::ConstVectorDataView
CollocatedVectorGrid<N>::DataView() const
{
    return CollocatedVectorGrid<N>::ConstVectorDataView{ m_data };
}

template <size_t N>
GridDataPositionFunc<N> CollocatedVectorGrid<N>::DataPosition() const
{
    Vector<double, N> dataOrigin = DataOrigin();
    Vector<double, N> gridSpacing = GridSpacing();
    return GridDataPositionFunc<N>(
        [dataOrigin,
         gridSpacing](const Vector<size_t, N>& idx) -> Vector<double, N> {
            return dataOrigin +
                   ElemMul(gridSpacing, idx.template CastTo<double>());
        });
}

template <size_t N>
void CollocatedVectorGrid<N>::ForEachDataPointIndex(
    const std::function<void(const Vector<size_t, N>&)>& func) const
{
    ForEachIndex(m_data.Size(), GetUnroll<void, N>::Unroll(func));
}

template <size_t N>
void CollocatedVectorGrid<N>::ParallelForEachDataPointIndex(
    const std::function<void(const Vector<size_t, N>&)>& func) const
{
    ParallelForEachIndex(m_data.Size(), GetUnroll<void, N>::Unroll(func));
}

template <size_t N>
void CollocatedVectorGrid<N>::SwapCollocatedVectorGrid(
    CollocatedVectorGrid* other)
{
    SwapGrid(other);

    m_data.Swap(other->m_data);
    std::swap(m_linearSampler, other->m_linearSampler);
    std::swap(m_sampler, other->m_sampler);
}

template <size_t N>
void CollocatedVectorGrid<N>::SetCollocatedVectorGrid(
    const CollocatedVectorGrid& other)
{
    SetGrid(other);

    m_data.CopyFrom(other.m_data);
    ResetSampler();
}

template <size_t N>
void CollocatedVectorGrid<N>::OnResize(const Vector<size_t, N>& resolution,
                                       const Vector<double, N>& gridSpacing,
                                       const Vector<double, N>& origin,
                                       const Vector<double, N>& initialValue)
{
    UNUSED_VARIABLE(resolution);
    UNUSED_VARIABLE(gridSpacing);
    UNUSED_VARIABLE(origin);

    m_data.Resize(DataSize(), initialValue);
    ResetSampler();
}

template <size_t N>
void CollocatedVectorGrid<N>::ResetSampler()
{
    m_linearSampler = LinearArraySampler<Vector<double, N>, N>(
        m_data, GridSpacing(), DataOrigin());
    m_sampler = m_linearSampler.Functor();
}

template <size_t N>
void CollocatedVectorGrid<N>::GetData(Array1<double>& data) const
{
    const size_t size = N * Product(DataSize(), ONE_SIZE);
    data.Resize(size);

    size_t cnt = 0;
    ForEachIndex(m_data.Size(), [this, &data, &cnt](auto... indices) {
        const Vector<double, N>& value = m_data(indices...);
        for (size_t c = 0; c < N; ++c)
        {
            data[cnt++] = value[c];
        }
    });
}

template <size_t N>
void CollocatedVectorGrid<N>::SetData(const ConstArrayView1<double>& data)
{
    assert(N * Product(DataSize(), ONE_SIZE) == data.Length());

    size_t cnt = 0;
    ForEachIndex(m_data.Size(), [this, &data, &cnt](auto... indices) {
        for (size_t c = 0; c < N; ++c)
        {
            m_data(indices...)[c] = data[cnt++];
        }
    });
}

template class CollocatedVectorGrid<2>;

template class CollocatedVectorGrid<3>;
}  // namespace CubbyFlow
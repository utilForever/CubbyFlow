// This code is based on CubbyFlow framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/FDM/FDMUtils.hpp>
#include <Core/Grid/ScalarGrid.hpp>
#include <Core/Utils/FlatbuffersHelper.hpp>

#include <Flatbuffers/generated/ScalarGrid2_generated.h>
#include <Flatbuffers/generated/ScalarGrid3_generated.h>

namespace CubbyFlow
{
template <size_t N>
struct GetFlatbuffersScalarGrid
{
    // Do nothing
};

template <>
struct GetFlatbuffersScalarGrid<2>
{
    static flatbuffers::Offset<fbs::ScalarGrid2> CreateScalarGrid(
        flatbuffers::FlatBufferBuilder& fbb, const fbs::Vector2UZ* resolution,
        const fbs::Vector2D* gridSpacing, const fbs::Vector2D* origin,
        flatbuffers::Offset<flatbuffers::Vector<double>> data)
    {
        return CreateScalarGrid2(fbb, resolution, gridSpacing, origin, data);
    }

    static const fbs::ScalarGrid2* GetScalarGrid(const uint8_t* buf)
    {
        return fbs::GetScalarGrid2(buf);
    }
};

template <>
struct GetFlatbuffersScalarGrid<3>
{
    static flatbuffers::Offset<fbs::ScalarGrid3> CreateScalarGrid(
        flatbuffers::FlatBufferBuilder& fbb, const fbs::Vector3UZ* resolution,
        const fbs::Vector3D* gridSpacing, const fbs::Vector3D* origin,
        flatbuffers::Offset<flatbuffers::Vector<double>> data)
    {
        return CreateScalarGrid3(fbb, resolution, gridSpacing, origin, data);
    }

    static const fbs::ScalarGrid3* GetScalarGrid(const uint8_t* buf)
    {
        return fbs::GetScalarGrid3(buf);
    }
};

template <size_t N>
ScalarGrid<N>::ScalarGrid()
    : m_linearSampler(LinearArraySampler<double, N>(
          m_data, Vector<double, N>::MakeConstant(1), Vector<double, N>{}))
{
    // Do nothing
}

template <size_t N>
ScalarGrid<N>::ScalarGrid(const ScalarGrid& other) : Grid<N>{ other }
{
    // Do nothing
}

template <size_t N>
ScalarGrid<N>::ScalarGrid(ScalarGrid&& other) noexcept
    : Grid<N>{ std::move(other) },
      m_data(std::move(other.m_data)),
      m_linearSampler(std::move(other.m_linearSampler))
{
    m_sampler = m_linearSampler.Functor();
}

template <size_t N>
ScalarGrid<N>& ScalarGrid<N>::operator=(const ScalarGrid& other)
{
    Grid<N>::operator=(other);
    return *this;
}

template <size_t N>
ScalarGrid<N>& ScalarGrid<N>::operator=(ScalarGrid&& other) noexcept
{
    Grid<N>::operator=(std::move(other));
    m_data = std::move(other.m_data);
    m_linearSampler = std::move(other.m_linearSampler);
    m_sampler = m_linearSampler.Functor();
    return *this;
}

template <size_t N>
void ScalarGrid<N>::Clear()
{
    Resize(Vector<size_t, N>{}, GridSpacing(), Origin(), 0.0);
}

template <size_t N>
void ScalarGrid<N>::Resize(const Vector<size_t, N>& resolution,
                           const Vector<double, N>& gridSpacing,
                           const Vector<double, N>& origin, double initialValue)
{
    SetSizeParameters(resolution, gridSpacing, origin);

    m_data.Resize(DataSize(), initialValue);
    ResetSampler();
}

template <size_t N>
void ScalarGrid<N>::Resize(const Vector<double, N>& gridSpacing,
                           const Vector<double, N>& origin)
{
    Resize(Resolution(), gridSpacing, origin);
}

template <size_t N>
const double& ScalarGrid<N>::operator()(const Vector<size_t, N>& idx) const
{
    return m_data(idx);
}

template <size_t N>
double& ScalarGrid<N>::operator()(const Vector<size_t, N>& idx)
{
    return m_data(idx);
}

template <size_t N>
Vector<double, N> ScalarGrid<N>::GradientAtDataPoint(
    const Vector<size_t, N>& idx) const
{
    return GetFDMUtils<N>::Gradient(m_data, GridSpacing(), idx);
}

template <size_t N>
double ScalarGrid<N>::LaplacianAtDataPoint(const Vector<size_t, N>& idx) const
{
    return GetFDMUtils<N>::Laplacian(m_data, GridSpacing(), idx);
}

template <size_t N>
double ScalarGrid<N>::Sample(const Vector<double, N>& x) const
{
    return m_sampler(x);
}

template <size_t N>
std::function<double(const Vector<double, N>&)> ScalarGrid<N>::Sampler() const
{
    return m_sampler;
}

template <size_t N>
Vector<double, N> ScalarGrid<N>::Gradient(const Vector<double, N>& x) const
{
    constexpr size_t numPoints = 1u << N;
    std::array<Vector<size_t, N>, numPoints> indices;
    std::array<double, numPoints> weights{};
    m_linearSampler.GetCoordinatesAndWeights(x, indices, weights);

    Vector<double, N> result;

    for (size_t i = 0; i < numPoints; ++i)
    {
        result += weights[i] * GradientAtDataPoint(indices[i]);
    }

    return result;
}

template <size_t N>
double ScalarGrid<N>::Laplacian(const Vector<double, N>& x) const
{
    constexpr size_t numPoints = 1u << N;
    std::array<Vector<size_t, N>, numPoints> indices;
    std::array<double, numPoints> weights{};
    m_linearSampler.GetCoordinatesAndWeights(x, indices, weights);

    double result = 0.0;

    for (size_t i = 0; i < numPoints; ++i)
    {
        result += weights[i] * LaplacianAtDataPoint(indices[i]);
    }

    return result;
}

template <size_t N>
typename ScalarGrid<N>::ScalarDataView ScalarGrid<N>::DataView()
{
    return ScalarDataView{ m_data };
}

template <size_t N>
typename ScalarGrid<N>::ConstScalarDataView ScalarGrid<N>::DataView() const
{
    return ConstScalarDataView{ m_data };
}

template <size_t N>
GridDataPositionFunc<N> ScalarGrid<N>::DataPosition() const
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
void ScalarGrid<N>::Fill(double value, ExecutionPolicy policy)
{
    ParallelForEachIndex(
        Vector<size_t, N>(), m_data.Size(),
        [this, value](auto... indices) { m_data(indices...) = value; }, policy);
}

template <size_t N>
void ScalarGrid<N>::Fill(
    const std::function<double(const Vector<double, N>&)>& func,
    ExecutionPolicy policy)
{
    auto pos = DataPosition();
    ParallelForEachIndex(
        Vector<size_t, N>(), m_data.Size(),
        [this, &func, &pos](auto... indices) {
            m_data(indices...) = func(pos(Vector<size_t, N>(indices...)));
        },
        policy);
}

template <size_t N>
void ScalarGrid<N>::ForEachDataPointIndex(
    const std::function<void(const Vector<size_t, N>&)>& func) const
{
    ForEachIndex(m_data.Size(), GetUnroll<void, N>::Unroll(func));
}

template <size_t N>
void ScalarGrid<N>::ParallelForEachDataPointIndex(
    const std::function<void(const Vector<size_t, N>&)>& func) const
{
    ParallelForEachIndex(m_data.Size(), GetUnroll<void, N>::Unroll(func));
}

template <size_t N>
void ScalarGrid<N>::Serialize(std::vector<uint8_t>* buffer) const
{
    flatbuffers::FlatBufferBuilder builder(1024);

    auto fbsResolution = CubbyFlowToFlatbuffers(Resolution());
    auto fbsGridSpacing = CubbyFlowToFlatbuffers(GridSpacing());
    auto fbsOrigin = CubbyFlowToFlatbuffers(Origin());

    Array1<double> gridData;
    GetData(gridData);
    flatbuffers::Offset<flatbuffers::Vector<double>> data =
        builder.CreateVector(gridData.data(), gridData.Length());

    auto fbsGrid = GetFlatbuffersScalarGrid<N>::CreateScalarGrid(
        builder, &fbsResolution, &fbsGridSpacing, &fbsOrigin, data);

    builder.Finish(fbsGrid);

    uint8_t* buf = builder.GetBufferPointer();
    const size_t size = builder.GetSize();

    buffer->resize(size);
    memcpy(buffer->data(), buf, size);
}

template <size_t N>
void ScalarGrid<N>::Deserialize(const std::vector<uint8_t>& buffer)
{
    auto fbsGrid = GetFlatbuffersScalarGrid<N>::GetScalarGrid(buffer.data());

    Resize(FlatbuffersToCubbyFlow(*fbsGrid->resolution()),
           FlatbuffersToCubbyFlow(*fbsGrid->gridSpacing()),
           FlatbuffersToCubbyFlow(*fbsGrid->origin()));

    auto data = fbsGrid->data();
    Array1<double> gridData(data->size());
    std::copy(data->begin(), data->end(), gridData.begin());

    SetData(gridData);
}

template <size_t N>
void ScalarGrid<N>::SwapScalarGrid(ScalarGrid* other)
{
    SwapGrid(other);

    m_data.Swap(other->m_data);
    std::swap(m_linearSampler, other->m_linearSampler);
    std::swap(m_sampler, other->m_sampler);
}

template <size_t N>
void ScalarGrid<N>::SetScalarGrid(const ScalarGrid& other)
{
    SetGrid(other);

    m_data.CopyFrom(other.m_data);
    ResetSampler();
}

template <size_t N>
void ScalarGrid<N>::ResetSampler()
{
    m_linearSampler =
        LinearArraySampler<double, N>(m_data, GridSpacing(), DataOrigin());
    m_sampler = m_linearSampler.Functor();
}

template <size_t N>
void ScalarGrid<N>::GetData(Array1<double>& data) const
{
    const size_t size = Product(DataSize(), ONE_SIZE);
    data.Resize(size);
    std::copy(m_data.begin(), m_data.end(), data.begin());
}

template <size_t N>
void ScalarGrid<N>::SetData(const ConstArrayView1<double>& data)
{
    assert(Product(DataSize(), ONE_SIZE) == data.Length());

    std::copy(data.begin(), data.end(), m_data.begin());
}

template class ScalarGrid<2>;

template class ScalarGrid<3>;

template class ScalarGridBuilder<2>;

template class ScalarGridBuilder<3>;
}  // namespace CubbyFlow

// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Grid/FaceCenteredGrid.hpp>

#include <algorithm>

namespace CubbyFlow
{
namespace Internal
{
double Divergence(const FaceCenteredGrid2& grid, const Vector2D& x)
{
    size_t i, j;
    double fx, fy;
    const Vector2D cellCenterOrigin = grid.Origin() + 0.5 * grid.GridSpacing();
    const Vector2D normalizedX =
        ElemDiv((x - cellCenterOrigin), grid.GridSpacing());

    GetBarycentric(normalizedX.x, 0, static_cast<ssize_t>(grid.Resolution().x),
                   i, fx);
    GetBarycentric(normalizedX.y, 0, static_cast<ssize_t>(grid.Resolution().y),
                   j, fy);

    std::array<Vector2UZ, 4> indices;
    std::array<double, 4> weights{};

    indices[0] = Vector2UZ{ i, j };
    indices[1] = Vector2UZ{ i + 1, j };
    indices[2] = Vector2UZ{ i, j + 1 };
    indices[3] = Vector2UZ{ i + 1, j + 1 };

    weights[0] = (1.0 - fx) * (1.0 - fy);
    weights[1] = fx * (1.0 - fy);
    weights[2] = (1.0 - fx) * fy;
    weights[3] = fx * fy;

    double result = 0.0;

    for (int n = 0; n < 4; ++n)
    {
        result += weights[n] * grid.DivergenceAtCellCenter(indices[n]);
    }

    return result;
}

double Curl(const FaceCenteredGrid2& grid, const Vector2UZ& idx)
{
    const size_t i = idx.x;
    const size_t j = idx.y;
    const Vector2D& gs = grid.GridSpacing();
    const Vector2UZ& res = grid.Resolution();

    assert(i < res.x && j < res.y);

    const Vector2D left = grid.ValueAtCellCenter((i > 0) ? i - 1 : i, j);
    const Vector2D right =
        grid.ValueAtCellCenter((i + 1 < res.x) ? i + 1 : i, j);
    const Vector2D bottom = grid.ValueAtCellCenter(i, (j > 0) ? j - 1 : j);
    const Vector2D top = grid.ValueAtCellCenter(i, (j + 1 < res.y) ? j + 1 : j);

    const double Fx_ym = bottom.x;
    const double Fx_yp = top.x;

    const double Fy_xm = left.y;
    const double Fy_xp = right.y;

    return 0.5 * (Fy_xp - Fy_xm) / gs.x - 0.5 * (Fx_yp - Fx_ym) / gs.y;
}

double Curl(const FaceCenteredGrid2& grid, const Vector2D& x)
{
    size_t i, j;
    double fx, fy;
    const Vector2D cellCenterOrigin = grid.Origin() + 0.5 * grid.GridSpacing();
    const Vector2D normalizedX =
        ElemDiv((x - cellCenterOrigin), grid.GridSpacing());

    GetBarycentric(normalizedX.x, static_cast<ssize_t>(grid.Resolution().x), i,
                   fx);
    GetBarycentric(normalizedX.y, static_cast<ssize_t>(grid.Resolution().y), j,
                   fy);

    std::array<Vector2UZ, 4> indices;
    std::array<double, 4> weights{};

    indices[0] = Vector2UZ{ i, j };
    indices[1] = Vector2UZ{ i + 1, j };
    indices[2] = Vector2UZ{ i, j + 1 };
    indices[3] = Vector2UZ{ i + 1, j + 1 };

    weights[0] = (1.0 - fx) * (1.0 - fy);
    weights[1] = fx * (1.0 - fy);
    weights[2] = (1.0 - fx) * fy;
    weights[3] = fx * fy;

    double result = 0.0;

    for (int n = 0; n < 4; ++n)
    {
        result += weights[n] * grid.CurlAtCellCenter(indices[n]);
    }

    return result;
}

double Divergence(const FaceCenteredGrid3& grid, const Vector3D& x)
{
    const Vector3UZ& res = grid.Resolution();
    size_t i, j, k;
    double fx, fy, fz;
    const Vector3D cellCenterOrigin = grid.Origin() + 0.5 * grid.GridSpacing();
    const Vector3D normalizedX =
        ElemDiv((x - cellCenterOrigin), grid.GridSpacing());

    GetBarycentric(normalizedX.x, static_cast<ssize_t>(res.x), i, fx);
    GetBarycentric(normalizedX.y, static_cast<ssize_t>(res.y), j, fy);
    GetBarycentric(normalizedX.z, static_cast<ssize_t>(res.z), k, fz);

    std::array<Vector3UZ, 8> indices;
    std::array<double, 8> weights{};

    indices[0] = Vector3UZ{ i, j, k };
    indices[1] = Vector3UZ{ i + 1, j, k };
    indices[2] = Vector3UZ{ i, j + 1, k };
    indices[3] = Vector3UZ{ i + 1, j + 1, k };
    indices[4] = Vector3UZ{ i, j, k + 1 };
    indices[5] = Vector3UZ{ i + 1, j, k + 1 };
    indices[6] = Vector3UZ{ i, j + 1, k + 1 };
    indices[7] = Vector3UZ{ i + 1, j + 1, k + 1 };

    weights[0] = (1.0 - fx) * (1.0 - fy) * (1.0 - fz);
    weights[1] = fx * (1.0 - fy) * (1.0 - fz);
    weights[2] = (1.0 - fx) * fy * (1.0 - fz);
    weights[3] = fx * fy * (1.0 - fz);
    weights[4] = (1.0 - fx) * (1.0 - fy) * fz;
    weights[5] = fx * (1.0 - fy) * fz;
    weights[6] = (1.0 - fx) * fy * fz;
    weights[7] = fx * fy * fz;

    double result = 0.0;

    for (int n = 0; n < 8; ++n)
    {
        result += weights[n] * grid.DivergenceAtCellCenter(
                                   indices[n].x, indices[n].y, indices[n].z);
    }

    return result;
}

Vector3D Curl(const FaceCenteredGrid3& grid, const Vector3UZ& idx)
{
    const size_t i = idx.x;
    const size_t j = idx.y;
    const size_t k = idx.z;
    const Vector3D& gs = grid.GridSpacing();
    const Vector3UZ& res = grid.Resolution();

    assert(i < res.x && j < res.y && k < res.z);

    const Vector3D left = grid.ValueAtCellCenter((i > 0) ? i - 1 : i, j, k);
    const Vector3D right =
        grid.ValueAtCellCenter((i + 1 < res.x) ? i + 1 : i, j, k);
    const Vector3D down = grid.ValueAtCellCenter(i, (j > 0) ? j - 1 : j, k);
    const Vector3D up =
        grid.ValueAtCellCenter(i, (j + 1 < res.y) ? j + 1 : j, k);
    const Vector3D back = grid.ValueAtCellCenter(i, j, (k > 0) ? k - 1 : k);
    const Vector3D front =
        grid.ValueAtCellCenter(i, j, (k + 1 < res.z) ? k + 1 : k);

    const double Fx_ym = down.x;
    const double Fx_yp = up.x;
    const double Fx_zm = back.x;
    const double Fx_zp = front.x;

    const double Fy_xm = left.y;
    const double Fy_xp = right.y;
    const double Fy_zm = back.y;
    const double Fy_zp = front.y;

    const double Fz_xm = left.z;
    const double Fz_xp = right.z;
    const double Fz_ym = down.z;
    const double Fz_yp = up.z;

    return Vector3D{
        0.5 * (Fz_yp - Fz_ym) / gs.y - 0.5 * (Fy_zp - Fy_zm) / gs.z,
        0.5 * (Fx_zp - Fx_zm) / gs.z - 0.5 * (Fz_xp - Fz_xm) / gs.x,
        0.5 * (Fy_xp - Fy_xm) / gs.x - 0.5 * (Fx_yp - Fx_ym) / gs.y
    };
}

Vector3D Curl(const FaceCenteredGrid3& grid, const Vector3D& x)
{
    const Vector3UZ& res = grid.Resolution();
    size_t i, j, k;
    double fx, fy, fz;
    const Vector3D cellCenterOrigin = grid.Origin() + 0.5 * grid.GridSpacing();
    const Vector3D normalizedX =
        ElemDiv((x - cellCenterOrigin), grid.GridSpacing());

    GetBarycentric(normalizedX.x, static_cast<ssize_t>(res.x), i, fx);
    GetBarycentric(normalizedX.y, static_cast<ssize_t>(res.y), j, fy);
    GetBarycentric(normalizedX.z, static_cast<ssize_t>(res.z), k, fz);

    std::array<Vector3UZ, 8> indices;
    std::array<double, 8> weights{};

    indices[0] = Vector3UZ{ i, j, k };
    indices[1] = Vector3UZ{ i + 1, j, k };
    indices[2] = Vector3UZ{ i, j + 1, k };
    indices[3] = Vector3UZ{ i + 1, j + 1, k };
    indices[4] = Vector3UZ{ i, j, k + 1 };
    indices[5] = Vector3UZ{ i + 1, j, k + 1 };
    indices[6] = Vector3UZ{ i, j + 1, k + 1 };
    indices[7] = Vector3UZ{ i + 1, j + 1, k + 1 };

    weights[0] = (1.0 - fx) * (1.0 - fy) * (1.0 - fz);
    weights[1] = fx * (1.0 - fy) * (1.0 - fz);
    weights[2] = (1.0 - fx) * fy * (1.0 - fz);
    weights[3] = fx * fy * (1.0 - fz);
    weights[4] = (1.0 - fx) * (1.0 - fy) * fz;
    weights[5] = fx * (1.0 - fy) * fz;
    weights[6] = (1.0 - fx) * fy * fz;
    weights[7] = fx * fy * fz;

    Vector3D result;

    for (int n = 0; n < 8; ++n)
    {
        result += weights[n] * grid.CurlAtCellCenter(indices[n]);
    }

    return result;
}
}  // namespace Internal

template <size_t N>
FaceCenteredGrid<N>::FaceCenteredGrid()
{
    for (size_t i = 0; i < N; ++i)
    {
        Vector<double, N> dataOrigin = Vector<double, N>::MakeConstant(0.5);
        dataOrigin[i] = 0.0;

        m_dataOrigins[i] = dataOrigin;
        m_linearSamplers[i] = LinearArraySampler<double, N>(
            m_data[i], Vector<double, N>::MakeConstant(1.0), dataOrigin);
    }
}

template <size_t N>
FaceCenteredGrid<N>::FaceCenteredGrid(const Vector<size_t, N>& resolution,
                                      const Vector<double, N>& gridSpacing,
                                      const Vector<double, N>& origin,
                                      const Vector<double, N>& initialValue)
    : FaceCenteredGrid{}
{
    Resize(resolution, gridSpacing, origin, initialValue);
}

template <size_t N>
FaceCenteredGrid<N>::FaceCenteredGrid(const FaceCenteredGrid& other)
    : VectorGrid<N>{ other }, m_dataOrigins(other.m_dataOrigins)
{
    for (size_t i = 0; i < N; ++i)
    {
        m_data[i].CopyFrom(other.m_data[i]);
    }

    ResetSampler();
}

template <size_t N>
FaceCenteredGrid<N>::FaceCenteredGrid(FaceCenteredGrid&& other) noexcept
    : VectorGrid<N>{ std::move(other) },
      m_data(std::move(other.m_data)),
      m_dataOrigins(std::move(other.m_dataOrigins))
{
    ResetSampler();
}

template <size_t N>
FaceCenteredGrid<N>& FaceCenteredGrid<N>::operator=(
    const FaceCenteredGrid& other)
{
    for (size_t i = 0; i < N; ++i)
    {
        m_data[i].CopyFrom(other.m_data[i]);
    }

    m_dataOrigins = other.m_dataOrigins;
    Grid<N>::operator=(other);
    ResetSampler();
    return *this;
}

template <size_t N>
FaceCenteredGrid<N>& FaceCenteredGrid<N>::operator=(
    FaceCenteredGrid&& other) noexcept
{
    m_data = std::move(other.m_data);
    m_dataOrigins = std::move(other.m_dataOrigins);
    Grid<N>::operator=(std::move(other));
    ResetSampler();
    return *this;
}

template <size_t N>
void FaceCenteredGrid<N>::Swap(Grid<N>* other)
{
    if (auto sameType = dynamic_cast<FaceCenteredGrid*>(other); sameType)
    {
        SwapGrid(sameType);

        for (size_t i = 0; i < N; ++i)
        {
            m_data[i].Swap(sameType->m_data[i]);
            std::swap(m_dataOrigins[i], sameType->m_dataOrigins[i]);
            std::swap(m_linearSamplers[i], sameType->m_linearSamplers[i]);
        }

        std::swap(m_sampler, sameType->m_sampler);
    }
}

template <size_t N>
void FaceCenteredGrid<N>::Set(const FaceCenteredGrid& other)
{
    SetGrid(other);

    for (size_t i = 0; i < N; ++i)
    {
        m_data[i].CopyFrom(other.m_data[i]);
    }

    m_dataOrigins = other.m_dataOrigins;

    ResetSampler();
}

template <size_t N>
double& FaceCenteredGrid<N>::U(const Vector<size_t, N>& idx)
{
    return m_data[0](idx);
}

template <size_t N>
const double& FaceCenteredGrid<N>::U(const Vector<size_t, N>& idx) const
{
    return m_data[0](idx);
}

template <size_t N>
double& FaceCenteredGrid<N>::V(const Vector<size_t, N>& idx)
{
    return m_data[1](idx);
}

template <size_t N>
const double& FaceCenteredGrid<N>::V(const Vector<size_t, N>& idx) const
{
    return m_data[1](idx);
}

template <size_t N>
Vector<double, N> FaceCenteredGrid<N>::ValueAtCellCenter(
    const Vector<size_t, N>& idx) const
{
    Vector<double, N> result;

    for (size_t i = 0; i < N; ++i)
    {
        assert(idx[i] < Resolution()[i]);
        result[i] = 0.5 * (m_data[i](idx) +
                           m_data[i](idx + Vector<size_t, N>::MakeUnit(i)));
    }

    return result;
}

template <size_t N>
double FaceCenteredGrid<N>::DivergenceAtCellCenter(
    const Vector<size_t, N>& idx) const
{
    const Vector<double, N>& gridSpacing = GridSpacing();
    double sum = 0.0;

    for (size_t i = 0; i < N; ++i)
    {
        assert(idx[i] < Resolution()[i]);
        sum +=
            (m_data[i](idx + Vector<size_t, N>::MakeUnit(i)) - m_data[i](idx)) /
            gridSpacing[i];
    }

    return sum;
}

template <size_t N>
typename GetCurl<N>::Type FaceCenteredGrid<N>::CurlAtCellCenter(
    const Vector<size_t, N>& idx) const
{
    return Internal::Curl(*this, idx);
}

template <size_t N>
typename FaceCenteredGrid<N>::ScalarDataView FaceCenteredGrid<N>::UView()
{
    return DataView(0);
}

template <size_t N>
typename FaceCenteredGrid<N>::ConstScalarDataView FaceCenteredGrid<N>::UView()
    const
{
    return DataView(0);
}

template <size_t N>
typename FaceCenteredGrid<N>::ScalarDataView FaceCenteredGrid<N>::VView()
{
    return DataView(1);
}

template <size_t N>
typename FaceCenteredGrid<N>::ConstScalarDataView FaceCenteredGrid<N>::VView()
    const
{
    return DataView(1);
}

template <size_t N>
typename FaceCenteredGrid<N>::ScalarDataView FaceCenteredGrid<N>::DataView(
    size_t i)
{
    return m_data[i].View();
}

template <size_t N>
typename FaceCenteredGrid<N>::ConstScalarDataView FaceCenteredGrid<N>::DataView(
    size_t i) const
{
    return m_data[i].View();
}

template <size_t N>
GridDataPositionFunc<N> FaceCenteredGrid<N>::UPosition() const
{
    return DataPosition(0);
}

template <size_t N>
GridDataPositionFunc<N> FaceCenteredGrid<N>::VPosition() const
{
    return DataPosition(1);
}

template <size_t N>
GridDataPositionFunc<N> FaceCenteredGrid<N>::DataPosition(size_t i) const
{
    Vector<double, N> gridSpacing = GridSpacing();
    Vector<double, N> dataOrigin = m_dataOrigins[i];

    return GridDataPositionFunc<N>(
        [gridSpacing,
         dataOrigin](const Vector<size_t, N>& idx) -> Vector<double, N> {
            return dataOrigin +
                   ElemMul(gridSpacing, idx.template CastTo<double>());
        });
}

template <size_t N>
Vector<size_t, N> FaceCenteredGrid<N>::USize() const
{
    return DataSize(0);
}

template <size_t N>
Vector<size_t, N> FaceCenteredGrid<N>::VSize() const
{
    return DataSize(1);
}

template <size_t N>
Vector<size_t, N> FaceCenteredGrid<N>::DataSize(size_t i) const
{
    return m_data[i].Size();
}

template <size_t N>
Vector<double, N> FaceCenteredGrid<N>::UOrigin() const
{
    return DataOrigin(0);
}

template <size_t N>
Vector<double, N> FaceCenteredGrid<N>::VOrigin() const
{
    return DataOrigin(1);
}

template <size_t N>
Vector<double, N> FaceCenteredGrid<N>::DataOrigin(size_t i) const
{
    return m_dataOrigins[i];
}

template <size_t N>
void FaceCenteredGrid<N>::Fill(const Vector<double, N>& value,
                               ExecutionPolicy policy)
{
    for (size_t i = 0; i < N; ++i)
    {
        ParallelForEachIndex(
            DataSize(i),
            [this, value, i](auto... indices) {
                m_data[i](indices...) = value[i];
            },
            policy);
    }
}

template <size_t N>
void FaceCenteredGrid<N>::Fill(
    const std::function<Vector<double, N>(const Vector<double, N>&)>& func,
    ExecutionPolicy policy)
{
    for (size_t i = 0; i < N; ++i)
    {
        auto pos = DataPosition(i);
        ParallelForEachIndex(
            DataSize(i),
            [this, &func, &pos, i](auto... indices) {
                m_data[i](indices...) = func(pos(indices...))[i];
            },
            policy);
    }
}

template <size_t N>
std::shared_ptr<VectorGrid<N>> FaceCenteredGrid<N>::Clone() const
{
    return std::make_shared<FaceCenteredGrid<N>>(*this);
}

template <size_t N>
void FaceCenteredGrid<N>::ForEachUIndex(
    const std::function<void(const Vector<size_t, N>&)>& func) const
{
    ForEachIndex(DataSize(0), GetUnroll<void, N>::Unroll(func));
}

template <size_t N>
void FaceCenteredGrid<N>::ParallelForEachUIndex(
    const std::function<void(const Vector<size_t, N>&)>& func) const
{
    ParallelForEachIndex(DataSize(0), GetUnroll<void, N>::Unroll(func));
}

template <size_t N>
void FaceCenteredGrid<N>::ForEachVIndex(
    const std::function<void(const Vector<size_t, N>&)>& func) const
{
    ForEachIndex(DataSize(1), GetUnroll<void, N>::Unroll(func));
}

template <size_t N>
void FaceCenteredGrid<N>::ParallelForEachVIndex(
    const std::function<void(const Vector<size_t, N>&)>& func) const
{
    ParallelForEachIndex(DataSize(1), GetUnroll<void, N>::Unroll(func));
}

template <size_t N>
Vector<double, N> FaceCenteredGrid<N>::Sample(const Vector<double, N>& x) const
{
    return m_sampler(x);
}

template <size_t N>
std::function<Vector<double, N>(const Vector<double, N>&)>
FaceCenteredGrid<N>::Sampler() const
{
    return m_sampler;
}

template <size_t N>
double FaceCenteredGrid<N>::Divergence(const Vector<double, N>& x) const
{
    return Internal::Divergence(*this, x);
}

template <size_t N>
typename GetCurl<N>::Type FaceCenteredGrid<N>::Curl(
    const Vector<double, N>& x) const
{
    return Internal::Curl(*this, x);
}

template <size_t N>
void FaceCenteredGrid<N>::OnResize(const Vector<size_t, N>& resolution,
                                   const Vector<double, N>& gridSpacing,
                                   const Vector<double, N>& origin,
                                   const Vector<double, N>& initialValue)
{
    for (size_t i = 0; i < N; ++i)
    {
        Vector<size_t, N> dataRes =
            (resolution != Vector<size_t, N>{})
                ? resolution + Vector<size_t, N>::MakeUnit(i)
                : resolution;
        m_data[i].Resize(dataRes, initialValue[i]);

        Vector<double, N> offset = 0.5 * gridSpacing;
        offset[i] = 0.0;

        m_dataOrigins[i] = origin + offset;
    }

    ResetSampler();
}

template <size_t N>
void FaceCenteredGrid<N>::ResetSampler()
{
    for (size_t i = 0; i < N; ++i)
    {
        m_linearSamplers[i] = LinearArraySampler<double, N>(
            m_data[i], GridSpacing(), m_dataOrigins[i]);
    }

    std::array<LinearArraySampler<double, N>, N> linSamplers = m_linearSamplers;

    m_sampler = [linSamplers](const Vector<double, N>& x) -> Vector<double, N> {
        Vector<double, N> result;
        for (size_t i = 0; i < N; ++i)
        {
            result[i] = linSamplers[i](x);
        }
        return result;
    };
}

template <size_t N>
typename FaceCenteredGrid<N>::Builder FaceCenteredGrid<N>::GetBuilder()
{
    return Builder{};
}

template <size_t N>
void FaceCenteredGrid<N>::GetData(Array1<double>& data) const
{
    size_t size = 0;
    for (size_t i = 0; i < N; ++i)
    {
        size += Product(DataSize(i), ONE_SIZE);
    }

    data.Resize(size);

    size_t cnt = 0;
    for (size_t i = 0; i < N; ++i)
    {
        std::ranges::for_each(
            m_data[i], [&data, &cnt](double value) { data[cnt++] = value; });
    }
}

template <size_t N>
void FaceCenteredGrid<N>::SetData(const ConstArrayView1<double>& data)
{
    size_t size = 0;
    for (size_t i = 0; i < N; ++i)
    {
        size += Product(DataSize(i), ONE_SIZE);
    }
    assert(size == data.Length());

    size_t cnt = 0;
    for (size_t i = 0; i < N; ++i)
    {
        ForEachIndex(m_data[i].Size(),
                     [this, &i, &data, &cnt](auto... indices) {
                         m_data[i](indices...) = data[cnt++];
                     });
    }
}

template <size_t N>
typename FaceCenteredGrid<N>::Builder&
FaceCenteredGrid<N>::Builder::WithResolution(
    const Vector<size_t, N>& resolution)
{
    m_resolution = resolution;
    return *this;
}

template <size_t N>
typename FaceCenteredGrid<N>::Builder&
FaceCenteredGrid<N>::Builder::WithGridSpacing(
    const Vector<double, N>& gridSpacing)
{
    m_gridSpacing = gridSpacing;
    return *this;
}

template <size_t N>
typename FaceCenteredGrid<N>::Builder& FaceCenteredGrid<N>::Builder::WithOrigin(
    const Vector<double, N>& gridOrigin)
{
    m_gridOrigin = gridOrigin;
    return *this;
}

template <size_t N>
typename FaceCenteredGrid<N>::Builder&
FaceCenteredGrid<N>::Builder::WithInitialValue(
    const Vector<double, N>& initialVal)
{
    m_initialVal = initialVal;
    return *this;
}

template <size_t N>
FaceCenteredGrid<N> FaceCenteredGrid<N>::Builder::Build() const
{
    return FaceCenteredGrid{ m_resolution, m_gridSpacing, m_gridOrigin,
                             m_initialVal };
}

template <size_t N>
std::shared_ptr<FaceCenteredGrid<N>> FaceCenteredGrid<N>::Builder::MakeShared()
    const
{
    return std::make_shared<FaceCenteredGrid>(m_resolution, m_gridSpacing,
                                              m_gridOrigin, m_initialVal);
}

template <size_t N>
std::shared_ptr<VectorGrid<N>> FaceCenteredGrid<N>::Builder::Build(
    const Vector<size_t, N>& resolution, const Vector<double, N>& gridSpacing,
    const Vector<double, N>& gridOrigin,
    const Vector<double, N>& initialVal) const
{
    return std::make_shared<FaceCenteredGrid>(resolution, gridSpacing,
                                              gridOrigin, initialVal);
}

template class FaceCenteredGrid<2>;

template class FaceCenteredGrid<3>;
}  // namespace CubbyFlow

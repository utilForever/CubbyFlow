// This code is based on CubbyFlow framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Grid/VectorGrid.hpp>
#include <Core/Utils/FlatbuffersHelper.hpp>

#include <Flatbuffers/generated/VectorGrid2_generated.h>
#include <Flatbuffers/generated/VectorGrid3_generated.h>

namespace CubbyFlow
{
template <size_t N>
struct GetFlatbuffersVectorGrid
{
    // Do nothing
};

template <>
struct GetFlatbuffersVectorGrid<2>
{
    static flatbuffers::Offset<fbs::VectorGrid2> CreateVectorGrid(
        flatbuffers::FlatBufferBuilder& fbb, const fbs::Vector2UZ* resolution,
        const fbs::Vector2D* gridSpacing, const fbs::Vector2D* origin,
        flatbuffers::Offset<flatbuffers::Vector<double>> data)
    {
        return CreateVectorGrid2(fbb, resolution, gridSpacing, origin, data);
    }

    static const fbs::VectorGrid2* GetVectorGrid(const uint8_t* buf)
    {
        return fbs::GetVectorGrid2(buf);
    }
};

template <>
struct GetFlatbuffersVectorGrid<3>
{
    static flatbuffers::Offset<fbs::VectorGrid3> CreateVectorGrid(
        flatbuffers::FlatBufferBuilder& fbb, const fbs::Vector3UZ* resolution,
        const fbs::Vector3D* gridSpacing, const fbs::Vector3D* origin,
        flatbuffers::Offset<flatbuffers::Vector<double>> data)
    {
        return CreateVectorGrid3(fbb, resolution, gridSpacing, origin, data);
    }

    static const fbs::VectorGrid3* GetVectorGrid(const uint8_t* buf)
    {
        return fbs::GetVectorGrid3(buf);
    }
};

template <size_t N>
VectorGrid<N>::VectorGrid(const VectorGrid& other) : Grid<N>{ other }
{
    // Do nothing
}

template <size_t N>
VectorGrid<N>::VectorGrid(VectorGrid&& other) noexcept
    : Grid<N>{ std::move(other) }
{
    // Do nothing
}

template <size_t N>
VectorGrid<N>& VectorGrid<N>::operator=(const VectorGrid& other)
{
    Grid<N>::operator=(other);
    return *this;
}

template <size_t N>
VectorGrid<N>& VectorGrid<N>::operator=(VectorGrid&& other) noexcept
{
    Grid<N>::operator=(std::move(other));
    return *this;
}

template <size_t N>
void VectorGrid<N>::Clear()
{
    Resize(Vector<size_t, N>{}, GridSpacing(), Origin(), Vector<double, N>{});
}

template <size_t N>
void VectorGrid<N>::Resize(const Vector<size_t, N>& resolution,
                           const Vector<double, N>& gridSpacing,
                           const Vector<double, N>& origin,
                           const Vector<double, N>& initialValue)
{
    SetSizeParameters(resolution, gridSpacing, origin);

    OnResize(resolution, gridSpacing, origin, initialValue);
}

template <size_t N>
void VectorGrid<N>::Resize(const Vector<double, N>& gridSpacing,
                           const Vector<double, N>& origin)
{
    Resize(Resolution(), gridSpacing, origin);
}

template <size_t N>
void VectorGrid<N>::Serialize(std::vector<uint8_t>* buffer) const
{
    flatbuffers::FlatBufferBuilder builder(1024);

    auto fbsResolution = CubbyFlowToFlatbuffers(Resolution());
    auto fbsGridSpacing = CubbyFlowToFlatbuffers(GridSpacing());
    auto fbsOrigin = CubbyFlowToFlatbuffers(Origin());

    Array1<double> gridData;
    GetData(gridData);
    flatbuffers::Offset<flatbuffers::Vector<double>> data =
        builder.CreateVector(gridData.data(), gridData.Length());

    auto fbsGrid = GetFlatbuffersVectorGrid<N>::CreateVectorGrid(
        builder, &fbsResolution, &fbsGridSpacing, &fbsOrigin, data);

    builder.Finish(fbsGrid);

    uint8_t* buf = builder.GetBufferPointer();
    const size_t size = builder.GetSize();

    buffer->resize(size);
    memcpy(buffer->data(), buf, size);
}

template <size_t N>
void VectorGrid<N>::Deserialize(const std::vector<uint8_t>& buffer)
{
    auto fbsGrid = GetFlatbuffersVectorGrid<N>::GetVectorGrid(buffer.data());

    Resize(FlatbuffersToCubbyFlow(*fbsGrid->resolution()),
           FlatbuffersToCubbyFlow(*fbsGrid->gridSpacing()),
           FlatbuffersToCubbyFlow(*fbsGrid->origin()));

    auto data = fbsGrid->data();
    Array1<double> gridData(data->size());
    std::copy(data->begin(), data->end(), gridData.begin());

    SetData(gridData);
}

template class VectorGrid<2>;

template class VectorGrid<3>;

template class VectorGridBuilder<2>;

template class VectorGridBuilder<3>;
}  // namespace CubbyFlow

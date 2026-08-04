// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Grid/GridSystemData.hpp>
#include <Core/Utils/Factory.hpp>
#include <Core/Utils/FlatbuffersHelper.hpp>

#include <Flatbuffers/generated/GridSystemData2_generated.h>
#include <Flatbuffers/generated/GridSystemData3_generated.h>

namespace CubbyFlow
{
template <size_t N>
GridSystemData<N>::GridSystemData()
    : GridSystemData(Vector<size_t, N>{}, Vector<double, N>::MakeConstant(1.0),
                     Vector<double, N>{})
{
    // Do nothing
}

template <size_t N>
GridSystemData<N>::GridSystemData(const Vector<size_t, N>& resolution,
                                  const Vector<double, N>& gridSpacing,
                                  const Vector<double, N>& origin)
{
    m_velocity = std::make_shared<FaceCenteredGrid<N>>();
    m_advectableVectorDataList.push_back(m_velocity);
    m_velocityIdx = 0;

    Resize(resolution, gridSpacing, origin);
}

template <size_t N>
GridSystemData<N>::GridSystemData(const GridSystemData& other)
    : m_resolution(other.m_resolution),
      m_gridSpacing(other.m_gridSpacing),
      m_origin(other.m_origin)
{
    for (auto& data : other.m_scalarDataList)
    {
        m_scalarDataList.push_back(data->Clone());
    }
    for (auto& data : other.m_vectorDataList)
    {
        m_vectorDataList.push_back(data->Clone());
    }
    for (auto& data : other.m_advectableScalarDataList)
    {
        m_advectableScalarDataList.push_back(data->Clone());
    }
    for (auto& data : other.m_advectableVectorDataList)
    {
        m_advectableVectorDataList.push_back(data->Clone());
    }

    assert(m_advectableVectorDataList.size() > 0);

    m_velocity = std::dynamic_pointer_cast<FaceCenteredGrid<N>>(
        m_advectableVectorDataList[0]);

    assert(m_velocity != nullptr);
}

template <size_t N>
GridSystemData<N>::GridSystemData(GridSystemData&& other) noexcept
    : m_resolution(std::move(other.m_resolution)),
      m_gridSpacing(std::move(other.m_gridSpacing)),
      m_origin(std::move(other.m_origin)),
      m_scalarDataList(std::move(other.m_scalarDataList)),
      m_vectorDataList(std::move(other.m_vectorDataList)),
      m_advectableScalarDataList(std::move(other.m_advectableScalarDataList)),
      m_advectableVectorDataList(std::move(other.m_advectableVectorDataList))
{
    m_velocity = std::dynamic_pointer_cast<FaceCenteredGrid<N>>(
        m_advectableVectorDataList[0]);
    assert(m_velocity != nullptr);
}

template <size_t N>
GridSystemData<N>& GridSystemData<N>::operator=(const GridSystemData& other)
{
    if (this != &other)
    {
        *this = GridSystemData(other);
    }

    return *this;
}

template <size_t N>
GridSystemData<N>& GridSystemData<N>::operator=(GridSystemData&& other) noexcept
{
    m_resolution = std::move(other.m_resolution);
    m_gridSpacing = std::move(other.m_gridSpacing);
    m_origin = std::move(other.m_origin);
    m_scalarDataList = std::move(other.m_scalarDataList);
    m_vectorDataList = std::move(other.m_vectorDataList);
    m_advectableScalarDataList = std::move(other.m_advectableScalarDataList);
    m_advectableVectorDataList = std::move(other.m_advectableVectorDataList);

    m_velocity = std::dynamic_pointer_cast<FaceCenteredGrid<N>>(
        m_advectableVectorDataList[0]);
    assert(m_advectableVectorDataList.size() > 0);

    m_velocityIdx = 0;
    return *this;
}

template <size_t N>
void GridSystemData<N>::Resize(const Vector<size_t, N>& resolution,
                               const Vector<double, N>& gridSpacing,
                               const Vector<double, N>& origin)
{
    m_resolution = resolution;
    m_gridSpacing = gridSpacing;
    m_origin = origin;

    for (auto& data : m_scalarDataList)
    {
        data->Resize(resolution, gridSpacing, origin);
    }
    for (auto& data : m_vectorDataList)
    {
        data->Resize(resolution, gridSpacing, origin);
    }
    for (auto& data : m_advectableScalarDataList)
    {
        data->Resize(resolution, gridSpacing, origin);
    }
    for (auto& data : m_advectableVectorDataList)
    {
        data->Resize(resolution, gridSpacing, origin);
    }
}

template <size_t N>
Vector<size_t, N> GridSystemData<N>::Resolution() const
{
    return m_resolution;
}

template <size_t N>
Vector<double, N> GridSystemData<N>::GridSpacing() const
{
    return m_gridSpacing;
}

template <size_t N>
Vector<double, N> GridSystemData<N>::Origin() const
{
    return m_origin;
}

template <size_t N>
BoundingBox<double, N> GridSystemData<N>::GetBoundingBox() const
{
    return m_velocity->GetBoundingBox();
}

template <size_t N>
size_t GridSystemData<N>::AddScalarData(
    const std::shared_ptr<ScalarGridBuilder<N>>& builder, double initialVal)
{
    const size_t attrIdx = m_scalarDataList.size();
    m_scalarDataList.push_back(
        builder->Build(Resolution(), GridSpacing(), Origin(), initialVal));
    return attrIdx;
}

template <size_t N>
size_t GridSystemData<N>::AddVectorData(
    const std::shared_ptr<VectorGridBuilder<N>>& builder,
    const Vector<double, N>& initialVal)
{
    const size_t attrIdx = m_vectorDataList.size();
    m_vectorDataList.push_back(
        builder->Build(Resolution(), GridSpacing(), Origin(), initialVal));
    return attrIdx;
}

template <size_t N>
size_t GridSystemData<N>::AddAdvectableScalarData(
    const std::shared_ptr<ScalarGridBuilder<N>>& builder, double initialVal)
{
    const size_t attrIdx = m_advectableScalarDataList.size();
    m_advectableScalarDataList.push_back(
        builder->Build(Resolution(), GridSpacing(), Origin(), initialVal));
    return attrIdx;
}

template <size_t N>
size_t GridSystemData<N>::AddAdvectableVectorData(
    const std::shared_ptr<VectorGridBuilder<N>>& builder,
    const Vector<double, N>& initialVal)
{
    const size_t attrIdx = m_advectableVectorDataList.size();
    m_advectableVectorDataList.push_back(
        builder->Build(Resolution(), GridSpacing(), Origin(), initialVal));
    return attrIdx;
}

template <size_t N>
const std::shared_ptr<FaceCenteredGrid<N>>& GridSystemData<N>::Velocity() const
{
    return m_velocity;
}

template <size_t N>
size_t GridSystemData<N>::VelocityIndex() const
{
    return m_velocityIdx;
}

template <size_t N>
const std::shared_ptr<ScalarGrid<N>>& GridSystemData<N>::ScalarDataAt(
    size_t idx) const
{
    return m_scalarDataList[idx];
}

template <size_t N>
const std::shared_ptr<VectorGrid<N>>& GridSystemData<N>::VectorDataAt(
    size_t idx) const
{
    return m_vectorDataList[idx];
}

template <size_t N>
const std::shared_ptr<ScalarGrid<N>>& GridSystemData<N>::AdvectableScalarDataAt(
    size_t idx) const
{
    return m_advectableScalarDataList[idx];
}

template <size_t N>
const std::shared_ptr<VectorGrid<N>>& GridSystemData<N>::AdvectableVectorDataAt(
    size_t idx) const
{
    return m_advectableVectorDataList[idx];
}

template <size_t N>
size_t GridSystemData<N>::NumberOfScalarData() const
{
    return m_scalarDataList.size();
}

template <size_t N>
size_t GridSystemData<N>::NumberOfVectorData() const
{
    return m_vectorDataList.size();
}

template <size_t N>
size_t GridSystemData<N>::NumberOfAdvectableScalarData() const
{
    return m_advectableScalarDataList.size();
}

template <size_t N>
size_t GridSystemData<N>::NumberOfAdvectableVectorData() const
{
    return m_advectableVectorDataList.size();
}

template <size_t N>
void GridSystemData<N>::Serialize(std::vector<uint8_t>* buffer) const
{
    Serialize(*this, buffer);
}

template <size_t N>
void GridSystemData<N>::Deserialize(const std::vector<uint8_t>& buffer)
{
    Deserialize(buffer, *this);
}

template <size_t N>
template <size_t M>
std::enable_if_t<M == 2, void> GridSystemData<N>::Serialize(
    const GridSystemData<2>& grid, std::vector<uint8_t>* buffer)
{
    flatbuffers::FlatBufferBuilder builder(1024);

    fbs::Vector2UZ resolution = CubbyFlowToFlatbuffers(grid.m_resolution);
    fbs::Vector2D gridSpacing = CubbyFlowToFlatbuffers(grid.m_gridSpacing);
    fbs::Vector2D origin = CubbyFlowToFlatbuffers(grid.m_origin);

    std::vector<flatbuffers::Offset<fbs::ScalarGridSerialized2>> scalarDataList;
    std::vector<flatbuffers::Offset<fbs::VectorGridSerialized2>> vectorDataList;
    std::vector<flatbuffers::Offset<fbs::ScalarGridSerialized2>>
        advScalarDataList;
    std::vector<flatbuffers::Offset<fbs::VectorGridSerialized2>>
        advVectorDataList;

    SerializeGrid(&builder, grid.m_scalarDataList,
                  fbs::CreateScalarGridSerialized2, &scalarDataList);
    SerializeGrid(&builder, grid.m_vectorDataList,
                  fbs::CreateVectorGridSerialized2, &vectorDataList);
    SerializeGrid(&builder, grid.m_advectableScalarDataList,
                  fbs::CreateScalarGridSerialized2, &advScalarDataList);
    SerializeGrid(&builder, grid.m_advectableVectorDataList,
                  fbs::CreateVectorGridSerialized2, &advVectorDataList);

    const flatbuffers::Offset<fbs::GridSystemData2> gridSystemData =
        CreateGridSystemData2(builder, &resolution, &gridSpacing, &origin,
                              grid.m_velocityIdx,
                              builder.CreateVector(scalarDataList),
                              builder.CreateVector(vectorDataList),
                              builder.CreateVector(advScalarDataList),
                              builder.CreateVector(advVectorDataList));

    builder.Finish(gridSystemData);

    uint8_t* buf = builder.GetBufferPointer();
    const size_t size = builder.GetSize();

    buffer->resize(size);
    memcpy(buffer->data(), buf, size);
}

template <size_t N>
template <size_t M>
std::enable_if_t<M == 3, void> GridSystemData<N>::Serialize(
    const GridSystemData<3>& grid, std::vector<uint8_t>* buffer)
{
    flatbuffers::FlatBufferBuilder builder(1024);

    fbs::Vector3UZ resolution = CubbyFlowToFlatbuffers(grid.m_resolution);
    fbs::Vector3D gridSpacing = CubbyFlowToFlatbuffers(grid.m_gridSpacing);
    fbs::Vector3D origin = CubbyFlowToFlatbuffers(grid.m_origin);

    std::vector<flatbuffers::Offset<fbs::ScalarGridSerialized3>> scalarDataList;
    std::vector<flatbuffers::Offset<fbs::VectorGridSerialized3>> vectorDataList;
    std::vector<flatbuffers::Offset<fbs::ScalarGridSerialized3>>
        advScalarDataList;
    std::vector<flatbuffers::Offset<fbs::VectorGridSerialized3>>
        advVectorDataList;

    SerializeGrid(&builder, grid.m_scalarDataList,
                  fbs::CreateScalarGridSerialized3, &scalarDataList);
    SerializeGrid(&builder, grid.m_vectorDataList,
                  fbs::CreateVectorGridSerialized3, &vectorDataList);
    SerializeGrid(&builder, grid.m_advectableScalarDataList,
                  fbs::CreateScalarGridSerialized3, &advScalarDataList);
    SerializeGrid(&builder, grid.m_advectableVectorDataList,
                  fbs::CreateVectorGridSerialized3, &advVectorDataList);

    flatbuffers::Offset<fbs::GridSystemData3> gridSystemData =
        CreateGridSystemData3(builder, &resolution, &gridSpacing, &origin,
                              grid.m_velocityIdx,
                              builder.CreateVector(scalarDataList),
                              builder.CreateVector(vectorDataList),
                              builder.CreateVector(advScalarDataList),
                              builder.CreateVector(advVectorDataList));

    builder.Finish(gridSystemData);

    uint8_t* buf = builder.GetBufferPointer();
    const size_t size = builder.GetSize();

    buffer->resize(size);
    memcpy(buffer->data(), buf, size);
}

template <size_t N>
template <size_t M>
std::enable_if_t<M == 2, void> GridSystemData<N>::Deserialize(
    const std::vector<uint8_t>& buffer, GridSystemData<2>& grid)
{
    const fbs::GridSystemData2* gridSystemData =
        fbs::GetGridSystemData2(buffer.data());

    grid.Resize(FlatbuffersToCubbyFlow(*gridSystemData->resolution()),
                FlatbuffersToCubbyFlow(*gridSystemData->gridSpacing()),
                FlatbuffersToCubbyFlow(*gridSystemData->origin()));

    grid.m_scalarDataList.clear();
    grid.m_vectorDataList.clear();
    grid.m_advectableScalarDataList.clear();
    grid.m_advectableVectorDataList.clear();

    DeserializeGrid(gridSystemData->scalarData(), Factory::BuildScalarGrid2,
                    &grid.m_scalarDataList);
    DeserializeGrid(gridSystemData->vectorData(), Factory::BuildVectorGrid2,
                    &grid.m_vectorDataList);
    DeserializeGrid(gridSystemData->advectableScalarData(),
                    Factory::BuildScalarGrid2,
                    &grid.m_advectableScalarDataList);
    DeserializeGrid(gridSystemData->advectableVectorData(),
                    Factory::BuildVectorGrid2,
                    &grid.m_advectableVectorDataList);

    grid.m_velocityIdx = gridSystemData->velocityIdx();
    grid.m_velocity = std::dynamic_pointer_cast<FaceCenteredGrid2>(
        grid.m_advectableVectorDataList[grid.m_velocityIdx]);
}

template <size_t N>
template <size_t M>
std::enable_if_t<M == 3, void> GridSystemData<N>::Deserialize(
    const std::vector<uint8_t>& buffer, GridSystemData<3>& grid)
{
    const fbs::GridSystemData3* gridSystemData =
        fbs::GetGridSystemData3(buffer.data());

    grid.Resize(FlatbuffersToCubbyFlow(*gridSystemData->resolution()),
                FlatbuffersToCubbyFlow(*gridSystemData->gridSpacing()),
                FlatbuffersToCubbyFlow(*gridSystemData->origin()));

    grid.m_scalarDataList.clear();
    grid.m_vectorDataList.clear();
    grid.m_advectableScalarDataList.clear();
    grid.m_advectableVectorDataList.clear();

    DeserializeGrid(gridSystemData->scalarData(), Factory::BuildScalarGrid3,
                    &grid.m_scalarDataList);
    DeserializeGrid(gridSystemData->vectorData(), Factory::BuildVectorGrid3,
                    &grid.m_vectorDataList);
    DeserializeGrid(gridSystemData->advectableScalarData(),
                    Factory::BuildScalarGrid3,
                    &grid.m_advectableScalarDataList);
    DeserializeGrid(gridSystemData->advectableVectorData(),
                    Factory::BuildVectorGrid3,
                    &grid.m_advectableVectorDataList);

    grid.m_velocityIdx = gridSystemData->velocityIdx();
    grid.m_velocity = std::dynamic_pointer_cast<FaceCenteredGrid3>(
        grid.m_advectableVectorDataList[grid.m_velocityIdx]);
}

template class GridSystemData<2>;

template class GridSystemData<3>;
}  // namespace CubbyFlow

// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Particle/ParticleSystemData.hpp>
#include <Core/Searcher/PointNeighborSearcher.hpp>
#include <Core/Searcher/PointParallelHashGridSearcher.hpp>
#include <Core/Utils/Factory.hpp>
#include <Core/Utils/FlatbuffersHelper.hpp>
#include <Core/Utils/Logging.hpp>
#include <Core/Utils/Parallel.hpp>
#include <Core/Utils/Timer.hpp>

#include <Flatbuffers/generated/ParticleSystemData2_generated.h>
#include <Flatbuffers/generated/ParticleSystemData3_generated.h>

namespace CubbyFlow
{
static const size_t DEFAULT_HASH_GRID_RESOLUTION = 64;

template <size_t N>
struct GetFlatbuffersParticleSystemData
{
    // Do nothing
};

template <>
struct GetFlatbuffersParticleSystemData<2>
{
    using Offset = flatbuffers::Offset<fbs::ParticleSystemData2>;

    static const fbs::ParticleSystemData2* GetParticleSystemData(
        const uint8_t* data)
    {
        return fbs::GetParticleSystemData2(data);
    }
};

template <>
struct GetFlatbuffersParticleSystemData<3>
{
    using Offset = flatbuffers::Offset<fbs::ParticleSystemData3>;

    static const fbs::ParticleSystemData3* GetParticleSystemData(
        const uint8_t* data)
    {
        return fbs::GetParticleSystemData3(data);
    }
};

template <size_t N>
ParticleSystemData<N>::ParticleSystemData() : ParticleSystemData{ 0 }
{
    // Do nothing
}

template <size_t N>
ParticleSystemData<N>::ParticleSystemData(size_t numberOfParticles)
{
    m_positionIdx = AddVectorData();
    m_velocityIdx = AddVectorData();
    m_forceIdx = AddVectorData();

    // Use PointParallelHashGridSearcher<N> by default
    m_neighborSearcher = std::make_shared<PointParallelHashGridSearcher<N>>(
        Vector<size_t, N>::MakeConstant(DEFAULT_HASH_GRID_RESOLUTION),
        2.0 * m_radius);

    Resize(numberOfParticles);
}

template <size_t N>
ParticleSystemData<N>::ParticleSystemData(const ParticleSystemData& other)
    : m_radius(other.m_radius),
      m_mass(other.m_mass),
      m_numberOfParticles(other.m_numberOfParticles),
      m_positionIdx(other.m_positionIdx),
      m_velocityIdx(other.m_velocityIdx),
      m_forceIdx(other.m_forceIdx),
      m_neighborSearcher(other.m_neighborSearcher->Clone()),
      m_neighborLists(other.m_neighborLists)
{
    for (auto& data : other.m_scalarDataList)
    {
        m_scalarDataList.Append(data);
    }

    for (auto& data : other.m_vectorDataList)
    {
        m_vectorDataList.Append(data);
    }
}

template <size_t N>
ParticleSystemData<N>::ParticleSystemData(ParticleSystemData&& other) noexcept
    : m_radius(std::exchange(other.m_radius, 1e-3)),
      m_mass(std::exchange(other.m_mass, 1e-3)),
      m_numberOfParticles(std::exchange(other.m_numberOfParticles, 0)),
      m_positionIdx(std::exchange(other.m_positionIdx, 0)),
      m_velocityIdx(std::exchange(other.m_velocityIdx, 0)),
      m_forceIdx(std::exchange(other.m_forceIdx, 0)),
      m_scalarDataList(std::move(other.m_scalarDataList)),
      m_vectorDataList(std::move(other.m_vectorDataList)),
      m_neighborSearcher(std::move(other.m_neighborSearcher)),
      m_neighborLists(std::move(other.m_neighborLists))
{
    // Do nothing
}

template <size_t N>
ParticleSystemData<N>& ParticleSystemData<N>::operator=(
    const ParticleSystemData& other)
{
    if (this != &other)
    {
        *this = ParticleSystemData(other);
    }

    return *this;
}

template <size_t N>
ParticleSystemData<N>& ParticleSystemData<N>::operator=(
    ParticleSystemData&& other) noexcept
{
    if (this == &other)
    {
        return *this;
    }

    m_radius = std::exchange(other.m_radius, 1e-3);
    m_mass = std::exchange(other.m_mass, 1e-3);
    m_numberOfParticles = std::exchange(other.m_numberOfParticles, 0);
    m_positionIdx = std::exchange(other.m_positionIdx, 0);
    m_velocityIdx = std::exchange(other.m_velocityIdx, 0);
    m_forceIdx = std::exchange(other.m_forceIdx, 0);
    m_scalarDataList = std::move(other.m_scalarDataList);
    m_vectorDataList = std::move(other.m_vectorDataList);
    m_neighborSearcher = std::move(other.m_neighborSearcher);
    m_neighborLists = std::move(other.m_neighborLists);
    return *this;
}

template <size_t N>
void ParticleSystemData<N>::Resize(size_t newNumberOfParticles)
{
    m_numberOfParticles = newNumberOfParticles;

    for (auto& attr : m_scalarDataList)
    {
        attr.Resize(newNumberOfParticles, 0.0);
    }

    for (auto& attr : m_vectorDataList)
    {
        attr.Resize(newNumberOfParticles, Vector<double, N>{});
    }
}

template <size_t N>
size_t ParticleSystemData<N>::NumberOfParticles() const
{
    return m_numberOfParticles;
}

template <size_t N>
size_t ParticleSystemData<N>::AddScalarData(double initialVal)
{
    const size_t attrIdx = m_scalarDataList.Length();
    m_scalarDataList.Append(ScalarData(NumberOfParticles(), initialVal));
    return attrIdx;
}

template <size_t N>
size_t ParticleSystemData<N>::AddVectorData(const Vector<double, N>& initialVal)
{
    const size_t attrIdx = m_vectorDataList.Length();
    m_vectorDataList.Append(VectorData(NumberOfParticles(), initialVal));
    return attrIdx;
}

template <size_t N>
double ParticleSystemData<N>::Radius() const
{
    return m_radius;
}

template <size_t N>
void ParticleSystemData<N>::SetRadius(double newRadius)
{
    m_radius = std::max(newRadius, 0.0);
}

template <size_t N>
double ParticleSystemData<N>::Mass() const
{
    return m_mass;
}

template <size_t N>
void ParticleSystemData<N>::SetMass(double newMass)
{
    m_mass = std::max(newMass, 0.0);
}

template <size_t N>
ConstArrayView1<Vector<double, N>> ParticleSystemData<N>::Positions() const
{
    return VectorDataAt(m_positionIdx);
}

template <size_t N>
ArrayView1<Vector<double, N>> ParticleSystemData<N>::Positions()
{
    return VectorDataAt(m_positionIdx);
}

template <size_t N>
ConstArrayView1<Vector<double, N>> ParticleSystemData<N>::Velocities() const
{
    return VectorDataAt(m_velocityIdx);
}

template <size_t N>
ArrayView1<Vector<double, N>> ParticleSystemData<N>::Velocities()
{
    return VectorDataAt(m_velocityIdx);
}

template <size_t N>
ConstArrayView1<Vector<double, N>> ParticleSystemData<N>::Forces() const
{
    return VectorDataAt(m_forceIdx);
}

template <size_t N>
ArrayView1<Vector<double, N>> ParticleSystemData<N>::Forces()
{
    return VectorDataAt(m_forceIdx);
}

template <size_t N>
ConstArrayView1<double> ParticleSystemData<N>::ScalarDataAt(size_t idx) const
{
    return ConstArrayView1<double>(m_scalarDataList[idx]);
}

template <size_t N>
ArrayView1<double> ParticleSystemData<N>::ScalarDataAt(size_t idx)
{
    return ArrayView1<double>(m_scalarDataList[idx]);
}

template <size_t N>
ConstArrayView1<Vector<double, N>> ParticleSystemData<N>::VectorDataAt(
    size_t idx) const
{
    return ConstArrayView1<Vector<double, N>>(m_vectorDataList[idx]);
}

template <size_t N>
ArrayView1<Vector<double, N>> ParticleSystemData<N>::VectorDataAt(size_t idx)
{
    return ArrayView1<Vector<double, N>>(m_vectorDataList[idx]);
}

template <size_t N>
void ParticleSystemData<N>::AddParticle(const Vector<double, N>& newPosition,
                                        const Vector<double, N>& newVelocity,
                                        const Vector<double, N>& newForce)
{
    Array1<Vector<double, N>> newPositions = { newPosition };
    Array1<Vector<double, N>> newVelocities = { newVelocity };
    Array1<Vector<double, N>> newForces = { newForce };

    AddParticles(newPositions, newVelocities, newForces);
}

template <size_t N>
void ParticleSystemData<N>::AddParticles(
    const ConstArrayView1<Vector<double, N>>& newPositions,
    const ConstArrayView1<Vector<double, N>>& newVelocities,
    const ConstArrayView1<Vector<double, N>>& newForces)
{
    if (newVelocities.Length() > 0 &&
        newVelocities.Length() != newPositions.Length())
    {
        throw std::invalid_argument{
            "newVelocities.Length() > 0 && newVelocities.Length() != "
            "newPositions.Length()"
        };
    }

    if (newForces.Length() > 0 && newForces.Length() != newPositions.Length())
    {
        throw std::invalid_argument{
            "newForces.Length() > 0 && newForces.Length() != "
            "newPositions.Length()"
        };
    }

    size_t oldNumberOfParticles = NumberOfParticles();
    const size_t newNumberOfParticles =
        oldNumberOfParticles + newPositions.Length();

    Resize(newNumberOfParticles);

    ArrayView1<Matrix<double, N, 1>> pos = Positions();
    ArrayView1<Matrix<double, N, 1>> vel = Velocities();
    ArrayView1<Matrix<double, N, 1>> frc = Forces();

    ParallelFor(ZERO_SIZE, newPositions.Length(),
                [&pos, &oldNumberOfParticles, &newPositions](size_t i) {
                    pos[i + oldNumberOfParticles] = newPositions[i];
                });

    if (newVelocities.Length() > 0)
    {
        ParallelFor(ZERO_SIZE, newPositions.Length(),
                    [&vel, &oldNumberOfParticles, &newVelocities](size_t i) {
                        vel[i + oldNumberOfParticles] = newVelocities[i];
                    });
    }

    if (newForces.Length() > 0)
    {
        ParallelFor(ZERO_SIZE, newPositions.Length(),
                    [&frc, &oldNumberOfParticles, &newForces](size_t i) {
                        frc[i + oldNumberOfParticles] = newForces[i];
                    });
    }
}

template <size_t N>
const std::shared_ptr<PointNeighborSearcher<N>>&
ParticleSystemData<N>::NeighborSearcher() const
{
    return m_neighborSearcher;
}

template <size_t N>
void ParticleSystemData<N>::SetNeighborSearcher(
    const std::shared_ptr<PointNeighborSearcher<N>>& newNeighborSearcher)
{
    m_neighborSearcher = newNeighborSearcher;
}

template <size_t N>
const Array1<Array1<size_t>>& ParticleSystemData<N>::NeighborLists() const
{
    return m_neighborLists;
}

template <size_t N>
void ParticleSystemData<N>::BuildNeighborSearcher(double maxSearchRadius)
{
    const Timer timer;

    assert(m_neighborSearcher != nullptr);

    m_neighborSearcher->Build(Positions(), maxSearchRadius);

    CUBBYFLOW_INFO << "Building neighbor searcher took: "
                   << timer.DurationInSeconds() << " seconds";
}

template <size_t N>
void ParticleSystemData<N>::BuildNeighborLists(double maxSearchRadius)
{
    const Timer timer;

    m_neighborLists.Resize(NumberOfParticles());

    ArrayView1<Matrix<double, N, 1>> points = Positions();

    for (size_t i = 0; i < NumberOfParticles(); ++i)
    {
        Vector<double, N> origin = points[i];
        m_neighborLists[i].Clear();

        m_neighborSearcher->ForEachNearbyPoint(
            origin, maxSearchRadius,
            [&i, this](size_t j, const Vector<double, N>&) {
                if (i != j)
                {
                    m_neighborLists[i].Append(j);
                }
            });
    }

    CUBBYFLOW_INFO << "Building neighbor list took: "
                   << timer.DurationInSeconds() << " seconds";
}

template <size_t N>
void ParticleSystemData<N>::Serialize(std::vector<uint8_t>* buffer) const
{
    flatbuffers::FlatBufferBuilder builder(1024);
    typename GetFlatbuffersParticleSystemData<N>::Offset fbsParticleSystemData;

    Serialize(*this, &builder, &fbsParticleSystemData);

    builder.Finish(fbsParticleSystemData);

    uint8_t* buf = builder.GetBufferPointer();
    const size_t size = builder.GetSize();

    buffer->resize(size);
    memcpy(buffer->data(), buf, size);
}

template <size_t N>
void ParticleSystemData<N>::Deserialize(const std::vector<uint8_t>& buffer)
{
    auto fbsParticleSystemData =
        GetFlatbuffersParticleSystemData<N>::GetParticleSystemData(
            buffer.data());
    Deserialize(fbsParticleSystemData, *this);
    Resize(NumberOfParticles());
}

template <size_t N>
void ParticleSystemData<N>::Set(const ParticleSystemData& other)
{
    *this = other;
    Resize(NumberOfParticles());
}

template <size_t N>
template <size_t M>
std::enable_if_t<M == 2, void> ParticleSystemData<N>::Serialize(
    const ParticleSystemData<2>& particles,
    flatbuffers::FlatBufferBuilder* builder,
    flatbuffers::Offset<fbs::ParticleSystemData2>* fbsParticleSystemData)
{
    // Copy data
    std::vector<flatbuffers::Offset<fbs::ScalarParticleData2>> scalarDataList;
    for (const auto& scalarData : particles.m_scalarDataList)
    {
        flatbuffers::Offset<fbs::ScalarParticleData2> fbsScalarData =
            fbs::CreateScalarParticleData2(
                *builder,
                builder->CreateVector(scalarData.data(), scalarData.Length()));
        scalarDataList.push_back(fbsScalarData);
    }
    const flatbuffers::Offset<
        flatbuffers::Vector<flatbuffers::Offset<fbs::ScalarParticleData2>>>
        fbsScalarDataList = builder->CreateVector(scalarDataList);

    std::vector<flatbuffers::Offset<fbs::VectorParticleData2>> vectorDataList;
    for (const auto& vectorData : particles.m_vectorDataList)
    {
        std::vector<fbs::Vector2D> newVectorData;
        for (const auto& v : vectorData)
        {
            newVectorData.push_back(CubbyFlowToFlatbuffers(v));
        }

        flatbuffers::Offset<fbs::VectorParticleData2> fbsVectorData =
            fbs::CreateVectorParticleData2(
                *builder, builder->CreateVectorOfStructs(newVectorData.data(),
                                                         newVectorData.size()));
        vectorDataList.push_back(fbsVectorData);
    }
    const flatbuffers::Offset<
        flatbuffers::Vector<flatbuffers::Offset<fbs::VectorParticleData2>>>
        fbsVectorDataList = builder->CreateVector(vectorDataList);

    // Copy neighbor searcher
    const flatbuffers::Offset<flatbuffers::String> neighborSearcherType =
        builder->CreateString(particles.m_neighborSearcher->TypeName());
    std::vector<uint8_t> neighborSearcherSerialized;
    particles.m_neighborSearcher->Serialize(&neighborSearcherSerialized);
    const flatbuffers::Offset<fbs::PointNeighborSearcherSerialized2>
        fbsNeighborSearcher = fbs::CreatePointNeighborSearcherSerialized2(
            *builder, neighborSearcherType,
            builder->CreateVector(neighborSearcherSerialized.data(),
                                  neighborSearcherSerialized.size()));

    // Copy neighbor lists
    std::vector<flatbuffers::Offset<fbs::ParticleNeighborList2>> neighborLists;
    for (const auto& neighbors : particles.m_neighborLists)
    {
        std::vector<uint64_t> neighbors64(neighbors.begin(), neighbors.end());
        flatbuffers::Offset<fbs::ParticleNeighborList2> fbsNeighborList =
            fbs::CreateParticleNeighborList2(
                *builder,
                builder->CreateVector(neighbors64.data(), neighbors64.size()));
        neighborLists.push_back(fbsNeighborList);
    }

    const flatbuffers::Offset<
        flatbuffers::Vector<flatbuffers::Offset<fbs::ParticleNeighborList2>>>
        fbsNeighborLists = builder->CreateVector(neighborLists);

    // Copy the searcher
    *fbsParticleSystemData = fbs::CreateParticleSystemData2(
        *builder, particles.m_radius, particles.m_mass, particles.m_positionIdx,
        particles.m_velocityIdx, particles.m_forceIdx, fbsScalarDataList,
        fbsVectorDataList, fbsNeighborSearcher, fbsNeighborLists);
}

template <size_t N>
template <size_t M>
std::enable_if_t<M == 3, void> ParticleSystemData<N>::Serialize(
    const ParticleSystemData<3>& particles,
    flatbuffers::FlatBufferBuilder* builder,
    flatbuffers::Offset<fbs::ParticleSystemData3>* fbsParticleSystemData)
{
    // Copy data
    std::vector<flatbuffers::Offset<fbs::ScalarParticleData3>> scalarDataList;
    for (const auto& scalarData : particles.m_scalarDataList)
    {
        flatbuffers::Offset<fbs::ScalarParticleData3> fbsScalarData =
            fbs::CreateScalarParticleData3(
                *builder,
                builder->CreateVector(scalarData.data(), scalarData.Length()));
        scalarDataList.push_back(fbsScalarData);
    }
    const flatbuffers::Offset<
        flatbuffers::Vector<flatbuffers::Offset<fbs::ScalarParticleData3>>>
        fbsScalarDataList = builder->CreateVector(scalarDataList);

    std::vector<flatbuffers::Offset<fbs::VectorParticleData3>> vectorDataList;
    for (const auto& vectorData : particles.m_vectorDataList)
    {
        std::vector<fbs::Vector3D> newVectorData;
        for (const auto& v : vectorData)
        {
            newVectorData.push_back(CubbyFlowToFlatbuffers(v));
        }

        flatbuffers::Offset<fbs::VectorParticleData3> fbsVectorData =
            fbs::CreateVectorParticleData3(
                *builder, builder->CreateVectorOfStructs(newVectorData.data(),
                                                         newVectorData.size()));
        vectorDataList.push_back(fbsVectorData);
    }
    const flatbuffers::Offset<
        flatbuffers::Vector<flatbuffers::Offset<fbs::VectorParticleData3>>>
        fbsVectorDataList = builder->CreateVector(vectorDataList);

    // Copy neighbor searcher
    const flatbuffers::Offset<flatbuffers::String> neighborSearcherType =
        builder->CreateString(particles.m_neighborSearcher->TypeName());
    std::vector<uint8_t> neighborSearcherSerialized;
    particles.m_neighborSearcher->Serialize(&neighborSearcherSerialized);
    const flatbuffers::Offset<fbs::PointNeighborSearcherSerialized3>
        fbsNeighborSearcher = fbs::CreatePointNeighborSearcherSerialized3(
            *builder, neighborSearcherType,
            builder->CreateVector(neighborSearcherSerialized.data(),
                                  neighborSearcherSerialized.size()));

    // Copy neighbor lists
    std::vector<flatbuffers::Offset<fbs::ParticleNeighborList3>> neighborLists;
    for (const auto& neighbors : particles.m_neighborLists)
    {
        std::vector<uint64_t> neighbors64(neighbors.begin(), neighbors.end());
        flatbuffers::Offset<fbs::ParticleNeighborList3> fbsNeighborList =
            fbs::CreateParticleNeighborList3(
                *builder,
                builder->CreateVector(neighbors64.data(), neighbors64.size()));
        neighborLists.push_back(fbsNeighborList);
    }

    const flatbuffers::Offset<
        flatbuffers::Vector<flatbuffers::Offset<fbs::ParticleNeighborList3>>>
        fbsNeighborLists = builder->CreateVector(neighborLists);

    // Copy the searcher
    *fbsParticleSystemData = fbs::CreateParticleSystemData3(
        *builder, particles.m_radius, particles.m_mass, particles.m_positionIdx,
        particles.m_velocityIdx, particles.m_forceIdx, fbsScalarDataList,
        fbsVectorDataList, fbsNeighborSearcher, fbsNeighborLists);
}

template <size_t N>
template <size_t M>
std::enable_if_t<M == 2, void> ParticleSystemData<N>::Deserialize(
    const fbs::ParticleSystemData2* fbsParticleSystemData,
    ParticleSystemData<2>& particles)
{
    particles.m_scalarDataList.Clear();
    particles.m_vectorDataList.Clear();

    // Copy scalars
    particles.m_radius = fbsParticleSystemData->radius();
    particles.m_mass = fbsParticleSystemData->mass();
    particles.m_positionIdx = fbsParticleSystemData->positionIdx();
    particles.m_velocityIdx = fbsParticleSystemData->velocityIdx();
    particles.m_forceIdx = fbsParticleSystemData->forceIdx();

    // Copy data
    const flatbuffers::Vector<flatbuffers::Offset<fbs::ScalarParticleData2>>*
        fbsScalarDataList = fbsParticleSystemData->scalarDataList();
    for (const auto& fbsScalarData : (*fbsScalarDataList))
    {
        const flatbuffers::Vector<double>* data = fbsScalarData->data();
        particles.m_scalarDataList.Append(ScalarData(data->size()));

        auto& newData = *(particles.m_scalarDataList.rbegin());
        for (uint32_t i = 0; i < data->size(); ++i)
        {
            newData[i] = data->Get(i);
        }
    }

    const flatbuffers::Vector<flatbuffers::Offset<fbs::VectorParticleData2>>*
        fbsVectorDataList = fbsParticleSystemData->vectorDataList();
    for (const auto& fbsVectorData : (*fbsVectorDataList))
    {
        const flatbuffers::Vector<const fbs::Vector2D*>* data =
            fbsVectorData->data();
        particles.m_vectorDataList.Append(VectorData(data->size()));

        auto& newData = *(particles.m_vectorDataList.rbegin());
        for (uint32_t i = 0; i < data->size(); ++i)
        {
            newData[i] = FlatbuffersToCubbyFlow(*data->Get(i));
        }
    }

    particles.m_numberOfParticles = particles.m_vectorDataList[0].Length();

    // Copy neighbor searcher
    const fbs::PointNeighborSearcherSerialized2* fbsNeighborSearcher =
        fbsParticleSystemData->neighborSearcher();
    particles.m_neighborSearcher = Factory::BuildPointNeighborSearcher2(
        fbsNeighborSearcher->type()->c_str());
    const std::vector<uint8_t> neighborSearcherSerialized(
        fbsNeighborSearcher->data()->begin(),
        fbsNeighborSearcher->data()->end());
    particles.m_neighborSearcher->Deserialize(neighborSearcherSerialized);

    // Copy neighbor list
    const flatbuffers::Vector<flatbuffers::Offset<fbs::ParticleNeighborList2>>*
        fbsNeighborLists = fbsParticleSystemData->neighborLists();
    particles.m_neighborLists.Resize(fbsNeighborLists->size());
    for (uint32_t i = 0; i < fbsNeighborLists->size(); ++i)
    {
        const flatbuffers::Vector<
            flatbuffers::Offset<fbs::ParticleNeighborList2>>::return_type
            fbsNeighborList = fbsNeighborLists->Get(i);
        particles.m_neighborLists[i].Resize(fbsNeighborList->data()->size());
        std::transform(fbsNeighborList->data()->begin(),
                       fbsNeighborList->data()->end(),
                       particles.m_neighborLists[i].begin(),
                       [](uint64_t val) { return val; });
    }
}

template <size_t N>
template <size_t M>
std::enable_if_t<M == 3, void> ParticleSystemData<N>::Deserialize(
    const fbs::ParticleSystemData3* fbsParticleSystemData,
    ParticleSystemData<3>& particles)
{
    particles.m_scalarDataList.Clear();
    particles.m_vectorDataList.Clear();

    // Copy scalars
    particles.m_radius = fbsParticleSystemData->radius();
    particles.m_mass = fbsParticleSystemData->mass();
    particles.m_positionIdx = fbsParticleSystemData->positionIdx();
    particles.m_velocityIdx = fbsParticleSystemData->velocityIdx();
    particles.m_forceIdx = fbsParticleSystemData->forceIdx();

    // Copy data
    const flatbuffers::Vector<flatbuffers::Offset<fbs::ScalarParticleData3>>*
        fbsScalarDataList = fbsParticleSystemData->scalarDataList();
    for (const auto& fbsScalarData : (*fbsScalarDataList))
    {
        const flatbuffers::Vector<double>* data = fbsScalarData->data();
        particles.m_scalarDataList.Append(ScalarData(data->size()));

        auto& newData = *(particles.m_scalarDataList.rbegin());
        for (uint32_t i = 0; i < data->size(); ++i)
        {
            newData[i] = data->Get(i);
        }
    }

    const flatbuffers::Vector<flatbuffers::Offset<fbs::VectorParticleData3>>*
        fbsVectorDataList = fbsParticleSystemData->vectorDataList();
    for (const auto& fbsVectorData : (*fbsVectorDataList))
    {
        const flatbuffers::Vector<const fbs::Vector3D*>* data =
            fbsVectorData->data();
        particles.m_vectorDataList.Append(VectorData(data->size()));

        auto& newData = *(particles.m_vectorDataList.rbegin());
        for (uint32_t i = 0; i < data->size(); ++i)
        {
            newData[i] = FlatbuffersToCubbyFlow(*data->Get(i));
        }
    }

    particles.m_numberOfParticles = particles.m_vectorDataList[0].Length();

    // Copy neighbor searcher
    const fbs::PointNeighborSearcherSerialized3* fbsNeighborSearcher =
        fbsParticleSystemData->neighborSearcher();
    particles.m_neighborSearcher = Factory::BuildPointNeighborSearcher3(
        fbsNeighborSearcher->type()->c_str());
    const std::vector<uint8_t> neighborSearcherSerialized(
        fbsNeighborSearcher->data()->begin(),
        fbsNeighborSearcher->data()->end());
    particles.m_neighborSearcher->Deserialize(neighborSearcherSerialized);

    // Copy neighbor list
    const flatbuffers::Vector<flatbuffers::Offset<fbs::ParticleNeighborList3>>*
        fbsNeighborLists = fbsParticleSystemData->neighborLists();
    particles.m_neighborLists.Resize(fbsNeighborLists->size());
    for (uint32_t i = 0; i < fbsNeighborLists->size(); ++i)
    {
        const flatbuffers::Vector<
            flatbuffers::Offset<fbs::ParticleNeighborList3>>::return_type
            fbsNeighborList = fbsNeighborLists->Get(i);
        particles.m_neighborLists[i].Resize(fbsNeighborList->data()->size());
        std::transform(fbsNeighborList->data()->begin(),
                       fbsNeighborList->data()->end(),
                       particles.m_neighborLists[i].begin(),
                       [](uint64_t val) { return val; });
    }
}

template class ParticleSystemData<2>;

template class ParticleSystemData<3>;
}  // namespace CubbyFlow

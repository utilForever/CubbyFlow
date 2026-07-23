// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Searcher/PointParallelHashGridSearcher.hpp>
#include <Core/Utils/FlatbuffersHelper.hpp>
#include <Core/Utils/Logging.hpp>
#include <Core/Utils/Parallel.hpp>
#include <Core/Utils/PointHashGridUtils.hpp>

#include <Flatbuffers/generated/PointParallelHashGridSearcher2_generated.h>
#include <Flatbuffers/generated/PointParallelHashGridSearcher3_generated.h>

#include <flatbuffers/flatbuffers.h>

namespace CubbyFlow
{
template <size_t N>
PointParallelHashGridSearcher<N>::PointParallelHashGridSearcher(
    const Vector<size_t, N>& resolution, double gridSpacing)
    : m_gridSpacing{ gridSpacing }
{
    m_resolution =
        Max(resolution.template CastTo<ssize_t>(),
            Vector<ssize_t, N>::MakeConstant(static_cast<ssize_t>(1)));
    const auto tableSize =
        static_cast<size_t>(Product(m_resolution, static_cast<ssize_t>(1)));
    m_startIndexTable.Resize(tableSize, std::numeric_limits<size_t>::max());
    m_endIndexTable.Resize(tableSize, std::numeric_limits<size_t>::max());
}

template <size_t N>
PointParallelHashGridSearcher<N>::PointParallelHashGridSearcher(
    const PointParallelHashGridSearcher& other)
    : m_gridSpacing(other.m_gridSpacing),
      m_resolution(other.m_resolution),
      m_points(other.m_points),
      m_keys(other.m_keys),
      m_startIndexTable(other.m_startIndexTable),
      m_endIndexTable(other.m_endIndexTable),
      m_sortedIndices(other.m_sortedIndices)
{
    // Do nothing
}

template <size_t N>
PointParallelHashGridSearcher<N>::PointParallelHashGridSearcher(
    PointParallelHashGridSearcher&& other) noexcept
    : m_gridSpacing(std::exchange(other.m_gridSpacing, 1.0)),
      m_resolution(std::move(other.m_resolution)),
      m_points(std::move(other.m_points)),
      m_keys(std::move(other.m_keys)),
      m_startIndexTable(std::move(other.m_startIndexTable)),
      m_endIndexTable(std::move(other.m_endIndexTable)),
      m_sortedIndices(std::move(other.m_sortedIndices))
{
    // Do nothing
}

template <size_t N>
PointParallelHashGridSearcher<N>& PointParallelHashGridSearcher<N>::operator=(
    const PointParallelHashGridSearcher& other)
{
    m_gridSpacing = other.m_gridSpacing;
    m_resolution = other.m_resolution;
    m_points = other.m_points;
    m_keys = other.m_keys;
    m_startIndexTable = other.m_startIndexTable;
    m_endIndexTable = other.m_endIndexTable;
    m_sortedIndices = other.m_sortedIndices;
    return *this;
}

template <size_t N>
PointParallelHashGridSearcher<N>& PointParallelHashGridSearcher<N>::operator=(
    PointParallelHashGridSearcher&& other) noexcept
{
    m_gridSpacing = std::exchange(other.m_gridSpacing, 1.0);
    m_resolution = std::move(other.m_resolution);
    m_points = std::move(other.m_points);
    m_keys = std::move(other.m_keys);
    m_startIndexTable = std::move(other.m_startIndexTable);
    m_endIndexTable = std::move(other.m_endIndexTable);
    m_sortedIndices = std::move(other.m_sortedIndices);
    return *this;
}

template <size_t N>
void PointParallelHashGridSearcher<N>::Build(
    const ConstArrayView1<Vector<double, N>>& points)
{
    m_points.Clear();
    m_keys.Clear();
    m_startIndexTable.Clear();
    m_endIndexTable.Clear();
    m_sortedIndices.Clear();

    // Allocate memory chunks
    size_t numberOfPoints = points.Length();
    Array1<size_t> tempKeys(numberOfPoints);
    const auto tableSize =
        static_cast<size_t>(Product(m_resolution, static_cast<ssize_t>(1)));

    m_startIndexTable.Resize(tableSize);
    m_endIndexTable.Resize(tableSize);

    ParallelFill(m_startIndexTable.begin(), m_startIndexTable.end(),
                 std::numeric_limits<size_t>::max());
    ParallelFill(m_endIndexTable.begin(), m_endIndexTable.end(),
                 std::numeric_limits<size_t>::max());

    m_keys.Resize(numberOfPoints);
    m_sortedIndices.Resize(numberOfPoints);
    m_points.Resize(numberOfPoints);

    if (numberOfPoints == 0)
    {
        return;
    }

    // Initialize indices array and generate hash key for each point
    ParallelFor(static_cast<size_t>(0), numberOfPoints, [&](size_t i) {
        m_sortedIndices[i] = i;
        m_points[i] = points[i];
        tempKeys[i] = PointHashGridUtils<N>::GetHashKeyFromPosition(
            points[i], m_gridSpacing, m_resolution);
    });

    // Sort indices based on hash key
    ParallelSort(m_sortedIndices.begin(), m_sortedIndices.end(),
                 [&tempKeys](size_t indexA, size_t indexB) {
                     return tempKeys[indexA] < tempKeys[indexB];
                 });

    // Re-order point and key arrays
    ParallelFor(static_cast<size_t>(0), numberOfPoints, [&](size_t i) {
        m_points[i] = points[m_sortedIndices[i]];
        m_keys[i] = tempKeys[m_sortedIndices[i]];
    });

    // Now m_points and m_keys are sorted by points' hash key values.
    // Let's fill in start/end index table with m_keys.

    // Assume that m_keys array looks like:
    // [5|8|8|10|10|10]
    // Then m_startIndexTable and m_endIndexTable should be like:
    // [.....|0|...|1|..|3|..]
    // [.....|1|...|3|..|6|..]
    //       ^5    ^8   ^10
    // So that m_endIndexTable[i] - m_startIndexTable[i] is the number points
    // in i-th table bucket.

    m_startIndexTable[m_keys[0]] = 0;
    m_endIndexTable[m_keys[numberOfPoints - 1]] = numberOfPoints;

    ParallelFor(static_cast<size_t>(1), numberOfPoints, [&](size_t i) {
        if (m_keys[i] > m_keys[i - 1])
        {
            m_startIndexTable[m_keys[i]] = i;
            m_endIndexTable[m_keys[i - 1]] = i;
        }
    });

    size_t sumNumberOfPointsPerBucket = 0;
    size_t maxNumberOfPointsPerBucket = 0;
    size_t numberOfNonEmptyBucket = 0;

    for (size_t i = 0; i < m_startIndexTable.Length(); ++i)
    {
        if (m_startIndexTable[i] != std::numeric_limits<size_t>::max())
        {
            size_t numberOfPointsInBucket =
                m_endIndexTable[i] - m_startIndexTable[i];
            sumNumberOfPointsPerBucket += numberOfPointsInBucket;
            maxNumberOfPointsPerBucket =
                std::max(maxNumberOfPointsPerBucket, numberOfPointsInBucket);
            ++numberOfNonEmptyBucket;
        }
    }

    CUBBYFLOW_INFO << "Average number of points per non-empty bucket: "
                   << static_cast<float>(sumNumberOfPointsPerBucket) /
                          static_cast<float>(numberOfNonEmptyBucket);
    CUBBYFLOW_INFO << "Max number of points per bucket: "
                   << maxNumberOfPointsPerBucket;
}

template <size_t N>
void PointParallelHashGridSearcher<N>::Build(
    const ConstArrayView1<Vector<double, N>>& points, double maxSearchRadius)
{
    m_gridSpacing = 2.0 * maxSearchRadius;

    Build(points);
}

template <size_t N>
void PointParallelHashGridSearcher<N>::ForEachNearbyPoint(
    const Vector<double, N>& origin, double radius,
    const ForEachNearbyPointFunc& callback) const
{
    constexpr int numKeys = 1 << N;
    size_t nearbyKeys[numKeys];

    PointHashGridUtils<N>::GetNearbyKeys(origin, m_gridSpacing, m_resolution,
                                         nearbyKeys);

    const double queryRadiusSquared = radius * radius;

    for (int i = 0; i < numKeys; i++)
    {
        const size_t nearbyKey = nearbyKeys[i];
        const size_t start = m_startIndexTable[nearbyKey];
        const size_t end = m_endIndexTable[nearbyKey];

        // Empty bucket -- continue to next bucket
        if (start == std::numeric_limits<size_t>::max())
        {
            continue;
        }

        for (size_t j = start; j < end; ++j)
        {
            Vector<double, N> direction = m_points[j] - origin;

            if (const double distanceSquared = direction.LengthSquared();
                distanceSquared <= queryRadiusSquared)
            {
                callback(m_sortedIndices[j], m_points[j]);
            }
        }
    }
}

template <size_t N>
bool PointParallelHashGridSearcher<N>::HasNearbyPoint(
    const Vector<double, N>& origin, double radius) const
{
    constexpr int numKeys = 1 << N;
    size_t nearbyKeys[numKeys];

    PointHashGridUtils<N>::GetNearbyKeys(origin, m_gridSpacing, m_resolution,
                                         nearbyKeys);

    const double queryRadiusSquared = radius * radius;

    for (int i = 0; i < numKeys; i++)
    {
        const size_t nearbyKey = nearbyKeys[i];
        const size_t start = m_startIndexTable[nearbyKey];
        const size_t end = m_endIndexTable[nearbyKey];

        // Empty bucket -- continue to next bucket
        if (start == std::numeric_limits<size_t>::max())
        {
            continue;
        }

        for (size_t j = start; j < end; ++j)
        {
            Vector<double, N> direction = m_points[j] - origin;

            if (const double distanceSquared = direction.LengthSquared();
                distanceSquared <= queryRadiusSquared)
            {
                return true;
            }
        }
    }

    return false;
}

template <size_t N>
ConstArrayView1<size_t> PointParallelHashGridSearcher<N>::Keys() const
{
    return m_keys;
}

template <size_t N>
ConstArrayView1<size_t> PointParallelHashGridSearcher<N>::StartIndexTable()
    const
{
    return m_startIndexTable;
}

template <size_t N>
ConstArrayView1<size_t> PointParallelHashGridSearcher<N>::EndIndexTable() const
{
    return m_endIndexTable;
}

template <size_t N>
ConstArrayView1<size_t> PointParallelHashGridSearcher<N>::SortedIndices() const
{
    return m_sortedIndices;
}

template <size_t N>
std::shared_ptr<PointNeighborSearcher<N>>
PointParallelHashGridSearcher<N>::Clone() const
{
    return std::make_shared<PointParallelHashGridSearcher>(*this);
}

template <size_t N>
void PointParallelHashGridSearcher<N>::Set(
    const PointParallelHashGridSearcher& other)
{
    m_gridSpacing = other.m_gridSpacing;
    m_resolution = other.m_resolution;
    m_points = other.m_points;
    m_keys = other.m_keys;
    m_startIndexTable = other.m_startIndexTable;
    m_endIndexTable = other.m_endIndexTable;
    m_sortedIndices = other.m_sortedIndices;
}

template <size_t N>
void PointParallelHashGridSearcher<N>::Serialize(
    std::vector<uint8_t>* buffer) const
{
    Serialize(*this, buffer);
}

template <size_t N>
void PointParallelHashGridSearcher<N>::Deserialize(
    const std::vector<uint8_t>& buffer)
{
    Deserialize(buffer, *this);
}

template <size_t N>
typename PointParallelHashGridSearcher<N>::Builder
PointParallelHashGridSearcher<N>::GetBuilder()
{
    return Builder{};
}

template <size_t N>
template <size_t M>
    requires(M == 2)
void PointParallelHashGridSearcher<N>::Serialize(
    const PointParallelHashGridSearcher<2>& searcher,
    std::vector<uint8_t>* buffer)
{
    flatbuffers::FlatBufferBuilder builder(1024);

    // Copy simple data
    fbs::Vector2UZ fbsResolution =
        fbs::Vector2UZ{ static_cast<uint64_t>(searcher.m_resolution.x),
                        static_cast<uint64_t>(searcher.m_resolution.y) };

    // Copy points
    std::vector<fbs::Vector2D> points;
    for (const auto& pt : searcher.m_points)
    {
        points.push_back(CubbyFlowToFlatbuffers(pt));
    }

    const flatbuffers::Offset<flatbuffers::Vector<const fbs::Vector2D*>>
        fbsPoints = builder.CreateVectorOfStructs(points.data(), points.size());

    // Copy key/tables
    std::vector<uint64_t> keys(searcher.m_keys.begin(), searcher.m_keys.end());
    std::vector<uint64_t> startIndexTable(searcher.m_startIndexTable.begin(),
                                          searcher.m_startIndexTable.end());
    std::vector<uint64_t> endIndexTable(searcher.m_endIndexTable.begin(),
                                        searcher.m_endIndexTable.end());
    std::vector<uint64_t> sortedIndices(searcher.m_sortedIndices.begin(),
                                        searcher.m_sortedIndices.end());

    const flatbuffers::Offset<flatbuffers::Vector<uint64_t>> fbsKeys =
        builder.CreateVector(keys.data(), keys.size());
    const flatbuffers::Offset<flatbuffers::Vector<uint64_t>>
        fbsStartIndexTable = builder.CreateVector(startIndexTable.data(),
                                                  startIndexTable.size());
    const flatbuffers::Offset<flatbuffers::Vector<uint64_t>> fbsEndIndexTable =
        builder.CreateVector(endIndexTable.data(), endIndexTable.size());
    const flatbuffers::Offset<flatbuffers::Vector<uint64_t>> fbsSortedIndices =
        builder.CreateVector(sortedIndices.data(), sortedIndices.size());

    // Copy the searcher
    const flatbuffers::Offset<fbs::PointParallelHashGridSearcher2> fbsSearcher =
        fbs::CreatePointParallelHashGridSearcher2(
            builder, searcher.m_gridSpacing, &fbsResolution, fbsPoints, fbsKeys,
            fbsStartIndexTable, fbsEndIndexTable, fbsSortedIndices);

    builder.Finish(fbsSearcher);

    uint8_t* buf = builder.GetBufferPointer();
    const size_t size = builder.GetSize();

    buffer->resize(size);
    memcpy(buffer->data(), buf, size);
}

template <size_t N>
template <size_t M>
    requires(M == 3)
void PointParallelHashGridSearcher<N>::Serialize(
    const PointParallelHashGridSearcher<3>& searcher,
    std::vector<uint8_t>* buffer)
{
    flatbuffers::FlatBufferBuilder builder(1024);

    // Copy simple data
    fbs::Vector3UZ fbsResolution =
        fbs::Vector3UZ{ static_cast<uint64_t>(searcher.m_resolution.x),
                        static_cast<uint64_t>(searcher.m_resolution.y),
                        static_cast<uint64_t>(searcher.m_resolution.z) };

    // Copy points
    std::vector<fbs::Vector3D> points;
    for (const auto& pt : searcher.m_points)
    {
        points.push_back(CubbyFlowToFlatbuffers(pt));
    }

    const flatbuffers::Offset<flatbuffers::Vector<const fbs::Vector3D*>>
        fbsPoints = builder.CreateVectorOfStructs(points.data(), points.size());

    // Copy key/tables
    std::vector<uint64_t> keys(searcher.m_keys.begin(), searcher.m_keys.end());
    std::vector<uint64_t> startIndexTable(searcher.m_startIndexTable.begin(),
                                          searcher.m_startIndexTable.end());
    std::vector<uint64_t> endIndexTable(searcher.m_endIndexTable.begin(),
                                        searcher.m_endIndexTable.end());
    std::vector<uint64_t> sortedIndices(searcher.m_sortedIndices.begin(),
                                        searcher.m_sortedIndices.end());

    const flatbuffers::Offset<flatbuffers::Vector<uint64_t>> fbsKeys =
        builder.CreateVector(keys.data(), keys.size());
    const flatbuffers::Offset<flatbuffers::Vector<uint64_t>>
        fbsStartIndexTable = builder.CreateVector(startIndexTable.data(),
                                                  startIndexTable.size());
    const flatbuffers::Offset<flatbuffers::Vector<uint64_t>> fbsEndIndexTable =
        builder.CreateVector(endIndexTable.data(), endIndexTable.size());
    const flatbuffers::Offset<flatbuffers::Vector<uint64_t>> fbsSortedIndices =
        builder.CreateVector(sortedIndices.data(), sortedIndices.size());

    // Copy the searcher
    const flatbuffers::Offset<fbs::PointParallelHashGridSearcher3> fbsSearcher =
        fbs::CreatePointParallelHashGridSearcher3(
            builder, searcher.m_gridSpacing, &fbsResolution, fbsPoints, fbsKeys,
            fbsStartIndexTable, fbsEndIndexTable, fbsSortedIndices);

    builder.Finish(fbsSearcher);

    uint8_t* buf = builder.GetBufferPointer();
    const size_t size = builder.GetSize();

    buffer->resize(size);
    memcpy(buffer->data(), buf, size);
}

template <size_t N>
template <size_t M>
    requires(M == 2)
void PointParallelHashGridSearcher<N>::Deserialize(
    const std::vector<uint8_t>& buffer,
    PointParallelHashGridSearcher<2>& searcher)
{
    const fbs::PointParallelHashGridSearcher2* fbsSearcher =
        fbs::GetPointParallelHashGridSearcher2(buffer.data());

    // Copy simple data
    const Vector2UZ res = FlatbuffersToCubbyFlow(*fbsSearcher->resolution());
    searcher.m_resolution = { static_cast<ssize_t>(res.x),
                              static_cast<ssize_t>(res.y) };
    searcher.m_gridSpacing = fbsSearcher->gridSpacing();

    // Copy points
    const flatbuffers::Vector<const fbs::Vector2D*>* fbsPoints =
        fbsSearcher->points();
    searcher.m_points.Resize(fbsPoints->size());
    for (uint32_t i = 0; i < fbsPoints->size(); ++i)
    {
        searcher.m_points[i] = FlatbuffersToCubbyFlow(*fbsPoints->Get(i));
    }

    // Copy key/tables
    const flatbuffers::Vector<uint64_t>* fbsKeys = fbsSearcher->keys();
    searcher.m_keys.Resize(fbsKeys->size());
    for (uint32_t i = 0; i < fbsKeys->size(); ++i)
    {
        searcher.m_keys[i] = static_cast<size_t>(fbsKeys->Get(i));
    }

    const flatbuffers::Vector<uint64_t>* fbsStartIndexTable =
        fbsSearcher->startIndexTable();
    searcher.m_startIndexTable.Resize(fbsStartIndexTable->size());
    for (uint32_t i = 0; i < fbsStartIndexTable->size(); ++i)
    {
        searcher.m_startIndexTable[i] =
            static_cast<size_t>(fbsStartIndexTable->Get(i));
    }

    const flatbuffers::Vector<uint64_t>* fbsEndIndexTable =
        fbsSearcher->endIndexTable();
    searcher.m_endIndexTable.Resize(fbsEndIndexTable->size());
    for (uint32_t i = 0; i < fbsEndIndexTable->size(); ++i)
    {
        searcher.m_endIndexTable[i] =
            static_cast<size_t>(fbsEndIndexTable->Get(i));
    }

    const flatbuffers::Vector<uint64_t>* fbsSortedIndices =
        fbsSearcher->sortedIndices();
    searcher.m_sortedIndices.Resize(fbsSortedIndices->size());
    for (uint32_t i = 0; i < fbsSortedIndices->size(); ++i)
    {
        searcher.m_sortedIndices[i] =
            static_cast<size_t>(fbsSortedIndices->Get(i));
    }
}

template <size_t N>
template <size_t M>
    requires(M == 3)
void PointParallelHashGridSearcher<N>::Deserialize(
    const std::vector<uint8_t>& buffer,
    PointParallelHashGridSearcher<3>& searcher)
{
    const fbs::PointParallelHashGridSearcher3* fbsSearcher =
        fbs::GetPointParallelHashGridSearcher3(buffer.data());

    // Copy simple data
    const Vector3UZ res = FlatbuffersToCubbyFlow(*fbsSearcher->resolution());
    searcher.m_resolution = { static_cast<ssize_t>(res.x),
                              static_cast<ssize_t>(res.y),
                              static_cast<ssize_t>(res.z) };
    searcher.m_gridSpacing = fbsSearcher->gridSpacing();

    // Copy points
    const flatbuffers::Vector<const fbs::Vector3D*>* fbsPoints =
        fbsSearcher->points();
    searcher.m_points.Resize(fbsPoints->size());
    for (uint32_t i = 0; i < fbsPoints->size(); ++i)
    {
        searcher.m_points[i] = FlatbuffersToCubbyFlow(*fbsPoints->Get(i));
    }

    // Copy key/tables
    const flatbuffers::Vector<uint64_t>* fbsKeys = fbsSearcher->keys();
    searcher.m_keys.Resize(fbsKeys->size());
    for (uint32_t i = 0; i < fbsKeys->size(); ++i)
    {
        searcher.m_keys[i] = static_cast<size_t>(fbsKeys->Get(i));
    }

    const flatbuffers::Vector<uint64_t>* fbsStartIndexTable =
        fbsSearcher->startIndexTable();
    searcher.m_startIndexTable.Resize(fbsStartIndexTable->size());
    for (uint32_t i = 0; i < fbsStartIndexTable->size(); ++i)
    {
        searcher.m_startIndexTable[i] =
            static_cast<size_t>(fbsStartIndexTable->Get(i));
    }

    const flatbuffers::Vector<uint64_t>* fbsEndIndexTable =
        fbsSearcher->endIndexTable();
    searcher.m_endIndexTable.Resize(fbsEndIndexTable->size());
    for (uint32_t i = 0; i < fbsEndIndexTable->size(); ++i)
    {
        searcher.m_endIndexTable[i] =
            static_cast<size_t>(fbsEndIndexTable->Get(i));
    }

    const flatbuffers::Vector<uint64_t>* fbsSortedIndices =
        fbsSearcher->sortedIndices();
    searcher.m_sortedIndices.Resize(fbsSortedIndices->size());
    for (uint32_t i = 0; i < fbsSortedIndices->size(); ++i)
    {
        searcher.m_sortedIndices[i] =
            static_cast<size_t>(fbsSortedIndices->Get(i));
    }
}

template <size_t N>
typename PointParallelHashGridSearcher<N>::Builder&
PointParallelHashGridSearcher<N>::Builder::WithResolution(
    const Vector<size_t, N>& resolution)
{
    m_resolution = resolution;
    return *this;
}

template <size_t N>
typename PointParallelHashGridSearcher<N>::Builder&
PointParallelHashGridSearcher<N>::Builder::WithGridSpacing(double gridSpacing)
{
    m_gridSpacing = gridSpacing;
    return *this;
}

template <size_t N>
PointParallelHashGridSearcher<N>
PointParallelHashGridSearcher<N>::Builder::Build() const
{
    return PointParallelHashGridSearcher{ m_resolution, m_gridSpacing };
}

template <size_t N>
std::shared_ptr<PointParallelHashGridSearcher<N>>
PointParallelHashGridSearcher<N>::Builder::MakeShared() const
{
    return std::make_shared<PointParallelHashGridSearcher>(m_resolution,
                                                           m_gridSpacing);
}

template <size_t N>
std::shared_ptr<PointNeighborSearcher<N>>
PointParallelHashGridSearcher<N>::Builder::BuildPointNeighborSearcher() const
{
    return MakeShared();
}

template class PointParallelHashGridSearcher<2>;

template class PointParallelHashGridSearcher<3>;
}  // namespace CubbyFlow

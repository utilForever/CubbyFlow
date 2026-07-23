// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Searcher/PointHashGridSearcher.hpp>
#include <Core/Utils/FlatbuffersHelper.hpp>
#include <Core/Utils/PointHashGridUtils.hpp>

#include <Flatbuffers/generated/PointHashGridSearcher2_generated.h>
#include <Flatbuffers/generated/PointHashGridSearcher3_generated.h>

#include <algorithm>
#include <span>

namespace CubbyFlow
{
template <size_t N>
PointHashGridSearcher<N>::PointHashGridSearcher(
    const Vector<size_t, N>& resolution, double gridSpacing)
{
    m_gridSpacing = gridSpacing;
    m_resolution =
        Max(resolution.template CastTo<ssize_t>(),
            Vector<ssize_t, N>::MakeConstant(static_cast<ssize_t>(1)));
}

template <size_t N>
PointHashGridSearcher<N>::PointHashGridSearcher(
    const PointHashGridSearcher& other)
    : m_gridSpacing(other.m_gridSpacing),
      m_resolution(other.m_resolution),
      m_points(other.m_points),
      m_buckets(other.m_buckets)
{
    // Do nothing
}

template <size_t N>
PointHashGridSearcher<N>::PointHashGridSearcher(
    PointHashGridSearcher&& other) noexcept
    : m_gridSpacing(std::exchange(other.m_gridSpacing, 1.0)),
      m_resolution(std::move(other.m_resolution)),
      m_points(std::move(other.m_points)),
      m_buckets(std::move(other.m_buckets))
{
    // Do nothing
}

template <size_t N>
PointHashGridSearcher<N>& PointHashGridSearcher<N>::operator=(
    const PointHashGridSearcher& other)
{
    m_gridSpacing = other.m_gridSpacing;
    m_resolution = other.m_resolution;
    m_points = other.m_points;
    m_buckets = other.m_buckets;
    return *this;
}

template <size_t N>
PointHashGridSearcher<N>& PointHashGridSearcher<N>::operator=(
    PointHashGridSearcher&& other) noexcept
{
    m_gridSpacing = std::exchange(other.m_gridSpacing, 1.0);
    m_resolution = std::move(other.m_resolution);
    m_points = std::move(other.m_points);
    m_buckets = std::move(other.m_buckets);
    return *this;
}

template <size_t N>
void PointHashGridSearcher<N>::Build(
    const ConstArrayView1<Vector<double, N>>& points, double maxSearchRadius)
{
    m_gridSpacing = 2.0 * maxSearchRadius;

    m_buckets.Clear();
    m_points.Clear();

    // Allocate memory chunks
    m_buckets.Resize(Product(m_resolution, static_cast<ssize_t>(1)));
    m_points.Resize(points.Length());

    if (points.IsEmpty())
    {
        return;
    }

    // Put points into buckets
    for (size_t i = 0; i < points.Length(); ++i)
    {
        m_points[i] = points[i];

        const size_t key = PointHashGridUtils<N>::GetHashKeyFromPosition(
            points[i], m_gridSpacing, m_resolution);
        m_buckets[key].Append(i);
    }
}

template <size_t N>
void PointHashGridSearcher<N>::ForEachNearbyPoint(
    const Vector<double, N>& origin, double radius,
    const ForEachNearbyPointFunc& callback) const
{
    if (m_buckets.IsEmpty())
    {
        return;
    }

    constexpr int numKeys = 1 << N;
    size_t nearbyKeys[numKeys];

    PointHashGridUtils<N>::GetNearbyKeys(origin, m_gridSpacing, m_resolution,
                                         nearbyKeys);

    const double queryRadiusSquared = radius * radius;

    for (int i = 0; i < numKeys; i++)
    {
        const Array1<size_t>& bucket = m_buckets[nearbyKeys[i]];
        const size_t numberOfPointsInBucket = bucket.Length();

        for (size_t j = 0; j < numberOfPointsInBucket; ++j)
        {
            size_t pointIndex = bucket[j];

            if (const double rSquared =
                    (m_points[pointIndex] - origin).LengthSquared();
                rSquared <= queryRadiusSquared)
            {
                callback(pointIndex, m_points[pointIndex]);
            }
        }
    }
}

template <size_t N>
bool PointHashGridSearcher<N>::HasNearbyPoint(const Vector<double, N>& origin,
                                              double radius) const
{
    if (m_buckets.IsEmpty())
    {
        return false;
    }

    constexpr int numKeys = 1 << N;
    size_t nearbyKeys[numKeys];

    PointHashGridUtils<N>::GetNearbyKeys(origin, m_gridSpacing, m_resolution,
                                         nearbyKeys);

    const double queryRadiusSquared = radius * radius;

    for (int i = 0; i < numKeys; i++)
    {
        const Array1<size_t>& bucket = m_buckets[nearbyKeys[i]];
        const size_t numberOfPointsInBucket = bucket.Length();

        for (size_t j = 0; j < numberOfPointsInBucket; ++j)
        {
            size_t pointIndex = bucket[j];

            if (const double rSquared =
                    (m_points[pointIndex] - origin).LengthSquared();
                rSquared <= queryRadiusSquared)
            {
                return true;
            }
        }
    }

    return false;
}

template <size_t N>
void PointHashGridSearcher<N>::Add(const Vector<double, N>& point)
{
    if (m_buckets.IsEmpty())
    {
        Array1<Vector<double, N>> arr = { point };
        Build(arr, 0.5 * m_gridSpacing);
    }
    else
    {
        const size_t i = m_points.Length();
        m_points.Append(point);

        const size_t key = PointHashGridUtils<N>::GetHashKeyFromPosition(
            point, m_gridSpacing, m_resolution);
        m_buckets[key].Append(i);
    }
}

template <size_t N>
const Array1<Array1<size_t>>& PointHashGridSearcher<N>::Buckets() const
{
    return m_buckets;
}

template <size_t N>
std::shared_ptr<PointNeighborSearcher<N>> PointHashGridSearcher<N>::Clone()
    const
{
    return std::make_shared<PointHashGridSearcher>(*this);
}

template <size_t N>
void PointHashGridSearcher<N>::Set(const PointHashGridSearcher& other)
{
    m_gridSpacing = other.m_gridSpacing;
    m_resolution = other.m_resolution;
    m_points = other.m_points;
    m_buckets = other.m_buckets;
}

template <size_t N>
void PointHashGridSearcher<N>::Serialize(std::vector<uint8_t>* buffer) const
{
    Serialize(*this, buffer);
}

template <size_t N>
void PointHashGridSearcher<N>::Deserialize(const std::vector<uint8_t>& buffer)
{
    Deserialize(buffer, *this);
}

template <size_t N>
typename PointHashGridSearcher<N>::Builder
PointHashGridSearcher<N>::GetBuilder()
{
    return Builder{};
}

template <size_t N>
template <size_t M>
CUBBYFLOW_REQUIRES(M == 2)
void PointHashGridSearcher<N>::Serialize(
    const PointHashGridSearcher<2>& searcher, std::vector<uint8_t>* buffer)
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

    // Copy buckets
    std::vector<flatbuffers::Offset<fbs::PointHashGridSearcherBucket2>> buckets;
    for (const auto& bucket : searcher.m_buckets)
    {
        std::vector<uint64_t> bucket64(bucket.begin(), bucket.end());
        flatbuffers::Offset<fbs::PointHashGridSearcherBucket2> fbsBucket =
            fbs::CreatePointHashGridSearcherBucket2(
                builder,
                builder.CreateVector(bucket64.data(), bucket64.size()));
        buckets.push_back(fbsBucket);
    }

    const flatbuffers::Offset<flatbuffers::Vector<
        flatbuffers::Offset<fbs::PointHashGridSearcherBucket2>>>
        fbsBuckets = builder.CreateVector(buckets);

    // Copy the searcher
    const flatbuffers::Offset<fbs::PointHashGridSearcher2> fbsSearcher =
        fbs::CreatePointHashGridSearcher2(builder, searcher.m_gridSpacing,
                                          &fbsResolution, fbsPoints,
                                          fbsBuckets);

    builder.Finish(fbsSearcher);

    uint8_t* buf = builder.GetBufferPointer();
    const size_t size = builder.GetSize();

    buffer->resize(size);
    memcpy(buffer->data(), buf, size);
}

template <size_t N>
template <size_t M>
CUBBYFLOW_REQUIRES(M == 3)
void PointHashGridSearcher<N>::Serialize(
    const PointHashGridSearcher<3>& searcher, std::vector<uint8_t>* buffer)
{
    flatbuffers::FlatBufferBuilder builder(1024);

    // Copy simple data
    auto fbsResolution =
        fbs::Vector3UZ(searcher.m_resolution.x, searcher.m_resolution.y,
                       searcher.m_resolution.z);

    // Copy points
    std::vector<fbs::Vector3D> points;
    for (const auto& pt : searcher.m_points)
    {
        points.push_back(CubbyFlowToFlatbuffers(pt));
    }

    const flatbuffers::Offset<flatbuffers::Vector<const fbs::Vector3D*>>
        fbsPoints = builder.CreateVectorOfStructs(points.data(), points.size());

    // Copy buckets
    std::vector<flatbuffers::Offset<fbs::PointHashGridSearcherBucket3>> buckets;
    for (const auto& bucket : searcher.m_buckets)
    {
        std::vector<uint64_t> bucket64(bucket.begin(), bucket.end());
        flatbuffers::Offset<fbs::PointHashGridSearcherBucket3> fbsBucket =
            fbs::CreatePointHashGridSearcherBucket3(
                builder,
                builder.CreateVector(bucket64.data(), bucket64.size()));
        buckets.push_back(fbsBucket);
    }

    const flatbuffers::Offset<flatbuffers::Vector<
        flatbuffers::Offset<fbs::PointHashGridSearcherBucket3>>>
        fbsBuckets = builder.CreateVector(buckets);

    // Copy the searcher
    const flatbuffers::Offset<fbs::PointHashGridSearcher3> fbsSearcher =
        fbs::CreatePointHashGridSearcher3(builder, searcher.m_gridSpacing,
                                          &fbsResolution, fbsPoints,
                                          fbsBuckets);

    builder.Finish(fbsSearcher);

    uint8_t* buf = builder.GetBufferPointer();
    const size_t size = builder.GetSize();

    buffer->resize(size);
    memcpy(buffer->data(), buf, size);
}

template <size_t N>
template <size_t M>
CUBBYFLOW_REQUIRES(M == 2)
void PointHashGridSearcher<N>::Deserialize(const std::vector<uint8_t>& buffer,
                                           PointHashGridSearcher<2>& searcher)
{
    const fbs::PointHashGridSearcher2* fbsSearcher =
        fbs::GetPointHashGridSearcher2(buffer.data());

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

    // Copy buckets
    const flatbuffers::Vector<
        flatbuffers::Offset<fbs::PointHashGridSearcherBucket2>>* fbsBuckets =
        fbsSearcher->buckets();
    searcher.m_buckets.Resize(fbsBuckets->size());
    for (uint32_t i = 0; i < fbsBuckets->size(); ++i)
    {
        const flatbuffers::Vector<
            flatbuffers::Offset<fbs::PointHashGridSearcherBucket2>>::return_type
            fbsBucket = fbsBuckets->Get(i);
        searcher.m_buckets[i].Resize(fbsBucket->data()->size());
        std::ranges::transform(
            std::span{ fbsBucket->data()->data(), fbsBucket->data()->size() },
            searcher.m_buckets[i].begin(),
            [](uint64_t val) { return static_cast<size_t>(val); });
    }
}

template <size_t N>
template <size_t M>
CUBBYFLOW_REQUIRES(M == 3)
void PointHashGridSearcher<N>::Deserialize(const std::vector<uint8_t>& buffer,
                                           PointHashGridSearcher<3>& searcher)
{
    const fbs::PointHashGridSearcher3* fbsSearcher =
        fbs::GetPointHashGridSearcher3(buffer.data());

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

    // Copy buckets
    const flatbuffers::Vector<
        flatbuffers::Offset<fbs::PointHashGridSearcherBucket3>>* fbsBuckets =
        fbsSearcher->buckets();
    searcher.m_buckets.Resize(fbsBuckets->size());
    for (uint32_t i = 0; i < fbsBuckets->size(); ++i)
    {
        const flatbuffers::Vector<
            flatbuffers::Offset<fbs::PointHashGridSearcherBucket3>>::return_type
            fbsBucket = fbsBuckets->Get(i);
        searcher.m_buckets[i].Resize(fbsBucket->data()->size());
        std::ranges::transform(
            std::span{ fbsBucket->data()->data(), fbsBucket->data()->size() },
            searcher.m_buckets[i].begin(),
            [](uint64_t val) { return static_cast<size_t>(val); });
    }
}

template <size_t N>
typename PointHashGridSearcher<N>::Builder&
PointHashGridSearcher<N>::Builder::WithResolution(
    const Vector<size_t, N>& resolution)
{
    m_resolution = resolution;
    return *this;
}

template <size_t N>
typename PointHashGridSearcher<N>::Builder&
PointHashGridSearcher<N>::Builder::WithGridSpacing(double gridSpacing)
{
    m_gridSpacing = gridSpacing;
    return *this;
}

template <size_t N>
PointHashGridSearcher<N> PointHashGridSearcher<N>::Builder::Build() const
{
    return PointHashGridSearcher{ m_resolution, m_gridSpacing };
}

template <size_t N>
std::shared_ptr<PointHashGridSearcher<N>>
PointHashGridSearcher<N>::Builder::MakeShared() const
{
    return std::make_shared<PointHashGridSearcher>(m_resolution, m_gridSpacing);
}

template <size_t N>
std::shared_ptr<PointNeighborSearcher<N>>
PointHashGridSearcher<N>::Builder::BuildPointNeighborSearcher() const
{
    return MakeShared();
}

template class PointHashGridSearcher<2>;

template class PointHashGridSearcher<3>;
}  // namespace CubbyFlow

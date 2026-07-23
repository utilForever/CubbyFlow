// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Searcher/PointSimpleListSearcher.hpp>
#include <Core/Utils/FlatbuffersHelper.hpp>

#include <Flatbuffers/generated/PointSimpleListSearcher2_generated.h>
#include <Flatbuffers/generated/PointSimpleListSearcher3_generated.h>

namespace CubbyFlow
{
template <size_t N>
PointSimpleListSearcher<N>::PointSimpleListSearcher(
    const PointSimpleListSearcher& other)
    : m_points(other.m_points)
{
    // Do nothing
}

template <size_t N>
PointSimpleListSearcher<N>::PointSimpleListSearcher(
    PointSimpleListSearcher&& other) noexcept
    : m_points(std::move(other.m_points))
{
    // Do nothing
}

template <size_t N>
PointSimpleListSearcher<N>& PointSimpleListSearcher<N>::operator=(
    const PointSimpleListSearcher& other)
{
    m_points = other.m_points;
    return *this;
}

template <size_t N>
PointSimpleListSearcher<N>& PointSimpleListSearcher<N>::operator=(
    PointSimpleListSearcher&& other) noexcept
{
    m_points = std::move(other.m_points);
    return *this;
}

template <size_t N>
void PointSimpleListSearcher<N>::Build(
    const ConstArrayView1<Vector<double, N>>& points, double maxSearchRadius)
{
    UNUSED_VARIABLE(maxSearchRadius);

    m_points.Resize(points.Length());
    std::copy(points.data(), points.data() + points.Length(), m_points.begin());
}

template <size_t N>
void PointSimpleListSearcher<N>::ForEachNearbyPoint(
    const Vector<double, N>& origin, double radius,
    const ForEachNearbyPointFunc& callback) const
{
    const double radiusSquared = radius * radius;

    for (size_t i = 0; i < m_points.Length(); ++i)
    {
        Vector<double, N> r = m_points[i] - origin;

        if (const double distanceSquared = r.Dot(r);
            distanceSquared <= radiusSquared)
        {
            callback(i, m_points[i]);
        }
    }
}

template <size_t N>
bool PointSimpleListSearcher<N>::HasNearbyPoint(const Vector<double, N>& origin,
                                                double radius) const
{
    const double radiusSquared = radius * radius;

    for (size_t i = 0; i < m_points.Length(); ++i)
    {
        Vector<double, N> r = m_points[i] - origin;

        if (const double distanceSquared = r.Dot(r);
            distanceSquared <= radiusSquared)
        {
            return true;
        }
    }

    return false;
}

template <size_t N>
std::shared_ptr<PointNeighborSearcher<N>> PointSimpleListSearcher<N>::Clone()
    const
{
    return std::make_shared<PointSimpleListSearcher>(*this);
}

template <size_t N>
void PointSimpleListSearcher<N>::Set(const PointSimpleListSearcher& other)
{
    m_points = other.m_points;
}

template <size_t N>
void PointSimpleListSearcher<N>::Serialize(std::vector<uint8_t>* buffer) const
{
    Serialize(*this, buffer);
}

template <size_t N>
void PointSimpleListSearcher<N>::Deserialize(const std::vector<uint8_t>& buffer)
{
    Deserialize(buffer, *this);
}

template <size_t N>
typename PointSimpleListSearcher<N>::Builder
PointSimpleListSearcher<N>::GetBuilder()
{
    return Builder{};
}

template <size_t N>
template <size_t M>
    requires(M == 2)
void PointSimpleListSearcher<N>::Serialize(
    const PointSimpleListSearcher<2>& searcher, std::vector<uint8_t>* buffer)
{
    flatbuffers::FlatBufferBuilder builder(1024);

    // Copy points
    Array1<fbs::Vector2D> points;
    for (const auto& pt : searcher.m_points)
    {
        points.Append(CubbyFlowToFlatbuffers(pt));
    }

    const flatbuffers::Offset<flatbuffers::Vector<const fbs::Vector2D*>>
        fbsPoints =
            builder.CreateVectorOfStructs(points.data(), points.Length());

    // Copy the searcher
    const flatbuffers::Offset<fbs::PointSimpleListSearcher2> fbsSearcher =
        fbs::CreatePointSimpleListSearcher2(builder, fbsPoints);

    builder.Finish(fbsSearcher);

    uint8_t* buf = builder.GetBufferPointer();
    const size_t size = builder.GetSize();

    buffer->resize(size);
    memcpy(buffer->data(), buf, size);
}

template <size_t N>
template <size_t M>
    requires(M == 3)
void PointSimpleListSearcher<N>::Serialize(
    const PointSimpleListSearcher<3>& searcher, std::vector<uint8_t>* buffer)
{
    flatbuffers::FlatBufferBuilder builder(1024);

    // Copy points
    std::vector<fbs::Vector3D> points;
    for (const auto& pt : searcher.m_points)
    {
        points.push_back(CubbyFlowToFlatbuffers(pt));
    }

    const flatbuffers::Offset<flatbuffers::Vector<const fbs::Vector3D*>>
        fbsPoints = builder.CreateVectorOfStructs(points.data(), points.size());

    // Copy the searcher
    const flatbuffers::Offset<fbs::PointSimpleListSearcher3> fbsSearcher =
        fbs::CreatePointSimpleListSearcher3(builder, fbsPoints);

    builder.Finish(fbsSearcher);

    uint8_t* buf = builder.GetBufferPointer();
    const size_t size = builder.GetSize();

    buffer->resize(size);
    memcpy(buffer->data(), buf, size);
}

template <size_t N>
template <size_t M>
    requires(M == 2)
void PointSimpleListSearcher<N>::Deserialize(
    const std::vector<uint8_t>& buffer, PointSimpleListSearcher<2>& searcher)
{
    const fbs::PointSimpleListSearcher2* fbsSearcher =
        fbs::GetPointSimpleListSearcher2(buffer.data());

    // Copy points
    const flatbuffers::Vector<const fbs::Vector2D*>* fbsPoints =
        fbsSearcher->points();
    searcher.m_points.Resize(fbsPoints->size());
    for (uint32_t i = 0; i < fbsPoints->size(); ++i)
    {
        searcher.m_points[i] = FlatbuffersToCubbyFlow(*fbsPoints->Get(i));
    }
}

template <size_t N>
template <size_t M>
    requires(M == 3)
void PointSimpleListSearcher<N>::Deserialize(
    const std::vector<uint8_t>& buffer, PointSimpleListSearcher<3>& searcher)
{
    const fbs::PointSimpleListSearcher3* fbsSearcher =
        fbs::GetPointSimpleListSearcher3(buffer.data());

    // Copy points
    const flatbuffers::Vector<const fbs::Vector3D*>* fbsPoints =
        fbsSearcher->points();
    searcher.m_points.Resize(fbsPoints->size());
    for (uint32_t i = 0; i < fbsPoints->size(); ++i)
    {
        searcher.m_points[i] = FlatbuffersToCubbyFlow(*fbsPoints->Get(i));
    }
}

template <size_t N>
PointSimpleListSearcher<N> PointSimpleListSearcher<N>::Builder::Build() const
{
    return PointSimpleListSearcher{};
}

template <size_t N>
std::shared_ptr<PointSimpleListSearcher<N>>
PointSimpleListSearcher<N>::Builder::MakeShared() const
{
    return std::make_shared<PointSimpleListSearcher>();
}

template <size_t N>
std::shared_ptr<PointNeighborSearcher<N>>
PointSimpleListSearcher<N>::Builder::BuildPointNeighborSearcher() const
{
    return MakeShared();
}

template class PointSimpleListSearcher<2>;

template class PointSimpleListSearcher<3>;
}  // namespace CubbyFlow

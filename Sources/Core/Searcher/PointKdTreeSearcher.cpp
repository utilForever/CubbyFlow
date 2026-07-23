// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Searcher/PointKdTreeSearcher.hpp>
#include <Core/Utils/FlatbuffersHelper.hpp>
#include <Core/Utils/Parallel.hpp>

#include <Flatbuffers/generated/PointKdTreeSearcher2_generated.h>
#include <Flatbuffers/generated/PointKdTreeSearcher3_generated.h>

#include <flatbuffers/flatbuffers.h>

namespace CubbyFlow
{
template <size_t N>
PointKdTreeSearcher<N>::PointKdTreeSearcher(const PointKdTreeSearcher& other)
    : m_tree(other.m_tree)
{
    // Do nothing
}

template <size_t N>
PointKdTreeSearcher<N>::PointKdTreeSearcher(
    PointKdTreeSearcher&& other) noexcept
    : m_tree(std::move(other.m_tree))
{
    // Do nothing
}

template <size_t N>
PointKdTreeSearcher<N>& PointKdTreeSearcher<N>::operator=(
    const PointKdTreeSearcher& other)
{
    m_tree = other.m_tree;
    return *this;
}

template <size_t N>
PointKdTreeSearcher<N>& PointKdTreeSearcher<N>::operator=(
    PointKdTreeSearcher&& other) noexcept
{
    m_tree = std::move(other.m_tree);
    return *this;
}

template <size_t N>
void PointKdTreeSearcher<N>::Build(
    const ConstArrayView1<Vector<double, N>>& points, double maxSearchRadius)
{
    UNUSED_VARIABLE(maxSearchRadius);

    m_tree.Build(points);
}

template <size_t N>
void PointKdTreeSearcher<N>::ForEachNearbyPoint(
    const Vector<double, N>& origin, double radius,
    const ForEachNearbyPointFunc& callback) const
{
    m_tree.ForEachNearbyPoint(origin, radius, callback);
}

template <size_t N>
bool PointKdTreeSearcher<N>::HasNearbyPoint(const Vector<double, N>& origin,
                                            double radius) const
{
    return m_tree.HasNearbyPoint(origin, radius);
}

template <size_t N>
std::shared_ptr<PointNeighborSearcher<N>> PointKdTreeSearcher<N>::Clone() const
{
    return std::make_shared<PointKdTreeSearcher>(*this);
}

template <size_t N>
void PointKdTreeSearcher<N>::Set(const PointKdTreeSearcher& other)
{
    m_tree = other.m_tree;
}

template <size_t N>
void PointKdTreeSearcher<N>::Serialize(std::vector<uint8_t>* buffer) const
{
    Serialize(*this, buffer);
}

template <size_t N>
void PointKdTreeSearcher<N>::Deserialize(const std::vector<uint8_t>& buffer)
{
    Deserialize(buffer, *this);
}

template <size_t N>
template <size_t M>
    requires(M == 2)
void PointKdTreeSearcher<N>::Serialize(const PointKdTreeSearcher<2>& searcher,
                                       std::vector<uint8_t>* buffer)
{
    flatbuffers::FlatBufferBuilder builder(1024);

    // Copy points
    std::vector<fbs::Vector2D> points;
    for (const auto& iter : searcher.m_tree)
    {
        points.push_back(CubbyFlowToFlatbuffers(iter));
    }

    const flatbuffers::Offset<flatbuffers::Vector<const fbs::Vector2D*>>
        fbsPoints = builder.CreateVectorOfStructs(points.data(), points.size());

    // Copy nodes
    std::vector<fbs::PointKdTreeSearcherNode2> nodes;
    for (auto iter = searcher.m_tree.BeginNode();
         iter != searcher.m_tree.EndNode(); ++iter)
    {
        nodes.emplace_back(iter->flags, iter->child, iter->item);
    }

    const flatbuffers::Offset<
        flatbuffers::Vector<const fbs::PointKdTreeSearcherNode2*>>
        fbsNodes = builder.CreateVectorOfStructs(nodes);

    // Copy the searcher
    const flatbuffers::Offset<fbs::PointKdTreeSearcher2> fbsSearcher =
        fbs::CreatePointKdTreeSearcher2(builder, fbsPoints, fbsNodes);

    // Finish
    builder.Finish(fbsSearcher);

    uint8_t* buf = builder.GetBufferPointer();
    const size_t size = builder.GetSize();

    buffer->resize(size);
    memcpy(buffer->data(), buf, size);
}

template <size_t N>
template <size_t M>
    requires(M == 3)
void PointKdTreeSearcher<N>::Serialize(const PointKdTreeSearcher<3>& searcher,
                                       std::vector<uint8_t>* buffer)
{
    flatbuffers::FlatBufferBuilder builder(1024);

    // Copy points
    std::vector<fbs::Vector3D> points;
    for (const auto& iter : searcher.m_tree)
    {
        points.push_back(CubbyFlowToFlatbuffers(iter));
    }

    const flatbuffers::Offset<flatbuffers::Vector<const fbs::Vector3D*>>
        fbsPoints = builder.CreateVectorOfStructs(points.data(), points.size());

    // Copy nodes
    std::vector<fbs::PointKdTreeSearcherNode3> nodes;
    for (auto iter = searcher.m_tree.BeginNode();
         iter != searcher.m_tree.EndNode(); ++iter)
    {
        nodes.emplace_back(iter->flags, iter->child, iter->item);
    }

    const flatbuffers::Offset<
        flatbuffers::Vector<const fbs::PointKdTreeSearcherNode3*>>
        fbsNodes = builder.CreateVectorOfStructs(nodes);

    // Copy the searcher
    const flatbuffers::Offset<fbs::PointKdTreeSearcher3> fbsSearcher =
        fbs::CreatePointKdTreeSearcher3(builder, fbsPoints, fbsNodes);

    // Finish
    builder.Finish(fbsSearcher);

    uint8_t* buf = builder.GetBufferPointer();
    const size_t size = builder.GetSize();

    buffer->resize(size);
    memcpy(buffer->data(), buf, size);
}

template <size_t N>
template <size_t M>
    requires(M == 2)
void PointKdTreeSearcher<N>::Deserialize(const std::vector<uint8_t>& buffer,
                                         PointKdTreeSearcher<2>& searcher)
{
    const fbs::PointKdTreeSearcher2* fbsSearcher =
        fbs::GetPointKdTreeSearcher2(buffer.data());

    const flatbuffers::Vector<const fbs::Vector2D*>* fbsPoints =
        fbsSearcher->points();
    const flatbuffers::Vector<const fbs::PointKdTreeSearcherNode2*>* fbsNodes =
        fbsSearcher->nodes();

    searcher.m_tree.Reserve(fbsPoints->size(), fbsNodes->size());

    // Copy points
    const auto pointsIter = searcher.m_tree.begin();
    for (uint32_t i = 0; i < fbsPoints->size(); ++i)
    {
        pointsIter[i] = FlatbuffersToCubbyFlow(*fbsPoints->Get(i));
    }

    // Copy nodes
    const auto nodesIter = searcher.m_tree.BeginNode();
    for (uint32_t i = 0; i < fbsNodes->size(); ++i)
    {
        const auto fbsNode = fbsNodes->Get(i);
        nodesIter[i].flags = fbsNode->flags();
        nodesIter[i].child = fbsNode->child();
        nodesIter[i].item = fbsNode->item();
        nodesIter[i].point = pointsIter[fbsNode->item()];
    }
}

template <size_t N>
template <size_t M>
    requires(M == 3)
void PointKdTreeSearcher<N>::Deserialize(const std::vector<uint8_t>& buffer,
                                         PointKdTreeSearcher<3>& searcher)
{
    const fbs::PointKdTreeSearcher3* fbsSearcher =
        fbs::GetPointKdTreeSearcher3(buffer.data());

    const flatbuffers::Vector<const fbs::Vector3D*>* fbsPoints =
        fbsSearcher->points();
    const flatbuffers::Vector<const fbs::PointKdTreeSearcherNode3*>* fbsNodes =
        fbsSearcher->nodes();

    searcher.m_tree.Reserve(fbsPoints->size(), fbsNodes->size());

    // Copy points
    const auto pointsIter = searcher.m_tree.begin();
    for (uint32_t i = 0; i < fbsPoints->size(); ++i)
    {
        pointsIter[i] = FlatbuffersToCubbyFlow(*fbsPoints->Get(i));
    }

    // Copy nodes
    const auto nodesIter = searcher.m_tree.BeginNode();
    for (uint32_t i = 0; i < fbsNodes->size(); ++i)
    {
        const auto fbsNode = fbsNodes->Get(i);
        nodesIter[i].flags = fbsNode->flags();
        nodesIter[i].child = fbsNode->child();
        nodesIter[i].item = fbsNode->item();
        nodesIter[i].point = pointsIter[fbsNode->item()];
    }
}

template <size_t N>
typename PointKdTreeSearcher<N>::Builder PointKdTreeSearcher<N>::GetBuilder()
{
    return Builder{};
}

template <size_t N>
PointKdTreeSearcher<N> PointKdTreeSearcher<N>::Builder::Build() const
{
    return PointKdTreeSearcher{};
}

template <size_t N>
std::shared_ptr<PointKdTreeSearcher<N>>
PointKdTreeSearcher<N>::Builder::MakeShared() const
{
    return std::make_shared<PointKdTreeSearcher>();
}

template <size_t N>
std::shared_ptr<PointNeighborSearcher<N>>
PointKdTreeSearcher<N>::Builder::BuildPointNeighborSearcher() const
{
    return MakeShared();
}

template class PointKdTreeSearcher<2>;

template class PointKdTreeSearcher<3>;
}  // namespace CubbyFlow

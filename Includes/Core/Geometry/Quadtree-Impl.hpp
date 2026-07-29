// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_QUADTREE_IMPL_HPP
#define CUBBYFLOW_QUADTREE_IMPL_HPP

#include <numeric>
#include <stack>

namespace CubbyFlow
{
template <typename T>
bool Quadtree<T>::Node::IsLeaf() const
{
    return firstChild == std::numeric_limits<size_t>::max();
}

template <typename T>
void Quadtree<T>::Build(const std::vector<T>& items, const BoundingBox2D& bound,
                        const BoxIntersectionTestFunc2<T>& testFunc,
                        size_t maxDepth)
{
    // Reset items
    m_maxDepth = maxDepth;
    m_items = items;
    m_nodes.clear();

    // Normalize bounding box
    m_bbox = bound;
    const double maxEdgeLen = std::max(m_bbox.Width(), m_bbox.Height());
    m_bbox.upperCorner =
        m_bbox.lowerCorner + Vector2D{ maxEdgeLen, maxEdgeLen };

    // Build
    m_nodes.resize(1);
    m_nodes[0].items.resize(m_items.size());
    std::iota(m_nodes[0].items.begin(), m_nodes[0].items.end(), ZERO_SIZE);

    Build(0, 1, m_bbox, testFunc);
}

template <typename T>
void Quadtree<T>::Clear()
{
    m_maxDepth = 1;
    m_items.clear();
    m_nodes.clear();
    m_bbox = BoundingBox2D();
}

template <typename T>
NearestNeighborQueryResult2<T> Quadtree<T>::Nearest(
    const Vector2D& pt,
    const NearestNeighborDistanceFunc2<T>& distanceFunc) const
{
    NearestNeighborQueryResult2<T> best;
    best.distance = std::numeric_limits<double>::max();
    best.item = nullptr;

    // Prepare to traverse quadtree
    std::stack<std::pair<const Node*, BoundingBox2D>> todo;

    // Traverse quadtree nodes
    const Node* node = m_nodes.data();
    BoundingBox2D bound = m_bbox;
    while (node != nullptr)
    {
        if (node->IsLeaf())
        {
            std::ranges::for_each(
                node->items, [this, &distanceFunc, &pt, &best](size_t itemIdx) {
                    const double distance = distanceFunc(m_items[itemIdx], pt);

                    if (distance < best.distance)
                    {
                        best.distance = distance;
                        best.item = &m_items[itemIdx];
                    }
                });
        }
        else
        {
            const double bestDistSqr = best.distance * best.distance;

            typedef std::tuple<const Node*, double, BoundingBox2D> NodeDistBox;
            std::array<NodeDistBox, 4> childDistSqrPairs;
            const auto MidPoint = bound.MidPoint();
            for (int i = 0; i < 4; ++i)
            {
                const Node* child = &m_nodes[node->firstChild + i];
                const auto childBound =
                    BoundingBox2D{ bound.Corner(i), MidPoint };
                Vector2D cp = childBound.Clamp(pt);
                double distMinSqr = cp.DistanceSquaredTo(pt);

                childDistSqrPairs[i] =
                    std::make_tuple(child, distMinSqr, childBound);
            }
            std::sort(childDistSqrPairs.begin(), childDistSqrPairs.end(),
                      [](const NodeDistBox& a, const NodeDistBox& b) {
                          return std::get<1>(a) > std::get<1>(b);
                      });

            std::ranges::for_each(
                childDistSqrPairs,
                [&todo, bestDistSqr](const NodeDistBox& childPair) {
                    if (std::get<1>(childPair) < bestDistSqr)
                    {
                        todo.emplace(std::get<0>(childPair),
                                     std::get<2>(childPair));
                    }
                });
        }

        // Grab next node to process from todo stack
        if (todo.empty())
        {
            break;
        }

        node = todo.top().first;
        bound = todo.top().second;
        todo.pop();
    }

    return best;
}

template <typename T>
bool Quadtree<T>::Intersects(const BoundingBox2D& box,
                             const BoxIntersectionTestFunc2<T>& testFunc) const
{
    return Intersects(box, testFunc, 0, m_bbox);
}

template <typename T>
bool Quadtree<T>::Intersects(const Ray2D& ray,
                             const RayIntersectionTestFunc2<T>& testFunc) const
{
    return Intersects(ray, testFunc, 0, m_bbox);
}

template <typename T>
void Quadtree<T>::ForEachIntersectingItem(
    const BoundingBox2D& box, const BoxIntersectionTestFunc2<T>& testFunc,
    const IntersectionVisitorFunc<T>& visitorFunc) const
{
    ForEachIntersectingItem(box, testFunc, visitorFunc, 0, m_bbox);
}

template <typename T>
void Quadtree<T>::ForEachIntersectingItem(
    const Ray2D& ray, const RayIntersectionTestFunc2<T>& testFunc,
    const IntersectionVisitorFunc<T>& visitorFunc) const
{
    ForEachIntersectingItem(ray, testFunc, visitorFunc, 0, m_bbox);
}

template <typename T>
ClosestIntersectionQueryResult2<T> Quadtree<T>::ClosestIntersection(
    const Ray2D& ray, const GetRayIntersectionFunc2<T>& testFunc) const
{
    ClosestIntersectionQueryResult2<T> best;
    best.distance = std::numeric_limits<double>::max();
    best.item = nullptr;

    return ClosestIntersection(ray, testFunc, 0, m_bbox, best);
}

template <typename T>
typename Quadtree<T>::Iterator Quadtree<T>::begin()
{
    return m_items.begin();
}

template <typename T>
typename Quadtree<T>::Iterator Quadtree<T>::end()
{
    return m_items.end();
}

template <typename T>
typename Quadtree<T>::ConstIterator Quadtree<T>::begin() const
{
    return m_items.begin();
}

template <typename T>
typename Quadtree<T>::ConstIterator Quadtree<T>::end() const
{
    return m_items.end();
}

template <typename T>
size_t Quadtree<T>::GetNumberOfItems() const
{
    return m_items.size();
}

template <typename T>
const T& Quadtree<T>::GetItem(size_t i) const
{
    return m_items[i];
}

template <typename T>
size_t Quadtree<T>::GetNumberOfNodes() const
{
    return m_nodes.size();
}

template <typename T>
const std::vector<size_t>& Quadtree<T>::GetItemsAtNode(size_t nodeIdx) const
{
    return m_nodes[nodeIdx].items;
}

template <typename T>
size_t Quadtree<T>::GetChildIndex(size_t nodeIdx, size_t childIdx) const
{
    return m_nodes[nodeIdx].firstChild + childIdx;
}

template <typename T>
const BoundingBox2D& Quadtree<T>::GetBoundingBox() const
{
    return m_bbox;
}

template <typename T>
size_t Quadtree<T>::GetMaxDepth() const
{
    return m_maxDepth;
}

template <typename T>
void Quadtree<T>::Build(size_t nodeIdx, size_t depth,
                        const BoundingBox2D& bound,
                        const BoxIntersectionTestFunc2<T>& testFunc)
{
    if (depth >= m_maxDepth || m_nodes[nodeIdx].items.empty())
    {
        return;
    }

    const size_t firstChild = m_nodes[nodeIdx].firstChild = m_nodes.size();
    m_nodes.resize(m_nodes[nodeIdx].firstChild + 4);

    BoundingBox2D bboxPerNode[4];

    for (int i = 0; i < 4; ++i)
    {
        bboxPerNode[i] = BoundingBox2D{ bound.Corner(i), bound.MidPoint() };
    }

    auto& currentItems = m_nodes[nodeIdx].items;
    for (size_t i = 0; i < currentItems.size(); ++i)
    {
        size_t currentItem = currentItems[i];
        for (int j = 0; j < 4; ++j)
        {
            if (testFunc(m_items[currentItem], bboxPerNode[j]))
            {
                m_nodes[firstChild + j].items.push_back(currentItem);
            }
        }
    }

    // Remove non-leaf data
    currentItems.clear();

    // Refine
    for (int i = 0; i < 4; ++i)
    {
        Build(firstChild + i, depth + 1, bboxPerNode[i], testFunc);
    }
}

template <typename T>
bool Quadtree<T>::Intersects(const BoundingBox2D& box,
                             const BoxIntersectionTestFunc2<T>& testFunc,
                             size_t nodeIdx, const BoundingBox2D& bound) const
{
    if (!box.Overlaps(bound))
    {
        return false;
    }

    const Node& node = m_nodes[nodeIdx];

    if (node.items.size() > 0)
    {
        for (size_t itemIdx : node.items)
        {
            if (testFunc(m_items[itemIdx], box))
            {
                return true;
            }
        }
    }

    if (node.firstChild != std::numeric_limits<size_t>::max())
    {
        for (int i = 0; i < 4; ++i)
        {
            if (Intersects(box, testFunc, node.firstChild + i,
                           BoundingBox2D{ bound.Corner(i), bound.MidPoint() }))
            {
                return true;
            }
        }
    }

    return false;
}

template <typename T>
bool Quadtree<T>::Intersects(const Ray2D& ray,
                             const RayIntersectionTestFunc2<T>& testFunc,
                             size_t nodeIdx, const BoundingBox2D& bound) const
{
    if (!bound.Intersects(ray))
    {
        return false;
    }

    const Node& node = m_nodes[nodeIdx];

    if (node.items.size() > 0)
    {
        for (size_t itemIdx : node.items)
        {
            if (testFunc(m_items[itemIdx], ray))
            {
                return true;
            }
        }
    }

    if (node.firstChild != std::numeric_limits<size_t>::max())
    {
        for (int i = 0; i < 4; ++i)
        {
            if (Intersects(ray, testFunc, node.firstChild + i,
                           BoundingBox2D{ bound.Corner(i), bound.MidPoint() }))
            {
                return true;
            }
        }
    }

    return false;
}

template <typename T>
void Quadtree<T>::ForEachIntersectingItem(
    const BoundingBox2D& box, const BoxIntersectionTestFunc2<T>& testFunc,
    const IntersectionVisitorFunc<T>& visitorFunc, size_t nodeIdx,
    const BoundingBox2D& bound) const
{
    if (!box.Overlaps(bound))
    {
        return;
    }

    const Node& node = m_nodes[nodeIdx];

    if (node.items.size() > 0)
    {
        for (size_t itemIdx : node.items)
        {
            if (testFunc(m_items[itemIdx], box))
            {
                visitorFunc(m_items[itemIdx]);
            }
        }
    }

    if (node.firstChild != std::numeric_limits<size_t>::max())
    {
        for (int i = 0; i < 4; ++i)
        {
            ForEachIntersectingItem(
                box, testFunc, visitorFunc, node.firstChild + i,
                BoundingBox2D{ bound.Corner(i), bound.MidPoint() });
        }
    }
}

template <typename T>
void Quadtree<T>::ForEachIntersectingItem(
    const Ray2D& ray, const RayIntersectionTestFunc2<T>& testFunc,
    const IntersectionVisitorFunc<T>& visitorFunc, size_t nodeIdx,
    const BoundingBox2D& bound) const
{
    if (!bound.Intersects(ray))
    {
        return;
    }

    const Node& node = m_nodes[nodeIdx];

    if (node.items.size() > 0)
    {
        for (size_t itemIdx : node.items)
        {
            if (testFunc(m_items[itemIdx], ray))
            {
                visitorFunc(m_items[itemIdx]);
            }
        }
    }

    if (node.firstChild != std::numeric_limits<size_t>::max())
    {
        for (int i = 0; i < 4; ++i)
        {
            ForEachIntersectingItem(
                ray, testFunc, visitorFunc, node.firstChild + i,
                BoundingBox2D{ bound.Corner(i), bound.MidPoint() });
        }
    }
}

template <typename T>
ClosestIntersectionQueryResult2<T> Quadtree<T>::ClosestIntersection(
    const Ray2D& ray, const GetRayIntersectionFunc2<T>& testFunc,
    size_t nodeIdx, const BoundingBox2D& bound,
    ClosestIntersectionQueryResult2<T> best) const
{
    if (!bound.Intersects(ray))
    {
        return best;
    }

    const Node& node = m_nodes[nodeIdx];

    if (node.items.size() > 0)
    {
        for (size_t itemIdx : node.items)
        {
            double dist = testFunc(m_items[itemIdx], ray);
            if (dist < best.distance)
            {
                best.distance = dist;
                best.item = &m_items[itemIdx];
            }
        }
    }

    if (node.firstChild != std::numeric_limits<size_t>::max())
    {
        for (int i = 0; i < 4; ++i)
        {
            best = ClosestIntersection(
                ray, testFunc, node.firstChild + i,
                BoundingBox2D{ bound.Corner(i), bound.MidPoint() }, best);
        }
    }

    return best;
}
}  // namespace CubbyFlow

#endif

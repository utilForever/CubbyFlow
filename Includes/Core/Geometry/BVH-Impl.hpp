// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_BVH_IMPL_HPP
#define CUBBYFLOW_BVH_IMPL_HPP

#include <numeric>

namespace CubbyFlow
{
template <typename T, size_t N>
BVH<T, N>::Node::Node() : flags(0)
{
    child = std::numeric_limits<size_t>::max();
}

template <typename T, size_t N>
void BVH<T, N>::Node::InitLeaf(size_t it, const BoundingBox<double, N>& b)
{
    flags = static_cast<char>(N);
    item = it;
    bound = b;
}

template <typename T, size_t N>
void BVH<T, N>::Node::InitInternal(uint8_t axis, size_t c,
                                   const BoundingBox<double, N>& b)
{
    flags = axis;
    child = c;
    bound = b;
}

template <typename T, size_t N>
bool BVH<T, N>::Node::IsLeaf() const
{
    return flags == static_cast<char>(N);
}

template <typename T, size_t N>
void BVH<T, N>::Build(
    const ConstArrayView1<T>& items,
    const ConstArrayView1<BoundingBox<double, N>>& itemsBounds)
{
    m_items = items;
    m_itemBounds = itemsBounds;

    if (m_items.IsEmpty())
    {
        return;
    }

    m_nodes.Clear();
    m_bound = BoundingBox<double, N>{};

    for (size_t i = 0; i < m_items.Length(); ++i)
    {
        m_bound.Merge(m_itemBounds[i]);
    }

    Array1<size_t> itemIndices(m_items.Length());
    std::iota(std::begin(itemIndices), std::end(itemIndices), 0);

    Build(0, itemIndices.data(), m_items.Length(), 0);
}

template <typename T, size_t N>
void BVH<T, N>::Clear()
{
    m_bound = BoundingBox<double, N>{};
    m_items.Clear();
    m_itemBounds.Clear();
    m_nodes.Clear();
}

template <typename T, size_t N>
inline NearestNeighborQueryResult<T, N> BVH<T, N>::Nearest(
    const Vector<double, N>& pt,
    const NearestNeighborDistanceFunc<T, N>& distanceFunc) const
{
    NearestNeighborQueryResult<T, N> best;
    best.distance = std::numeric_limits<double>::max();
    best.item = nullptr;

    // Prepare to traverse BVH
    static const int kMaxTreeDepth = 8 * sizeof(size_t);
    const Node* todo[kMaxTreeDepth];
    size_t todoPos = 0;

    // Traverse BVH nodes
    const Node* node = m_nodes.data();
    while (node != nullptr)
    {
        if (node->IsLeaf())
        {
            if (double dist = distanceFunc(m_items[node->item], pt);
                dist < best.distance)
            {
                best.distance = dist;
                best.item = &m_items[node->item];
            }

            // Grab next node to process from todo stack
            if (todoPos > 0)
            {
                // Dequeue
                --todoPos;
                node = todo[todoPos];
            }
            else
            {
                break;
            }
        }
        else
        {
            const double bestDistSqr = best.distance * best.distance;

            const Node* left = node + 1;
            const Node* right = &m_nodes[node->child];

            // If pt is inside the box, then the closestLeft and Right will be
            // identical to pt. This will make distMinLeftSqr and
            // distMinRightSqr zero, meaning that such a box will have higher
            // priority.
            Vector<double, N> closestLeft = left->bound.Clamp(pt);
            Vector<double, N> closestRight = right->bound.Clamp(pt);

            const double distMinLeftSqr = closestLeft.DistanceSquaredTo(pt);
            const double distMinRightSqr = closestRight.DistanceSquaredTo(pt);

            const bool shouldVisitLeft = distMinLeftSqr < bestDistSqr;
            const bool shouldVisitRight = distMinRightSqr < bestDistSqr;

            if (shouldVisitLeft && shouldVisitRight)
            {
                const Node* secondChild;
                const Node* firstChild;

                if (distMinLeftSqr < distMinRightSqr)
                {
                    firstChild = left;
                    secondChild = right;
                }
                else
                {
                    firstChild = right;
                    secondChild = left;
                }

                // Enqueue secondChild in todo stack
                todo[todoPos] = secondChild;
                ++todoPos;
                node = firstChild;
            }
            else if (shouldVisitLeft)
            {
                node = left;
            }
            else if (shouldVisitRight)
            {
                node = right;
            }
            else
            {
                if (todoPos > 0)
                {
                    // Dequeue
                    --todoPos;
                    node = todo[todoPos];
                }
                else
                {
                    break;
                }
            }
        }
    }

    return best;
}

template <typename T, size_t N>
bool BVH<T, N>::Intersects(const BoundingBox<double, N>& box,
                           const BoxIntersectionTestFunc<T, N>& testFunc) const
{
    if (!m_bound.Overlaps(box))
    {
        return false;
    }

    // prepare to traverse BVH for box
    static const int MAX_TREE_DEPTH = 8 * sizeof(size_t);
    const Node* todo[MAX_TREE_DEPTH];
    size_t todoPos = 0;

    // traverse BVH nodes for box
    const Node* node = m_nodes.data();

    while (node != nullptr)
    {
        if (node->IsLeaf())
        {
            if (testFunc(m_items[node->item], box))
            {
                return true;
            }

            // grab next node to process from todo stack
            if (todoPos > 0)
            {
                // Dequeue
                --todoPos;
                node = todo[todoPos];
            }
            else
            {
                break;
            }
        }
        else
        {
            // get node children pointers for box
            const Node* firstChild = node + 1;
            const Node* secondChild = &m_nodes[node->child];

            // advance to next child node, possibly enqueue other child
            if (!firstChild->bound.Overlaps(box))
            {
                node = secondChild;
            }
            else if (!secondChild->bound.Overlaps(box))
            {
                node = firstChild;
            }
            else
            {
                // enqueue secondChild in todo stack
                todo[todoPos] = secondChild;
                ++todoPos;
                node = firstChild;
            }
        }
    }

    return false;
}

template <typename T, size_t N>
bool BVH<T, N>::Intersects(const Ray<double, N>& ray,
                           const RayIntersectionTestFunc<T, N>& testFunc) const
{
    if (!m_bound.Intersects(ray))
    {
        return false;
    }

    // prepare to traverse BVH for ray
    static const int kMaxTreeDepth = 8 * sizeof(size_t);
    const Node* todo[kMaxTreeDepth];
    size_t todoPos = 0;

    // traverse BVH nodes for ray
    const Node* node = m_nodes.data();

    while (node != nullptr)
    {
        if (node->IsLeaf())
        {
            if (testFunc(m_items[node->item], ray))
            {
                return true;
            }

            // grab next node to process from todo stack
            if (todoPos > 0)
            {
                // Dequeue
                --todoPos;
                node = todo[todoPos];
            }
            else
            {
                break;
            }
        }
        else
        {
            // get node children pointers for ray
            const Node* firstChild;
            const Node* secondChild;

            if (ray.direction[node->flags] > 0.0)
            {
                firstChild = node + 1;
                secondChild = &m_nodes[node->child];
            }
            else
            {
                firstChild = &m_nodes[node->child];
                secondChild = node + 1;
            }

            // advance to next child node, possibly enqueue other child
            if (!firstChild->bound.Intersects(ray))
            {
                node = secondChild;
            }
            else if (!secondChild->bound.Intersects(ray))
            {
                node = firstChild;
            }
            else
            {
                // enqueue secondChild in todo stack
                todo[todoPos] = secondChild;
                ++todoPos;
                node = firstChild;
            }
        }
    }

    return false;
}

template <typename T, size_t N>
void BVH<T, N>::ForEachIntersectingItem(
    const BoundingBox<double, N>& box,
    const BoxIntersectionTestFunc<T, N>& testFunc,
    const IntersectionVisitorFunc<T>& visitorFunc) const
{
    if (!m_bound.Overlaps(box))
    {
        return;
    }

    // prepare to traverse BVH for box
    static const int MAX_TREE_DEPTH = 8 * sizeof(size_t);
    const Node* todo[MAX_TREE_DEPTH];
    size_t todoPos = 0;

    // traverse BVH nodes for box
    const Node* node = m_nodes.data();

    while (node != nullptr)
    {
        if (node->IsLeaf())
        {
            if (testFunc(m_items[node->item], box))
            {
                visitorFunc(m_items[node->item]);
            }

            // grab next node to process from todo stack
            if (todoPos > 0)
            {
                // Dequeue
                --todoPos;
                node = todo[todoPos];
            }
            else
            {
                break;
            }
        }
        else
        {
            // get node children pointers for box
            const Node* firstChild = node + 1;
            const Node* secondChild = &m_nodes[node->child];

            // advance to next child node, possibly enqueue other child
            if (!firstChild->bound.Overlaps(box))
            {
                node = secondChild;
            }
            else if (!secondChild->bound.Overlaps(box))
            {
                node = firstChild;
            }
            else
            {
                // enqueue secondChild in todo stack
                todo[todoPos] = secondChild;
                ++todoPos;
                node = firstChild;
            }
        }
    }
}

template <typename T, size_t N>
void BVH<T, N>::ForEachIntersectingItem(
    const Ray<double, N>& ray, const RayIntersectionTestFunc<T, N>& testFunc,
    const IntersectionVisitorFunc<T>& visitorFunc) const
{
    if (!m_bound.Intersects(ray))
    {
        return;
    }

    // prepare to traverse BVH for ray
    static const int MAX_TREE_DEPTH = 8 * sizeof(size_t);
    const Node* todo[MAX_TREE_DEPTH];
    size_t todoPos = 0;

    // traverse BVH nodes for ray
    const Node* node = m_nodes.data();

    while (node != nullptr)
    {
        if (node->IsLeaf())
        {
            if (testFunc(m_items[node->item], ray))
            {
                visitorFunc(m_items[node->item]);
            }

            // grab next node to process from todo stack
            if (todoPos > 0)
            {
                // Dequeue
                --todoPos;
                node = todo[todoPos];
            }
            else
            {
                break;
            }
        }
        else
        {
            // get node children pointers for ray
            const Node* firstChild;
            const Node* secondChild;

            if (ray.direction[node->flags] > 0.0)
            {
                firstChild = node + 1;
                secondChild = &m_nodes[node->child];
            }
            else
            {
                firstChild = &m_nodes[node->child];
                secondChild = node + 1;
            }

            // advance to next child node, possibly enqueue other child
            if (!firstChild->bound.Intersects(ray))
            {
                node = secondChild;
            }
            else if (!secondChild->bound.Intersects(ray))
            {
                node = firstChild;
            }
            else
            {
                // enqueue secondChild in todo stack
                todo[todoPos] = secondChild;
                ++todoPos;
                node = firstChild;
            }
        }
    }
}

template <typename T, size_t N>
ClosestIntersectionQueryResult<T, N> BVH<T, N>::ClosestIntersection(
    const Ray<double, N>& ray,
    const GetRayIntersectionFunc<T, N>& testFunc) const
{
    ClosestIntersectionQueryResult<T, N> best;
    best.distance = std::numeric_limits<double>::max();
    best.item = nullptr;

    if (!m_bound.Intersects(ray))
    {
        return best;
    }

    // prepare to traverse BVH for ray
    static const int MAX_TREE_DEPTH = 8 * sizeof(size_t);
    const Node* todo[MAX_TREE_DEPTH];
    size_t todoPos = 0;

    // traverse BVH nodes for ray
    const Node* node = m_nodes.data();

    while (node != nullptr)
    {
        if (node->IsLeaf())
        {
            if (double dist = testFunc(m_items[node->item], ray);
                dist < best.distance)
            {
                best.distance = dist;
                best.item = m_items.data() + node->item;
            }

            // grab next node to process from todo stack
            if (todoPos > 0)
            {
                // Dequeue
                --todoPos;
                node = todo[todoPos];
            }
            else
            {
                break;
            }
        }
        else
        {
            // get node children pointers for ray
            const Node* firstChild;
            const Node* secondChild;

            if (ray.direction[node->flags] > 0.0)
            {
                firstChild = node + 1;
                secondChild = &m_nodes[node->child];
            }
            else
            {
                firstChild = &m_nodes[node->child];
                secondChild = node + 1;
            }

            // advance to next child node, possibly enqueue other child
            if (!firstChild->bound.Intersects(ray))
            {
                node = secondChild;
            }
            else if (!secondChild->bound.Intersects(ray))
            {
                node = firstChild;
            }
            else
            {
                // enqueue secondChild in todo stack
                todo[todoPos] = secondChild;
                ++todoPos;
                node = firstChild;
            }
        }
    }

    return best;
}

template <typename T, size_t N>
const BoundingBox<double, N>& BVH<T, N>::GetBoundingBox() const
{
    return m_bound;
}

template <typename T, size_t N>
typename BVH<T, N>::Iterator BVH<T, N>::begin()
{
    return m_items.begin();
}

template <typename T, size_t N>
typename BVH<T, N>::Iterator BVH<T, N>::end()
{
    return m_items.end();
}

template <typename T, size_t N>
typename BVH<T, N>::ConstIterator BVH<T, N>::begin() const
{
    return m_items.begin();
}

template <typename T, size_t N>
typename BVH<T, N>::ConstIterator BVH<T, N>::end() const
{
    return m_items.end();
}

template <typename T, size_t N>
size_t BVH<T, N>::NumberOfItems() const
{
    return m_items.Length();
}

template <typename T, size_t N>
const T& BVH<T, N>::Item(size_t i) const
{
    return m_items[i];
}

template <typename T, size_t N>
size_t BVH<T, N>::NumberOfNodes() const
{
    return m_nodes.Length();
}

template <typename T, size_t N>
std::pair<size_t, size_t> BVH<T, N>::Children(size_t i) const
{
    if (IsLeaf(i))
    {
        return std::make_pair(std::numeric_limits<size_t>::max(),
                              std::numeric_limits<size_t>::max());
    }

    return std::make_pair(i + 1, m_nodes[i].child);
}

template <typename T, size_t N>
bool BVH<T, N>::IsLeaf(size_t i) const
{
    return m_nodes[i].IsLeaf();
}

template <typename T, size_t N>
const BoundingBox<double, N>& BVH<T, N>::NodeBound(size_t i) const
{
    return m_nodes[i].bound;
}

template <typename T, size_t N>
typename BVH<T, N>::Iterator BVH<T, N>::ItemOfNode(size_t i)
{
    if (IsLeaf(i))
    {
        return m_nodes[i].item + begin();
    }

    return end();
}

template <typename T, size_t N>
typename BVH<T, N>::ConstIterator BVH<T, N>::ItemOfNode(size_t i) const
{
    if (IsLeaf(i))
    {
        return m_nodes[i].item + begin();
    }

    return end();
}

template <typename T, size_t N>
size_t BVH<T, N>::Build(size_t nodeIndex, size_t* itemIndices, size_t nItems,
                        size_t currentDepth)
{
    // add a node
    m_nodes.Append(Node{});

    // initialize leaf node if termination criteria met
    if (nItems == 1)
    {
        m_nodes[nodeIndex].InitLeaf(itemIndices[0],
                                    m_itemBounds[itemIndices[0]]);
        return currentDepth + 1;
    }

    // find the mid-point of the bounding box to use as a qsplit pivot
    BoundingBox<double, N> nodeBound;
    for (size_t i = 0; i < nItems; ++i)
    {
        nodeBound.Merge(m_itemBounds[itemIndices[i]]);
    }

    Vector<double, N> d = nodeBound.upperCorner - nodeBound.lowerCorner;

    // choose which axis to split along
    auto axis = static_cast<uint8_t>(d.DominantAxis());

    const double pivot =
        0.5 * (nodeBound.upperCorner[axis] + nodeBound.lowerCorner[axis]);

    // classify primitives with respect to split
    const size_t midPoint = QSplit(itemIndices, nItems, pivot, axis);

    // recursively initialize children _nodes
    const size_t d0 =
        Build(nodeIndex + 1, itemIndices, midPoint, currentDepth + 1);
    m_nodes[nodeIndex].InitInternal(axis, m_nodes.Length(), nodeBound);
    const size_t d1 = Build(m_nodes[nodeIndex].child, itemIndices + midPoint,
                            nItems - midPoint, currentDepth + 1);

    return std::max(d0, d1);
}

template <typename T, size_t N>
size_t BVH<T, N>::QSplit(size_t* itemIndices, size_t numItems, double pivot,
                         uint8_t axis)
{
    size_t ret = 0;

    for (size_t i = 0; i < numItems; ++i)
    {
        BoundingBox<double, N> b = m_itemBounds[itemIndices[i]];

        if (const double centroid =
                0.5f * (b.lowerCorner[axis] + b.upperCorner[axis]);
            centroid < pivot)
        {
            std::swap(itemIndices[i], itemIndices[ret]);
            ret++;
        }
    }

    if (ret == 0 || ret == numItems)
    {
        ret = numItems >> 1;
    }

    return ret;
}
}  // namespace CubbyFlow

#endif

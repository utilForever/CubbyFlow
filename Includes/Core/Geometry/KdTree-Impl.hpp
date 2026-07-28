// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_KDTREE_IMPL_HPP
#define CUBBYFLOW_KDTREE_IMPL_HPP

#include <numeric>

namespace CubbyFlow
{
template <typename T, size_t K>
void KdTree<T, K>::Node::InitLeaf(size_t it, const Point& pt)
{
    flags = K;
    item = it;
    child = std::numeric_limits<size_t>::max();
    point = pt;
}

template <typename T, size_t K>
void KdTree<T, K>::Node::InitInternal(size_t axis, size_t it, size_t c,
                                      const Point& pt)
{
    flags = axis;
    item = it;
    child = c;
    point = pt;
}

template <typename T, size_t K>
bool KdTree<T, K>::Node::IsLeaf() const
{
    return flags == K;
}

template <typename T, size_t K>
void KdTree<T, K>::Build(const ConstArrayView1<Point>& points)
{
    m_points.resize(points.Length());
    std::copy(points.begin(), points.end(), m_points.begin());

    if (m_points.empty())
    {
        return;
    }

    m_nodes.clear();

    std::vector<size_t> itemIndices(m_points.size());
    std::iota(std::begin(itemIndices), std::end(itemIndices), 0);

    [[maybe_unused]] const size_t d =
        Build(0, itemIndices.data(), m_points.size(), 0);
}

template <typename T, size_t K>
void KdTree<T, K>::ForEachNearbyPoint(
    const Point& origin, T radius,
    const std::function<void(size_t, const Point&)>& callback) const
{
    const T r2 = radius * radius;

    // prepare to traverse the tree for sphere
    static const int maxTreeDepth = 8 * sizeof(size_t);
    const Node* todo[maxTreeDepth];
    size_t todoPos = 0;

    // traverse the tree nodes for sphere
    const Node* node = m_nodes.data();

    while (node != nullptr)
    {
        if (node->item != std::numeric_limits<size_t>::max() &&
            (node->point - origin).LengthSquared() <= r2)
        {
            callback(node->item, node->point);
        }

        if (node->IsLeaf())
        {
            // grab next node to process from todo stack
            if (todoPos > 0)
            {
                // dequeue
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
            // get node children pointers for sphere
            const Node* firstChild = node + 1;
            const Node* secondChild = &m_nodes[node->child];

            // advance to next child node, possibly enqueue other child
            const size_t axis = node->flags;
            const T plane = node->point[axis];

            if (plane - origin[axis] > radius)
            {
                node = firstChild;
            }
            else if (origin[axis] - plane > radius)
            {
                node = secondChild;
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

template <typename T, size_t K>
bool KdTree<T, K>::HasNearbyPoint(const Point& origin, T radius) const
{
    const T r2 = radius * radius;

    // prepare to traverse the tree for sphere
    static const int maxTreeDepth = 8 * sizeof(size_t);
    const Node* todo[maxTreeDepth];
    size_t todoPos = 0;

    // traverse the tree nodes for sphere
    const Node* node = m_nodes.data();

    while (node != nullptr)
    {
        if (node->item != std::numeric_limits<size_t>::max() &&
            (node->point - origin).LengthSquared() <= r2)
        {
            return true;
        }

        if (node->IsLeaf())
        {
            // grab next node to process from todo stack
            if (todoPos > 0)
            {
                // dequeue
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
            // get node children pointers for sphere
            const Node* firstChild = node + 1;
            const Node* secondChild = &m_nodes[node->child];

            // advance to next child node, possibly enqueue other child
            const size_t axis = node->flags;
            const T plane = node->point[axis];

            if (origin[axis] < plane && plane - origin[axis] > radius)
            {
                node = firstChild;
            }
            else if (origin[axis] > plane && origin[axis] - plane > radius)
            {
                node = secondChild;
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

template <typename T, size_t K>
size_t KdTree<T, K>::GetNearestPoint(const Point& origin) const
{
    // prepare to traverse the tree for sphere
    static const int maxTreeDepth = 8 * sizeof(size_t);
    const Node* todo[maxTreeDepth];
    size_t todoPos = 0;

    // traverse the tree nodes for sphere
    const Node* node = m_nodes.data();
    size_t nearest = 0;
    T minDist2 = (node->point - origin).LengthSquared();

    while (node != nullptr)
    {
        const T newDist2 = (node->point - origin).LengthSquared();
        if (newDist2 <= minDist2)
        {
            nearest = node->item;
            minDist2 = newDist2;
        }

        if (node->IsLeaf())
        {
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
            // get node children pointers for sphere
            const Node* firstChild = node + 1;
            const Node* secondChild = static_cast<Node*>(&m_nodes[node->child]);

            // advance to next child node, possibly enqueue other child
            const size_t axis = node->flags;
            const T plane = node->point[axis];
            const T minDist = std::sqrt(minDist2);

            if (plane - origin[axis] > minDist)
            {
                node = firstChild;
            }
            else if (origin[axis] - plane > minDist)
            {
                node = secondChild;
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

    return nearest;
}

template <typename T, size_t K>
void KdTree<T, K>::Reserve(size_t numPoints, size_t numNodes)
{
    m_points.resize(numPoints);
    m_nodes.resize(numNodes);
}

template <typename T, size_t K>
typename KdTree<T, K>::Iterator KdTree<T, K>::begin()
{
    return m_points.begin();
};

template <typename T, size_t K>
typename KdTree<T, K>::Iterator KdTree<T, K>::end()
{
    return m_points.end();
};

template <typename T, size_t K>
typename KdTree<T, K>::ConstIterator KdTree<T, K>::begin() const
{
    return m_points.begin();
};

template <typename T, size_t K>
typename KdTree<T, K>::ConstIterator KdTree<T, K>::end() const
{
    return m_points.end();
};

template <typename T, size_t K>
typename KdTree<T, K>::NodeIterator KdTree<T, K>::BeginNode()
{
    return m_nodes.begin();
};

template <typename T, size_t K>
typename KdTree<T, K>::NodeIterator KdTree<T, K>::EndNode()
{
    return m_nodes.end();
};

template <typename T, size_t K>
typename KdTree<T, K>::ConstNodeIterator KdTree<T, K>::BeginNode() const
{
    return m_nodes.begin();
};

template <typename T, size_t K>
typename KdTree<T, K>::ConstNodeIterator KdTree<T, K>::EndNode() const
{
    return m_nodes.end();
};

template <typename T, size_t K>
size_t KdTree<T, K>::Build(size_t nodeIndex, size_t* itemIndices, size_t nItems,
                           size_t currentDepth)
{
    // add a node
    m_nodes.emplace_back();

    // initialize leaf node if termination criteria met
    if (nItems == 0)
    {
        m_nodes[nodeIndex].InitLeaf(std::numeric_limits<size_t>::max(), {});
        return currentDepth + 1;
    }
    if (nItems == 1)
    {
        m_nodes[nodeIndex].InitLeaf(itemIndices[0], m_points[itemIndices[0]]);
        return currentDepth + 1;
    }

    // choose which axis to split along
    BBox nodeBound;
    for (size_t i = 0; i < nItems; ++i)
    {
        nodeBound.Merge(m_points[itemIndices[i]]);
    }
    Point d = nodeBound.upperCorner - nodeBound.lowerCorner;
    const size_t axis = static_cast<size_t>(d.DominantAxis());

    // pick mid point
    std::nth_element(itemIndices, itemIndices + nItems / 2,
                     itemIndices + nItems, [this, &axis](size_t a, size_t b) {
                         return m_points[a][axis] < m_points[b][axis];
                     });
    const size_t midPoint = nItems / 2;

    // recursively initialize children nodes
    const size_t d0 =
        Build(nodeIndex + 1, itemIndices, midPoint, currentDepth + 1);
    m_nodes[nodeIndex].InitInternal(axis, itemIndices[midPoint], m_nodes.size(),
                                    m_points[itemIndices[midPoint]]);
    const size_t d1 =
        Build(m_nodes[nodeIndex].child, itemIndices + midPoint + 1,
              nItems - midPoint - 1, currentDepth + 1);

    return std::max(d0, d1);
}
}  // namespace CubbyFlow

#endif

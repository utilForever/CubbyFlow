// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Geometry/ColliderSet.hpp>
#include <Core/Geometry/SurfaceSet.hpp>

#include <limits>

namespace CubbyFlow
{
template <size_t N>
ColliderSet<N>::ColliderSet()
    : ColliderSet{ Array1<std::shared_ptr<Collider<N>>>{} }
{
    // Do nothing
}

template <size_t N>
ColliderSet<N>::ColliderSet(
    const ConstArrayView1<std::shared_ptr<Collider<N>>>& others)
{
    SetSurface(std::make_shared<SurfaceSet<N>>());

    for (const auto& collider : others)
    {
        AddCollider(collider);
    }
}

template <size_t N>
Vector<double, N> ColliderSet<N>::VelocityAt(
    const Vector<double, N>& point) const
{
    size_t closestCollider = std::numeric_limits<size_t>::max();
    double closestDist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < m_colliders.Length(); ++i)
    {
        if (const double dist =
                m_colliders[i]->GetSurface()->ClosestDistance(point);
            dist < closestDist)
        {
            closestDist = dist;
            closestCollider = i;
        }
    }

    if (closestCollider != std::numeric_limits<size_t>::max())
    {
        return m_colliders[closestCollider]->VelocityAt(point);
    }

    return Vector<double, N>{};
}

template <size_t N>
void ColliderSet<N>::AddCollider(const std::shared_ptr<Collider<N>>& collider)
{
    std::shared_ptr<SurfaceSet<N>> surfaceSet =
        std::dynamic_pointer_cast<SurfaceSet<N>>(GetSurface());
    m_colliders.Append(collider);
    surfaceSet->AddSurface(collider->GetSurface());
}

template <size_t N>
size_t ColliderSet<N>::NumberOfColliders() const
{
    return m_colliders.Length();
}

template <size_t N>
std::shared_ptr<Collider<N>> ColliderSet<N>::GetCollider(size_t i) const
{
    return m_colliders[i];
}

template <size_t N>
typename ColliderSet<N>::Builder ColliderSet<N>::GetBuilder()
{
    return Builder{};
}

template <size_t N>
typename ColliderSet<N>::Builder& ColliderSet<N>::Builder::WithColliders(
    const ConstArrayView1<std::shared_ptr<Collider<N>>>& others)
{
    m_colliders = others;
    return *this;
}

template <size_t N>
ColliderSet<N> ColliderSet<N>::Builder::Build() const
{
    return ColliderSet{ m_colliders };
}

template <size_t N>
std::shared_ptr<ColliderSet<N>> ColliderSet<N>::Builder::MakeShared() const
{
    return std::make_shared<ColliderSet>(m_colliders);
}

template class ColliderSet<2>;

template class ColliderSet<3>;
}  // namespace CubbyFlow
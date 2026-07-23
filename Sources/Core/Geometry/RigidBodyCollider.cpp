// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Geometry/RigidBodyCollider.hpp>

namespace CubbyFlow
{
template <size_t N>
RigidBodyCollider<N>::RigidBodyCollider(
    const std::shared_ptr<Surface<N>>& surface)
{
    SetSurface(surface);
}

template <size_t N>
RigidBodyCollider<N>::RigidBodyCollider(
    const std::shared_ptr<Surface<N>>& surface,
    const Vector<double, N>& _linearVelocity,
    const AngularVelocity<N>& _angularVelocity)
    : linearVelocity(_linearVelocity), angularVelocity(_angularVelocity)
{
    SetSurface(surface);
}

template <size_t N>
Vector<double, N> RigidBodyCollider<N>::VelocityAt(
    const Vector<double, N>& point) const
{
    Vector<double, N> r = point - GetSurface()->transform.GetTranslation();
    return linearVelocity + angularVelocity.Cross(r);
}

template <size_t N>
typename RigidBodyCollider<N>::Builder RigidBodyCollider<N>::GetBuilder()
{
    return Builder{};
}

template <size_t N>
typename RigidBodyCollider<N>::Builder&
RigidBodyCollider<N>::Builder::WithSurface(
    const std::shared_ptr<Surface<N>>& surface)
{
    m_surface = surface;
    return *this;
}

template <size_t N>
typename RigidBodyCollider<N>::Builder&
RigidBodyCollider<N>::Builder::WithLinearVelocity(
    const Vector<double, N>& _linearVelocity)
{
    m_linearVelocity = _linearVelocity;
    return *this;
}

template <size_t N>
typename RigidBodyCollider<N>::Builder&
RigidBodyCollider<N>::Builder::WithAngularVelocity(
    const AngularVelocity<N>& _angularVelocity)
{
    m_angularVelocity = _angularVelocity;
    return *this;
}

template <size_t N>
RigidBodyCollider<N> RigidBodyCollider<N>::Builder::Build() const
{
    return RigidBodyCollider{ m_surface, m_linearVelocity, m_angularVelocity };
}

template <size_t N>
std::shared_ptr<RigidBodyCollider<N>>
RigidBodyCollider<N>::Builder::MakeShared() const
{
    return std::make_shared<RigidBodyCollider>(m_surface, m_linearVelocity,
                                               m_angularVelocity);
}

template class RigidBodyCollider<2>;

template class RigidBodyCollider<3>;
}  // namespace CubbyFlow
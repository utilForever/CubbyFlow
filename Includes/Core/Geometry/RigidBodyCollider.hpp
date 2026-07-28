// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_RIGID_BODY_COLLIDER_HPP
#define CUBBYFLOW_RIGID_BODY_COLLIDER_HPP

#include <Core/Geometry/Collider.hpp>

#include <utility>

namespace CubbyFlow
{
template <size_t N>
class AngularVelocity
{
    // Do nothing
};

template <>
class AngularVelocity<2>
{
 public:
    AngularVelocity() = default;

    AngularVelocity(double _value) : value(_value)
    {
        // Do nothing
    }

    [[nodiscard]] Vector2D Cross(const Vector2D& r) const
    {
        return value * Vector2D{ -r.y, r.x };
    }

    double value = 0.0;
};

template <>
class AngularVelocity<3>
{
 public:
    AngularVelocity() = default;

    AngularVelocity(std::initializer_list<double> lst) : value(lst)
    {
        // Do nothing
    }

    AngularVelocity(Vector3D _value) : value(std::move(_value))
    {
        // Do nothing
    }

    [[nodiscard]] Vector3D Cross(const Vector3D& r) const
    {
        return value.Cross(r);
    }

    Vector3D value;
};

using AngularVelocity2 = AngularVelocity<2>;

using AngularVelocity3 = AngularVelocity<3>;

//!
//! \brief N-D rigid body collider class.
//!
//! This class implements N-D rigid body collider. The collider can only take
//! rigid body motion with linear and rotational velocities.
//!
template <size_t N>
class RigidBodyCollider final : public Collider<N>
{
 public:
    class Builder;

    using Collider<N>::GetSurface;
    using Collider<N>::SetSurface;

    //! Constructs a collider with a surface.
    explicit RigidBodyCollider(const std::shared_ptr<Surface<N>>& surface);

    //! Constructs a collider with a surface and other parameters.
    RigidBodyCollider(const std::shared_ptr<Surface<N>>& surface,
                      const Vector<double, N>& _linearVelocity,
                      const AngularVelocity<N>& _angularVelocity);

    //! Returns the velocity of the collider at given \p point.
    [[nodiscard]] Vector<double, N> VelocityAt(
        const Vector<double, N>& point) const override;

    //! Returns builder fox RigidBodyCollider.
    static Builder GetBuilder();

    //! Linear velocity of the rigid body.
    Vector<double, N> linearVelocity;

    //! Angular velocity of the rigid body.
    AngularVelocity<N> angularVelocity;
};

//! 2-D RigidBodyCollider type.
using RigidBodyCollider2 = RigidBodyCollider<2>;

//! 3-D RigidBodyCollider type.
using RigidBodyCollider3 = RigidBodyCollider<3>;

//! Shared pointer for the RigidBodyCollider2 type.
using RigidBodyCollider2Ptr = std::shared_ptr<RigidBodyCollider2>;

//! Shared pointer for the RigidBodyCollider3 type.
using RigidBodyCollider3Ptr = std::shared_ptr<RigidBodyCollider3>;

//!
//! \brief Front-end to create RigidBodyCollider objects step by step.
//!
template <size_t N>
class RigidBodyCollider<N>::Builder final
{
 public:
    //! Returns builder with surface.
    Builder& WithSurface(const std::shared_ptr<Surface<N>>& surface);

    //! Returns builder with linear velocity.
    Builder& WithLinearVelocity(const Vector<double, N>& _linearVelocity);

    //! Returns builder with angular velocity.
    Builder& WithAngularVelocity(const AngularVelocity<N>& _angularVelocity);

    //! Builds RigidBodyCollider.
    RigidBodyCollider Build() const;

    //! Builds shared pointer of RigidBodyCollider instance.
    std::shared_ptr<RigidBodyCollider<N>> MakeShared() const;

 private:
    std::shared_ptr<Surface<N>> m_surface;
    Vector<double, N> m_linearVelocity;
    AngularVelocity<N> m_angularVelocity;
};
}  // namespace CubbyFlow

#endif

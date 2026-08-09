// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Geometry/Collider.hpp>

#include <cassert>

namespace CubbyFlow
{
template <size_t N>
void Collider<N>::ResolveCollision(double radius, double restitutionCoefficient,
                                   Vector<double, N>* newPosition,
                                   Vector<double, N>* newVelocity)
{
    assert(m_surface);

    if (!m_surface->IsValidGeometry())
    {
        return;
    }

    ColliderQueryResult colliderPoint;

    GetClosestPoint(m_surface, *newPosition, &colliderPoint);

    // Check if the new position is penetrating the surface
    if (IsPenetrating(colliderPoint, *newPosition, radius))
    {
        // Target point is the closest non-penetrating position from the
        // new position.
        Vector<double, N> targetNormal = colliderPoint.normal;
        Vector<double, N> targetPoint =
            colliderPoint.point + radius * targetNormal;
        Vector<double, N> colliderVelAtTargetPoint = colliderPoint.velocity;

        // Get new candidate relative velocity from the target point.
        Vector<double, N> relativeVel = *newVelocity - colliderVelAtTargetPoint;
        double normalDotRelativeVel = targetNormal.Dot(relativeVel);
        Vector<double, N> relativeVelN = normalDotRelativeVel * targetNormal;
        Vector<double, N> relativeVelT = relativeVel - relativeVelN;

        // Check if the velocity is facing opposite direction of the surface
        // normal
        if (normalDotRelativeVel < 0.0)
        {
            // Apply restitution coefficient to the surface normal component of
            // the velocity
            Vector<double, N> deltaRelativeVelN =
                (-restitutionCoefficient - 1.0) * relativeVelN;
            relativeVelN *= -restitutionCoefficient;

            // Apply friction to the tangential component of the velocity
            // From Bridson et al., Robust Treatment of Collisions, Contact and
            // Friction for Cloth Animation, 2002
            // http://graphics.stanford.edu/papers/cloth-sig02/cloth.pdf
            if (relativeVelT.LengthSquared() > 0.0)
            {
                double frictionScale = std::max(
                    1.0 - m_frictionCoefficient * deltaRelativeVelN.Length() /
                              relativeVelT.Length(),
                    0.0);
                relativeVelT *= frictionScale;
            }

            // Reassemble the components
            *newVelocity =
                relativeVelN + relativeVelT + colliderVelAtTargetPoint;
        }

        // Geometric fix
        *newPosition = targetPoint;
    }
}

template <size_t N>
double Collider<N>::GetFrictionCoefficient() const
{
    return m_frictionCoefficient;
}

template <size_t N>
void Collider<N>::SetFrictionCoefficient(double newFrictionCoefficient)
{
    m_frictionCoefficient = std::max(newFrictionCoefficient, 0.0);
}

template <size_t N>
const std::shared_ptr<Surface<N>>& Collider<N>::GetSurface() const
{
    return m_surface;
}

template <size_t N>
void Collider<N>::SetSurface(const std::shared_ptr<Surface<N>>& newSurface)
{
    m_surface = newSurface;
}

template <size_t N>
void Collider<N>::GetClosestPoint(const std::shared_ptr<Surface<N>>& surface,
                                  const Vector<double, N>& queryPoint,
                                  ColliderQueryResult* result) const
{
    result->distance = surface->ClosestDistance(queryPoint);
    result->point = surface->ClosestPoint(queryPoint);
    result->normal = surface->ClosestNormal(queryPoint);
    result->velocity = VelocityAt(queryPoint);
}

template <size_t N>
bool Collider<N>::IsPenetrating(const ColliderQueryResult& colliderPoint,
                                const Vector<double, N>& position,
                                double radius)
{
    // If the new candidate position of the particle is on the other side of
    // the surface OR the new distance to the surface is less than or equal to
    // the particle's radius, this particle is in colliding state.
    return (position - colliderPoint.point).Dot(colliderPoint.normal) <= 0.0 ||
           colliderPoint.distance <= radius;
}

template <size_t N>
void Collider<N>::Update(double currentTimeInSeconds,
                         double timeIntervalInSeconds)
{
    assert(m_surface);

    if (!m_surface->IsValidGeometry())
    {
        return;
    }

    m_surface->UpdateQueryEngine();

    if (m_onUpdateCallback)
    {
        m_onUpdateCallback(this, currentTimeInSeconds, timeIntervalInSeconds);
    }
}

template <size_t N>
void Collider<N>::SetOnBeginUpdateCallback(
    const OnBeginUpdateCallback& callback)
{
    m_onUpdateCallback = callback;
}

template class Collider<2>;

template class Collider<3>;
}  // namespace CubbyFlow

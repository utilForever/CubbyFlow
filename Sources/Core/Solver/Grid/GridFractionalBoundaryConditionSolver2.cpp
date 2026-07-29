// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Array/ArrayUtils.hpp>
#include <Core/Geometry/ImplicitSurface.hpp>
#include <Core/Geometry/SurfaceToImplicit.hpp>
#include <Core/Solver/Grid/GridFractionalBoundaryConditionSolver2.hpp>
#include <Core/Utils/LevelSetUtils.hpp>
#include <Core/Utils/PhysicsHelpers.hpp>

namespace CubbyFlow
{
static Vector2D ProjectVelocityToCollider(const Vector2D& point,
                                          const FaceCenteredGrid2& velocity,
                                          const ScalarField2& colliderSDF,
                                          const Collider2& collider)
{
    const Vector2D colliderVelocity = collider.VelocityAt(point);
    const Vector2D gradient = colliderSDF.Gradient(point);
    if (gradient.LengthSquared() == 0.0)
    {
        return colliderVelocity;
    }

    const Vector2D relativeVelocity = velocity.Sample(point) - colliderVelocity;
    const Vector2D normal = gradient.Normalized();
    return ProjectAndApplyFriction(relativeVelocity, normal,
                                   collider.GetFrictionCoefficient()) +
           colliderVelocity;
}

void GridFractionalBoundaryConditionSolver2::ConstrainVelocity(
    FaceCenteredGrid2* velocity, unsigned int extrapolationDepth)
{
    const Vector2UZ size = velocity->Resolution();

    if (m_colliderSDF == nullptr || m_colliderSDF->Resolution() != size)
    {
        UpdateCollider(GetCollider(), size, velocity->GridSpacing(),
                       velocity->Origin());
    }

    ArrayView2<double> u = velocity->UView();
    ArrayView2<double> v = velocity->VView();
    auto uPos = velocity->UPosition();
    auto vPos = velocity->VPosition();

    Array2<double> uTemp{ u.Size() };
    Array2<double> vTemp{ v.Size() };
    Array2<char> uMarker{ u.Size(), 1 };
    Array2<char> vMarker{ v.Size(), 1 };

    Vector2D h = velocity->GridSpacing();

    // Assign collider's velocity first and initialize markers
    velocity->ParallelForEachUIndex(
        [&uPos, this, &h, &uMarker, &u](const Vector2UZ& idx) {
            const Vector2D pt = uPos(idx);
            const double phi0 =
                m_colliderSDF->Sample(pt - Vector2D{ 0.5 * h.x, 0.0 });
            const double phi1 =
                m_colliderSDF->Sample(pt + Vector2D{ 0.5 * h.x, 0.0 });
            double frac = FractionInsideSDF(phi0, phi1);
            frac = 1.0 - std::clamp(frac, 0.0, 1.0);

            if (frac > 0.0)
            {
                uMarker(idx) = 1;
            }
            else
            {
                const Vector2D colliderVel = GetCollider()->VelocityAt(pt);
                u(idx) = colliderVel.x;
                uMarker(idx) = 0;
            }
        });

    velocity->ParallelForEachVIndex(
        [&vPos, this, &h, &vMarker, &v](const Vector2UZ& idx) {
            const Vector2D pt = vPos(idx);
            const double phi0 =
                m_colliderSDF->Sample(pt - Vector2D{ 0.0, 0.5 * h.y });
            const double phi1 =
                m_colliderSDF->Sample(pt + Vector2D{ 0.0, 0.5 * h.y });
            double frac = FractionInsideSDF(phi0, phi1);
            frac = 1.0 - std::clamp(frac, 0.0, 1.0);

            if (frac > 0.0)
            {
                vMarker(idx) = 1;
            }
            else
            {
                const Vector2D colliderVel = GetCollider()->VelocityAt(pt);
                v(idx) = colliderVel.y;
                vMarker(idx) = 0;
            }
        });

    // Free-slip: Extrapolate fluid velocity into the collider
    ExtrapolateToRegion(velocity->UView(), uMarker, extrapolationDepth, u);
    ExtrapolateToRegion(velocity->VView(), vMarker, extrapolationDepth, v);

    // No-flux: project the extrapolated velocity to the collider's surface
    // normal
    velocity->ParallelForEachUIndex(
        [&uPos, this, &velocity, &uTemp, &u](const Vector2UZ& idx) {
            const Vector2D pt = uPos(idx);
            if (!IsInsideSDF(m_colliderSDF->Sample(pt)))
            {
                uTemp(idx) = u(idx);
                return;
            }
            uTemp(idx) = ProjectVelocityToCollider(
                             pt, *velocity, *m_colliderSDF, *GetCollider())
                             .x;
        });

    velocity->ParallelForEachVIndex(
        [&vPos, this, &velocity, &vTemp, &v](const Vector2UZ& idx) {
            const Vector2D pt = vPos(idx);
            if (!IsInsideSDF(m_colliderSDF->Sample(pt)))
            {
                vTemp(idx) = v(idx);
                return;
            }
            vTemp(idx) = ProjectVelocityToCollider(
                             pt, *velocity, *m_colliderSDF, *GetCollider())
                             .y;
        });

    // Transfer results
    ParallelForEachIndex(
        u.Size(), [&u, &uTemp](size_t i, size_t j) { u(i, j) = uTemp(i, j); });
    ParallelForEachIndex(
        v.Size(), [&v, &vTemp](size_t i, size_t j) { v(i, j) = vTemp(i, j); });

    // No-flux: Project velocity on the domain boundary if closed
    if (GetClosedDomainBoundaryFlag() & DIRECTION_LEFT)
    {
        for (size_t j = 0; j < u.Size().y; ++j)
        {
            u(0, j) = 0;
        }
    }
    if (GetClosedDomainBoundaryFlag() & DIRECTION_RIGHT)
    {
        for (size_t j = 0; j < u.Size().y; ++j)
        {
            u(u.Size().x - 1, j) = 0;
        }
    }
    if (GetClosedDomainBoundaryFlag() & DIRECTION_DOWN)
    {
        for (size_t i = 0; i < v.Size().x; ++i)
        {
            v(i, 0) = 0;
        }
    }
    if (GetClosedDomainBoundaryFlag() & DIRECTION_UP)
    {
        for (size_t i = 0; i < v.Size().x; ++i)
        {
            v(i, v.Size().y - 1) = 0;
        }
    }
}

ScalarField2Ptr GridFractionalBoundaryConditionSolver2::GetColliderSDF() const
{
    return m_colliderSDF;
}

VectorField2Ptr
GridFractionalBoundaryConditionSolver2::GetColliderVelocityField() const
{
    return m_colliderVel;
}

void GridFractionalBoundaryConditionSolver2::OnColliderUpdated(
    const Vector2UZ& gridSize, const Vector2D& gridSpacing,
    const Vector2D& gridOrigin)
{
    if (m_colliderSDF == nullptr)
    {
        m_colliderSDF = std::make_shared<CellCenteredScalarGrid2>();
    }

    m_colliderSDF->Resize(gridSize, gridSpacing, gridOrigin);

    if (GetCollider() != nullptr)
    {
        Surface2Ptr surface = GetCollider()->GetSurface();
        ImplicitSurface2Ptr implicitSurface =
            std::dynamic_pointer_cast<ImplicitSurface2>(surface);
        if (implicitSurface == nullptr)
        {
            implicitSurface = std::make_shared<SurfaceToImplicit2>(surface);
        }

        m_colliderSDF->Fill([&implicitSurface](const Vector2D& pt) {
            return implicitSurface->SignedDistance(pt);
        });

        m_colliderVel = CustomVectorField2::Builder{}
                            .WithFunction([this](const Vector2D& x) {
                                return GetCollider()->VelocityAt(x);
                            })
                            .WithDerivativeResolution(gridSpacing.x)
                            .MakeShared();
    }
    else
    {
        m_colliderSDF->Fill(std::numeric_limits<double>::max());

        m_colliderVel =
            CustomVectorField2::Builder{}
                .WithFunction([](const Vector2D&) { return Vector2D{}; })
                .WithDerivativeResolution(gridSpacing.x)
                .MakeShared();
    }
}
}  // namespace CubbyFlow

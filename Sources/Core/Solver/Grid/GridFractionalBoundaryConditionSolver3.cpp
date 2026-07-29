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
#include <Core/Solver/Grid/GridFractionalBoundaryConditionSolver3.hpp>
#include <Core/Utils/LevelSetUtils.hpp>
#include <Core/Utils/PhysicsHelpers.hpp>

namespace CubbyFlow
{
static Vector3D ProjectVelocityToCollider(const Vector3D& point,
                                          const FaceCenteredGrid3& velocity,
                                          const ScalarField3& colliderSDF,
                                          const Collider3& collider)
{
    const Vector3D colliderVelocity = collider.VelocityAt(point);
    const Vector3D gradient = colliderSDF.Gradient(point);

    if (gradient.LengthSquared() == 0.0)
    {
        return colliderVelocity;
    }

    const Vector3D relativeVelocity = velocity.Sample(point) - colliderVelocity;
    const Vector3D normal = gradient.Normalized();

    return ProjectAndApplyFriction(relativeVelocity, normal,
                                   collider.GetFrictionCoefficient()) +
           colliderVelocity;
}

void GridFractionalBoundaryConditionSolver3::ConstrainVelocity(
    FaceCenteredGrid3* velocity, unsigned int extrapolationDepth)
{
    Vector3UZ size = velocity->Resolution();

    if (m_colliderSDF == nullptr || m_colliderSDF->Resolution() != size)
    {
        UpdateCollider(GetCollider(), size, velocity->GridSpacing(),
                       velocity->Origin());
    }

    ArrayView3<double> u = velocity->UView();
    ArrayView3<double> v = velocity->VView();
    ArrayView3<double> w = velocity->WView();
    auto uPos = velocity->UPosition();
    auto vPos = velocity->VPosition();
    auto wPos = velocity->WPosition();

    Array3<double> uTemp{ u.Size() };
    Array3<double> vTemp{ v.Size() };
    Array3<double> wTemp{ w.Size() };
    Array3<char> uMarker{ u.Size(), 1 };
    Array3<char> vMarker{ v.Size(), 1 };
    Array3<char> wMarker{ w.Size(), 1 };

    Vector3D h = velocity->GridSpacing();

    // Assign collider's velocity first and initialize markers
    velocity->ParallelForEachUIndex(
        [&uPos, this, &h, &uMarker, &u](const Vector3UZ& idx) {
            const Vector3D pt = uPos(idx);
            const double phi0 =
                m_colliderSDF->Sample(pt - Vector3D{ 0.5 * h.x, 0.0, 0.0 });
            const double phi1 =
                m_colliderSDF->Sample(pt + Vector3D{ 0.5 * h.x, 0.0, 0.0 });
            double frac = FractionInsideSDF(phi0, phi1);
            frac = 1.0 - std::clamp(frac, 0.0, 1.0);

            if (frac > 0.0)
            {
                uMarker(idx) = 1;
            }
            else
            {
                const Vector3D colliderVel = GetCollider()->VelocityAt(pt);
                u(idx) = colliderVel.x;
                uMarker(idx) = 0;
            }
        });

    velocity->ParallelForEachVIndex(
        [&vPos, this, &h, &vMarker, &v](const Vector3UZ& idx) {
            const Vector3D pt = vPos(idx);
            const double phi0 =
                m_colliderSDF->Sample(pt - Vector3D{ 0.0, 0.5 * h.y, 0.0 });
            const double phi1 =
                m_colliderSDF->Sample(pt + Vector3D{ 0.0, 0.5 * h.y, 0.0 });
            double frac = FractionInsideSDF(phi0, phi1);
            frac = 1.0 - std::clamp(frac, 0.0, 1.0);

            if (frac > 0.0)
            {
                vMarker(idx) = 1;
            }
            else
            {
                const Vector3D colliderVel = GetCollider()->VelocityAt(pt);
                v(idx) = colliderVel.y;
                vMarker(idx) = 0;
            }
        });

    velocity->ParallelForEachWIndex(
        [&wPos, this, &h, &wMarker, &w](const Vector3UZ& idx) {
            const Vector3D pt = wPos(idx);
            const double phi0 =
                m_colliderSDF->Sample(pt - Vector3D{ 0.0, 0.0, 0.5 * h.z });
            const double phi1 =
                m_colliderSDF->Sample(pt + Vector3D{ 0.0, 0.0, 0.5 * h.z });
            double frac = FractionInsideSDF(phi0, phi1);
            frac = 1.0 - std::clamp(frac, 0.0, 1.0);

            if (frac > 0.0)
            {
                wMarker(idx) = 1;
            }
            else
            {
                const Vector3D colliderVel = GetCollider()->VelocityAt(pt);
                w(idx) = colliderVel.z;
                wMarker(idx) = 0;
            }
        });

    // Free-slip: Extrapolate fluid velocity into the collider
    ExtrapolateToRegion(velocity->UView(), uMarker, extrapolationDepth, u);
    ExtrapolateToRegion(velocity->VView(), vMarker, extrapolationDepth, v);
    ExtrapolateToRegion(velocity->WView(), wMarker, extrapolationDepth, w);

    // No-flux: project the extrapolated velocity to the collider's surface
    // normal
    velocity->ParallelForEachUIndex(
        [&uPos, this, &velocity, &uTemp, &u](const Vector3UZ& idx) {
            const Vector3D pt = uPos(idx);
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
        [&vPos, this, &velocity, &vTemp, &v](const Vector3UZ& idx) {
            const Vector3D pt = vPos(idx);
            if (!IsInsideSDF(m_colliderSDF->Sample(pt)))
            {
                vTemp(idx) = v(idx);
                return;
            }
            vTemp(idx) = ProjectVelocityToCollider(
                             pt, *velocity, *m_colliderSDF, *GetCollider())
                             .y;
        });

    velocity->ParallelForEachWIndex(
        [&wPos, this, &velocity, &wTemp, &w](const Vector3UZ& idx) {
            const Vector3D pt = wPos(idx);
            if (!IsInsideSDF(m_colliderSDF->Sample(pt)))
            {
                wTemp(idx) = w(idx);
                return;
            }
            wTemp(idx) = ProjectVelocityToCollider(
                             pt, *velocity, *m_colliderSDF, *GetCollider())
                             .z;
        });

    // Transfer results
    ParallelForEachIndex(u.Size(), [&u, &uTemp](size_t i, size_t j, size_t k) {
        u(i, j, k) = uTemp(i, j, k);
    });
    ParallelForEachIndex(v.Size(), [&v, &vTemp](size_t i, size_t j, size_t k) {
        v(i, j, k) = vTemp(i, j, k);
    });
    ParallelForEachIndex(w.Size(), [&w, &wTemp](size_t i, size_t j, size_t k) {
        w(i, j, k) = wTemp(i, j, k);
    });

    // No-flux: Project velocity on the domain boundary if closed
    if (GetClosedDomainBoundaryFlag() & DIRECTION_LEFT)
    {
        for (size_t k = 0; k < u.Size().z; ++k)
        {
            for (size_t j = 0; j < u.Size().y; ++j)
            {
                u(0, j, k) = 0;
            }
        }
    }
    if (GetClosedDomainBoundaryFlag() & DIRECTION_RIGHT)
    {
        for (size_t k = 0; k < u.Size().z; ++k)
        {
            for (size_t j = 0; j < u.Size().y; ++j)
            {
                u(u.Size().x - 1, j, k) = 0;
            }
        }
    }
    if (GetClosedDomainBoundaryFlag() & DIRECTION_DOWN)
    {
        for (size_t k = 0; k < v.Size().z; ++k)
        {
            for (size_t i = 0; i < v.Size().x; ++i)
            {
                v(i, 0, k) = 0;
            }
        }
    }
    if (GetClosedDomainBoundaryFlag() & DIRECTION_UP)
    {
        for (size_t k = 0; k < v.Size().z; ++k)
        {
            for (size_t i = 0; i < v.Size().x; ++i)
            {
                v(i, v.Size().y - 1, k) = 0;
            }
        }
    }
    if (GetClosedDomainBoundaryFlag() & DIRECTION_BACK)
    {
        for (size_t j = 0; j < w.Size().y; ++j)
        {
            for (size_t i = 0; i < w.Size().x; ++i)
            {
                w(i, j, 0) = 0;
            }
        }
    }
    if (GetClosedDomainBoundaryFlag() & DIRECTION_FRONT)
    {
        for (size_t j = 0; j < w.Size().y; ++j)
        {
            for (size_t i = 0; i < w.Size().x; ++i)
            {
                w(i, j, w.Size().z - 1) = 0;
            }
        }
    }
}

ScalarField3Ptr GridFractionalBoundaryConditionSolver3::GetColliderSDF() const
{
    return m_colliderSDF;
}

VectorField3Ptr
GridFractionalBoundaryConditionSolver3::GetColliderVelocityField() const
{
    return m_colliderVel;
}

void GridFractionalBoundaryConditionSolver3::OnColliderUpdated(
    const Vector3UZ& gridSize, const Vector3D& gridSpacing,
    const Vector3D& gridOrigin)
{
    if (m_colliderSDF == nullptr)
    {
        m_colliderSDF = std::make_shared<CellCenteredScalarGrid3>();
    }

    m_colliderSDF->Resize(gridSize, gridSpacing, gridOrigin);

    if (GetCollider() != nullptr)
    {
        Surface3Ptr surface = GetCollider()->GetSurface();
        ImplicitSurface3Ptr implicitSurface =
            std::dynamic_pointer_cast<ImplicitSurface3>(surface);
        if (implicitSurface == nullptr)
        {
            implicitSurface = std::make_shared<SurfaceToImplicit3>(surface);
        }

        m_colliderSDF->Fill([&implicitSurface](const Vector3D& pt) {
            return implicitSurface->SignedDistance(pt);
        });

        m_colliderVel = CustomVectorField3::Builder{}
                            .WithFunction([this](const Vector3D& x) {
                                return GetCollider()->VelocityAt(x);
                            })
                            .WithDerivativeResolution(gridSpacing.x)
                            .MakeShared();
    }
    else
    {
        m_colliderSDF->Fill(std::numeric_limits<double>::max());

        m_colliderVel =
            CustomVectorField3::Builder{}
                .WithFunction([](const Vector3D&) { return Vector3D{}; })
                .WithDerivativeResolution(gridSpacing.x)
                .MakeShared();
    }
}
}  // namespace CubbyFlow

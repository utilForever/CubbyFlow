// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Solver/Grid/GridBlockedBoundaryConditionSolver3.hpp>
#include <Core/Utils/LevelSetUtils.hpp>

namespace CubbyFlow
{
static const char FLUID = 1;
static const char COLLIDER = 0;

void GridBlockedBoundaryConditionSolver3::ConstrainVelocity(
    FaceCenteredGrid3* velocity, unsigned int extrapolationDepth)
{
    GridFractionalBoundaryConditionSolver3::ConstrainVelocity(
        velocity, extrapolationDepth);

    // No-flux: project the velocity at the marker interface
    Vector3UZ size = velocity->Resolution();
    ArrayView3<double> u = velocity->UView();
    ArrayView3<double> v = velocity->VView();
    ArrayView3<double> w = velocity->WView();
    GridDataPositionFunc<3> uPos = velocity->UPosition();
    GridDataPositionFunc<3> vPos = velocity->VPosition();
    GridDataPositionFunc<3> wPos = velocity->WPosition();

    ForEachIndex(m_marker.Size(), [this, &uPos, &u, &size, &vPos, &v, &wPos,
                                   &w](size_t i, size_t j, size_t k) {
        if (m_marker(i, j, k) == COLLIDER)
        {
            if (i > 0 && m_marker(i - 1, j, k) == FLUID)
            {
                const Vector3D colliderVel =
                    GetCollider()->VelocityAt(uPos(i, j, k));
                u(i, j, k) = colliderVel.x;
            }
            if (i < size.x - 1 && m_marker(i + 1, j, k) == FLUID)
            {
                const Vector3D colliderVel =
                    GetCollider()->VelocityAt(uPos(i + 1, j, k));
                u(i + 1, j, k) = colliderVel.x;
            }
            if (j > 0 && m_marker(i, j - 1, k) == FLUID)
            {
                const Vector3D colliderVel =
                    GetCollider()->VelocityAt(vPos(i, j, k));
                v(i, j, k) = colliderVel.y;
            }
            if (j < size.y - 1 && m_marker(i, j + 1, k) == FLUID)
            {
                const Vector3D colliderVel =
                    GetCollider()->VelocityAt(vPos(i, j + 1, k));
                v(i, j + 1, k) = colliderVel.y;
            }
            if (k > 0 && m_marker(i, j, k - 1) == FLUID)
            {
                const Vector3D colliderVel =
                    GetCollider()->VelocityAt(wPos(i, j, k));
                w(i, j, k) = colliderVel.z;
            }
            if (k < size.z - 1 && m_marker(i, j, k + 1) == FLUID)
            {
                const Vector3D colliderVel =
                    GetCollider()->VelocityAt(wPos(i, j, k + 1));
                w(i, j, k + 1) = colliderVel.z;
            }
        }
    });
}

const Array3<char>& GridBlockedBoundaryConditionSolver3::GetMarker() const
{
    return m_marker;
}

void GridBlockedBoundaryConditionSolver3::OnColliderUpdated(
    const Vector3UZ& gridSize, const Vector3D& gridSpacing,
    const Vector3D& gridOrigin)
{
    GridFractionalBoundaryConditionSolver3::OnColliderUpdated(
        gridSize, gridSpacing, gridOrigin);

    const auto sdf =
        std::dynamic_pointer_cast<CellCenteredScalarGrid3>(GetColliderSDF());

    m_marker.Resize(gridSize);
    ParallelForEachIndex(m_marker.Size(),
                         [&sdf, this](size_t i, size_t j, size_t k) {
                             if (IsInsideSDF((*sdf)(i, j, k)))
                             {
                                 m_marker(i, j, k) = COLLIDER;
                             }
                             else
                             {
                                 m_marker(i, j, k) = FLUID;
                             }
                         });
}
}  // namespace CubbyFlow
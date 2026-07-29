// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Solver/Grid/GridBlockedBoundaryConditionSolver2.hpp>
#include <Core/Utils/LevelSetUtils.hpp>

namespace CubbyFlow
{
static const char FLUID = 1;
static const char COLLIDER = 0;

namespace
{
struct VelocityConstraintData2
{
    const Array2<char>& marker;
    const Vector2UZ& size;
    const GridDataPositionFunc<2>& uPos;
    const GridDataPositionFunc<2>& vPos;
    const Collider2& collider;
    ArrayView2<double>& u;
    ArrayView2<double>& v;
};

void ConstrainU(size_t i, size_t j, const VelocityConstraintData2& data)
{
    if (i > 0 && data.marker(i - 1, j) == FLUID)
    {
        data.u(i, j) = data.collider.VelocityAt(data.uPos(i, j)).x;
    }

    if (i < data.size.x - 1 && data.marker(i + 1, j) == FLUID)
    {
        data.u(i + 1, j) = data.collider.VelocityAt(data.uPos(i + 1, j)).x;
    }
}

void ConstrainV(size_t i, size_t j, const VelocityConstraintData2& data)
{
    if (j > 0 && data.marker(i, j - 1) == FLUID)
    {
        data.v(i, j) = data.collider.VelocityAt(data.vPos(i, j)).y;
    }

    if (j < data.size.y - 1 && data.marker(i, j + 1) == FLUID)
    {
        data.v(i, j + 1) = data.collider.VelocityAt(data.vPos(i, j + 1)).y;
    }
}

void ConstrainVelocityAt(size_t i, size_t j,
                         const VelocityConstraintData2& data)
{
    if (data.marker(i, j) != COLLIDER)
    {
        return;
    }

    ConstrainU(i, j, data);
    ConstrainV(i, j, data);
}
}  // namespace

void GridBlockedBoundaryConditionSolver2::ConstrainVelocity(
    FaceCenteredGrid2* velocity, unsigned int extrapolationDepth)
{
    GridFractionalBoundaryConditionSolver2::ConstrainVelocity(
        velocity, extrapolationDepth);

    // No-flux: project the velocity at the marker interface
    Vector2UZ size = velocity->Resolution();
    ArrayView2<double> u = velocity->UView();
    ArrayView2<double> v = velocity->VView();
    GridDataPositionFunc<2> uPos = velocity->UPosition();
    GridDataPositionFunc<2> vPos = velocity->VPosition();
    const VelocityConstraintData2 data{ m_marker,       size, uPos, vPos,
                                        *GetCollider(), u,    v };

    ForEachIndex(m_marker.Size(), [&data](size_t i, size_t j) {
        ConstrainVelocityAt(i, j, data);
    });
}

const Array2<char>& GridBlockedBoundaryConditionSolver2::GetMarker() const
{
    return m_marker;
}

void GridBlockedBoundaryConditionSolver2::OnColliderUpdated(
    const Vector2UZ& gridSize, const Vector2D& gridSpacing,
    const Vector2D& gridOrigin)
{
    GridFractionalBoundaryConditionSolver2::OnColliderUpdated(
        gridSize, gridSpacing, gridOrigin);

    const auto sdf =
        std::dynamic_pointer_cast<CellCenteredScalarGrid2>(GetColliderSDF());

    m_marker.Resize(gridSize);
    ParallelForEachIndex(m_marker.Size(), [&sdf, this](size_t i, size_t j) {
        if (IsInsideSDF((*sdf)(i, j)))
        {
            m_marker(i, j) = COLLIDER;
        }
        else
        {
            m_marker(i, j) = FLUID;
        }
    });
}
}  // namespace CubbyFlow

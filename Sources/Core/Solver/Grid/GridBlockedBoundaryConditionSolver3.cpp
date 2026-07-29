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

namespace
{
struct VelocityConstraintData3
{
    const Array3<char>& marker;
    const Vector3UZ& size;
    const GridDataPositionFunc<3>& uPos;
    const GridDataPositionFunc<3>& vPos;
    const GridDataPositionFunc<3>& wPos;
    const Collider3& collider;
    ArrayView3<double>& u;
    ArrayView3<double>& v;
    ArrayView3<double>& w;
};

void ConstrainU(size_t i, size_t j, size_t k,
                const VelocityConstraintData3& data)
{
    if (i > 0 && data.marker(i - 1, j, k) == FLUID)
    {
        data.u(i, j, k) = data.collider.VelocityAt(data.uPos(i, j, k)).x;
    }

    if (i < data.size.x - 1 && data.marker(i + 1, j, k) == FLUID)
    {
        data.u(i + 1, j, k) =
            data.collider.VelocityAt(data.uPos(i + 1, j, k)).x;
    }
}

void ConstrainV(size_t i, size_t j, size_t k,
                const VelocityConstraintData3& data)
{
    if (j > 0 && data.marker(i, j - 1, k) == FLUID)
    {
        data.v(i, j, k) = data.collider.VelocityAt(data.vPos(i, j, k)).y;
    }

    if (j < data.size.y - 1 && data.marker(i, j + 1, k) == FLUID)
    {
        data.v(i, j + 1, k) =
            data.collider.VelocityAt(data.vPos(i, j + 1, k)).y;
    }
}

void ConstrainW(size_t i, size_t j, size_t k,
                const VelocityConstraintData3& data)
{
    if (k > 0 && data.marker(i, j, k - 1) == FLUID)
    {
        data.w(i, j, k) = data.collider.VelocityAt(data.wPos(i, j, k)).z;
    }

    if (k < data.size.z - 1 && data.marker(i, j, k + 1) == FLUID)
    {
        data.w(i, j, k + 1) =
            data.collider.VelocityAt(data.wPos(i, j, k + 1)).z;
    }
}

void ConstrainVelocityAt(size_t i, size_t j, size_t k,
                         const VelocityConstraintData3& data)
{
    if (data.marker(i, j, k) != COLLIDER)
    {
        return;
    }

    ConstrainU(i, j, k, data);
    ConstrainV(i, j, k, data);
    ConstrainW(i, j, k, data);
}
}  // namespace

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
    const VelocityConstraintData3 data{ m_marker,       size, uPos, vPos, wPos,
                                        *GetCollider(), u,    v,    w };

    ForEachIndex(m_marker.Size(), [&data](size_t i, size_t j, size_t k) {
        ConstrainVelocityAt(i, j, k, data);
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

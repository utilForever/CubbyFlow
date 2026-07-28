// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Solver/FDM/FDMICCGSolver3.hpp>
#include <Core/Solver/Grid/GridBackwardEulerDiffusionSolver3.hpp>
#include <Core/Utils/LevelSetUtils.hpp>

namespace CubbyFlow
{
const char FLUID = 0;
const char AIR = 1;
const char BOUNDARY = 2;

GridBackwardEulerDiffusionSolver3::GridBackwardEulerDiffusionSolver3(
    BoundaryType boundaryType)
    : m_boundaryType(boundaryType)
{
    m_systemSolver = std::make_shared<FDMICCGSolver3>(
        100, std::numeric_limits<double>::epsilon());
}

void GridBackwardEulerDiffusionSolver3::Solve(const ScalarGrid3& source,
                                              double diffusionCoefficient,
                                              double timeIntervalInSeconds,
                                              ScalarGrid3* dest,
                                              const ScalarField3& boundarySDF,
                                              const ScalarField3& fluidSDF)
{
    const GridDataPositionFunc<3> pos = source.DataPosition();
    const Vector3D& h = source.GridSpacing();
    const Vector3D c =
        timeIntervalInSeconds * diffusionCoefficient / ElemMul(h, h);

    BuildMarkers(source.DataSize(), pos, boundarySDF, fluidSDF);
    BuildMatrix(source.DataSize(), c);
    BuildVectors(source.DataView(), c);

    if (m_systemSolver != nullptr)
    {
        // Solve the system
        m_systemSolver->Solve(&m_system);

        // Assign the solution
        source.ParallelForEachDataPointIndex(
            [&dest, this](size_t i, size_t j, size_t k) {
                (*dest)(i, j, k) = m_system.x(i, j, k);
            });
    }
}

void GridBackwardEulerDiffusionSolver3::Solve(
    const CollocatedVectorGrid3& source, double diffusionCoefficient,
    double timeIntervalInSeconds, CollocatedVectorGrid3* dest,
    const ScalarField3& boundarySDF, const ScalarField3& fluidSDF)
{
    const GridDataPositionFunc<3> pos = source.DataPosition();
    const Vector3D& h = source.GridSpacing();
    const Vector3D c =
        timeIntervalInSeconds * diffusionCoefficient / ElemMul(h, h);

    BuildMarkers(source.DataSize(), pos, boundarySDF, fluidSDF);
    BuildMatrix(source.DataSize(), c);

    // u
    BuildVectors(source.DataView(), c, 0);

    if (m_systemSolver != nullptr)
    {
        // Solve the system
        m_systemSolver->Solve(&m_system);

        // Assign the solution
        source.ParallelForEachDataPointIndex(
            [&dest, this](const Vector3UZ& idx) {
                (*dest)(idx).x = m_system.x(idx);
            });
    }

    // v
    BuildVectors(source.DataView(), c, 1);

    if (m_systemSolver != nullptr)
    {
        // Solve the system
        m_systemSolver->Solve(&m_system);

        // Assign the solution
        source.ParallelForEachDataPointIndex(
            [&dest, this](const Vector3UZ& idx) {
                (*dest)(idx).y = m_system.x(idx);
            });
    }

    // w
    BuildVectors(source.DataView(), c, 2);

    if (m_systemSolver != nullptr)
    {
        // Solve the system
        m_systemSolver->Solve(&m_system);

        // Assign the solution
        source.ParallelForEachDataPointIndex(
            [&dest, this](const Vector3UZ& idx) {
                (*dest)(idx).z = m_system.x(idx);
            });
    }
}

void GridBackwardEulerDiffusionSolver3::Solve(const FaceCenteredGrid3& source,
                                              double diffusionCoefficient,
                                              double timeIntervalInSeconds,
                                              FaceCenteredGrid3* dest,
                                              const ScalarField3& boundarySDF,
                                              const ScalarField3& fluidSDF)
{
    const Vector3D& h = source.GridSpacing();
    const Vector3D c =
        timeIntervalInSeconds * diffusionCoefficient / ElemMul(h, h);

    // u
    const auto uPos = source.UPosition();
    BuildMarkers(source.USize(), uPos, boundarySDF, fluidSDF);
    BuildMatrix(source.USize(), c);
    BuildVectors(source.UView(), c);

    if (m_systemSolver != nullptr)
    {
        // Solve the system
        m_systemSolver->Solve(&m_system);

        // Assign the solution
        source.ParallelForEachUIndex([&dest, this](const Vector3UZ& idx) {
            dest->U(idx) = m_system.x(idx);
        });
    }

    // v
    const auto vPos = source.VPosition();
    BuildMarkers(source.VSize(), vPos, boundarySDF, fluidSDF);
    BuildMatrix(source.VSize(), c);
    BuildVectors(source.VView(), c);

    if (m_systemSolver != nullptr)
    {
        // Solve the system
        m_systemSolver->Solve(&m_system);

        // Assign the solution
        source.ParallelForEachVIndex([&dest, this](const Vector3UZ& idx) {
            dest->V(idx) = m_system.x(idx);
        });
    }

    // w
    const auto wPos = source.WPosition();
    BuildMarkers(source.WSize(), wPos, boundarySDF, fluidSDF);
    BuildMatrix(source.WSize(), c);
    BuildVectors(source.WView(), c);

    if (m_systemSolver != nullptr)
    {
        // Solve the system
        m_systemSolver->Solve(&m_system);

        // Assign the solution
        source.ParallelForEachWIndex([&dest, this](const Vector3UZ& idx) {
            dest->W(idx) = m_system.x(idx);
        });
    }
}

void GridBackwardEulerDiffusionSolver3::SetLinearSystemSolver(
    const FDMLinearSystemSolver3Ptr& Solver)
{
    m_systemSolver = Solver;
}

void GridBackwardEulerDiffusionSolver3::BuildMarkers(
    const Vector3UZ& size,
    const std::function<Vector3D(size_t, size_t, size_t)>& pos,
    const ScalarField3& boundarySDF, const ScalarField3& fluidSDF)
{
    m_markers.Resize(size);

    ParallelForEachIndex(
        m_markers.Size(),
        [&boundarySDF, &pos, this, &fluidSDF](size_t i, size_t j, size_t k) {
            if (IsInsideSDF(boundarySDF.Sample(pos(i, j, k))))
            {
                m_markers(i, j, k) = BOUNDARY;
            }
            else if (IsInsideSDF(fluidSDF.Sample(pos(i, j, k))))
            {
                m_markers(i, j, k) = FLUID;
            }
            else
            {
                m_markers(i, j, k) = AIR;
            }
        });
}

void GridBackwardEulerDiffusionSolver3::BuildMatrix(const Vector3UZ& size,
                                                    const Vector3D& c)
{
    m_system.A.Resize(size);

    bool isBoundaryType = (m_boundaryType == BoundaryType::Dirichlet);

    // Build linear system
    ParallelForEachIndex(m_system.A.Size(), [this, &size, &isBoundaryType, &c](
                                                size_t i, size_t j, size_t k) {
        FDMMatrixRow3& row = m_system.A(i, j, k);

        // Initialize
        row.center = 1.0;
        row.right = row.up = row.front = 0.0;

        if (m_markers(i, j, k) == FLUID)
        {
            if (i + 1 < size.x)
            {
                if ((isBoundaryType && m_markers(i + 1, j, k) != AIR) ||
                    m_markers(i + 1, j, k) == FLUID)
                {
                    row.center += c.x;
                }

                if (m_markers(i + 1, j, k) == FLUID)
                {
                    row.right -= c.x;
                }
            }

            if (i > 0 && ((isBoundaryType && m_markers(i - 1, j, k) != AIR) ||
                          m_markers(i - 1, j, k) == FLUID))
            {
                row.center += c.x;
            }

            if (j + 1 < size.y)
            {
                if ((isBoundaryType && m_markers(i, j + 1, k) != AIR) ||
                    m_markers(i, j + 1, k) == FLUID)
                {
                    row.center += c.y;
                }

                if (m_markers(i, j + 1, k) == FLUID)
                {
                    row.up -= c.y;
                }
            }

            if (j > 0 && ((isBoundaryType && m_markers(i, j - 1, k) != AIR) ||
                          m_markers(i, j - 1, k) == FLUID))
            {
                row.center += c.y;
            }

            if (k + 1 < size.z)
            {
                if ((isBoundaryType && m_markers(i, j, k + 1) != AIR) ||
                    m_markers(i, j, k + 1) == FLUID)
                {
                    row.center += c.z;
                }

                if (m_markers(i, j, k + 1) == FLUID)
                {
                    row.front -= c.z;
                }
            }

            if (k > 0 && ((isBoundaryType && m_markers(i, j, k - 1) != AIR) ||
                          m_markers(i, j, k - 1) == FLUID))
            {
                row.center += c.z;
            }
        }
    });
}

void GridBackwardEulerDiffusionSolver3::BuildVectors(
    const ConstArrayView3<double>& f, const Vector3D& c)
{
    Vector3UZ size = f.Size();

    m_system.x.Resize(size, 0.0);
    m_system.b.Resize(size, 0.0);

    // Build linear system
    ParallelForEachIndex(
        m_system.x.Size(), [this, &f, &size, &c](size_t i, size_t j, size_t k) {
            m_system.b(i, j, k) = m_system.x(i, j, k) = f(i, j, k);

            if (m_boundaryType == BoundaryType::Dirichlet &&
                m_markers(i, j, k) == FLUID)
            {
                if (i + 1 < size.x && m_markers(i + 1, j, k) == BOUNDARY)
                {
                    m_system.b(i, j, k) += c.x * f(i + 1, j, k);
                }

                if (i > 0 && m_markers(i - 1, j, k) == BOUNDARY)
                {
                    m_system.b(i, j, k) += c.x * f(i - 1, j, k);
                }

                if (j + 1 < size.y && m_markers(i, j + 1, k) == BOUNDARY)
                {
                    m_system.b(i, j, k) += c.y * f(i, j + 1, k);
                }

                if (j > 0 && m_markers(i, j - 1, k) == BOUNDARY)
                {
                    m_system.b(i, j, k) += c.y * f(i, j - 1, k);
                }

                if (k + 1 < size.z && m_markers(i, j, k + 1) == BOUNDARY)
                {
                    m_system.b(i, j, k) += c.z * f(i, j, k + 1);
                }

                if (k > 0 && m_markers(i, j, k - 1) == BOUNDARY)
                {
                    m_system.b(i, j, k) += c.z * f(i, j, k - 1);
                }
            }
        });
}

void GridBackwardEulerDiffusionSolver3::BuildVectors(
    const ConstArrayView3<Vector3D>& f, const Vector3D& c, size_t component)
{
    Vector3UZ size = f.Size();

    m_system.x.Resize(size, 0.0);
    m_system.b.Resize(size, 0.0);

    // Build linear system
    ParallelForEachIndex(m_system.x.Size(), [this, &f, &component, &size, &c](
                                                size_t i, size_t j, size_t k) {
        m_system.b(i, j, k) = m_system.x(i, j, k) = f(i, j, k)[component];

        if (m_boundaryType == BoundaryType::Dirichlet &&
            m_markers(i, j, k) == FLUID)
        {
            if (i + 1 < size.x && m_markers(i + 1, j, k) == BOUNDARY)
            {
                m_system.b(i, j, k) += c.x * f(i + 1, j, k)[component];
            }

            if (i > 0 && m_markers(i - 1, j, k) == BOUNDARY)
            {
                m_system.b(i, j, k) += c.x * f(i - 1, j, k)[component];
            }

            if (j + 1 < size.y && m_markers(i, j + 1, k) == BOUNDARY)
            {
                m_system.b(i, j, k) += c.y * f(i, j + 1, k)[component];
            }

            if (j > 0 && m_markers(i, j - 1, k) == BOUNDARY)
            {
                m_system.b(i, j, k) += c.y * f(i, j - 1, k)[component];
            }

            if (k + 1 < size.z && m_markers(i, j, k + 1) == BOUNDARY)
            {
                m_system.b(i, j, k) += c.z * f(i, j, k + 1)[component];
            }

            if (k > 0 && m_markers(i, j, k - 1) == BOUNDARY)
            {
                m_system.b(i, j, k) += c.z * f(i, j, k - 1)[component];
            }
        }
    });
}
}  // namespace CubbyFlow
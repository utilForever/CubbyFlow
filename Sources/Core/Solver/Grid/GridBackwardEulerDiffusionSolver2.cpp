// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Solver/FDM/FDMICCGSolver2.hpp>
#include <Core/Solver/Grid/GridBackwardEulerDiffusionSolver2.hpp>
#include <Core/Utils/LevelSetUtils.hpp>

namespace CubbyFlow
{
const char FLUID = 0;
const char AIR = 1;
const char BOUNDARY = 2;

GridBackwardEulerDiffusionSolver2::GridBackwardEulerDiffusionSolver2(
    BoundaryType boundaryType)
    : m_boundaryType(boundaryType)
{
    m_systemSolver = std::make_shared<FDMICCGSolver2>(
        100, std::numeric_limits<double>::epsilon());
}

void GridBackwardEulerDiffusionSolver2::Solve(const ScalarGrid2& source,
                                              double diffusionCoefficient,
                                              double timeIntervalInSeconds,
                                              ScalarGrid2* dest,
                                              const ScalarField2& boundarySDF,
                                              const ScalarField2& fluidSDF)
{
    if (m_systemSolver != nullptr)
    {
        const GridDataPositionFunc<2> pos = source.DataPosition();
        const Vector2D& h = source.GridSpacing();
        const Vector2D c =
            timeIntervalInSeconds * diffusionCoefficient / ElemMul(h, h);

        BuildMarkers(source.DataSize(), pos, boundarySDF, fluidSDF);
        BuildMatrix(source.DataSize(), c);
        BuildVectors(source.DataView(), c);

        // Solve the system
        m_systemSolver->Solve(&m_system);

        // Assign the solution
        source.ParallelForEachDataPointIndex(
            [&dest, this](const Vector2UZ& idx) {
                (*dest)(idx) = m_system.x(idx);
            });
    }
}

void GridBackwardEulerDiffusionSolver2::Solve(
    const CollocatedVectorGrid2& source, double diffusionCoefficient,
    double timeIntervalInSeconds, CollocatedVectorGrid2* dest,
    const ScalarField2& boundarySDF, const ScalarField2& fluidSDF)
{
    if (m_systemSolver != nullptr)
    {
        const GridDataPositionFunc<2> pos = source.DataPosition();
        const Vector2D& h = source.GridSpacing();
        const Vector2D c =
            timeIntervalInSeconds * diffusionCoefficient / ElemMul(h, h);

        BuildMarkers(source.DataSize(), pos, boundarySDF, fluidSDF);
        BuildMatrix(source.DataSize(), c);

        // u
        BuildVectors(source.DataView(), c, 0);

        // Solve the system
        m_systemSolver->Solve(&m_system);

        // Assign the solution
        source.ParallelForEachDataPointIndex(
            [&dest, this](const Vector2UZ& idx) {
                (*dest)(idx).x = m_system.x(idx);
            });

        // v
        BuildVectors(source.DataView(), c, 1);

        // Solve the system
        m_systemSolver->Solve(&m_system);

        // Assign the solution
        source.ParallelForEachDataPointIndex(
            [&dest, this](const Vector2UZ& idx) {
                (*dest)(idx).y = m_system.x(idx);
            });
    }
}

void GridBackwardEulerDiffusionSolver2::Solve(const FaceCenteredGrid2& source,
                                              double diffusionCoefficient,
                                              double timeIntervalInSeconds,
                                              FaceCenteredGrid2* dest,
                                              const ScalarField2& boundarySDF,
                                              const ScalarField2& fluidSDF)
{
    if (m_systemSolver != nullptr)
    {
        const Vector2D& h = source.GridSpacing();
        const Vector2D c =
            timeIntervalInSeconds * diffusionCoefficient / ElemMul(h, h);

        // u
        const auto uPos = source.UPosition();
        BuildMarkers(source.USize(), uPos, boundarySDF, fluidSDF);
        BuildMatrix(source.USize(), c);
        BuildVectors(source.UView(), c);

        // Solve the system
        m_systemSolver->Solve(&m_system);

        // Assign the solution
        source.ParallelForEachUIndex([&dest, this](const Vector2UZ& idx) {
            dest->U(idx) = m_system.x(idx);
        });

        // v
        const auto vPos = source.VPosition();
        BuildMarkers(source.VSize(), vPos, boundarySDF, fluidSDF);
        BuildMatrix(source.VSize(), c);
        BuildVectors(source.VView(), c);

        // Solve the system
        m_systemSolver->Solve(&m_system);

        // Assign the solution
        source.ParallelForEachVIndex([&dest, this](const Vector2UZ& idx) {
            dest->V(idx) = m_system.x(idx);
        });
    }
}

void GridBackwardEulerDiffusionSolver2::SetLinearSystemSolver(
    const FDMLinearSystemSolver2Ptr& Solver)
{
    m_systemSolver = Solver;
}

void GridBackwardEulerDiffusionSolver2::BuildMarkers(
    const Vector2UZ& size, const std::function<Vector2D(size_t, size_t)>& pos,
    const ScalarField2& boundarySDF, const ScalarField2& fluidSDF)
{
    m_markers.Resize(size);

    ParallelForEachIndex(m_markers.Size(), [&boundarySDF, &pos, this,
                                            &fluidSDF](size_t i, size_t j) {
        if (IsInsideSDF(boundarySDF.Sample(pos(i, j))))
        {
            m_markers(i, j) = BOUNDARY;
        }
        else if (IsInsideSDF(fluidSDF.Sample(pos(i, j))))
        {
            m_markers(i, j) = FLUID;
        }
        else
        {
            m_markers(i, j) = AIR;
        }
    });
}

void GridBackwardEulerDiffusionSolver2::BuildMatrix(const Vector2UZ& size,
                                                    const Vector2D& c)
{
    m_system.A.Resize(size);

    bool isBoundaryType = (m_boundaryType == BoundaryType::Dirichlet);

    // Build linear system
    ParallelForEachIndex(m_system.A.Size(), [this, &size, &isBoundaryType, &c](
                                                size_t i, size_t j) {
        FDMMatrixRow2& row = m_system.A(i, j);

        // Initialize
        row.center = 1.0;
        row.right = row.up = 0.0;

        if (m_markers(i, j) == FLUID)
        {
            if (i + 1 < size.x)
            {
                if ((isBoundaryType && m_markers(i + 1, j) != AIR) ||
                    m_markers(i + 1, j) == FLUID)
                {
                    row.center += c.x;
                }

                if (m_markers(i + 1, j) == FLUID)
                {
                    row.right -= c.x;
                }
            }

            if (i > 0 && ((isBoundaryType && m_markers(i - 1, j) != AIR) ||
                          m_markers(i - 1, j) == FLUID))
            {
                row.center += c.x;
            }

            if (j + 1 < size.y)
            {
                if ((isBoundaryType && m_markers(i, j + 1) != AIR) ||
                    m_markers(i, j + 1) == FLUID)
                {
                    row.center += c.y;
                }

                if (m_markers(i, j + 1) == FLUID)
                {
                    row.up -= c.y;
                }
            }

            if (j > 0 && ((isBoundaryType && m_markers(i, j - 1) != AIR) ||
                          m_markers(i, j - 1) == FLUID))
            {
                row.center += c.y;
            }
        }
    });
}

void GridBackwardEulerDiffusionSolver2::BuildVectors(
    const ConstArrayView2<double>& f, const Vector2D& c)
{
    Vector2UZ size = f.Size();

    m_system.x.Resize(size, 0.0);
    m_system.b.Resize(size, 0.0);

    // Build linear system
    ParallelForEachIndex(
        m_system.x.Size(), [this, &f, &size, &c](size_t i, size_t j) {
            m_system.b(i, j) = m_system.x(i, j) = f(i, j);

            if (m_boundaryType == BoundaryType::Dirichlet &&
                m_markers(i, j) == FLUID)
            {
                if (i + 1 < size.x && m_markers(i + 1, j) == BOUNDARY)
                {
                    m_system.b(i, j) += c.x * f(i + 1, j);
                }

                if (i > 0 && m_markers(i - 1, j) == BOUNDARY)
                {
                    m_system.b(i, j) += c.x * f(i - 1, j);
                }

                if (j + 1 < size.y && m_markers(i, j + 1) == BOUNDARY)
                {
                    m_system.b(i, j) += c.y * f(i, j + 1);
                }

                if (j > 0 && m_markers(i, j - 1) == BOUNDARY)
                {
                    m_system.b(i, j) += c.y * f(i, j - 1);
                }
            }
        });
}

void GridBackwardEulerDiffusionSolver2::BuildVectors(
    const ConstArrayView2<Vector2D>& f, const Vector2D& c, size_t component)
{
    Vector2UZ size = f.Size();

    m_system.x.Resize(size, 0.0);
    m_system.b.Resize(size, 0.0);

    // Build linear system
    ParallelForEachIndex(m_system.x.Size(), [this, &f, &component, &size, &c](
                                                size_t i, size_t j) {
        m_system.b(i, j) = m_system.x(i, j) = f(i, j)[component];

        if (m_boundaryType == BoundaryType::Dirichlet &&
            m_markers(i, j) == FLUID)
        {
            if (i + 1 < size.x && m_markers(i + 1, j) == BOUNDARY)
            {
                m_system.b(i, j) += c.x * f(i + 1, j)[component];
            }

            if (i > 0 && m_markers(i - 1, j) == BOUNDARY)
            {
                m_system.b(i, j) += c.x * f(i - 1, j)[component];
            }

            if (j + 1 < size.y && m_markers(i, j + 1) == BOUNDARY)
            {
                m_system.b(i, j) += c.y * f(i, j + 1)[component];
            }

            if (j > 0 && m_markers(i, j - 1) == BOUNDARY)
            {
                m_system.b(i, j) += c.y * f(i, j - 1)[component];
            }
        }
    });
}
}  // namespace CubbyFlow
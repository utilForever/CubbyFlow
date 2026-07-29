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

static FDMMatrixRow2 BuildMatrixRow(size_t i, size_t j, const Vector2UZ& size,
                                    const Vector2D& c,
                                    const Array2<char>& markers,
                                    bool isDirichlet)
{
    FDMMatrixRow2 row;
    row.center = 1.0;
    row.right = row.up = 0.0;

    if (markers(i, j) != FLUID)
    {
        return row;
    }

    if (i + 1 < size.x)
    {
        if ((isDirichlet && markers(i + 1, j) != AIR) ||
            markers(i + 1, j) == FLUID)
        {
            row.center += c.x;
        }

        if (markers(i + 1, j) == FLUID)
        {
            row.right -= c.x;
        }
    }

    if (i > 0 && ((isDirichlet && markers(i - 1, j) != AIR) ||
                  markers(i - 1, j) == FLUID))
    {
        row.center += c.x;
    }

    if (j + 1 < size.y)
    {
        if ((isDirichlet && markers(i, j + 1) != AIR) ||
            markers(i, j + 1) == FLUID)
        {
            row.center += c.y;
        }

        if (markers(i, j + 1) == FLUID)
        {
            row.up -= c.y;
        }
    }

    if (j > 0 && ((isDirichlet && markers(i, j - 1) != AIR) ||
                  markers(i, j - 1) == FLUID))
    {
        row.center += c.y;
    }

    return row;
}

template <typename ValueAt>
static double BuildRHS(size_t i, size_t j, const Vector2UZ& size,
                       const Vector2D& c, const Array2<char>& markers,
                       bool isDirichlet, const ValueAt& valueAt)
{
    double result = valueAt(i, j);

    if (!isDirichlet || markers(i, j) != FLUID)
    {
        return result;
    }

    if (i + 1 < size.x && markers(i + 1, j) == BOUNDARY)
    {
        result += c.x * valueAt(i + 1, j);
    }

    if (i > 0 && markers(i - 1, j) == BOUNDARY)
    {
        result += c.x * valueAt(i - 1, j);
    }

    if (j + 1 < size.y && markers(i, j + 1) == BOUNDARY)
    {
        result += c.y * valueAt(i, j + 1);
    }

    if (j > 0 && markers(i, j - 1) == BOUNDARY)
    {
        result += c.y * valueAt(i, j - 1);
    }

    return result;
}

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

template <typename PositionFunc>
void GridBackwardEulerDiffusionSolver2::BuildMarkers(
    const Vector2UZ& size, const PositionFunc& pos,
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
        m_system.A(i, j) =
            BuildMatrixRow(i, j, size, c, m_markers, isBoundaryType);
    });
}

void GridBackwardEulerDiffusionSolver2::BuildVectors(
    const ConstArrayView2<double>& f, const Vector2D& c)
{
    Vector2UZ size = f.Size();

    m_system.x.Resize(size, 0.0);
    m_system.b.Resize(size, 0.0);

    // Build linear system
    ParallelForEachIndex(m_system.x.Size(), [this, &f, &size, &c](size_t i,
                                                                  size_t j) {
        m_system.x(i, j) = f(i, j);
        m_system.b(i, j) = BuildRHS(
            i, j, size, c, m_markers, m_boundaryType == BoundaryType::Dirichlet,
            [&f](size_t x, size_t y) { return f(x, y); });
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
        m_system.x(i, j) = f(i, j)[component];
        m_system.b(i, j) = BuildRHS(
            i, j, size, c, m_markers, m_boundaryType == BoundaryType::Dirichlet,
            [&f, component](size_t x, size_t y) { return f(x, y)[component]; });
    });
}
}  // namespace CubbyFlow

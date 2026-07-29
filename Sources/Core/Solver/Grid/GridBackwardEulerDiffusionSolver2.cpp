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

namespace
{
struct MatrixRowData2
{
    const Vector2UZ& size;
    const Vector2D& c;
    const Array2<char>& markers;
    bool isDirichlet;
};

bool AddsCenter(char marker, bool isDirichlet)
{
    return marker == FLUID || (isDirichlet && marker != AIR);
}

void AddXTerms(size_t i, size_t j, const MatrixRowData2& data,
               FDMMatrixRow2* row)
{
    if (i + 1 < data.size.x)
    {
        const char marker = data.markers(i + 1, j);

        if (AddsCenter(marker, data.isDirichlet))
        {
            row->center += data.c.x;
        }

        if (marker == FLUID)
        {
            row->right -= data.c.x;
        }
    }

    if (i > 0 && AddsCenter(data.markers(i - 1, j), data.isDirichlet))
    {
        row->center += data.c.x;
    }
}

void AddYTerms(size_t i, size_t j, const MatrixRowData2& data,
               FDMMatrixRow2* row)
{
    if (j + 1 < data.size.y)
    {
        const char marker = data.markers(i, j + 1);

        if (AddsCenter(marker, data.isDirichlet))
        {
            row->center += data.c.y;
        }

        if (marker == FLUID)
        {
            row->up -= data.c.y;
        }
    }

    if (j > 0 && AddsCenter(data.markers(i, j - 1), data.isDirichlet))
    {
        row->center += data.c.y;
    }
}

FDMMatrixRow2 BuildMatrixRow(size_t i, size_t j, const MatrixRowData2& data)
{
    FDMMatrixRow2 row{};
    row.center = 1.0;

    if (data.markers(i, j) != FLUID)
    {
        return row;
    }

    AddXTerms(i, j, data, &row);
    AddYTerms(i, j, data, &row);

    return row;
}

template <typename ValueAt>
struct RHSData2
{
    const Vector2UZ& size;
    const Vector2D& c;
    const Array2<char>& markers;
    bool isDirichlet;
    const ValueAt& valueAt;
};

template <typename ValueAt>
double BoundaryContribution(size_t i, size_t j, bool isValid,
                            double coefficient, const RHSData2<ValueAt>& data)
{
    if (!isValid || data.markers(i, j) != BOUNDARY)
    {
        return 0.0;
    }

    return coefficient * data.valueAt(i, j);
}

template <typename ValueAt>
double BuildRHS(size_t i, size_t j, const RHSData2<ValueAt>& data)
{
    double result = data.valueAt(i, j);

    if (!data.isDirichlet || data.markers(i, j) != FLUID)
    {
        return result;
    }

    result +=
        BoundaryContribution(i + 1, j, i + 1 < data.size.x, data.c.x, data);
    result += BoundaryContribution(i > 0 ? i - 1 : i, j, i > 0, data.c.x, data);
    result +=
        BoundaryContribution(i, j + 1, j + 1 < data.size.y, data.c.y, data);
    result += BoundaryContribution(i, j > 0 ? j - 1 : j, j > 0, data.c.y, data);

    return result;
}
}  // namespace

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
    const MatrixRowData2 data{ size, c, m_markers, isBoundaryType };

    // Build linear system
    ParallelForEachIndex(m_system.A.Size(), [this, &data](size_t i, size_t j) {
        m_system.A(i, j) = BuildMatrixRow(i, j, data);
    });
}

void GridBackwardEulerDiffusionSolver2::BuildVectors(
    const ConstArrayView2<double>& f, const Vector2D& c)
{
    Vector2UZ size = f.Size();

    m_system.x.Resize(size, 0.0);
    m_system.b.Resize(size, 0.0);

    const auto valueAt = [&f](size_t x, size_t y) { return f(x, y); };
    const RHSData2<decltype(valueAt)> data{
        size, c, m_markers, m_boundaryType == BoundaryType::Dirichlet, valueAt
    };

    // Build linear system
    ParallelForEachIndex(m_system.x.Size(),
                         [this, &f, &data](size_t i, size_t j) {
                             m_system.x(i, j) = f(i, j);
                             m_system.b(i, j) = BuildRHS(i, j, data);
                         });
}

void GridBackwardEulerDiffusionSolver2::BuildVectors(
    const ConstArrayView2<Vector2D>& f, const Vector2D& c, size_t component)
{
    Vector2UZ size = f.Size();

    m_system.x.Resize(size, 0.0);
    m_system.b.Resize(size, 0.0);

    const auto valueAt = [&f, component](size_t x, size_t y) {
        return f(x, y)[component];
    };
    const RHSData2<decltype(valueAt)> data{
        size, c, m_markers, m_boundaryType == BoundaryType::Dirichlet, valueAt
    };

    // Build linear system
    ParallelForEachIndex(m_system.x.Size(),
                         [this, &f, component, &data](size_t i, size_t j) {
                             m_system.x(i, j) = f(i, j)[component];
                             m_system.b(i, j) = BuildRHS(i, j, data);
                         });
}
}  // namespace CubbyFlow

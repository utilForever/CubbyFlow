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
#include <Core/Solver/Grid/GridBlockedBoundaryConditionSolver2.hpp>
#include <Core/Solver/Grid/GridSinglePhasePressureSolver2.hpp>
#include <Core/Utils/LevelSetUtils.hpp>

#include <array>

namespace CubbyFlow
{
const char FLUID = 0;
const char AIR = 1;
const char BOUNDARY = 2;

const double DEFAULT_TOLERANCE = 1e-6;

namespace
{
struct SingleRowData2
{
    const Vector2UZ& size;
    const Vector2D& invHSqr;
    const Array2<char>& markers;
    const FaceCenteredGrid2& input;
};

void BuildSingleRow(size_t i, size_t j, const SingleRowData2& data,
                    FDMMatrixRow2* row, double* rhs)
{
    const auto& [size, invHSqr, markers, input] = data;
    *row = {};
    *rhs = 0.0;

    if (markers(i, j) != FLUID)
    {
        row->center = 1.0;
        return;
    }

    *rhs = input.DivergenceAtCellCenter(i, j);

    const auto addNeighbor = [&](bool valid, size_t x, size_t y, double weight,
                                 double* offDiagonal) {
        if (!valid || markers(x, y) == BOUNDARY)
        {
            return;
        }

        row->center += weight;

        if (offDiagonal != nullptr && markers(x, y) == FLUID)
        {
            *offDiagonal -= weight;
        }
    };

    addNeighbor(i + 1 < size.x, i + 1, j, invHSqr.x, &row->right);
    addNeighbor(i > 0, i - 1, j, invHSqr.x, nullptr);
    addNeighbor(j + 1 < size.y, i, j + 1, invHSqr.y, &row->up);
    addNeighbor(j > 0, i, j - 1, invHSqr.y, nullptr);
}

void BuildCompressedRow(size_t i, size_t j, const Vector2UZ& size,
                        const Vector2D& invHSqr,
                        const ConstArrayView2<char>& markers,
                        const Array2<size_t>& coordToIndex, MatrixCSRD* A,
                        VectorND* b, const FaceCenteredGrid2& input)
{
    const size_t center = markers.Index(i, j);

    if (markers[center] != FLUID)
    {
        return;
    }

    b->AddElement(input.DivergenceAtCellCenter(i, j));

    std::vector<double> row(1, 0.0);
    std::vector<size_t> columns(1, coordToIndex[center]);

    const auto addNeighbor = [&](bool valid, size_t x, size_t y,
                                 double weight) {
        if (!valid || markers(x, y) == BOUNDARY)
        {
            return;
        }

        row[0] += weight;

        const size_t neighbor = markers.Index(x, y);

        if (markers[neighbor] == FLUID)
        {
            row.push_back(-weight);
            columns.push_back(coordToIndex[neighbor]);
        }
    };

    addNeighbor(i + 1 < size.x, i + 1, j, invHSqr.x);
    addNeighbor(i > 0, i - 1, j, invHSqr.x);
    addNeighbor(j + 1 < size.y, i, j + 1, invHSqr.y);
    addNeighbor(j > 0, i, j - 1, invHSqr.y);
    A->AddRow(row, columns);
}

char CoarsenMarker(size_t i, size_t j, const Vector2UZ& size,
                   const Array2<char>& finer)
{
    const std::array<size_t, 4> iIndices{ (i > 0) ? 2 * i - 1 : 2 * i, 2 * i,
                                          2 * i + 1,
                                          (i + 1 < size.x) ? 2 * i + 2
                                                           : 2 * i + 1 };
    const std::array<size_t, 4> jIndices{ (j > 0) ? 2 * j - 1 : 2 * j, 2 * j,
                                          2 * j + 1,
                                          (j + 1 < size.y) ? 2 * j + 2
                                                           : 2 * j + 1 };
    std::array<int, 3> counts{};

    for (size_t y : jIndices)
    {
        for (size_t x : iIndices)
        {
            ++counts[static_cast<int>(finer(x, y))];
        }
    }

    return static_cast<char>(ArgMax3(counts[0], counts[1], counts[2]));
}

void BuildSingleSystem(FDMMatrix2* A, FDMVector2* b,
                       const Array2<char>& markers,
                       const FaceCenteredGrid2& input)
{
    Vector2UZ size = input.Resolution();
    const Vector2D invH = 1.0 / input.GridSpacing();
    Vector2D invHSqr = ElemMul(invH, invH);
    const SingleRowData2 data{ size, invHSqr, markers, input };

    ParallelForEachIndex(A->Size(), [&A, &b, &data](size_t i, size_t j) {
        BuildSingleRow(i, j, data, &(*A)(i, j), &(*b)(i, j));
    });
}

void BuildSingleSystem(MatrixCSRD* A, VectorND* x, VectorND* b,
                       const Array2<char>& markers,
                       const FaceCenteredGrid2& input)
{
    Vector2UZ size = input.Resolution();
    const Vector2D invH = 1.0 / input.GridSpacing();
    Vector2D invHSqr = ElemMul(invH, invH);

    ConstArrayView2<char> markerAcc{ markers };

    A->Clear();
    b->Clear();

    size_t numRows = 0;
    Array2<size_t> coordToIndex{ size };
    ForEachIndex(markers.Size(),
                 [&markerAcc, &coordToIndex, &numRows](size_t i, size_t j) {
                     const size_t cIdx = markerAcc.Index(i, j);

                     if (markerAcc[cIdx] == FLUID)
                     {
                         coordToIndex[cIdx] = numRows++;
                     }
                 });

    ForEachIndex(markers.Size(), [&markerAcc, &b, &input, &coordToIndex, &size,
                                  &invHSqr, &A](size_t i, size_t j) {
        BuildCompressedRow(i, j, size, invHSqr, markerAcc, coordToIndex, A, b,
                           input);
    });

    x->Resize(b->GetRows(), 0.0);
}
}  // namespace

GridSinglePhasePressureSolver2::GridSinglePhasePressureSolver2()
{
    m_systemSolver = std::make_shared<FDMICCGSolver2>(100, DEFAULT_TOLERANCE);
}

void GridSinglePhasePressureSolver2::Solve(const FaceCenteredGrid2& input,
                                           double timeIntervalInSeconds,
                                           FaceCenteredGrid2* output,
                                           const ScalarField2& boundarySDF,
                                           const VectorField2& boundaryVelocity,
                                           const ScalarField2& fluidSDF,
                                           bool useCompressed)
{
    UNUSED_VARIABLE(timeIntervalInSeconds);
    UNUSED_VARIABLE(boundaryVelocity);

    const GridDataPositionFunc<2> pos = input.CellCenterPosition();

    BuildMarkers(input.Resolution(), pos, boundarySDF, fluidSDF);
    BuildSystem(input, useCompressed);

    if (m_systemSolver != nullptr)
    {
        // Solve the system
        if (m_mgSystemSolver == nullptr)
        {
            if (useCompressed)
            {
                m_system.Clear();
                m_systemSolver->SolveCompressed(&m_compSystem);
                DecompressSolution();
            }
            else
            {
                m_compSystem.Clear();
                m_systemSolver->Solve(&m_system);
            }
        }
        else
        {
            m_mgSystemSolver->Solve(&m_mgSystem);
        }

        // Apply pressure gradient
        ApplyPressureGradient(input, output);
    }
}

GridBoundaryConditionSolver2Ptr
GridSinglePhasePressureSolver2::SuggestedBoundaryConditionSolver() const
{
    return std::make_shared<GridBlockedBoundaryConditionSolver2>();
}

const FDMLinearSystemSolver2Ptr&
GridSinglePhasePressureSolver2::GetLinearSystemSolver() const
{
    return m_systemSolver;
}

void GridSinglePhasePressureSolver2::SetLinearSystemSolver(
    const FDMLinearSystemSolver2Ptr& solver)
{
    m_systemSolver = solver;
    m_mgSystemSolver = std::dynamic_pointer_cast<FDMMGSolver2>(m_systemSolver);

    if (m_mgSystemSolver == nullptr)
    {
        // In case of non-mg system, use flat structure.
        m_mgSystem.Clear();
    }
    else
    {
        // In case of mg system, use multi-level structure.
        m_system.Clear();
        m_compSystem.Clear();
    }
}

const FDMVector2& GridSinglePhasePressureSolver2::GetPressure() const
{
    if (m_mgSystemSolver == nullptr)
    {
        return m_system.x;
    }

    return m_mgSystem.x.levels.front();
}

template <typename PositionFunc>
void GridSinglePhasePressureSolver2::BuildMarkers(
    const Vector2UZ& size, const PositionFunc& pos,
    const ScalarField2& boundarySDF, const ScalarField2& fluidSDF)
{
    // Build levels
    size_t maxLevels = 1;
    if (m_mgSystemSolver != nullptr)
    {
        maxLevels = m_mgSystemSolver->GetParams().maxNumberOfLevels;
    }
    FDMMGUtils2::ResizeArrayWithFinest(size, maxLevels, &m_markers);

    // Build top-level markers
    ParallelForEachIndex(m_markers[0].Size(), [&pos, &boundarySDF, this,
                                               &fluidSDF](size_t i, size_t j) {
        const Vector2D pt = pos(i, j);

        if (IsInsideSDF(boundarySDF.Sample(pt)))
        {
            m_markers[0](i, j) = BOUNDARY;
        }
        else if (IsInsideSDF(fluidSDF.Sample(pt)))
        {
            m_markers[0](i, j) = FLUID;
        }
        else
        {
            m_markers[0](i, j) = AIR;
        }
    });

    // Build sub-level markers
    for (size_t l = 1; l < m_markers.size(); ++l)
    {
        const Array2<char>& finer = m_markers[l - 1];
        Array2<char>& coarser = m_markers[l];
        const Vector2UZ n = coarser.Size();

        ParallelRangeFor(ZERO_SIZE, n.x, ZERO_SIZE, n.y,
                         [&n, &finer, &coarser](size_t iBegin, size_t iEnd,
                                                size_t jBegin, size_t jEnd) {
                             for (size_t j = jBegin; j < jEnd; ++j)
                             {
                                 for (size_t i = iBegin; i < iEnd; ++i)
                                 {
                                     coarser(i, j) =
                                         CoarsenMarker(i, j, n, finer);
                                 }
                             }
                         });
    }
}

void GridSinglePhasePressureSolver2::DecompressSolution()
{
    ConstArrayView2<char> acc{ m_markers[0] };
    m_system.x.Resize(acc.Size());

    size_t row = 0;
    ForEachIndex(m_markers[0].Size(), [&acc, this, &row](size_t i, size_t j) {
        if (acc(i, j) == FLUID)
        {
            m_system.x(i, j) = m_compSystem.x[row];
            ++row;
        }
    });
}

void GridSinglePhasePressureSolver2::BuildSystem(const FaceCenteredGrid2& input,
                                                 bool useCompressed)
{
    const Vector2UZ size = input.Resolution();
    size_t numLevels = 1;

    if (m_mgSystemSolver == nullptr)
    {
        if (!useCompressed)
        {
            m_system.Resize(size);
        }
    }
    else
    {
        // Build levels
        const size_t maxLevels =
            m_mgSystemSolver->GetParams().maxNumberOfLevels;
        FDMMGUtils2::ResizeArrayWithFinest(size, maxLevels,
                                           &m_mgSystem.A.levels);
        FDMMGUtils2::ResizeArrayWithFinest(size, maxLevels,
                                           &m_mgSystem.x.levels);
        FDMMGUtils2::ResizeArrayWithFinest(size, maxLevels,
                                           &m_mgSystem.b.levels);

        numLevels = m_mgSystem.A.levels.size();
    }

    // Build top level
    const FaceCenteredGrid2* finer = &input;
    if (m_mgSystemSolver == nullptr)
    {
        if (useCompressed)
        {
            BuildSingleSystem(&m_compSystem.A, &m_compSystem.x, &m_compSystem.b,
                              m_markers[0], *finer);
        }
        else
        {
            BuildSingleSystem(&m_system.A, &m_system.b, m_markers[0], *finer);
        }
    }
    else
    {
        BuildSingleSystem(&m_mgSystem.A.levels.front(),
                          &m_mgSystem.b.levels.front(), m_markers[0], *finer);
    }

    // Build sub-levels
    FaceCenteredGrid2 coarser;
    for (size_t l = 1; l < numLevels; ++l)
    {
        Vector2UZ res = finer->Resolution();
        Vector2D h = finer->GridSpacing();
        const Vector2D& o = finer->Origin();
        res.x = res.x >> 1;
        res.y = res.y >> 1;
        h *= 2.0;

        // Down sample
        coarser.Resize(res, h, o);
        coarser.Fill(finer->Sampler());

        BuildSingleSystem(&m_mgSystem.A.levels[l], &m_mgSystem.b.levels[l],
                          m_markers[l], coarser);

        finer = &coarser;
    }
}

void GridSinglePhasePressureSolver2::ApplyPressureGradient(
    const FaceCenteredGrid2& input, FaceCenteredGrid2* output)
{
    Vector2UZ size = input.Resolution();
    ConstArrayView2<double> u = input.UView();
    ConstArrayView2<double> v = input.VView();
    ArrayView2<double> u0 = output->UView();
    ArrayView2<double> v0 = output->VView();

    const FDMVector2& x = GetPressure();

    Vector2D invH = 1.0 / input.GridSpacing();

    ParallelForEachIndex(x.Size(), [this, &size, &u0, &u, &invH, &x, &v0, &v](
                                       size_t i, size_t j) {
        if (m_markers[0](i, j) == FLUID)
        {
            if (i + 1 < size.x && m_markers[0](i + 1, j) != BOUNDARY)
            {
                u0(i + 1, j) = u(i + 1, j) + invH.x * (x(i + 1, j) - x(i, j));
            }
            if (j + 1 < size.y && m_markers[0](i, j + 1) != BOUNDARY)
            {
                v0(i, j + 1) = v(i, j + 1) + invH.y * (x(i, j + 1) - x(i, j));
            }
        }
    });
}
}  // namespace CubbyFlow

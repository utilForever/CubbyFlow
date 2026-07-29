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
#include <Core/Solver/Grid/GridBlockedBoundaryConditionSolver3.hpp>
#include <Core/Solver/Grid/GridSinglePhasePressureSolver3.hpp>
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
void BuildSingleRow(size_t i, size_t j, size_t k, const Vector3UZ& size,
                    const Vector3D& invHSqr, const Array3<char>& markers,
                    const FaceCenteredGrid3& input, FDMMatrixRow3* row,
                    double* rhs)
{
    *row = {};
    *rhs = 0.0;

    if (markers(i, j, k) != FLUID)
    {
        row->center = 1.0;
        return;
    }

    *rhs = input.DivergenceAtCellCenter(i, j, k);

    const auto addNeighbor = [&](bool valid, size_t x, size_t y, size_t z,
                                 double weight, double* offDiagonal) {
        if (!valid || markers(x, y, z) == BOUNDARY)
        {
            return;
        }

        row->center += weight;

        if (offDiagonal != nullptr && markers(x, y, z) == FLUID)
        {
            *offDiagonal -= weight;
        }
    };

    addNeighbor(i + 1 < size.x, i + 1, j, k, invHSqr.x, &row->right);
    addNeighbor(i > 0, i - 1, j, k, invHSqr.x, nullptr);
    addNeighbor(j + 1 < size.y, i, j + 1, k, invHSqr.y, &row->up);
    addNeighbor(j > 0, i, j - 1, k, invHSqr.y, nullptr);
    addNeighbor(k + 1 < size.z, i, j, k + 1, invHSqr.z, &row->front);
    addNeighbor(k > 0, i, j, k - 1, invHSqr.z, nullptr);
}

void BuildCompressedRow(size_t i, size_t j, size_t k, const Vector3UZ& size,
                        const Vector3D& invHSqr,
                        const ConstArrayView3<char>& markers,
                        const Array3<size_t>& coordToIndex, MatrixCSRD* A,
                        VectorND* b, const FaceCenteredGrid3& input)
{
    const size_t center = markers.Index(i, j, k);

    if (markers[center] != FLUID)
    {
        return;
    }

    b->AddElement(input.DivergenceAtCellCenter(i, j, k));

    std::vector<double> row(1, 0.0);
    std::vector<size_t> columns(1, coordToIndex[center]);

    const auto addNeighbor = [&](bool valid, size_t x, size_t y, size_t z,
                                 double weight) {
        if (!valid || markers(x, y, z) == BOUNDARY)
        {
            return;
        }

        row[0] += weight;

        const size_t neighbor = markers.Index(x, y, z);

        if (markers[neighbor] == FLUID)
        {
            row.push_back(-weight);
            columns.push_back(coordToIndex[neighbor]);
        }
    };

    addNeighbor(i + 1 < size.x, i + 1, j, k, invHSqr.x);
    addNeighbor(i > 0, i - 1, j, k, invHSqr.x);
    addNeighbor(j + 1 < size.y, i, j + 1, k, invHSqr.y);
    addNeighbor(j > 0, i, j - 1, k, invHSqr.y);
    addNeighbor(k + 1 < size.z, i, j, k + 1, invHSqr.z);
    addNeighbor(k > 0, i, j, k - 1, invHSqr.z);
    A->AddRow(row, columns);
}

char CoarsenMarker(size_t i, size_t j, size_t k, const Vector3UZ& size,
                   const Array3<char>& finer)
{
    const std::array<size_t, 4> iIndices{ (i > 0) ? 2 * i - 1 : 2 * i, 2 * i,
                                          2 * i + 1,
                                          (i + 1 < size.x) ? 2 * i + 2
                                                           : 2 * i + 1 };
    const std::array<size_t, 4> jIndices{ (j > 0) ? 2 * j - 1 : 2 * j, 2 * j,
                                          2 * j + 1,
                                          (j + 1 < size.y) ? 2 * j + 2
                                                           : 2 * j + 1 };
    const std::array<size_t, 4> kIndices{ (k > 0) ? 2 * k - 1 : 2 * k, 2 * k,
                                          2 * k + 1,
                                          (k + 1 < size.z) ? 2 * k + 2
                                                           : 2 * k + 1 };
    std::array<int, 3> counts{};

    for (size_t z : kIndices)
    {
        for (size_t y : jIndices)
        {
            for (size_t x : iIndices)
            {
                ++counts[static_cast<int>(finer(x, y, z))];
            }
        }
    }

    return static_cast<char>(ArgMax3(counts[0], counts[1], counts[2]));
}

void ApplyPressureGradientAt(size_t i, size_t j, size_t k,
                             const Vector3UZ& size, const Array3<char>& markers,
                             const ConstArrayView3<double>& u,
                             const ConstArrayView3<double>& v,
                             const ConstArrayView3<double>& w,
                             ArrayView3<double> u0, ArrayView3<double> v0,
                             ArrayView3<double> w0, const Vector3D& invH,
                             const FDMVector3& pressure)
{
    if (markers(i, j, k) != FLUID)
    {
        return;
    }

    if (i + 1 < size.x && markers(i + 1, j, k) != BOUNDARY)
    {
        u0(i + 1, j, k) = u(i + 1, j, k) +
                          invH.x * (pressure(i + 1, j, k) - pressure(i, j, k));
    }

    if (j + 1 < size.y && markers(i, j + 1, k) != BOUNDARY)
    {
        v0(i, j + 1, k) = v(i, j + 1, k) +
                          invH.y * (pressure(i, j + 1, k) - pressure(i, j, k));
    }

    if (k + 1 < size.z && markers(i, j, k + 1) != BOUNDARY)
    {
        w0(i, j, k + 1) = w(i, j, k + 1) +
                          invH.z * (pressure(i, j, k + 1) - pressure(i, j, k));
    }
}

void BuildSingleSystem(FDMMatrix3* A, FDMVector3* b,
                       const Array3<char>& markers,
                       const FaceCenteredGrid3& input)
{
    Vector3UZ size = input.Resolution();
    const Vector3D invH = 1.0 / input.GridSpacing();
    Vector3D invHSqr = ElemMul(invH, invH);

    // Build linear system
    ParallelForEachIndex(A->Size(), [&A, &b, &markers, &input, &size, &invHSqr](
                                        size_t i, size_t j, size_t k) {
        BuildSingleRow(i, j, k, size, invHSqr, markers, input, &(*A)(i, j, k),
                       &(*b)(i, j, k));
    });
}

void BuildSingleSystem(MatrixCSRD* A, VectorND* x, VectorND* b,
                       const Array3<char>& markers,
                       const FaceCenteredGrid3& input)
{
    Vector3UZ size = input.Resolution();
    const Vector3D invH = 1.0 / input.GridSpacing();
    Vector3D invHSqr = ElemMul(invH, invH);

    ConstArrayView3<char> markerAcc{ markers };

    A->Clear();
    b->Clear();

    size_t numRows = 0;
    Array3<size_t> coordToIndex{ size };
    ForEachIndex(markers.Size(), [&markerAcc, &coordToIndex, &numRows](
                                     size_t i, size_t j, size_t k) {
        const size_t cIdx = markerAcc.Index(i, j, k);

        if (markerAcc[cIdx] == FLUID)
        {
            coordToIndex[cIdx] = numRows++;
        }
    });

    ForEachIndex(markers.Size(), [&markerAcc, &b, &input, &coordToIndex, &size,
                                  &invHSqr, &A](size_t i, size_t j, size_t k) {
        BuildCompressedRow(i, j, k, size, invHSqr, markerAcc, coordToIndex, A,
                           b, input);
    });

    x->Resize(b->GetRows(), 0.0);
}
}  // namespace

GridSinglePhasePressureSolver3::GridSinglePhasePressureSolver3()
{
    m_systemSolver = std::make_shared<FDMICCGSolver3>(100, DEFAULT_TOLERANCE);
}

void GridSinglePhasePressureSolver3::Solve(const FaceCenteredGrid3& input,
                                           double timeIntervalInSeconds,
                                           FaceCenteredGrid3* output,
                                           const ScalarField3& boundarySDF,
                                           const VectorField3& boundaryVelocity,
                                           const ScalarField3& fluidSDF,
                                           bool useCompressed)
{
    UNUSED_VARIABLE(timeIntervalInSeconds);
    UNUSED_VARIABLE(boundaryVelocity);

    const GridDataPositionFunc<3> pos = input.CellCenterPosition();

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

GridBoundaryConditionSolver3Ptr
GridSinglePhasePressureSolver3::SuggestedBoundaryConditionSolver() const
{
    return std::make_shared<GridBlockedBoundaryConditionSolver3>();
}

const FDMLinearSystemSolver3Ptr&
GridSinglePhasePressureSolver3::GetLinearSystemSolver() const
{
    return m_systemSolver;
}

void GridSinglePhasePressureSolver3::SetLinearSystemSolver(
    const FDMLinearSystemSolver3Ptr& solver)
{
    m_systemSolver = solver;
    m_mgSystemSolver = std::dynamic_pointer_cast<FDMMGSolver3>(m_systemSolver);

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

const FDMVector3& GridSinglePhasePressureSolver3::GetPressure() const
{
    if (m_mgSystemSolver == nullptr)
    {
        return m_system.x;
    }

    return m_mgSystem.x.levels.front();
}

template <typename PositionFunc>
void GridSinglePhasePressureSolver3::BuildMarkers(
    const Vector3UZ& size, const PositionFunc& pos,
    const ScalarField3& boundarySDF, const ScalarField3& fluidSDF)
{
    // Build levels
    size_t maxLevels = 1;
    if (m_mgSystemSolver != nullptr)
    {
        maxLevels = m_mgSystemSolver->GetParams().maxNumberOfLevels;
    }
    FDMMGUtils3::ResizeArrayWithFinest(size, maxLevels, &m_markers);

    // Build top-level markers
    ParallelForEachIndex(
        m_markers[0].Size(),
        [&pos, &boundarySDF, this, &fluidSDF](size_t i, size_t j, size_t k) {
            const Vector3D pt = pos(i, j, k);

            if (IsInsideSDF(boundarySDF.Sample(pt)))
            {
                m_markers[0](i, j, k) = BOUNDARY;
            }
            else if (IsInsideSDF(fluidSDF.Sample(pt)))
            {
                m_markers[0](i, j, k) = FLUID;
            }
            else
            {
                m_markers[0](i, j, k) = AIR;
            }
        });

    // Build sub-level markers
    for (size_t l = 1; l < m_markers.size(); ++l)
    {
        const Array3<char>& finer = m_markers[l - 1];
        Array3<char>& coarser = m_markers[l];
        const Vector3UZ n = coarser.Size();

        ParallelRangeFor(
            ZERO_SIZE, n.x, ZERO_SIZE, n.y, ZERO_SIZE, n.z,
            [&n, &finer, &coarser](size_t iBegin, size_t iEnd, size_t jBegin,
                                   size_t jEnd, size_t kBegin, size_t kEnd) {
                for (size_t k = kBegin; k < kEnd; ++k)
                {
                    for (size_t j = jBegin; j < jEnd; ++j)
                    {
                        for (size_t i = iBegin; i < iEnd; ++i)
                        {
                            coarser(i, j, k) = CoarsenMarker(i, j, k, n, finer);
                        }
                    }
                }
            });
    }
}

void GridSinglePhasePressureSolver3::DecompressSolution()
{
    ConstArrayView3<char> acc{ m_markers[0] };
    m_system.x.Resize(acc.Size());

    size_t row = 0;
    ForEachIndex(m_markers[0].Size(),
                 [&acc, this, &row](size_t i, size_t j, size_t k) {
                     if (acc(i, j, k) == FLUID)
                     {
                         m_system.x(i, j, k) = m_compSystem.x[row];
                         ++row;
                     }
                 });
}

void GridSinglePhasePressureSolver3::BuildSystem(const FaceCenteredGrid3& input,
                                                 bool useCompressed)
{
    const Vector3UZ size = input.Resolution();
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
        FDMMGUtils3::ResizeArrayWithFinest(size, maxLevels,
                                           &m_mgSystem.A.levels);
        FDMMGUtils3::ResizeArrayWithFinest(size, maxLevels,
                                           &m_mgSystem.x.levels);
        FDMMGUtils3::ResizeArrayWithFinest(size, maxLevels,
                                           &m_mgSystem.b.levels);

        numLevels = m_mgSystem.A.levels.size();
    }

    // Build top level
    const FaceCenteredGrid3* finer = &input;
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
    FaceCenteredGrid3 coarser;
    for (size_t l = 1; l < numLevels; ++l)
    {
        Vector3UZ res = finer->Resolution();
        Vector3D h = finer->GridSpacing();
        const Vector3D& o = finer->Origin();
        res.x = res.x >> 1;
        res.y = res.y >> 1;
        res.z = res.z >> 1;
        h *= 2.0;

        // Down sample
        coarser.Resize(res, h, o);
        coarser.Fill(finer->Sampler());

        BuildSingleSystem(&m_mgSystem.A.levels[l], &m_mgSystem.b.levels[l],
                          m_markers[l], coarser);

        finer = &coarser;
    }
}

void GridSinglePhasePressureSolver3::ApplyPressureGradient(
    const FaceCenteredGrid3& input, FaceCenteredGrid3* output)
{
    Vector3UZ size = input.Resolution();
    ConstArrayView3<double> u = input.UView();
    ConstArrayView3<double> v = input.VView();
    ConstArrayView3<double> w = input.WView();
    ArrayView3<double> u0 = output->UView();
    ArrayView3<double> v0 = output->VView();
    ArrayView3<double> w0 = output->WView();

    const FDMVector3& x = GetPressure();

    Vector3D invH = 1.0 / input.GridSpacing();

    ParallelForEachIndex(x.Size(), [this, &size, &u0, &u, &invH, &x, &v0, &v,
                                    &w0, &w](size_t i, size_t j, size_t k) {
        ApplyPressureGradientAt(i, j, k, size, m_markers[0], u, v, w, u0, v0,
                                w0, invH, x);
    });
}
}  // namespace CubbyFlow

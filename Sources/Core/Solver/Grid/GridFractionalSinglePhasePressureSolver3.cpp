// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

//
// Adopted the code from:
// http://www.cs.ubc.ca/labs/imager/tr/2007/Batty_VariationalFluids/
// and
// https://github.com/christopherbatty/FluidRigidCoupling2D
//

#include <Core/Solver/FDM/FDMICCGSolver3.hpp>
#include <Core/Solver/Grid/GridFractionalBoundaryConditionSolver3.hpp>
#include <Core/Solver/Grid/GridFractionalSinglePhasePressureSolver3.hpp>
#include <Core/Utils/LevelSetUtils.hpp>

namespace CubbyFlow
{
const double DEFAULT_TOLERANCE = 1e-6;
const double MIN_WEIGHT = 0.01;

namespace
{
std::array<size_t, 4> FineIndices(size_t i, size_t size, int kernelSize)
{
    std::array<size_t, 4> indices{};
    indices[0] = (i > 0) ? 2 * i - 1 : 2 * i;
    indices[1] = 2 * i;
    indices[2] = (kernelSize == 3 && i + 1 >= size) ? 2 * i : 2 * i + 1;
    indices[3] = (i + 1 < size) ? 2 * i + 2 : 2 * i + 1;

    return indices;
}

double RestrictPoint(size_t i, size_t j, size_t k, const Vector3UZ& size,
                     const std::array<int, 3>& kernelSize,
                     const std::array<std::array<double, 4>, 3>& kernels,
                     const Array3<double>& finer)
{
    const auto iIndices = FineIndices(i, size.x, kernelSize[0]);
    const auto jIndices = FineIndices(j, size.y, kernelSize[1]);
    const auto kIndices = FineIndices(k, size.z, kernelSize[2]);
    double sum = 0.0;

    for (int z = 0; z < kernelSize[2]; ++z)
    {
        for (int y = 0; y < kernelSize[1]; ++y)
        {
            for (int x = 0; x < kernelSize[0]; ++x)
            {
                sum += kernels[0][x] * kernels[1][y] * kernels[2][z] *
                       finer(iIndices[x], jIndices[y], kIndices[z]);
            }
        }
    }

    return sum;
}

struct PressureGradientData3
{
    const Vector3UZ& size;
    const Array3<double>& fluidSDF;
    const Array3<double>& uWeights;
    const Array3<double>& vWeights;
    const Array3<double>& wWeights;
    const ConstArrayView3<double>& u;
    const ConstArrayView3<double>& v;
    const ConstArrayView3<double>& w;
    ArrayView3<double>& u0;
    ArrayView3<double>& v0;
    ArrayView3<double>& w0;
    const Vector3D& invH;
    const FDMVector3& pressure;
};

double PressureTheta(double centerPhi, double neighborPhi)
{
    return std::max(FractionInsideSDF(centerPhi, neighborPhi), 0.01);
}

void ApplyUGradient(size_t i, size_t j, size_t k, double centerPhi,
                    const PressureGradientData3& data)
{
    if (i + 1 < data.size.x && data.uWeights(i + 1, j, k) > 0.0 &&
        (IsInsideSDF(centerPhi) || IsInsideSDF(data.fluidSDF(i + 1, j, k))))
    {
        const double theta =
            PressureTheta(centerPhi, data.fluidSDF(i + 1, j, k));
        data.u0(i + 1, j, k) =
            data.u(i + 1, j, k) +
            data.invH.x / theta *
                (data.pressure(i + 1, j, k) - data.pressure(i, j, k));
    }
}

void ApplyVGradient(size_t i, size_t j, size_t k, double centerPhi,
                    const PressureGradientData3& data)
{
    if (j + 1 < data.size.y && data.vWeights(i, j + 1, k) > 0.0 &&
        (IsInsideSDF(centerPhi) || IsInsideSDF(data.fluidSDF(i, j + 1, k))))
    {
        const double theta =
            PressureTheta(centerPhi, data.fluidSDF(i, j + 1, k));
        data.v0(i, j + 1, k) =
            data.v(i, j + 1, k) +
            data.invH.y / theta *
                (data.pressure(i, j + 1, k) - data.pressure(i, j, k));
    }
}

void ApplyWGradient(size_t i, size_t j, size_t k, double centerPhi,
                    const PressureGradientData3& data)
{
    if (k + 1 < data.size.z && data.wWeights(i, j, k + 1) > 0.0 &&
        (IsInsideSDF(centerPhi) || IsInsideSDF(data.fluidSDF(i, j, k + 1))))
    {
        const double theta =
            PressureTheta(centerPhi, data.fluidSDF(i, j, k + 1));
        data.w0(i, j, k + 1) =
            data.w(i, j, k + 1) +
            data.invH.z / theta *
                (data.pressure(i, j, k + 1) - data.pressure(i, j, k));
    }
}

void ApplyPressureGradientAt(size_t i, size_t j, size_t k,
                             const PressureGradientData3& data)
{
    const double centerPhi = data.fluidSDF(i, j, k);
    ApplyUGradient(i, j, k, centerPhi, data);
    ApplyVGradient(i, j, k, centerPhi, data);
    ApplyWGradient(i, j, k, centerPhi, data);
}

void Restrict(const Array3<double>& finer, Array3<double>* coarser)
{
    // --*--|--*--|--*--|--*--
    //  1/8   3/8   3/8   1/8
    //           to
    // -----|-----*-----|-----
    static const std::array<double, 4> centeredKernel = { { 0.125, 0.375, 0.375,
                                                            0.125 } };

    // -|----|----|----|----|-
    //      1/4  1/2  1/4
    //           to
    // -|---------|---------|-
    static const std::array<double, 4> staggeredKernel = { { 0.0, 1.0, 0.0,
                                                             0.0 } };

    std::array<int, 3> kernelSize{};
    kernelSize[0] = finer.Size().x != 2 * coarser->Size().x ? 3 : 4;
    kernelSize[1] = finer.Size().y != 2 * coarser->Size().y ? 3 : 4;
    kernelSize[2] = finer.Size().z != 2 * coarser->Size().z ? 3 : 4;

    std::array<std::array<double, 4>, 3> kernels{};
    kernels[0] = (kernelSize[0] == 3) ? staggeredKernel : centeredKernel;
    kernels[1] = (kernelSize[1] == 3) ? staggeredKernel : centeredKernel;
    kernels[2] = (kernelSize[2] == 3) ? staggeredKernel : centeredKernel;

    const Vector3UZ n = coarser->Size();

    ParallelRangeFor(ZERO_SIZE, n.x, ZERO_SIZE, n.y, ZERO_SIZE, n.z,
                     [&kernelSize, &n, &kernels, &finer, &coarser](
                         size_t iBegin, size_t iEnd, size_t jBegin, size_t jEnd,
                         size_t kBegin, size_t kEnd) {
                         for (size_t k = kBegin; k < kEnd; ++k)
                         {
                             for (size_t j = jBegin; j < jEnd; ++j)
                             {
                                 for (size_t i = iBegin; i < iEnd; ++i)
                                 {
                                     (*coarser)(i, j, k) =
                                         RestrictPoint(i, j, k, n, kernelSize,
                                                       kernels, finer);
                                 }
                             }
                         }
                     });
}

struct PressureRow3
{
    double center = 0.0;
    double rhs = 0.0;
    std::array<double, 6> offDiagonal{};
    std::array<bool, 6> coupled{};
};

template <typename BoundaryVelocityFunc>
struct PressureRowData3
{
    const Vector3UZ& size;
    const Vector3D& invH;
    const Vector3D& invHSqr;
    const Array3<double>& fluidSDF;
    const Array3<double>& uWeights;
    const Array3<double>& vWeights;
    const Array3<double>& wWeights;
    const GridDataPositionFunc<3>& uPos;
    const GridDataPositionFunc<3>& vPos;
    const GridDataPositionFunc<3>& wPos;
    const BoundaryVelocityFunc& boundaryVel;
    const FaceCenteredGrid3& input;
};

void AddCoefficient(double centerPhi, double neighborPhi, double term,
                    size_t direction, PressureRow3* row)
{
    if (IsInsideSDF(neighborPhi))
    {
        row->center += term;
        row->offDiagonal[direction] = -term;
        row->coupled[direction] = true;
    }
    else
    {
        const double theta =
            std::max(FractionInsideSDF(centerPhi, neighborPhi), 0.01);
        row->center += term / theta;
    }
}

template <typename BoundaryVelocityFunc>
void AddXTerms(size_t i, size_t j, size_t k, double centerPhi,
               const PressureRowData3<BoundaryVelocityFunc>& data,
               PressureRow3* row)
{
    if (i + 1 < data.size.x)
    {
        AddCoefficient(centerPhi, data.fluidSDF(i + 1, j, k),
                       data.uWeights(i + 1, j, k) * data.invHSqr.x, 0, row);
        row->rhs += data.uWeights(i + 1, j, k) * data.input.U(i + 1, j, k) *
                    data.invH.x;
    }
    else
    {
        row->rhs += data.input.U(i + 1, j, k) * data.invH.x;
    }

    if (i > 0)
    {
        AddCoefficient(centerPhi, data.fluidSDF(i - 1, j, k),
                       data.uWeights(i, j, k) * data.invHSqr.x, 1, row);
        row->rhs -=
            data.uWeights(i, j, k) * data.input.U(i, j, k) * data.invH.x;
    }
    else
    {
        row->rhs -= data.input.U(i, j, k) * data.invH.x;
    }
}

template <typename BoundaryVelocityFunc>
void AddYTerms(size_t i, size_t j, size_t k, double centerPhi,
               const PressureRowData3<BoundaryVelocityFunc>& data,
               PressureRow3* row)
{
    if (j + 1 < data.size.y)
    {
        AddCoefficient(centerPhi, data.fluidSDF(i, j + 1, k),
                       data.vWeights(i, j + 1, k) * data.invHSqr.y, 2, row);
        row->rhs += data.vWeights(i, j + 1, k) * data.input.V(i, j + 1, k) *
                    data.invH.y;
    }
    else
    {
        row->rhs += data.input.V(i, j + 1, k) * data.invH.y;
    }

    if (j > 0)
    {
        AddCoefficient(centerPhi, data.fluidSDF(i, j - 1, k),
                       data.vWeights(i, j, k) * data.invHSqr.y, 3, row);
        row->rhs -=
            data.vWeights(i, j, k) * data.input.V(i, j, k) * data.invH.y;
    }
    else
    {
        row->rhs -= data.input.V(i, j, k) * data.invH.y;
    }
}

template <typename BoundaryVelocityFunc>
void AddZTerms(size_t i, size_t j, size_t k, double centerPhi,
               const PressureRowData3<BoundaryVelocityFunc>& data,
               PressureRow3* row)
{
    if (k + 1 < data.size.z)
    {
        AddCoefficient(centerPhi, data.fluidSDF(i, j, k + 1),
                       data.wWeights(i, j, k + 1) * data.invHSqr.z, 4, row);
        row->rhs += data.wWeights(i, j, k + 1) * data.input.W(i, j, k + 1) *
                    data.invH.z;
    }
    else
    {
        row->rhs += data.input.W(i, j, k + 1) * data.invH.z;
    }

    if (k > 0)
    {
        AddCoefficient(centerPhi, data.fluidSDF(i, j, k - 1),
                       data.wWeights(i, j, k) * data.invHSqr.z, 5, row);
        row->rhs -=
            data.wWeights(i, j, k) * data.input.W(i, j, k) * data.invH.z;
    }
    else
    {
        row->rhs -= data.input.W(i, j, k) * data.invH.z;
    }
}

template <typename BoundaryVelocityFunc>
void AddBoundaryVelocity(size_t i, size_t j, size_t k,
                         const PressureRowData3<BoundaryVelocityFunc>& data,
                         PressureRow3* row)
{
    row->rhs += (1.0 - data.uWeights(i + 1, j, k)) *
                    data.boundaryVel(data.uPos(i + 1, j, k)).x * data.invH.x -
                (1.0 - data.uWeights(i, j, k)) *
                    data.boundaryVel(data.uPos(i, j, k)).x * data.invH.x +
                (1.0 - data.vWeights(i, j + 1, k)) *
                    data.boundaryVel(data.vPos(i, j + 1, k)).y * data.invH.y -
                (1.0 - data.vWeights(i, j, k)) *
                    data.boundaryVel(data.vPos(i, j, k)).y * data.invH.y +
                (1.0 - data.wWeights(i, j, k + 1)) *
                    data.boundaryVel(data.wPos(i, j, k + 1)).z * data.invH.z -
                (1.0 - data.wWeights(i, j, k)) *
                    data.boundaryVel(data.wPos(i, j, k)).z * data.invH.z;
}

template <typename BoundaryVelocityFunc>
PressureRow3 BuildPressureRow(
    size_t i, size_t j, size_t k,
    const PressureRowData3<BoundaryVelocityFunc>& data)
{
    PressureRow3 row;
    const double centerPhi = data.fluidSDF(i, j, k);
    if (!IsInsideSDF(centerPhi))
    {
        row.center = 1.0;
        return row;
    }

    AddXTerms(i, j, k, centerPhi, data, &row);
    AddYTerms(i, j, k, centerPhi, data, &row);
    AddZTerms(i, j, k, centerPhi, data, &row);
    AddBoundaryVelocity(i, j, k, data, &row);
    if (row.center < std::numeric_limits<double>::epsilon())
    {
        row.center = 1.0;
        row.rhs = 0.0;
    }

    return row;
}

template <typename BoundaryVelocityFunc>
void AppendCompressedRow(
    size_t i, size_t j, size_t k, const Vector3UZ& size, const Vector3D& invH,
    const Vector3D& invHSqr, const Array3<double>& fluidSDF,
    const Array3<double>& uWeights, const Array3<double>& vWeights,
    const Array3<double>& wWeights, const GridDataPositionFunc<3>& uPos,
    const GridDataPositionFunc<3>& vPos, const GridDataPositionFunc<3>& wPos,
    const BoundaryVelocityFunc& boundaryVel, const FaceCenteredGrid3& input,
    const Array3<size_t>& coordToIndex, MatrixCSRD* A, VectorND* b)
{
    if (!IsInsideSDF(fluidSDF(i, j, k)))
    {
        return;
    }

    const PressureRowData3<BoundaryVelocityFunc> rowData{
        size,     invH, invHSqr, fluidSDF, uWeights,    vWeights,
        wWeights, uPos, vPos,    wPos,     boundaryVel, input
    };
    const PressureRow3 pressureRow = BuildPressureRow(i, j, k, rowData);
    std::vector<double> row{ pressureRow.center };
    std::vector<size_t> columns{ coordToIndex(i, j, k) };

    const auto addColumn = [&](size_t direction, size_t x, size_t y, size_t z) {
        if (pressureRow.coupled[direction])
        {
            row.push_back(pressureRow.offDiagonal[direction]);
            columns.push_back(coordToIndex(x, y, z));
        }
    };

    addColumn(0, i + 1, j, k);
    addColumn(1, i - 1, j, k);
    addColumn(2, i, j + 1, k);
    addColumn(3, i, j - 1, k);
    addColumn(4, i, j, k + 1);
    addColumn(5, i, j, k - 1);
    A->AddRow(row, columns);
    b->AddElement(pressureRow.rhs);
}

template <typename BoundaryVelocityFunc>
void BuildSingleSystem(FDMMatrix3* A, FDMVector3* b,
                       const Array3<double>& fluidSDF,
                       const Array3<double>& uWeights,
                       const Array3<double>& vWeights,
                       const Array3<double>& wWeights,
                       const BoundaryVelocityFunc& boundaryVel,
                       const FaceCenteredGrid3& input)
{
    const Vector3UZ size = input.Resolution();
    const GridDataPositionFunc<3> uPos = input.UPosition();
    const GridDataPositionFunc<3> vPos = input.VPosition();
    const GridDataPositionFunc<3> wPos = input.WPosition();

    const Vector3D invH = 1.0 / input.GridSpacing();
    const Vector3D invHSqr = ElemMul(invH, invH);
    const PressureRowData3<BoundaryVelocityFunc> rowData{
        size,     invH, invHSqr, fluidSDF, uWeights,    vWeights,
        wWeights, uPos, vPos,    wPos,     boundaryVel, input
    };

    // Build linear system
    ParallelForEachIndex(
        A->Size(), [&A, &b, &rowData](size_t i, size_t j, size_t k) {
            const PressureRow3 data = BuildPressureRow(i, j, k, rowData);
            FDMMatrixRow3& row = (*A)(i, j, k);
            row.center = data.center;
            row.right = data.offDiagonal[0];
            row.up = data.offDiagonal[2];
            row.front = data.offDiagonal[4];
            (*b)(i, j, k) = data.rhs;
        });
}

template <typename BoundaryVelocityFunc>
void BuildSingleSystem(MatrixCSRD* A, VectorND* x, VectorND* b,
                       const Array3<double>& fluidSDF,
                       const Array3<double>& uWeights,
                       const Array3<double>& vWeights,
                       const Array3<double>& wWeights,
                       const BoundaryVelocityFunc& boundaryVel,
                       const FaceCenteredGrid3& input)
{
    const Vector3UZ size = input.Resolution();
    const GridDataPositionFunc<3> uPos = input.UPosition();
    const GridDataPositionFunc<3> vPos = input.VPosition();
    const GridDataPositionFunc<3> wPos = input.WPosition();

    const Vector3D invH = 1.0 / input.GridSpacing();
    const Vector3D invHSqr = ElemMul(invH, invH);

    ConstArrayView3<double> fluidSDFAcc{ fluidSDF };

    A->Clear();
    b->Clear();

    size_t numRows = 0;
    Array3<size_t> coordToIndex{ size };
    ForEachIndex(fluidSDF.Size(), [&fluidSDFAcc, &fluidSDF, &coordToIndex,
                                   &numRows](size_t i, size_t j, size_t k) {
        const size_t cIdx = fluidSDFAcc.Index(i, j, k);
        const double centerPhi = fluidSDF[cIdx];

        if (IsInsideSDF(centerPhi))
        {
            coordToIndex[cIdx] = numRows++;
        }
    });

    ForEachIndex(fluidSDF.Size(), [&fluidSDF, &coordToIndex, &size, &uWeights,
                                   &invHSqr, &input, &invH, &vWeights,
                                   &wWeights, &boundaryVel, &uPos, &vPos, &wPos,
                                   &A, &b](size_t i, size_t j, size_t k) {
        AppendCompressedRow(i, j, k, size, invH, invHSqr, fluidSDF, uWeights,
                            vWeights, wWeights, uPos, vPos, wPos, boundaryVel,
                            input, coordToIndex, A, b);
    });

    x->Resize(b->GetRows(), 0.0);
}
}  // namespace

GridFractionalSinglePhasePressureSolver3::
    GridFractionalSinglePhasePressureSolver3()
{
    m_systemSolver = std::make_shared<FDMICCGSolver3>(100, DEFAULT_TOLERANCE);
}

void GridFractionalSinglePhasePressureSolver3::Solve(
    const FaceCenteredGrid3& input, double timeIntervalInSeconds,
    FaceCenteredGrid3* output, const ScalarField3& boundarySDF,
    const VectorField3& boundaryVelocity, const ScalarField3& fluidSDF,
    bool useCompressed)
{
    UNUSED_VARIABLE(timeIntervalInSeconds);

    BuildWeights(input, boundarySDF, boundaryVelocity, fluidSDF);
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
    }

    // Apply pressure gradient
    ApplyPressureGradient(input, output);
}

GridBoundaryConditionSolver3Ptr
GridFractionalSinglePhasePressureSolver3::SuggestedBoundaryConditionSolver()
    const
{
    return std::make_shared<GridFractionalBoundaryConditionSolver3>();
}

const FDMLinearSystemSolver3Ptr&
GridFractionalSinglePhasePressureSolver3::GetLinearSystemSolver() const
{
    return m_systemSolver;
}

void GridFractionalSinglePhasePressureSolver3::SetLinearSystemSolver(
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

const FDMVector3& GridFractionalSinglePhasePressureSolver3::GetPressure() const
{
    if (m_mgSystemSolver == nullptr)
    {
        return m_system.x;
    }

    return m_mgSystem.x.levels.front();
}

void GridFractionalSinglePhasePressureSolver3::BuildWeights(
    const FaceCenteredGrid3& input, const ScalarField3& boundarySDF,
    const VectorField3& boundaryVelocity, const ScalarField3& fluidSDF)
{
    const Vector3UZ& size = input.Resolution();

    // Build levels
    size_t maxLevels = 1;
    if (m_mgSystemSolver != nullptr)
    {
        maxLevels = m_mgSystemSolver->GetParams().maxNumberOfLevels;
    }

    FDMMGUtils3::ResizeArrayWithFinest(size, maxLevels, &m_fluidSDF);
    m_uWeights.resize(m_fluidSDF.size());
    m_vWeights.resize(m_fluidSDF.size());
    m_wWeights.resize(m_fluidSDF.size());
    for (size_t l = 0; l < m_fluidSDF.size(); ++l)
    {
        m_uWeights[l].Resize(m_fluidSDF[l].Size() + Vector3UZ{ 1, 0, 0 });
        m_vWeights[l].Resize(m_fluidSDF[l].Size() + Vector3UZ{ 0, 1, 0 });
        m_wWeights[l].Resize(m_fluidSDF[l].Size() + Vector3UZ{ 0, 0, 1 });
    }

    // Build top-level grids
    GridDataPositionFunc<3> cellPos = input.CellCenterPosition();
    GridDataPositionFunc<3> uPos = input.UPosition();
    GridDataPositionFunc<3> vPos = input.VPosition();
    GridDataPositionFunc<3> wPos = input.WPosition();
    m_boundaryVel = boundaryVelocity.Sampler();
    Vector3D h = input.GridSpacing();

    ParallelForEachIndex(
        m_fluidSDF[0].Size(),
        [this, &fluidSDF, &cellPos](size_t i, size_t j, size_t k) {
            m_fluidSDF[0](i, j, k) = fluidSDF.Sample(cellPos(i, j, k));
        });

    ParallelForEachIndex(
        m_uWeights[0].Size(),
        [&uPos, &boundarySDF, &h, this](size_t i, size_t j, size_t k) {
            const Vector3D pt = uPos(i, j, k);
            const double phi0 = boundarySDF.Sample(
                pt + Vector3D{ 0.0, -0.5 * h.y, -0.5 * h.z });
            const double phi1 =
                boundarySDF.Sample(pt + Vector3D{ 0.0, 0.5 * h.y, -0.5 * h.z });
            const double phi2 =
                boundarySDF.Sample(pt + Vector3D{ 0.0, -0.5 * h.y, 0.5 * h.z });
            const double phi3 =
                boundarySDF.Sample(pt + Vector3D{ 0.0, 0.5 * h.y, 0.5 * h.z });
            const double frac = FractionInside(phi0, phi1, phi2, phi3);
            double weight = std::clamp(1.0 - frac, 0.0, 1.0);

            // Clamp non-zero weight to kMinWeight. Having nearly-zero element
            // in the matrix can be an issue.
            if (weight < MIN_WEIGHT && weight > 0.0)
            {
                weight = MIN_WEIGHT;
            }

            m_uWeights[0](i, j, k) = weight;
        });

    ParallelForEachIndex(
        m_vWeights[0].Size(),
        [&vPos, &boundarySDF, &h, this](size_t i, size_t j, size_t k) {
            const Vector3D pt = vPos(i, j, k);
            const double phi0 = boundarySDF.Sample(
                pt + Vector3D{ -0.5 * h.x, 0.0, -0.5 * h.z });
            const double phi1 =
                boundarySDF.Sample(pt + Vector3D{ -0.5 * h.x, 0.0, 0.5 * h.z });
            const double phi2 =
                boundarySDF.Sample(pt + Vector3D{ 0.5 * h.x, 0.0, -0.5 * h.z });
            const double phi3 =
                boundarySDF.Sample(pt + Vector3D{ 0.5 * h.x, 0.0, 0.5 * h.z });
            const double frac = FractionInside(phi0, phi1, phi2, phi3);
            double weight = std::clamp(1.0 - frac, 0.0, 1.0);

            // Clamp non-zero weight to kMinWeight. Having nearly-zero element
            // in the matrix can be an issue.
            if (weight < MIN_WEIGHT && weight > 0.0)
            {
                weight = MIN_WEIGHT;
            }

            m_vWeights[0](i, j, k) = weight;
        });

    ParallelForEachIndex(
        m_wWeights[0].Size(),
        [&wPos, &boundarySDF, &h, this](size_t i, size_t j, size_t k) {
            const Vector3D pt = wPos(i, j, k);
            const double phi0 = boundarySDF.Sample(
                pt + Vector3D{ -0.5 * h.x, -0.5 * h.y, 0.0 });
            const double phi1 =
                boundarySDF.Sample(pt + Vector3D{ -0.5 * h.x, 0.5 * h.y, 0.0 });
            const double phi2 =
                boundarySDF.Sample(pt + Vector3D{ 0.5 * h.x, -0.5 * h.y, 0.0 });
            const double phi3 =
                boundarySDF.Sample(pt + Vector3D{ 0.5 * h.x, 0.5 * h.y, 0.0 });
            const double frac = FractionInside(phi0, phi1, phi2, phi3);
            double weight = std::clamp(1.0 - frac, 0.0, 1.0);

            // Clamp non-zero weight to kMinWeight. Having nearly-zero element
            // in the matrix can be an issue.
            if (weight < MIN_WEIGHT && weight > 0.0)
            {
                weight = MIN_WEIGHT;
            }

            m_wWeights[0](i, j, k) = weight;
        });

    // Build sub-levels
    for (size_t l = 1; l < m_fluidSDF.size(); ++l)
    {
        const Array3<double>& finerFluidSdf = m_fluidSDF[l - 1];
        Array3<double>& coarserFluidSdf = m_fluidSDF[l];
        const Array3<double>& finerUWeight = m_uWeights[l - 1];
        Array3<double>& coarserUWeight = m_uWeights[l];
        const Array3<double>& finerVWeight = m_vWeights[l - 1];
        Array3<double>& coarserVWeight = m_vWeights[l];
        const Array3<double>& finerWWeight = m_wWeights[l - 1];
        Array3<double>& coarserWWeight = m_wWeights[l];

        // Fluid SDF
        Restrict(finerFluidSdf, &coarserFluidSdf);
        Restrict(finerUWeight, &coarserUWeight);
        Restrict(finerVWeight, &coarserVWeight);
        Restrict(finerWWeight, &coarserWWeight);
    }
}

void GridFractionalSinglePhasePressureSolver3::DecompressSolution()
{
    ConstArrayView3<double> acc{ m_fluidSDF[0] };
    m_system.x.Resize(acc.Size());

    size_t row = 0;
    ForEachIndex(m_fluidSDF[0].Size(),
                 [&acc, this, &row](size_t i, size_t j, size_t k) {
                     if (IsInsideSDF(acc(i, j, k)))
                     {
                         m_system.x(i, j, k) = m_compSystem.x[row];
                         ++row;
                     }
                 });
}

void GridFractionalSinglePhasePressureSolver3::BuildSystem(
    const FaceCenteredGrid3& input, bool useCompressed)
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
                              m_fluidSDF[0], m_uWeights[0], m_vWeights[0],
                              m_wWeights[0], m_boundaryVel, *finer);
        }
        else
        {
            BuildSingleSystem(&m_system.A, &m_system.b, m_fluidSDF[0],
                              m_uWeights[0], m_vWeights[0], m_wWeights[0],
                              m_boundaryVel, *finer);
        }
    }
    else
    {
        BuildSingleSystem(&m_mgSystem.A.levels.front(),
                          &m_mgSystem.b.levels.front(), m_fluidSDF[0],
                          m_uWeights[0], m_vWeights[0], m_wWeights[0],
                          m_boundaryVel, *finer);
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
                          m_fluidSDF[l], m_uWeights[l], m_vWeights[l],
                          m_wWeights[l], m_boundaryVel, coarser);

        finer = &coarser;
    }
}

void GridFractionalSinglePhasePressureSolver3::ApplyPressureGradient(
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
    const PressureGradientData3 data{ size,
                                      m_fluidSDF[0],
                                      m_uWeights[0],
                                      m_vWeights[0],
                                      m_wWeights[0],
                                      u,
                                      v,
                                      w,
                                      u0,
                                      v0,
                                      w0,
                                      invH,
                                      x };

    ParallelForEachIndex(x.Size(), [&data](size_t i, size_t j, size_t k) {
        ApplyPressureGradientAt(i, j, k, data);
    });
}
}  // namespace CubbyFlow

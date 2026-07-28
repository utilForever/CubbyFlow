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
    ParallelRangeFor(
        ZERO_SIZE, n.x, ZERO_SIZE, n.y, ZERO_SIZE, n.z,
        [&kernelSize, &n, &kernels, &finer, &coarser](
            size_t iBegin, size_t iEnd, size_t jBegin, size_t jEnd,
            size_t kBegin, size_t kEnd) {
            std::array<size_t, 4> kIndices{};

            for (size_t k = kBegin; k < kEnd; ++k)
            {
                if (kernelSize[2] == 3)
                {
                    kIndices[0] = (k > 0) ? 2 * k - 1 : 2 * k;
                    kIndices[1] = 2 * k;
                    kIndices[2] = (k + 1 < n.z) ? 2 * k + 1 : 2 * k;
                }
                else
                {
                    kIndices[0] = (k > 0) ? 2 * k - 1 : 2 * k;
                    kIndices[1] = 2 * k;
                    kIndices[2] = 2 * k + 1;
                    kIndices[3] = (k + 1 < n.z) ? 2 * k + 2 : 2 * k + 1;
                }

                std::array<size_t, 4> jIndices{};

                for (size_t j = jBegin; j < jEnd; ++j)
                {
                    if (kernelSize[1] == 3)
                    {
                        jIndices[0] = (j > 0) ? 2 * j - 1 : 2 * j;
                        jIndices[1] = 2 * j;
                        jIndices[2] = (j + 1 < n.y) ? 2 * j + 1 : 2 * j;
                    }
                    else
                    {
                        jIndices[0] = (j > 0) ? 2 * j - 1 : 2 * j;
                        jIndices[1] = 2 * j;
                        jIndices[2] = 2 * j + 1;
                        jIndices[3] = (j + 1 < n.y) ? 2 * j + 2 : 2 * j + 1;
                    }

                    std::array<size_t, 4> iIndices{};
                    for (size_t i = iBegin; i < iEnd; ++i)
                    {
                        if (kernelSize[0] == 3)
                        {
                            iIndices[0] = (i > 0) ? 2 * i - 1 : 2 * i;
                            iIndices[1] = 2 * i;
                            iIndices[2] = (i + 1 < n.x) ? 2 * i + 1 : 2 * i;
                        }
                        else
                        {
                            iIndices[0] = (i > 0) ? 2 * i - 1 : 2 * i;
                            iIndices[1] = 2 * i;
                            iIndices[2] = 2 * i + 1;
                            iIndices[3] = (i + 1 < n.x) ? 2 * i + 2 : 2 * i + 1;
                        }

                        double sum = 0.0;
                        for (int z = 0; z < kernelSize[2]; ++z)
                        {
                            for (int y = 0; y < kernelSize[1]; ++y)
                            {
                                for (int x = 0; x < kernelSize[0]; ++x)
                                {
                                    const double w = kernels[0][x] *
                                                     kernels[1][y] *
                                                     kernels[2][z];
                                    sum += w * finer(iIndices[x], jIndices[y],
                                                     kIndices[z]);
                                }
                            }
                        }

                        (*coarser)(i, j, k) = sum;
                    }
                }
            }
        });
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

    // Build linear system
    ParallelForEachIndex(A->Size(), [&A, &b, &fluidSDF, &size, &uWeights,
                                     &invHSqr, &input, &invH, &vWeights,
                                     &wWeights, &boundaryVel, &uPos, &vPos,
                                     &wPos](size_t i, size_t j, size_t k) {
        auto& row = (*A)(i, j, k);

        // initialize
        row.center = row.right = row.up = row.front = 0.0;
        (*b)(i, j, k) = 0.0;

        const double centerPhi = fluidSDF(i, j, k);

        if (IsInsideSDF(centerPhi))
        {
            double term;

            if (i + 1 < size.x)
            {
                term = uWeights(i + 1, j, k) * invHSqr.x;
                const double rightPhi = fluidSDF(i + 1, j, k);

                if (IsInsideSDF(rightPhi))
                {
                    row.center += term;
                    row.right -= term;
                }
                else
                {
                    double theta = FractionInsideSDF(centerPhi, rightPhi);
                    theta = std::max(theta, 0.01);
                    row.center += term / theta;
                }

                (*b)(i, j, k) +=
                    uWeights(i + 1, j, k) * input.U(i + 1, j, k) * invH.x;
            }
            else
            {
                (*b)(i, j, k) += input.U(i + 1, j, k) * invH.x;
            }

            if (i > 0)
            {
                term = uWeights(i, j, k) * invHSqr.x;
                const double leftPhi = fluidSDF(i - 1, j, k);

                if (IsInsideSDF(leftPhi))
                {
                    row.center += term;
                }
                else
                {
                    double theta = FractionInsideSDF(centerPhi, leftPhi);
                    theta = std::max(theta, 0.01);
                    row.center += term / theta;
                }

                (*b)(i, j, k) -= uWeights(i, j, k) * input.U(i, j, k) * invH.x;
            }
            else
            {
                (*b)(i, j, k) -= input.U(i, j, k) * invH.x;
            }

            if (j + 1 < size.y)
            {
                term = vWeights(i, j + 1, k) * invHSqr.y;
                const double upPhi = fluidSDF(i, j + 1, k);

                if (IsInsideSDF(upPhi))
                {
                    row.center += term;
                    row.up -= term;
                }
                else
                {
                    double theta = FractionInsideSDF(centerPhi, upPhi);
                    theta = std::max(theta, 0.01);
                    row.center += term / theta;
                }

                (*b)(i, j, k) +=
                    vWeights(i, j + 1, k) * input.V(i, j + 1, k) * invH.y;
            }
            else
            {
                (*b)(i, j, k) += input.V(i, j + 1, k) * invH.y;
            }

            if (j > 0)
            {
                term = vWeights(i, j, k) * invHSqr.y;
                const double downPhi = fluidSDF(i, j - 1, k);

                if (IsInsideSDF(downPhi))
                {
                    row.center += term;
                }
                else
                {
                    double theta = FractionInsideSDF(centerPhi, downPhi);
                    theta = std::max(theta, 0.01);
                    row.center += term / theta;
                }

                (*b)(i, j, k) -= vWeights(i, j, k) * input.V(i, j, k) * invH.y;
            }
            else
            {
                (*b)(i, j, k) -= input.V(i, j, k) * invH.y;
            }

            if (k + 1 < size.z)
            {
                term = wWeights(i, j, k + 1) * invHSqr.z;
                const double frontPhi = fluidSDF(i, j, k + 1);

                if (IsInsideSDF(frontPhi))
                {
                    row.center += term;
                    row.front -= term;
                }
                else
                {
                    double theta = FractionInsideSDF(centerPhi, frontPhi);
                    theta = std::max(theta, 0.01);
                    row.center += term / theta;
                }

                (*b)(i, j, k) +=
                    wWeights(i, j, k + 1) * input.W(i, j, k + 1) * invH.z;
            }
            else
            {
                (*b)(i, j, k) += input.W(i, j, k + 1) * invH.z;
            }

            if (k > 0)
            {
                term = wWeights(i, j, k) * invHSqr.z;
                const double backPhi = fluidSDF(i, j, k - 1);

                if (IsInsideSDF(backPhi))
                {
                    row.center += term;
                }
                else
                {
                    double theta = FractionInsideSDF(centerPhi, backPhi);
                    theta = std::max(theta, 0.01);
                    row.center += term / theta;
                }

                (*b)(i, j, k) -= wWeights(i, j, k) * input.W(i, j, k) * invH.z;
            }
            else
            {
                (*b)(i, j, k) -= input.W(i, j, k) * invH.z;
            }

            // Accumulate contributions from the moving boundary
            const double boundaryContribution =
                (1.0 - uWeights(i + 1, j, k)) *
                    boundaryVel(uPos(i + 1, j, k)).x * invH.x -
                (1.0 - uWeights(i, j, k)) * boundaryVel(uPos(i, j, k)).x *
                    invH.x +
                (1.0 - vWeights(i, j + 1, k)) *
                    boundaryVel(vPos(i, j + 1, k)).y * invH.y -
                (1.0 - vWeights(i, j, k)) * boundaryVel(vPos(i, j, k)).y *
                    invH.y +
                (1.0 - wWeights(i, j, k + 1)) *
                    boundaryVel(wPos(i, j, k + 1)).z * invH.z -
                (1.0 - wWeights(i, j, k)) * boundaryVel(wPos(i, j, k)).z *
                    invH.z;
            (*b)(i, j, k) += boundaryContribution;

            // If row.center is near-zero, the cell is likely inside a solid
            // boundary.
            if (row.center < std::numeric_limits<double>::epsilon())
            {
                row.center = 1.0;
                (*b)(i, j, k) = 0.0;
            }
        }
        else
        {
            row.center = 1.0;
        }
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

    ForEachIndex(fluidSDF.Size(), [&fluidSDFAcc, &fluidSDF, &coordToIndex,
                                   &size, &uWeights, &invHSqr, &input, &invH,
                                   &vWeights, &wWeights, &boundaryVel, &uPos,
                                   &vPos, &wPos, &A,
                                   &b](size_t i, size_t j, size_t k) {
        const size_t cIdx = fluidSDFAcc.Index(i, j, k);
        const double centerPhi = fluidSDF[cIdx];

        if (IsInsideSDF(centerPhi))
        {
            double bijk = 0.0;

            std::vector<double> row(1, 0.0);
            std::vector<size_t> colIdx(1, coordToIndex[cIdx]);

            double term;

            if (i + 1 < size.x)
            {
                term = uWeights(i + 1, j, k) * invHSqr.x;
                const double rightPhi = fluidSDF(i + 1, j, k);

                if (IsInsideSDF(rightPhi))
                {
                    row[0] += term;
                    row.push_back(-term);
                    colIdx.push_back(coordToIndex(i + 1, j, k));
                }
                else
                {
                    double theta = FractionInsideSDF(centerPhi, rightPhi);
                    theta = std::max(theta, 0.01);
                    row[0] += term / theta;
                }

                bijk += uWeights(i + 1, j, k) * input.U(i + 1, j, k) * invH.x;
            }
            else
            {
                bijk += input.U(i + 1, j, k) * invH.x;
            }

            if (i > 0)
            {
                term = uWeights(i, j, k) * invHSqr.x;
                const double leftPhi = fluidSDF(i - 1, j, k);

                if (IsInsideSDF(leftPhi))
                {
                    row[0] += term;
                    row.push_back(-term);
                    colIdx.push_back(coordToIndex(i - 1, j, k));
                }
                else
                {
                    double theta = FractionInsideSDF(centerPhi, leftPhi);
                    theta = std::max(theta, 0.01);
                    row[0] += term / theta;
                }

                bijk -= uWeights(i, j, k) * input.U(i, j, k) * invH.x;
            }
            else
            {
                bijk -= input.U(i, j, k) * invH.x;
            }

            if (j + 1 < size.y)
            {
                term = vWeights(i, j + 1, k) * invHSqr.y;
                const double upPhi = fluidSDF(i, j + 1, k);

                if (IsInsideSDF(upPhi))
                {
                    row[0] += term;
                    row.push_back(-term);
                    colIdx.push_back(coordToIndex(i, j + 1, k));
                }
                else
                {
                    double theta = FractionInsideSDF(centerPhi, upPhi);
                    theta = std::max(theta, 0.01);
                    row[0] += term / theta;
                }

                bijk += vWeights(i, j + 1, k) * input.V(i, j + 1, k) * invH.y;
            }
            else
            {
                bijk += input.V(i, j + 1, k) * invH.y;
            }

            if (j > 0)
            {
                term = vWeights(i, j, k) * invHSqr.y;
                const double downPhi = fluidSDF(i, j - 1, k);

                if (IsInsideSDF(downPhi))
                {
                    row[0] += term;
                    row.push_back(-term);
                    colIdx.push_back(coordToIndex(i, j - 1, k));
                }
                else
                {
                    double theta = FractionInsideSDF(centerPhi, downPhi);
                    theta = std::max(theta, 0.01);
                    row[0] += term / theta;
                }

                bijk -= vWeights(i, j, k) * input.V(i, j, k) * invH.y;
            }
            else
            {
                bijk -= input.V(i, j, k) * invH.y;
            }

            if (k + 1 < size.z)
            {
                term = wWeights(i, j, k + 1) * invHSqr.z;
                const double frontPhi = fluidSDF(i, j, k + 1);

                if (IsInsideSDF(frontPhi))
                {
                    row[0] += term;
                    row.push_back(-term);
                    colIdx.push_back(coordToIndex(i, j, k + 1));
                }
                else
                {
                    double theta = FractionInsideSDF(centerPhi, frontPhi);
                    theta = std::max(theta, 0.01);
                    row[0] += term / theta;
                }

                bijk += wWeights(i, j, k + 1) * input.W(i, j, k + 1) * invH.z;
            }
            else
            {
                bijk += input.W(i, j, k + 1) * invH.z;
            }

            if (k > 0)
            {
                term = wWeights(i, j, k) * invHSqr.z;
                const double backPhi = fluidSDF(i, j, k - 1);

                if (IsInsideSDF(backPhi))
                {
                    row[0] += term;
                    row.push_back(-term);
                    colIdx.push_back(coordToIndex(i, j, k - 1));
                }
                else
                {
                    double theta = FractionInsideSDF(centerPhi, backPhi);
                    theta = std::max(theta, 0.01);
                    row[0] += term / theta;
                }

                bijk -= wWeights(i, j, k) * input.W(i, j, k) * invH.z;
            }
            else
            {
                bijk -= input.W(i, j, k) * invH.z;
            }

            // Accumulate contributions from the moving boundary
            const double boundaryContribution =
                (1.0 - uWeights(i + 1, j, k)) *
                    boundaryVel(uPos(i + 1, j, k)).x * invH.x -
                (1.0 - uWeights(i, j, k)) * boundaryVel(uPos(i, j, k)).x *
                    invH.x +
                (1.0 - vWeights(i, j + 1, k)) *
                    boundaryVel(vPos(i, j + 1, k)).y * invH.y -
                (1.0 - vWeights(i, j, k)) * boundaryVel(vPos(i, j, k)).y *
                    invH.y +
                (1.0 - wWeights(i, j, k + 1)) *
                    boundaryVel(wPos(i, j, k + 1)).z * invH.z -
                (1.0 - wWeights(i, j, k)) * boundaryVel(wPos(i, j, k)).z *
                    invH.z;
            bijk += boundaryContribution;

            // If row.center is near-zero, the cell is likely inside a solid
            // boundary.
            if (row[0] < std::numeric_limits<double>::epsilon())
            {
                row[0] = 1.0;
                bijk = 0.0;
            }

            A->AddRow(row, colIdx);
            b->AddElement(bijk);
        }
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

    ParallelForEachIndex(x.Size(), [this, &size, &u0, &u, &invH, &x, &v0, &v,
                                    &w0, &w](size_t i, size_t j, size_t k) {
        const double centerPhi = m_fluidSDF[0](i, j, k);

        if (i + 1 < size.x && m_uWeights[0](i + 1, j, k) > 0.0 &&
            (IsInsideSDF(centerPhi) || IsInsideSDF(m_fluidSDF[0](i + 1, j, k))))
        {
            const double rightPhi = m_fluidSDF[0](i + 1, j, k);
            double theta = FractionInsideSDF(centerPhi, rightPhi);
            theta = std::max(theta, 0.01);

            u0(i + 1, j, k) =
                u(i + 1, j, k) + invH.x / theta * (x(i + 1, j, k) - x(i, j, k));
        }

        if (j + 1 < size.y && m_vWeights[0](i, j + 1, k) > 0.0 &&
            (IsInsideSDF(centerPhi) || IsInsideSDF(m_fluidSDF[0](i, j + 1, k))))
        {
            const double upPhi = m_fluidSDF[0](i, j + 1, k);
            double theta = FractionInsideSDF(centerPhi, upPhi);
            theta = std::max(theta, 0.01);

            v0(i, j + 1, k) =
                v(i, j + 1, k) + invH.y / theta * (x(i, j + 1, k) - x(i, j, k));
        }

        if (k + 1 < size.z && m_wWeights[0](i, j, k + 1) > 0.0 &&
            (IsInsideSDF(centerPhi) || IsInsideSDF(m_fluidSDF[0](i, j, k + 1))))
        {
            const double frontPhi = m_fluidSDF[0](i, j, k + 1);
            double theta = FractionInsideSDF(centerPhi, frontPhi);
            theta = std::max(theta, 0.01);

            w0(i, j, k + 1) =
                w(i, j, k + 1) + invH.z / theta * (x(i, j, k + 1) - x(i, j, k));
        }
    });
}
}  // namespace CubbyFlow

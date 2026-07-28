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

#include <Core/Solver/FDM/FDMICCGSolver2.hpp>
#include <Core/Solver/Grid/GridFractionalBoundaryConditionSolver2.hpp>
#include <Core/Solver/Grid/GridFractionalSinglePhasePressureSolver2.hpp>
#include <Core/Utils/LevelSetUtils.hpp>

namespace CubbyFlow
{
const double DEFAULT_TOLERANCE = 1e-6;
const double MIN_WEIGHT = 0.01;

namespace
{
void Restrict(const Array2<double>& finer, Array2<double>* coarser)
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

    std::array<int, 2> kernelSize{};
    kernelSize[0] = finer.Size().x != 2 * coarser->Size().x ? 3 : 4;
    kernelSize[1] = finer.Size().y != 2 * coarser->Size().y ? 3 : 4;

    std::array<std::array<double, 4>, 2> kernels{};
    kernels[0] = (kernelSize[0] == 3) ? staggeredKernel : centeredKernel;
    kernels[1] = (kernelSize[1] == 3) ? staggeredKernel : centeredKernel;

    const Vector2UZ n = coarser->Size();
    ParallelRangeFor(
        ZERO_SIZE, n.x, ZERO_SIZE, n.y,
        [&kernelSize, &n, &kernels, &finer, &coarser](
            size_t iBegin, size_t iEnd, size_t jBegin, size_t jEnd) {
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
                    for (int y = 0; y < kernelSize[1]; ++y)
                    {
                        for (int x = 0; x < kernelSize[0]; ++x)
                        {
                            const double w = kernels[0][x] * kernels[1][y];
                            sum += w * finer(iIndices[x], jIndices[y]);
                        }
                    }

                    (*coarser)(i, j) = sum;
                }
            }
        });
}

template <typename BoundaryVelocityFunc>
void BuildSingleSystem(FDMMatrix2* A, FDMVector2* b,
                       const Array2<double>& fluidSDF,
                       const Array2<double>& uWeights,
                       const Array2<double>& vWeights,
                       const BoundaryVelocityFunc& boundaryVel,
                       const FaceCenteredGrid2& input)
{
    const Vector2UZ size = input.Resolution();
    const GridDataPositionFunc<2> uPos = input.UPosition();
    const GridDataPositionFunc<2> vPos = input.VPosition();

    const Vector2D invH = 1.0 / input.GridSpacing();
    const Vector2D invHSqr = ElemMul(invH, invH);

    // Build linear system
    ParallelForEachIndex(A->Size(), [&A, &b, &fluidSDF, &size, &uWeights,
                                     &invHSqr, &input, &invH, &vWeights,
                                     &boundaryVel, &uPos,
                                     &vPos](size_t i, size_t j) {
        FDMMatrixRow2& row = (*A)(i, j);

        // initialize
        row.center = row.right = row.up = 0.0;
        (*b)(i, j) = 0.0;

        const double centerPhi = fluidSDF(i, j);

        if (IsInsideSDF(centerPhi))
        {
            double term;

            if (i + 1 < size.x)
            {
                term = uWeights(i + 1, j) * invHSqr.x;
                const double rightPhi = fluidSDF(i + 1, j);

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

                (*b)(i, j) += uWeights(i + 1, j) * input.U(i + 1, j) * invH.x;
            }
            else
            {
                (*b)(i, j) += input.U(i + 1, j) * invH.x;
            }

            if (i > 0)
            {
                term = uWeights(i, j) * invHSqr.x;
                const double leftPhi = fluidSDF(i - 1, j);

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

                (*b)(i, j) -= uWeights(i, j) * input.U(i, j) * invH.x;
            }
            else
            {
                (*b)(i, j) -= input.U(i, j) * invH.x;
            }

            if (j + 1 < size.y)
            {
                term = vWeights(i, j + 1) * invHSqr.y;
                const double upPhi = fluidSDF(i, j + 1);

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

                (*b)(i, j) += vWeights(i, j + 1) * input.V(i, j + 1) * invH.y;
            }
            else
            {
                (*b)(i, j) += input.V(i, j + 1) * invH.y;
            }

            if (j > 0)
            {
                term = vWeights(i, j) * invHSqr.y;
                const double downPhi = fluidSDF(i, j - 1);

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

                (*b)(i, j) -= vWeights(i, j) * input.V(i, j) * invH.y;
            }
            else
            {
                (*b)(i, j) -= input.V(i, j) * invH.y;
            }

            // Accumulate contributions from the moving boundary
            const double boundaryContribution =
                (1.0 - uWeights(i + 1, j)) * boundaryVel(uPos(i + 1, j)).x *
                    invH.x -
                (1.0 - uWeights(i, j)) * boundaryVel(uPos(i, j)).x * invH.x +
                (1.0 - vWeights(i, j + 1)) * boundaryVel(vPos(i, j + 1)).y *
                    invH.y -
                (1.0 - vWeights(i, j)) * boundaryVel(vPos(i, j)).y * invH.y;
            (*b)(i, j) += boundaryContribution;

            // If row.center is near-zero, the cell is likely inside a solid
            // boundary.
            if (row.center < std::numeric_limits<double>::epsilon())
            {
                row.center = 1.0;
                (*b)(i, j) = 0.0;
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
                       const Array2<double>& fluidSDF,
                       const Array2<double>& uWeights,
                       const Array2<double>& vWeights,
                       const BoundaryVelocityFunc& boundaryVel,
                       const FaceCenteredGrid2& input)
{
    const Vector2UZ size = input.Resolution();
    const GridDataPositionFunc<2> uPos = input.UPosition();
    const GridDataPositionFunc<2> vPos = input.VPosition();

    const Vector2D invH = 1.0 / input.GridSpacing();
    const Vector2D invHSqr = ElemMul(invH, invH);

    ConstArrayView2<double> fluidSDFAcc{ fluidSDF };

    A->Clear();
    b->Clear();

    size_t numRows = 0;
    Array2<size_t> coordToIndex{ size };
    ForEachIndex(fluidSDF.Size(), [&fluidSDFAcc, &fluidSDF, &coordToIndex,
                                   &numRows](size_t i, size_t j) {
        const size_t cIdx = fluidSDFAcc.Index(i, j);
        const double centerPhi = fluidSDF[cIdx];

        if (IsInsideSDF(centerPhi))
        {
            coordToIndex[cIdx] = numRows++;
        }
    });

    ForEachIndex(fluidSDF.Size(), [&fluidSDFAcc, &fluidSDF, &coordToIndex,
                                   &size, &uWeights, &invHSqr, &input, &invH,
                                   &vWeights, &boundaryVel, &uPos, &vPos, &A,
                                   &b](size_t i, size_t j) {
        const size_t cIdx = fluidSDFAcc.Index(i, j);
        const double centerPhi = fluidSDF(i, j);

        if (IsInsideSDF(centerPhi))
        {
            double bij = 0.0;

            std::vector<double> row(1, 0.0);
            std::vector<size_t> colIdx(1, coordToIndex[cIdx]);

            double term;

            if (i + 1 < size.x)
            {
                term = uWeights(i + 1, j) * invHSqr.x;
                const double rightPhi = fluidSDF(i + 1, j);

                if (IsInsideSDF(rightPhi))
                {
                    row[0] += term;
                    row.push_back(-term);
                    colIdx.push_back(coordToIndex(i + 1, j));
                }
                else
                {
                    double theta = FractionInsideSDF(centerPhi, rightPhi);
                    theta = std::max(theta, 0.01);
                    row[0] += term / theta;
                }

                bij += uWeights(i + 1, j) * input.U(i + 1, j) * invH.x;
            }
            else
            {
                bij += input.U(i + 1, j) * invH.x;
            }

            if (i > 0)
            {
                term = uWeights(i, j) * invHSqr.x;
                const double leftPhi = fluidSDF(i - 1, j);

                if (IsInsideSDF(leftPhi))
                {
                    row[0] += term;
                    row.push_back(-term);
                    colIdx.push_back(coordToIndex(i - 1, j));
                }
                else
                {
                    double theta = FractionInsideSDF(centerPhi, leftPhi);
                    theta = std::max(theta, 0.01);
                    row[0] += term / theta;
                }

                bij -= uWeights(i, j) * input.U(i, j) * invH.x;
            }
            else
            {
                bij -= input.U(i, j) * invH.x;
            }

            if (j + 1 < size.y)
            {
                term = vWeights(i, j + 1) * invHSqr.y;
                const double upPhi = fluidSDF(i, j + 1);

                if (IsInsideSDF(upPhi))
                {
                    row[0] += term;
                    row.push_back(-term);
                    colIdx.push_back(coordToIndex(i, j + 1));
                }
                else
                {
                    double theta = FractionInsideSDF(centerPhi, upPhi);
                    theta = std::max(theta, 0.01);
                    row[0] += term / theta;
                }

                bij += vWeights(i, j + 1) * input.V(i, j + 1) * invH.y;
            }
            else
            {
                bij += input.V(i, j + 1) * invH.y;
            }

            if (j > 0)
            {
                term = vWeights(i, j) * invHSqr.y;
                const double downPhi = fluidSDF(i, j - 1);

                if (IsInsideSDF(downPhi))
                {
                    row[0] += term;
                    row.push_back(-term);
                    colIdx.push_back(coordToIndex(i, j - 1));
                }
                else
                {
                    double theta = FractionInsideSDF(centerPhi, downPhi);
                    theta = std::max(theta, 0.01);
                    row[0] += term / theta;
                }

                bij -= vWeights(i, j) * input.V(i, j) * invH.y;
            }
            else
            {
                bij -= input.V(i, j) * invH.y;
            }

            // Accumulate contributions from the moving boundary
            const double boundaryContribution =
                (1.0 - uWeights(i + 1, j)) * boundaryVel(uPos(i + 1, j)).x *
                    invH.x -
                (1.0 - uWeights(i, j)) * boundaryVel(uPos(i, j)).x * invH.x +
                (1.0 - vWeights(i, j + 1)) * boundaryVel(vPos(i, j + 1)).y *
                    invH.y -
                (1.0 - vWeights(i, j)) * boundaryVel(vPos(i, j)).y * invH.y;
            bij += boundaryContribution;

            // If row.center is near-zero, the cell is likely inside a solid
            // boundary.
            if (row[0] < std::numeric_limits<double>::epsilon())
            {
                row[0] = 1.0;
                bij = 0.0;
            }

            A->AddRow(row, colIdx);
            b->AddElement(bij);
        }
    });

    x->Resize(b->GetRows(), 0.0);
}
}  // namespace

GridFractionalSinglePhasePressureSolver2::
    GridFractionalSinglePhasePressureSolver2()
{
    m_systemSolver = std::make_shared<FDMICCGSolver2>(100, DEFAULT_TOLERANCE);
}

void GridFractionalSinglePhasePressureSolver2::Solve(
    const FaceCenteredGrid2& input, double timeIntervalInSeconds,
    FaceCenteredGrid2* output, const ScalarField2& boundarySDF,
    const VectorField2& boundaryVelocity, const ScalarField2& fluidSDF,
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

        // Apply pressure gradient
        ApplyPressureGradient(input, output);
    }
}

GridBoundaryConditionSolver2Ptr
GridFractionalSinglePhasePressureSolver2::SuggestedBoundaryConditionSolver()
    const
{
    return std::make_shared<GridFractionalBoundaryConditionSolver2>();
}

const FDMLinearSystemSolver2Ptr&
GridFractionalSinglePhasePressureSolver2::GetLinearSystemSolver() const
{
    return m_systemSolver;
}

void GridFractionalSinglePhasePressureSolver2::SetLinearSystemSolver(
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

const FDMVector2& GridFractionalSinglePhasePressureSolver2::GetPressure() const
{
    if (m_mgSystemSolver == nullptr)
    {
        return m_system.x;
    }

    return m_mgSystem.x.levels.front();
}

void GridFractionalSinglePhasePressureSolver2::BuildWeights(
    const FaceCenteredGrid2& input, const ScalarField2& boundarySDF,
    const VectorField2& boundaryVelocity, const ScalarField2& fluidSDF)
{
    const Vector2UZ& size = input.Resolution();

    // Build levels
    size_t maxLevels = 1;
    if (m_mgSystemSolver != nullptr)
    {
        maxLevels = m_mgSystemSolver->GetParams().maxNumberOfLevels;
    }

    FDMMGUtils2::ResizeArrayWithFinest(size, maxLevels, &m_fluidSDF);
    m_uWeights.resize(m_fluidSDF.size());
    m_vWeights.resize(m_fluidSDF.size());
    for (size_t l = 0; l < m_fluidSDF.size(); ++l)
    {
        m_uWeights[l].Resize(m_fluidSDF[l].Size() + Vector2UZ{ 1, 0 });
        m_vWeights[l].Resize(m_fluidSDF[l].Size() + Vector2UZ{ 0, 1 });
    }

    // Build top-level grids
    GridDataPositionFunc<2> cellPos = input.CellCenterPosition();
    GridDataPositionFunc<2> uPos = input.UPosition();
    GridDataPositionFunc<2> vPos = input.VPosition();
    m_boundaryVel = boundaryVelocity.Sampler();
    Vector2D h = input.GridSpacing();

    ParallelForEachIndex(
        m_fluidSDF[0].Size(), [this, &fluidSDF, &cellPos](size_t i, size_t j) {
            m_fluidSDF[0](i, j) = fluidSDF.Sample(cellPos(i, j));
        });

    ParallelForEachIndex(m_uWeights[0].Size(), [&uPos, &boundarySDF, &h, this](
                                                   size_t i, size_t j) {
        const Vector2D pt = uPos(i, j);
        const double phi0 = boundarySDF.Sample(pt - Vector2D{ 0.5 * h.x, 0.0 });
        const double phi1 = boundarySDF.Sample(pt + Vector2D{ 0.5 * h.x, 0.0 });
        const double frac = FractionInsideSDF(phi0, phi1);
        double weight = std::clamp(1.0 - frac, 0.0, 1.0);

        // Clamp non-zero weight to kMinWeight. Having nearly-zero element
        // in the matrix can be an issue.
        if (weight < MIN_WEIGHT && weight > 0.0)
        {
            weight = MIN_WEIGHT;
        }

        m_uWeights[0](i, j) = weight;
    });

    ParallelForEachIndex(m_vWeights[0].Size(), [&vPos, &boundarySDF, &h, this](
                                                   size_t i, size_t j) {
        const Vector2D pt = vPos(i, j);
        const double phi0 = boundarySDF.Sample(pt - Vector2D{ 0.0, 0.5 * h.y });
        const double phi1 = boundarySDF.Sample(pt + Vector2D{ 0.0, 0.5 * h.y });
        const double frac = FractionInsideSDF(phi0, phi1);
        double weight = std::clamp(1.0 - frac, 0.0, 1.0);

        // Clamp non-zero weight to kMinWeight. Having nearly-zero element
        // in the matrix can be an issue.
        if (weight < MIN_WEIGHT && weight > 0.0)
        {
            weight = MIN_WEIGHT;
        }

        m_vWeights[0](i, j) = weight;
    });

    // Build sub-levels
    for (size_t l = 1; l < m_fluidSDF.size(); ++l)
    {
        const auto& finerFluidSDF = m_fluidSDF[l - 1];
        auto& coarserFluidSDF = m_fluidSDF[l];
        const auto& finerUWeight = m_uWeights[l - 1];
        auto& coarserUWeight = m_uWeights[l];
        const auto& finerVWeight = m_vWeights[l - 1];
        auto& coarserVWeight = m_vWeights[l];

        // Fluid SDF
        Restrict(finerFluidSDF, &coarserFluidSDF);
        Restrict(finerUWeight, &coarserUWeight);
        Restrict(finerVWeight, &coarserVWeight);
    }
}

void GridFractionalSinglePhasePressureSolver2::DecompressSolution()
{
    ConstArrayView2<double> acc{ m_fluidSDF[0] };
    m_system.x.Resize(acc.Size());

    size_t row = 0;
    ForEachIndex(m_fluidSDF[0].Size(), [&acc, this, &row](size_t i, size_t j) {
        if (IsInsideSDF(acc(i, j)))
        {
            m_system.x(i, j) = m_compSystem.x[row];
            ++row;
        }
    });
}

void GridFractionalSinglePhasePressureSolver2::BuildSystem(
    const FaceCenteredGrid2& input, bool useCompressed)
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
                              m_fluidSDF[0], m_uWeights[0], m_vWeights[0],
                              m_boundaryVel, *finer);
        }
        else
        {
            BuildSingleSystem(&m_system.A, &m_system.b, m_fluidSDF[0],
                              m_uWeights[0], m_vWeights[0], m_boundaryVel,
                              *finer);
        }
    }
    else
    {
        BuildSingleSystem(&m_mgSystem.A.levels.front(),
                          &m_mgSystem.b.levels.front(), m_fluidSDF[0],
                          m_uWeights[0], m_vWeights[0], m_boundaryVel, *finer);
    }

    // Build sub-levels
    FaceCenteredGrid2 coarser;
    for (size_t l = 1; l < numLevels; ++l)
    {
        Vector2UZ res = finer->Resolution();
        Vector2D h = finer->GridSpacing();
        const Vector2D o = finer->Origin();
        res.x = res.x >> 1;
        res.y = res.y >> 1;
        h *= 2.0;

        // Down sample
        coarser.Resize(res, h, o);
        coarser.Fill(finer->Sampler());

        BuildSingleSystem(&m_mgSystem.A.levels[l], &m_mgSystem.b.levels[l],
                          m_fluidSDF[l], m_uWeights[l], m_vWeights[l],
                          m_boundaryVel, coarser);

        finer = &coarser;
    }
}

void GridFractionalSinglePhasePressureSolver2::ApplyPressureGradient(
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
        const double centerPhi = m_fluidSDF[0](i, j);

        if (i + 1 < size.x && m_uWeights[0](i + 1, j) > 0.0 &&
            (IsInsideSDF(centerPhi) || IsInsideSDF(m_fluidSDF[0](i + 1, j))))
        {
            const double rightPhi = m_fluidSDF[0](i + 1, j);
            double theta = FractionInsideSDF(centerPhi, rightPhi);
            theta = std::max(theta, 0.01);

            u0(i + 1, j) =
                u(i + 1, j) + invH.x / theta * (x(i + 1, j) - x(i, j));
        }

        if (j + 1 < size.y && m_vWeights[0](i, j + 1) > 0.0 &&
            (IsInsideSDF(centerPhi) || IsInsideSDF(m_fluidSDF[0](i, j + 1))))
        {
            const double upPhi = m_fluidSDF[0](i, j + 1);
            double theta = FractionInsideSDF(centerPhi, upPhi);
            theta = std::max(theta, 0.01);

            v0(i, j + 1) =
                v(i, j + 1) + invH.y / theta * (x(i, j + 1) - x(i, j));
        }
    });
}
}  // namespace CubbyFlow

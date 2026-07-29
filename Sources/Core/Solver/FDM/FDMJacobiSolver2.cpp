// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Solver/FDM/FDMJacobiSolver2.hpp>

namespace CubbyFlow
{
FDMJacobiSolver2::FDMJacobiSolver2(unsigned int maxNumberOfIterations,
                                   unsigned int residualCheckInterval,
                                   double tolerance)
    : m_maxNumberOfIterations{ maxNumberOfIterations },
      m_lastNumberOfIterations{ 0 },
      m_residualCheckInterval{ residualCheckInterval },
      m_tolerance{ tolerance },
      m_lastResidual{ std::numeric_limits<double>::max() }
{
    // Do nothing
}

bool FDMJacobiSolver2::Solve(FDMLinearSystem2* system)
{
    ClearCompressedVectors();

    m_xTemp.Resize(system->x.Size());
    m_residual.Resize(system->x.Size());

    m_lastNumberOfIterations = m_maxNumberOfIterations;

    for (unsigned int iter = 0; iter < m_maxNumberOfIterations; ++iter)
    {
        Relax(system->A, system->b, &system->x, &m_xTemp);
        m_xTemp.Swap(system->x);

        if (iter != 0 && iter % m_residualCheckInterval == 0)
        {
            FDMBLAS2::Residual(system->A, system->x, system->b, &m_residual);

            if (FDMBLAS2::L2Norm(m_residual) < m_tolerance)
            {
                m_lastNumberOfIterations = iter + 1;
                break;
            }
        }
    }

    FDMBLAS2::Residual(system->A, system->x, system->b, &m_residual);
    m_lastResidual = FDMBLAS2::L2Norm(m_residual);

    return m_lastResidual < m_tolerance;
}

bool FDMJacobiSolver2::SolveCompressed(FDMCompressedLinearSystem2* system)
{
    ClearUncompressedVectors();

    m_xTempComp.Resize(system->x.GetRows());
    m_residualComp.Resize(system->x.GetRows());

    m_lastNumberOfIterations = m_maxNumberOfIterations;

    for (unsigned int iter = 0; iter < m_maxNumberOfIterations; ++iter)
    {
        Relax(system->A, system->b, &system->x, &m_xTempComp);

        m_xTempComp.Swap(system->x);

        if (iter != 0 && iter % m_residualCheckInterval == 0)
        {
            FDMCompressedBLAS2::Residual(system->A, system->x, system->b,
                                         &m_residualComp);

            if (FDMCompressedBLAS2::L2Norm(m_residualComp) < m_tolerance)
            {
                m_lastNumberOfIterations = iter + 1;
                break;
            }
        }
    }

    FDMCompressedBLAS2::Residual(system->A, system->x, system->b,
                                 &m_residualComp);
    m_lastResidual = FDMCompressedBLAS2::L2Norm(m_residualComp);

    return m_lastResidual < m_tolerance;
}

unsigned int FDMJacobiSolver2::GetMaxNumberOfIterations() const
{
    return m_maxNumberOfIterations;
}

unsigned int FDMJacobiSolver2::GetLastNumberOfIterations() const
{
    return m_lastNumberOfIterations;
}

double FDMJacobiSolver2::GetTolerance() const
{
    return m_tolerance;
}

double FDMJacobiSolver2::GetLastResidual() const
{
    return m_lastResidual;
}

void FDMJacobiSolver2::Relax(const FDMMatrix2& A, const FDMVector2& b,
                             FDMVector2* x, FDMVector2* xTemp)
{
    Vector2UZ size = A.Size();
    FDMVector2& xRef = *x;
    FDMVector2& xTempRef = *xTemp;

    ParallelForEachIndex(
        size, [&A, &xRef, &size, &xTempRef, &b](size_t i, size_t j) {
            const double r =
                ((i > 0) ? A(i - 1, j).right * xRef(i - 1, j) : 0.0) +
                ((i + 1 < size.x) ? A(i, j).right * xRef(i + 1, j) : 0.0) +
                ((j > 0) ? A(i, j - 1).up * xRef(i, j - 1) : 0.0) +
                ((j + 1 < size.y) ? A(i, j).up * xRef(i, j + 1) : 0.0);

            xTempRef(i, j) = (b(i, j) - r) / A(i, j).center;
        });
}

void FDMJacobiSolver2::Relax(const MatrixCSRD& A, const VectorND& b,
                             VectorND* x, VectorND* xTemp)
{
    const auto rp = A.RowPointersBegin();
    const auto ci = A.ColumnIndicesBegin();
    const auto nnz = A.NonZeroBegin();

    VectorND& xRef = *x;
    VectorND& xTempRef = *xTemp;

    ParallelForEachIndex(b.GetRows(),
                         [&rp, &ci, &nnz, &xRef, &xTempRef, &b](size_t i) {
                             const size_t rowBegin = rp[i];
                             const size_t rowEnd = rp[i + 1];

                             double r = 0.0;
                             double diag = 1.0;
                             for (size_t jj = rowBegin; jj < rowEnd; ++jj)
                             {
                                 const size_t j = ci[jj];

                                 if (i == j)
                                 {
                                     diag = nnz[jj];
                                 }
                                 else
                                 {
                                     r += nnz[jj] * xRef[j];
                                 }
                             }

                             xTempRef[i] = (b[i] - r) / diag;
                         });
}

void FDMJacobiSolver2::ClearUncompressedVectors()
{
    m_xTempComp.Clear();
    m_residualComp.Clear();
}

void FDMJacobiSolver2::ClearCompressedVectors()
{
    m_xTemp.Clear();
    m_residual.Clear();
}
}  // namespace CubbyFlow
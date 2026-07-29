// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Solver/FDM/FDMGaussSeidelSolver2.hpp>

namespace CubbyFlow
{
FDMGaussSeidelSolver2::FDMGaussSeidelSolver2(unsigned int maxNumberOfIterations,
                                             unsigned int residualCheckInterval,
                                             double tolerance, double sorFactor,
                                             bool useRedBlackOrdering)
    : m_maxNumberOfIterations{ maxNumberOfIterations },
      m_lastNumberOfIterations{ 0 },
      m_residualCheckInterval{ residualCheckInterval },
      m_tolerance{ tolerance },
      m_lastResidual{ std::numeric_limits<double>::max() },
      m_sorFactor{ sorFactor },
      m_useRedBlackOrdering{ useRedBlackOrdering }
{
    // Do nothing
}

bool FDMGaussSeidelSolver2::Solve(FDMLinearSystem2* system)
{
    ClearCompressedVectors();

    m_residual.Resize(system->x.Size());

    m_lastNumberOfIterations = m_maxNumberOfIterations;

    for (unsigned int iter = 0; iter < m_maxNumberOfIterations; ++iter)
    {
        if (m_useRedBlackOrdering)
        {
            RelaxRedBlack(system->A, system->b, m_sorFactor, &system->x);
        }
        else
        {
            Relax(system->A, system->b, m_sorFactor, &system->x);
        }

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

bool FDMGaussSeidelSolver2::SolveCompressed(FDMCompressedLinearSystem2* system)
{
    ClearUncompressedVectors();

    m_residualComp.Resize(system->x.GetRows());

    m_lastNumberOfIterations = m_maxNumberOfIterations;

    for (unsigned int iter = 0; iter < m_maxNumberOfIterations; ++iter)
    {
        Relax(system->A, system->b, m_sorFactor, &system->x);

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

unsigned int FDMGaussSeidelSolver2::GetMaxNumberOfIterations() const
{
    return m_maxNumberOfIterations;
}

unsigned int FDMGaussSeidelSolver2::GetLastNumberOfIterations() const
{
    return m_lastNumberOfIterations;
}

double FDMGaussSeidelSolver2::GetTolerance() const
{
    return m_tolerance;
}

double FDMGaussSeidelSolver2::GetLastResidual() const
{
    return m_lastResidual;
}

double FDMGaussSeidelSolver2::GetSORFactor() const
{
    return m_sorFactor;
}

bool FDMGaussSeidelSolver2::GetUseRedBlackOrdering() const
{
    return m_useRedBlackOrdering;
}

void FDMGaussSeidelSolver2::Relax(const FDMMatrix2& A, const FDMVector2& b,
                                  double sorFactor, FDMVector2* x)
{
    Vector2UZ size = A.Size();
    FDMVector2& refX = *x;

    ForEachIndex(size, [&A, &refX, &size, &sorFactor, &b](size_t i, size_t j) {
        const double r =
            ((i > 0) ? A(i - 1, j).right * refX(i - 1, j) : 0.0) +
            ((i + 1 < size.x) ? A(i, j).right * refX(i + 1, j) : 0.0) +
            ((j > 0) ? A(i, j - 1).up * refX(i, j - 1) : 0.0) +
            ((j + 1 < size.y) ? A(i, j).up * refX(i, j + 1) : 0.0);

        refX(i, j) = (1.0 - sorFactor) * refX(i, j) +
                     sorFactor * (b(i, j) - r) / A(i, j).center;
    });
}

void FDMGaussSeidelSolver2::Relax(const MatrixCSRD& A, const VectorND& b,
                                  double sorFactor, VectorND* x)
{
    const auto rp = A.RowPointersBegin();
    const auto ci = A.ColumnIndicesBegin();
    const auto nnz = A.NonZeroBegin();

    VectorND& xRef = *x;

    ForEachIndex(b.GetRows(), [&rp, &ci, &nnz, &xRef, &sorFactor,
                               &b](size_t i) {
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

        xRef[i] = (1.0 - sorFactor) * xRef[i] + sorFactor * (b[i] - r) / diag;
    });
}

void FDMGaussSeidelSolver2::RelaxRedBlack(const FDMMatrix2& A,
                                          const FDMVector2& b, double sorFactor,
                                          FDMVector2* x)
{
    Vector2UZ size = A.Size();
    FDMVector2& xRef = *x;

    // Red update
    ParallelRangeFor(
        ZERO_SIZE, size.x, ZERO_SIZE, size.y,
        [&A, &xRef, &size, &sorFactor, &b](size_t iBegin, size_t iEnd,
                                           size_t jBegin, size_t jEnd) {
            for (size_t j = jBegin; j < jEnd; ++j)
            {
                // i.e. (0, 0)
                size_t i = j % 2 + iBegin;

                for (; i < iEnd; i += 2)
                {
                    const double r =
                        ((i > 0) ? A(i - 1, j).right * xRef(i - 1, j) : 0.0) +
                        ((i + 1 < size.x) ? A(i, j).right * xRef(i + 1, j)
                                          : 0.0) +
                        ((j > 0) ? A(i, j - 1).up * xRef(i, j - 1) : 0.0) +
                        ((j + 1 < size.y) ? A(i, j).up * xRef(i, j + 1) : 0.0);

                    xRef(i, j) = (1.0 - sorFactor) * xRef(i, j) +
                                 sorFactor * (b(i, j) - r) / A(i, j).center;
                }
            }
        });

    // Black update
    ParallelRangeFor(
        ZERO_SIZE, size.x, ZERO_SIZE, size.y,
        [&A, &xRef, &size, &sorFactor, &b](size_t iBegin, size_t iEnd,
                                           size_t jBegin, size_t jEnd) {
            for (size_t j = jBegin; j < jEnd; ++j)
            {
                size_t i = 1 - j % 2 + iBegin;  // i.e. (1, 0)

                for (; i < iEnd; i += 2)
                {
                    const double r =
                        ((i > 0) ? A(i - 1, j).right * xRef(i - 1, j) : 0.0) +
                        ((i + 1 < size.x) ? A(i, j).right * xRef(i + 1, j)
                                          : 0.0) +
                        ((j > 0) ? A(i, j - 1).up * xRef(i, j - 1) : 0.0) +
                        ((j + 1 < size.y) ? A(i, j).up * xRef(i, j + 1) : 0.0);

                    xRef(i, j) = (1.0 - sorFactor) * xRef(i, j) +
                                 sorFactor * (b(i, j) - r) / A(i, j).center;
                }
            }
        });
}

void FDMGaussSeidelSolver2::ClearUncompressedVectors()
{
    m_residual.Clear();
}

void FDMGaussSeidelSolver2::ClearCompressedVectors()
{
    m_residualComp.Clear();
}
}  // namespace CubbyFlow
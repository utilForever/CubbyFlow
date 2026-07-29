// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#include <Core/Solver/FDM/FDMGaussSeidelSolver3.hpp>

namespace CubbyFlow
{
namespace
{
struct RelaxData3
{
    const FDMMatrix3& A;
    const FDMVector3& b;
    double sorFactor;
    FDMVector3* x;
    Vector3UZ size;
};

double NeighborSum(size_t i, size_t j, size_t k, const RelaxData3& data)
{
    return ((i > 0) ? data.A(i - 1, j, k).right * (*data.x)(i - 1, j, k)
                    : 0.0) +
           ((i + 1 < data.size.x)
                ? data.A(i, j, k).right * (*data.x)(i + 1, j, k)
                : 0.0) +
           ((j > 0) ? data.A(i, j - 1, k).up * (*data.x)(i, j - 1, k) : 0.0) +
           ((j + 1 < data.size.y) ? data.A(i, j, k).up * (*data.x)(i, j + 1, k)
                                  : 0.0) +
           ((k > 0) ? data.A(i, j, k - 1).front * (*data.x)(i, j, k - 1)
                    : 0.0) +
           ((k + 1 < data.size.z)
                ? data.A(i, j, k).front * (*data.x)(i, j, k + 1)
                : 0.0);
}

void RelaxPoint(size_t i, size_t j, size_t k, const RelaxData3& data)
{
    const double residual = NeighborSum(i, j, k, data);
    (*data.x)(i, j, k) =
        (1.0 - data.sorFactor) * (*data.x)(i, j, k) +
        data.sorFactor * (data.b(i, j, k) - residual) / data.A(i, j, k).center;
}

void RelaxRange(const RelaxData3& data, size_t color, const Vector3UZ& begin,
                const Vector3UZ& end)
{
    for (size_t k = begin.z; k < end.z; ++k)
    {
        for (size_t j = begin.y; j < end.y; ++j)
        {
            for (size_t i = (color + j + k) % 2 + begin.x; i < end.x; i += 2)
            {
                RelaxPoint(i, j, k, data);
            }
        }
    }
}

void RelaxCompressedRow(size_t i, MatrixCSRD::ConstIndexIterator rowPointers,
                        MatrixCSRD::ConstIndexIterator columnIndices,
                        MatrixCSRD::ConstNonZeroIterator nonZeros,
                        const VectorND& b, double sorFactor, VectorND* x)
{
    double residual = 0.0;
    double diagonal = 1.0;

    for (size_t jj = rowPointers[i]; jj < rowPointers[i + 1]; ++jj)
    {
        const size_t j = columnIndices[jj];

        if (i == j)
        {
            diagonal = nonZeros[jj];
        }
        else
        {
            residual += nonZeros[jj] * (*x)[j];
        }
    }

    (*x)[i] =
        (1.0 - sorFactor) * (*x)[i] + sorFactor * (b[i] - residual) / diagonal;
}
}  // namespace

FDMGaussSeidelSolver3::FDMGaussSeidelSolver3(unsigned int maxNumberOfIterations,
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

bool FDMGaussSeidelSolver3::Solve(FDMLinearSystem3* system)
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
            FDMBLAS3::Residual(system->A, system->x, system->b, &m_residual);

            if (FDMBLAS3::L2Norm(m_residual) < m_tolerance)
            {
                m_lastNumberOfIterations = iter + 1;
                break;
            }
        }
    }

    FDMBLAS3::Residual(system->A, system->x, system->b, &m_residual);
    m_lastResidual = FDMBLAS3::L2Norm(m_residual);

    return m_lastResidual < m_tolerance;
}

bool FDMGaussSeidelSolver3::SolveCompressed(FDMCompressedLinearSystem3* system)
{
    ClearUncompressedVectors();

    m_residualComp.Resize(system->x.GetRows());

    m_lastNumberOfIterations = m_maxNumberOfIterations;

    for (unsigned int iter = 0; iter < m_maxNumberOfIterations; ++iter)
    {
        Relax(system->A, system->b, m_sorFactor, &system->x);

        if (iter != 0 && iter % m_residualCheckInterval == 0)
        {
            FDMCompressedBLAS3::Residual(system->A, system->x, system->b,
                                         &m_residualComp);

            if (FDMCompressedBLAS3::L2Norm(m_residualComp) < m_tolerance)
            {
                m_lastNumberOfIterations = iter + 1;
                break;
            }
        }
    }

    FDMCompressedBLAS3::Residual(system->A, system->x, system->b,
                                 &m_residualComp);
    m_lastResidual = FDMCompressedBLAS3::L2Norm(m_residualComp);

    return m_lastResidual < m_tolerance;
}

unsigned int FDMGaussSeidelSolver3::GetMaxNumberOfIterations() const
{
    return m_maxNumberOfIterations;
}

unsigned int FDMGaussSeidelSolver3::GetLastNumberOfIterations() const
{
    return m_lastNumberOfIterations;
}

double FDMGaussSeidelSolver3::GetTolerance() const
{
    return m_tolerance;
}

double FDMGaussSeidelSolver3::GetLastResidual() const
{
    return m_lastResidual;
}

double FDMGaussSeidelSolver3::GetSORFactor() const
{
    return m_sorFactor;
}

bool FDMGaussSeidelSolver3::GetUseRedBlackOrdering() const
{
    return m_useRedBlackOrdering;
}

void FDMGaussSeidelSolver3::Relax(const FDMMatrix3& A, const FDMVector3& b,
                                  double sorFactor, FDMVector3* x)
{
    Vector3UZ size = A.Size();
    FDMVector3& xRef = *x;

    ForEachIndex(size, [&A, &xRef, &size, &sorFactor, &b](size_t i, size_t j,
                                                          size_t k) {
        const double r =
            ((i > 0) ? A(i - 1, j, k).right * xRef(i - 1, j, k) : 0.0) +
            ((i + 1 < size.x) ? A(i, j, k).right * xRef(i + 1, j, k) : 0.0) +
            ((j > 0) ? A(i, j - 1, k).up * xRef(i, j - 1, k) : 0.0) +
            ((j + 1 < size.y) ? A(i, j, k).up * xRef(i, j + 1, k) : 0.0) +
            ((k > 0) ? A(i, j, k - 1).front * xRef(i, j, k - 1) : 0.0) +
            ((k + 1 < size.z) ? A(i, j, k).front * xRef(i, j, k + 1) : 0.0);

        xRef(i, j, k) = (1.0 - sorFactor) * xRef(i, j, k) +
                        sorFactor * (b(i, j, k) - r) / A(i, j, k).center;
    });
}

void FDMGaussSeidelSolver3::Relax(const MatrixCSRD& A, const VectorND& b,
                                  double sorFactor, VectorND* x)
{
    const auto rp = A.RowPointersBegin();
    const auto ci = A.ColumnIndicesBegin();
    const auto nnz = A.NonZeroBegin();

    VectorND& xRef = *x;

    ForEachIndex(b.GetRows(),
                 [&rp, &ci, &nnz, &xRef, &sorFactor, &b](size_t i) {
                     RelaxCompressedRow(i, rp, ci, nnz, b, sorFactor, &xRef);
                 });
}

void FDMGaussSeidelSolver3::RelaxRedBlack(const FDMMatrix3& A,
                                          const FDMVector3& b, double sorFactor,
                                          FDMVector3* x)
{
    Vector3UZ size = A.Size();
    FDMVector3& xRef = *x;
    const RelaxData3 data{ A, b, sorFactor, &xRef, size };

    for (size_t color = 0; color < 2; ++color)
    {
        ParallelRangeFor(
            ZERO_SIZE, size.x, ZERO_SIZE, size.y, ZERO_SIZE, size.z,
            [&data, color](size_t iBegin, size_t iEnd, size_t jBegin,
                           size_t jEnd, size_t kBegin, size_t kEnd) {
                RelaxRange(data, color, { iBegin, jBegin, kBegin },
                           { iEnd, jEnd, kEnd });
            });
    }
}

void FDMGaussSeidelSolver3::ClearUncompressedVectors()
{
    m_residual.Clear();
}

void FDMGaussSeidelSolver3::ClearCompressedVectors()
{
    m_residualComp.Clear();
}
}  // namespace CubbyFlow
